/*******************************************************************************
 * wjmb Feather M0 Beehive LoRa code
 *
 * Many thanks to Maarten Westenberg, Thomas Telkamp and Matthijs Kooijman 
 * porting the LMIC stack to Arduino IDE and Gerben den Hartog for his tiny 
 * stack implementation with the AES library that we used in the LMIC stack.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (30s/day airtime)
 * 
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h. 
 * 
 * For a good overview of what to do and what not to forget, see:
 * https://community.hiveeyes.org/t/using-the-adafruit-feather-m0-lora-rfm95-and-ttn/528
 * 
 * TODO:
 * - Look into thing that can be done to lower power during inactive moments.
 *   (use RTCZero.h to detach USB and set in standbyMode())???
 * - Receive parameter from TTN to adjust "audio gain" via a parameter called
 *   NR_OF_SPECTRA. Somehow Rx doesn't receive in combination with I2S ... not sure 
 *   where the collision is. Perhaps in the interrrupt disabling of I2S.read(). 
 * - Read more literature to figure out which spectral features to extract. 
 * - Multiple temperature sensors (need different library for that, or other pin).
 * - Dynamic calculation of TX_INTERVAl to implement 10s/day airtime TTN fair use.
 *******************************************************************************/
#include <lmic.h>             // https://github.com/matthijskooijman/arduino-lmic
#include <hal/hal.h>          // https://github.com/matthijskooijman/arduino-lmic
#include <SPI.h>              // https://www.arduino.cc/en/Reference/SPI
#include <OneWire.h>          // https://www.pjrc.com/teensy/td_libs_OneWire.html
#include <DS18B20.h>          // https://github.com/RobTillaart/Arduino/tree/master/libraries/DS18B20
#include <I2S.h>              // https://www.arduino.cc/en/Reference/I2S .. part of "Adafruit SAMD Boards"
#include <Adafruit_ZeroFFT.h> // https://github.com/adafruit/Adafruit_ZeroFFT
#include "ttn_credentials.h"

// Pin mapping
#define REDLED 13
#define ONEW_BUS 10
#define VBATPIN A7

const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, 11},
};

void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
// TTN fair use policy: 30 seconds per day airtime ... so 625ms per message every half hour
const unsigned TX_INTERVAL = 1800;

// Sound: according to the data sheet of the SPH0645LM4H-B, the sample rate must be between 32khz  
// and 64khz. It also must be a multiple of (MasterClock/64). For SAMD21, 48MHz/samplerate must 
// therefore be a multiple of 64. According to Adafruit lower sampling rates than 32kHz can be  
// used. No luck under 3000Hz though.
// Warning: if SAMPLE_RATE and/or DATA_SIZE is/are altered, noisefloor is not valid anymore !!!!
#define SAMPLE_RATE 3125 // max freq detectable is half of this according to Nyquist
#define DATA_SIZE 256 // needs to be a multiple of 64.
#define SPECTRUM_SIZE (DATA_SIZE/2) // half of DATA_SIZE.
uint8_t NR_OF_SPECTRA = 32; // nr of freq. spectra to add and get a less discretised spectrum
#define NR_OF_BINS_TO_COMBINE 4 // nr of bins to combine to get a course spectrum. ~50 Hz with 3125/512
#define STARTBIN 8 // 97.66 Hz lower frequency of the relevant freq window
#define ENDBIN 48 // 585.94 Hz upper frequency of the relevant freq window
#define NR_OF_COURSE_BINS ((ENDBIN-STARTBIN)/NR_OF_BINS_TO_COMBINE)

// for the above values SAMPLERATE 3125, DATASIZE 256, NR_OF_SPECTRA 32, this is the noise floor 
// if measured in a "silent" space. 30 spectra were recorded and this is the minumum value of each bin.
uint16_t noisefloor[SPECTRUM_SIZE] = {93,64,58,66,54,66,61,70,23,26,46,45,48,62,58,65,17,0,17,23,23,
                                      35,30,34,25,24,36,39,38,50,45,53,5,0,5,6,6,12,10,12,6,8,12,16,
                                      8,14,19,13,12,1,12,14,12,20,23,19,11,15,18,21,23,27,28,30,0,0,
                                      0,1,0,2,0,2,1,1,2,1,2,2,3,3,0,0,1,3,2,4,4,4,0,1,3,2,1,2,5,3,2,
                                      0,2,2,1,3,2,1,2,2,5,6,3,3,4,3,3,0,2,4,3,3,6,3,0,2,3,4,5,4,5,7 };

// One wire
OneWire onewire(ONEW_BUS);
DS18B20 ds18b20_sensor(&onewire);
#define NR_OF_TSENSORS 1;

// I2S variables.
uint8_t I2Sbuffer[512];
volatile int I2Savailable = 0;

// sensor data (not all used yet)
int16_t t = 0; // temperature (degrC*100)
int16_t t_i = 0; // temperature inside (degrC*100)
uint8_t h = 0; // humidity (perc)
uint16_t bv = 0; // battery voltage (mV)
uint16_t s_tot = 0; // not sure what the unit is coming out of the FFT .. power, pressure, energy??
uint16_t s_SPL = 0; // sound pressure level (dB) ... uncalibrated though.
uint8_t spectrum[NR_OF_COURSE_BINS]; // spectrum in relevant range

void onEvent (ev_t ev) 
{
    Serial.print((float)os_getTime()/OSTICKS_PER_SEC); // OSTICKS_PER_SEC = 62500
    Serial.print("s: ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            // Disable link check validation (automatically enabled
            // during join, but not supported by TTN at this time).
            LMIC_setLinkCheckMode(0);
            // after join succeeded, we can start sending.
            do_send(&sendjob);
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break; // what is the purpose of this second break?
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              // for some reason this is never the case.
              Serial.print(F("Received ")); Serial.print(LMIC.dataLen); Serial.println(F(" bytes of payload (expecting one byte)"));
              uint8_t downlink[LMIC.dataLen];
              memcpy(&downlink,&(LMIC.frame+LMIC.dataBeg)[0],LMIC.dataLen);
              // so we take the first byte
              NR_OF_SPECTRA = downlink[0];
              Serial.print(F(" NR_OF_SPECTRA set to: ")); Serial.println(NR_OF_SPECTRA);
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void get_temperature()
{
  ds18b20_sensor.requestTemperatures();
  while (!ds18b20_sensor.isConversionComplete());  // wait until sensor is ready
  t_i = round(ds18b20_sensor.getTempC() * 100);
  Serial.print(F("t_i (x100 degreeC) : ")); Serial.println(t_i);
}

void get_battery()
{
  // whoops: A7 pin = pin 9 is used for microphone. I2S.end() sets this pin to input
  // again, so this should work.
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2 (voltage divider), so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  bv = round(measuredvbat*1000);
  Serial.print(F("bv (mV)            : ")); Serial.println(bv);
}

void get_weight()
{
}

void onI2SReceive() 
{
  // This function will run at a frequency of (sampleRate / 64). At 3.125khz, this is every 20 ms
  // so make sure this is called again within that time if a contiguous set of data is needed.
  I2S.read(I2Sbuffer, 512);
  I2Savailable = 1;
}

void get_audiosample(int16_t *sampledata)
{  
  while (!I2Savailable); // wait until data is available again.
  int *values = (int *) I2Sbuffer;
  float avg = 0;
  for (int i=0; i<64; i++) 
  {
    // If the SEL pin is low, samples will be in odd numbered positions.
    // If you connect SEL to high, data will be in even positions.
    sampledata[i] = values[(2*i) + 1] >> 14;
    avg += sampledata[i];
  }
  avg /= 64;
  for (int i=0; i<64; i++) sampledata[i] -= avg;
  I2Savailable = 0;
}

void get_audiodata()
{
  // start I2S with 32-bits per sample, as needed by SPH0645LM4H-B.
  I2S.onReceive(onI2SReceive);
  if (!I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 32)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
  I2S.read(); // apparently this get things going.
  delay(10000); // this microphone/I2S thing has a huge startup effect :-( Fortunately we have time.
  
  int16_t data[DATA_SIZE]; // is there the risk of overflow, because the SPH0645LM4H-B returns 18 bits?
  uint16_t spctrm[SPECTRUM_SIZE]; 
    
  // collect microphone input, determine RMS of raw data and do FFT. We simply take a number of spectra 
  // and average these. Alternatives would be to calculate a moving average or exponential smoothing on the spectrum.
  memset(spctrm,0,sizeof(spctrm));
  uint32_t rms = 0;
  for (int i=0; i<NR_OF_SPECTRA; i++) 
  {
    // collect data .. 64 values at the time.
    for (int j=0; j<DATA_SIZE/64; j++) get_audiosample(&data[j*64]);
    for (int j=0; j<DATA_SIZE; j++) rms += data[j]*data[j];
    //for (int j=0; j<DATA_SIZE; j++) Serial.println(data[j]);
    // run the FFT
    ZeroFFT(data, DATA_SIZE);
    // add spectrum to array used to average a number of them. No need to divide by NR_OF_SPECTRA as uint16_t is large enough
    for (int j=0; j<SPECTRUM_SIZE; j++) spctrm[j]+=data[j];
  }
  rms = sqrt(rms/(NR_OF_SPECTRA*DATA_SIZE));
  I2S.end();
  
  for (int i=0; i<SPECTRUM_SIZE; i++) 
  {
    uint16_t nfl = noisefloor[i]*(NR_OF_SPECTRA/32); // to keep the possibility open that NR_OF_SPECTRA is altered by downlink message.
    if (spctrm[i]<=nfl) spctrm[i]=0; // in case the noisefloor isn't the absolute minimum.
    else spctrm[i] -= nfl;
  }
  /*
  for (int i=0; i<SPECTRUM_SIZE; i++) { 
    Serial.print(FFT_BIN(i,SAMPLE_RATE, DATA_SIZE)); 
    Serial.print(" "); 
    Serial.println(spctrm[i]); } // debug
  */
  // SPH0645LM4H-B: -26 dBFS Sensitivity and 65 dBA signal to noise (both at 94 dB SPL) 
  // Therefore, full scale = 94 dB + 26 dB = 120 dB = FS
  // SNR w.r.t. 94 dB reference -> 94 dB - 65 dBA = 29 dB noise floor (approx)
  // FS = 2^17-1 = 131071 (if 18 bits signed is true .. with this strange DC component).
  s_SPL = 100*(float)(20.0*log10(sqrt(2)*rms/131071)+120); // uncalibrated.
  Serial.print(F("s_SPL (x100 dB)    : ")); Serial.println(s_SPL);

  // Make a condensed and normalised version of the relevant part of spectrum [STARTBIN,ENDBIN).
  uint32_t cspctrm[NR_OF_COURSE_BINS]; // course spectrum (q15_t overflows)
  memset(cspctrm,0,sizeof(cspctrm));
  uint32_t maxval=0;
  float sum=0;
  for (int i=0; i<NR_OF_COURSE_BINS; i++) 
  {
    for (int j=0; j<NR_OF_BINS_TO_COMBINE; j++) 
    {
      cspctrm[i] += spctrm[(STARTBIN+i*NR_OF_BINS_TO_COMBINE+j)];
    }
    if (cspctrm[i]>maxval) maxval=cspctrm[i];
    sum += cspctrm[i];
  }
  s_tot = sum/NR_OF_BINS_TO_COMBINE;
    
  Serial.print(F("s_tot (a.u.):      : ")); Serial.println(s_tot);
  for (int i=0; i<NR_OF_COURSE_BINS; i++) 
  {
    spectrum[i] = 255*((float)cspctrm[i]/maxval);
    // a messy set of lines, but it displays what is needed:
    Serial.print(F("s_bin")); if (FFT_BIN((STARTBIN+i*NR_OF_BINS_TO_COMBINE), SAMPLE_RATE, DATA_SIZE)<100) Serial.print("0"); 
      Serial.print(FFT_BIN((STARTBIN+i*NR_OF_BINS_TO_COMBINE), SAMPLE_RATE, DATA_SIZE),0); Serial.print("_"); 
      Serial.print(FFT_BIN((STARTBIN+(i+1)*NR_OF_BINS_TO_COMBINE), SAMPLE_RATE, DATA_SIZE),0); Serial.print(F("Hz     : ")); 
      Serial.println(spectrum[i]);
  } 
}

extern "C" char *sbrk(int i);
int FreeRam () 
{
  // for checking out how much RAM we have.
  char stack_dummy = 0;
  return &stack_dummy - sbrk(0);
}

void blink()
{
  digitalWrite(REDLED, HIGH);
  delay(200);
  digitalWrite(REDLED, LOW);
  delay(200);
}

void do_send(osjob_t* j)
{
  // Read sensor values
  get_battery();
  get_temperature();
  get_weight();
  get_audiodata();

  // prepare data to be sent
  byte mydata[6+NR_OF_COURSE_BINS];
  mydata[0] = t_i >> 8;
  mydata[1] = t_i;
  mydata[2] = s_SPL >> 8;
  mydata[3] = s_SPL;
  mydata[4] = s_tot >> 8;
  mydata[5] = s_tot;
  for (unsigned i=0;i<NR_OF_COURSE_BINS;i++)
  {
    mydata[6+i] = spectrum[i];
  }

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
      // Prepare upstream data transmission at the next possible time.
      LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
      Serial.print((float)os_getTime()/OSTICKS_PER_SEC); Serial.println(F("s: Packet queued"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void setup() 
{
  delay(10000); // wjmb: to allow me to set the serial console.
  
  pinMode(REDLED, OUTPUT);
  
  Serial.begin(9600);
  Serial.println(F("Starting"));
  
  // start temperature sensor.
  ds18b20_sensor.begin();
  ds18b20_sensor.setResolution(9);
    
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // tell LMIC to make the receive windows bigger, in case your clock is 1% faster or slower.  
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // join
  LMIC_startJoining();
  blink(); blink(); blink();
  // Some test today show that using the following will NOT make TTN send ADR:
  //LMIC_setAdrMode(1);
  //LMIC_setLinkCheckMode(0);
  // Leaving out the latter makes ADR work just great.
}

void loop() 
{
  os_runloop_once();
}
