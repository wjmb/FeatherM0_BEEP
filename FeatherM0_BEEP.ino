/*******************************************************************************
 * wjmb Feather M0 Beehive LoRa code
 *
 * Many thanks to Maarten Westenberg, Thomas Telkamp and Matthijs Kooijman 
 * porting the LMIC stack to Arduino IDE and Gerben den Hartog for his tiny 
 * stack implementation with the AES library that was used in the LMIC stack.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (30s/day airtime)
 * 
 * Do not forget to define the radio type correctly in config.h. 
 * 
 * For a good overview of what to do and what not to forget, see:
 * https://community.hiveeyes.org/t/using-the-adafruit-feather-m0-lora-rfm95-and-ttn/528
 * 
 * TODO:
 * - Look into things that can be done to lower power during inactive moments.
 *   (use RTCZero.h to detach USB and set in standbyMode())???
 * - Dynamic calculation of TX_INTERVAl to implement 10s/day airtime TTN fair use.
 *******************************************************************************/

#include <lmic.h>             // https://github.com/matthijskooijman/arduino-lmic
#include <hal/hal.h>          // https://github.com/matthijskooijman/arduino-lmic
#include <SPI.h>              // https://www.arduino.cc/en/Reference/SPI
#include <Adafruit_Sensor.h>  // https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_BME280.h>  // https://github.com/adafruit/Adafruit_BME280_Library
#include <Adafruit_ZeroFFT.h> // https://github.com/adafruit/Adafruit_ZeroFFT
#include <Adafruit_ZeroI2S.h> // https://github.com/adafruit/Adafruit_ZeroI2S
#include <math.h>
#include "ttn_credentials.h"  // this is where the APPEUI, DEVEUI and APPKEY are defined

// Pin mapping
#define REDLED 13
#define VBATPIN A7 // ugh, pin also used by I2S

const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, 11},
};

// ttn_credentials.h contains APPEUI, DEVEUI and APPKEY; makes it easier to share code without sharing credentials
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty cycle limitations).
// TTN fair use policy: 30 seconds per day airtime.
const unsigned TX_INTERVAL = 150;
const unsigned TX_PER_REJOIN = 24*3600/TX_INTERVAL;

// Samplerate must be a multiple of (MasterClock/64). For SAMD21, 48MHz/samplerate must 
// therefore be a multiple of 64. 
#define SAMPLE_RATE 12500 // max freq detectable is half of this according to Nyquist
#define DATA_SIZE 1024 // needs to be a multiple of 64.
#define SPECTRUM_SIZE (DATA_SIZE/2) // half of DATA_SIZE.
uint8_t NR_OF_SPECTRA = 1; // nr of freq. spectra to add and get a less discretised spectrum

// for condensing the spectrum into a course spectrum to be broadcast over LoRa
#define NR_OF_BINS_TO_COMBINE 4 // nr of bins to combine to get a course spectrum.
#define STARTBIN 8 // 98 Hz lower frequency of the relevant freq window
#define ENDBIN 48 // 586 Hz upper frequency of the relevant freq window
#define NR_OF_COURSE_BINS ((ENDBIN-STARTBIN)/NR_OF_BINS_TO_COMBINE)

// BME280 over i2c
Adafruit_BME280 bme; // I2C

/* create a buffer for both the left and right channel data */
int32_t left[DATA_SIZE];
int32_t right[DATA_SIZE];

Adafruit_ZeroI2S i2s(0, 1, 9, 2);

// sensor data (not all used yet)
int16_t t = 0; // temperature (degrC*100)
int16_t t_i = 0; // temperature inside (degrC*100)
uint8_t h = 0; // humidity (perc)
uint16_t p = 0; // pressure (hPa)
uint16_t bv = 0; // battery voltage (mV)
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
            // LMIC_setLinkCheckMode(0);
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
              Serial.print(F("Received ")); Serial.print(LMIC.dataLen); Serial.println(F(" bytes of payload (expecting one byte)"));
              uint8_t downlink[LMIC.dataLen];
              memcpy(&downlink,&(LMIC.frame+LMIC.dataBeg)[0],LMIC.dataLen);
              // so we take the first byte and use as a kind of gain parameter.
              NR_OF_SPECTRA = downlink[0];
              Serial.print(F(" NR_OF_SPECTRA set to: ")); Serial.println(NR_OF_SPECTRA);
            }
            if (LMIC.seqnoUp>TX_PER_REJOIN) {
              Serial.println(F(" Resetting and rejoining "));
              LMIC_reset();
              LMIC_startJoining();
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            //for (int i=0; i<int(TX_INTERVAL/8); i++) {
            //  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
            //}
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
  t_i = round(bme.readTemperature() * 100);
  Serial.print(F("t_i (x100 degreeC) : ")); Serial.println(t_i);
  h = round(bme.readHumidity());
  Serial.print(F("h   (%)            : ")); Serial.println(h);
  p = round(bme.readPressure()-87000);
  Serial.print(F("p   (hPa)          : ")); Serial.println((p+87000)/100.0);
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
  Serial.print(F("bv  (mV)           : ")); Serial.println(bv);
}

void get_weight()
{
}

void get_audiosample(int32_t *sampledata)
{  
  for (unsigned i=0; i<DATA_SIZE; ++i) i2s.read(&left[i], &right[i]);
  for (int i = 0; i<DATA_SIZE; i++) 
  {
    left[i] >>= 7;
    right[i] >>= 7;
    // FOR SOME REALLY ODD REASON, DATA COMES IN OCCASIONALLY ON THE LEFT AND OCCASIONALLY ON THE RIGHT CHANNEL.
    // WORKAROUND! the other channel contains -1 or 0 (not sure why), so adding it gives only as small deviation. 
    sampledata[i] = left[i]+right[i];
  }
}

void get_audiodata()
{
  uint16_t spctrm[SPECTRUM_SIZE];
  memset(spctrm,0,sizeof(spctrm));
  uint64_t ss = 0; // sum of squares, 24 bits squared is 48 bits range, so need >32 bits
    
  for (int i=0; i<NR_OF_SPECTRA; i++) 
  {
    int32_t data[DATA_SIZE];
    // sample microphone.
    get_audiosample(data);
    
    // take out the DC component, calculate max ampl, and rms value.
    int32_t mean = 0; // we sum 24 bits oscillating data 2^10 times, so max range is 34 bits. 32 bits is sufficient.
    for (int j=0; j<DATA_SIZE; j++) mean += data[j];
    mean/=DATA_SIZE;
    
    uint32_t mx = 0; // max value
    for (int j=0; j<DATA_SIZE; j++) 
    {
      data[j] -= mean;
      ss += data[j]*data[j]; 
      if (abs(data[j])>mx) mx=abs(data[j]);
    }

    // need to fit 24 bits into 16 bits for FFT, so we scale:
    byte shift=0;
    while (mx>((1UL<<15)-1)) 
    {
      shift++;
      mx>>=1;
    }
    int16_t data16[DATA_SIZE];
    for (int j=0; j<DATA_SIZE; j++) data16[j] = data[j]>>shift;
    
    ZeroFFT(data16, DATA_SIZE);

    // WRONG: cannot just add spectra of different scaling factors! Guess, we could add them shifted back again???
    for (int j=0; j<SPECTRUM_SIZE; j++) spctrm[j]+=data16[j]; // potential overflow, I guess, we could devide by NR_OF_SPECTRA.
  }
  
  uint32_t rms = sqrt(ss/DATA_SIZE);
  const int FULL_SCALE_DBSPL = 120; // FULL SCALE dBSPL (AOP = 116dB SPL)
  const double FULL_SCALE_DBFS = 20*log10(pow(2,23)); // BIT LENGTH = 24 for ICS43432 ... or should this be 23 (half range)
  s_SPL = 100*(FULL_SCALE_DBSPL-(FULL_SCALE_DBFS-20*log10(sqrt(2) * rms)));
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

void blink(uint16_t length)
{
  digitalWrite(REDLED, HIGH);
  delay(length);
  digitalWrite(REDLED, LOW);
  delay(length);
}

void do_send(osjob_t* j)
{
  Serial.print(F("seqnoUp            : ")); Serial.println(LMIC.seqnoUp);
  // Read sensor values
  //get_battery();
  get_temperature();
  get_audiodata();
  // prepare data to be sent
  byte mydata[7+NR_OF_COURSE_BINS];
  mydata[0] = t_i >> 8;
  mydata[1] = t_i;
  mydata[2] = p >> 8;
  mydata[3] = p;
  mydata[4] = h;
  mydata[5] = s_SPL >> 8;
  mydata[6] = s_SPL;
  for (unsigned i=0;i<NR_OF_COURSE_BINS;i++) mydata[7+i] = spectrum[i];
    
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
  delay(10000); // to allow me to set the serial console.
  
  pinMode(REDLED, OUTPUT);
  
  Serial.begin(115200);
  Serial.println(F("Starting"));
  
  // start temperature sensor.
  bool bme_status;
    
  // (you can also pass in a Wire library object like &Wire2)
  bme_status = bme.begin();  
  if (!bme_status) {
        Serial.println("Could not find a valid BME280 sensor");
  }

  // start I2S with 32-bits per sample, as needed by SPH0645LM4H-B and ICS43432.
  i2s.begin(I2S_32_BIT, SAMPLE_RATE);
  i2s.enableRx();
  
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
  // tell LMIC to make the receive windows bigger, in case your clock is 1% faster or slower.  
  LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
  // join
  LMIC_startJoining();
  blink(200); blink(200); blink(400);
  // Some test today show that using the following will NOT make TTN send ADR:
  LMIC_setAdrMode(1);
  // Does TTN support LinkCheckMode?
  LMIC_setLinkCheckMode(1);
}

void loop() 
{
  os_runloop_once();
}
