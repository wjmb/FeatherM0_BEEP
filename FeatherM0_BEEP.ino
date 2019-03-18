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

#define LORA // use LoRa or, if not defined, just run the sensor measurements and report to Serial

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
unsigned TX_INTERVAL = 600;
const unsigned TX_PER_REJOIN = 24*3600/TX_INTERVAL;

// Samplerate must be a multiple of (MasterClock/64). For SAMD21, 48MHz/samplerate must 
// therefore be a multiple of 64. Be carefull: these parameters change affect calc_spectralcode();
// TODO: nomenclature is somewhat confusing.
#define SAMPLE_RATE 6250 // 6.25 kHz is the minumum sampling frequency of ICS43434
#define SAMPLE_SIZE 1024 // needs to be a multiple of 64.
#define SPECTRUM_SIZE (SAMPLE_SIZE/2) // half of SAMPLE_SIZE.
#define NR_OF_SAMPLES 100 // nr of times we take an audiosample (should be at least 120).
#define NR_OF_SPECTRA 10 // nr of spectra we average over.
// 1000 samples of 1024 size and 6250 rate, including FFT, takes approx 360 seconds

// for condensing the spectrum into a course spectrum to be broadcast over LoRa
#define NR_OF_BINS_TO_COMBINE 8 // nr of bins to combine to get a course spectrum.
#define STARTBIN 16 // 98 Hz lower frequency of the relevant freq window
#define ENDBIN 96 // 586 Hz upper frequency of the relevant freq window
#define NR_OF_COURSE_BINS ((ENDBIN-STARTBIN)/NR_OF_BINS_TO_COMBINE)

// BME280 over i2c
Adafruit_BME280 bme; // I2C

/* create a buffer for both the left and right channel data */
int32_t left[SAMPLE_SIZE];
int32_t right[SAMPLE_SIZE];

Adafruit_ZeroI2S i2s(0, 1, 9, 2);

// sensor data (not all used yet)
int16_t t = 0; // temperature (degrC*100)
int16_t t_i = 0; // temperature inside (degrC*100)
uint8_t h = 0; // humidity (perc)
uint16_t p = 0; // pressure (hPa)
uint16_t bv = 0; // battery voltage (mV)
uint16_t s_SPL = 0; // sound pressure level (dB) ... uncalibrated though.
uint8_t s_rugo = 0; // rugosity.
uint8_t s_spectrum[NR_OF_COURSE_BINS]; // spectrum in relevant range.
uint8_t s_code[64]; // spectral codes ordered in dominance.
uint16_t s_codehistogram[64]; // frequency of these codes.

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
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received ")); Serial.print(LMIC.dataLen); Serial.println(F(" bytes of payload"));
              uint8_t dwnlnk[LMIC.dataLen];
              memcpy(&dwnlnk,&(LMIC.frame+LMIC.dataBeg)[0],LMIC.dataLen);
              if (LMIC.dataLen==2)
              {
                TX_INTERVAL = (dwnlnk[0]<<8) | dwnlnk[1];
                if (TX_INTERVAL>1800) TX_INTERVAL=1800; // safetynet ... no more than half an hour
              }
              Serial.print(F(" TX_INTERVAL set to: ")); Serial.println(TX_INTERVAL);
            }
            if (LMIC.seqnoUp>TX_PER_REJOIN) {
              Serial.println(F(" Resetting and rejoining "));
              LMIC_reset();
              LMIC_startJoining();
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

void get_audiosample(int32_t *data)
{  
  // first read the data.
  for (uint16_t i=0; i<SAMPLE_SIZE; ++i) i2s.read(&left[i], &right[i]);
  
  // then loop over it and shift it.
  for (uint16_t i=0; i<SAMPLE_SIZE; i++) 
  {
    left[i] >>= 7;
    right[i] >>= 7;
    // FOR SOME REALLY ODD REASON, DATA COMES IN OCCASIONALLY ON THE LEFT AND OCCASIONALLY ON THE RIGHT CHANNEL.
    // WORKAROUND! the other channel contains -1 or 0 (not sure why), so adding it gives only as small deviation. 
    data[i] = left[i]+right[i];
  }
  
  // take out the DC component (why actually?)
  int32_t mean = 0; // we sum 24 bits oscillating data 2^10 times, so max range is 34 bits. 32 bits is sufficient.
  for (uint16_t i=0; i<SAMPLE_SIZE; i++) 
  {
    mean += data[i];
  }
  mean/=SAMPLE_SIZE;
  for (uint16_t i=0; i<SAMPLE_SIZE; i++) 
  {
    data[i] -= mean;
  }
}

uint8_t get_audiospectrum(int32_t *data, uint16_t *spectrum)
{
  // expensive in terms of RAM to make copies.
  uint32_t mx = 0; // max value
  for (uint16_t i=0; i<SAMPLE_SIZE; i++) 
  {
    if (abs(data[i])>mx) mx=abs(data[i]);
  }

  // need to fit 24 bits into 16 bits for FFT, so we scale:
  uint8_t shift=0;
  while (mx>((1UL<<15)-1)) 
  {
    shift++;
    mx>>=1;
  }
  int16_t data16[SAMPLE_SIZE];
  
  for (uint16_t i=0; i<SAMPLE_SIZE; i++) 
  {
    data16[i] = data[i]>>shift;
  }
    
  ZeroFFT(data16, SAMPLE_SIZE);

  for (uint16_t i=0; i<SPECTRUM_SIZE; i++) 
  {
    spectrum[i]=data16[i];
  }
  
  //for (uint16_t j=0; j<SPECTRUM_SIZE; j++) { Serial.print(FFT_BIN(j, SAMPLE_RATE, SAMPLE_SIZE)); Serial.print("\t"); Serial.println(data16[j]); }
  //Serial.print(F("FreeRam (fft)      : ")); Serial.println(FreeRam());
  
  return shift;
}

double calc_dB(double sos, uint32_t ss) // sum of squares , sample size
{
  const double rms = sqrt(sos/ss);
  const uint8_t FULL_SCALE_DBSPL = 120; // FULL SCALE dBSPL (AOP = 116dB SPL)
  const double FULL_SCALE_DBFS = 20*log10(pow(2,24)); // BIT LENGTH = 24 for ICS43432
  return FULL_SCALE_DBSPL-(FULL_SCALE_DBFS-20*log10(sqrt(2) * rms));  
}

double calc_dB(int32_t *data)
{
  double sos = 0; 
  for (int i=0; i<SAMPLE_SIZE; i++) 
  {
    sos += pow(data[i],2); 
    // sos += data[i]*data[i]; // this produces ovf sometimes. WHY?
  }
  return calc_dB(sos, SAMPLE_SIZE);
}

double calc_spectralentropy(uint32_t *spectrum)
{
  double se = 0;
  double sum = 0;
  // calculate normalisation coefficient:
  for (uint16_t i=0; i<SPECTRUM_SIZE;i++)
  {
    sum += spectrum[i];
  }
  
  for (uint16_t i=0; i<SPECTRUM_SIZE;i++)
  {
    // log_x(n) = log_y(n) / log_y(x)
    const double p = spectrum[i]/sum;
    if (p) se -= p*(log10(p)/log10(2.0));
  }
  return se;
}

uint8_t calc_spectralcode(uint32_t *spectrum)
{
  uint32_t u1 = 0;
  uint32_t u2 = 0;
  uint32_t u3 = 0;
  uint32_t u4 = 0;

  // This classifies the spectrum in 24 "shapes". According to publications by A.F. Rybochkin et al. from Kursk. 
  // Moshen2015_methods.... lists 205-225, 280-300, 325-345, and 390-410 Hz windows.
  // Moshen2015_avtoreferat lists 215-245, 295-325, 365-395, and 405-435 Hz windows.

  if (SAMPLE_SIZE==512) 
  {
    // salient frequency windows. // these frequency windows are pretty hardcoded (for 512 samples at 6250 Hz)
    u1 = spectrum[17]+spectrum[18]; // approx 205-225 Hz
    u2 = spectrum[23]+spectrum[24]; // approx 280-300 Hz
    u3 = spectrum[27]+spectrum[28]; // approx 325-345 Hz
    u4 = spectrum[32]+spectrum[33]; // approx 390-410 Hz
  }
  else if (SAMPLE_SIZE==1024)
  {
    // salient frequency windows. // these frequency windows are pretty hardcoded (for 1024 samples at 6250 Hz)
    u1 = spectrum[33]+spectrum[34]+spectrum[35]+spectrum[36]; // approx 205-225 Hz
    u2 = spectrum[46]+spectrum[47]+spectrum[48]+spectrum[49]; // approx 280-300 Hz
    u3 = spectrum[53]+spectrum[54]+spectrum[55]+spectrum[56]; // approx 325-345 Hz
    u4 = spectrum[64]+spectrum[65]+spectrum[66]+spectrum[67]; // approx 390-410 Hz
  }
  else Serial.println(F("SAMPLE_SIZE doesn't match calc_spectralcode() implementation!"));

  // comperator bank.
  uint8_t code = 0;
  code+=(u1>u2)?(1<<5):0;
  code+=(u1>u3)?(1<<4):0;
  code+=(u1>u4)?(1<<3):0;
  code+=(u2>u3)?(1<<2):0;
  code+=(u2>u4)?(1<<1):0;
  code+=(u3>u4)?(1<<0):0;
  
  return code;
}

void sort(uint8_t *index, uint16_t *data, uint8_t nr)
{
  // first fill the index array.  
  for (uint8_t i=0; i<nr; i++)
  {
    index[i] = i;
  }
  bool swapped; 
  for (uint8_t i=0; i<nr-1; i++) 
  { 
    swapped = false; 
    for (uint8_t j=0; j<nr-1-i; j++) 
    { 
      if (data[j]<data[j+1])
      { 
        const uint16_t tmp = data[j];
        data[j] = data[j+1];
        data[j+1] = tmp;
        const uint8_t tmpi = index[j];
        index[j] = index[j+1];
        index[j+1] = tmpi;
        swapped=true;
      } 
    }
    // If no two elements were swapped by inner loop, then break 
    if (swapped == false) break; 
  }  
}

void get_audiodata()
{
  // this is very memory intensive! 
  int32_t audiosample[SAMPLE_SIZE];
  uint16_t audiospectrum[SPECTRUM_SIZE];
  uint32_t averagespectrum[SPECTRUM_SIZE];
  memset(averagespectrum,0,sizeof(averagespectrum));
  memset(s_codehistogram,0,sizeof(s_codehistogram));
  double sos = 0; // sum of squares.
  double sosd = 0; // sum of squared differences.
  double average_entr = 0; // mean entropy
  
  for (uint16_t i=0; i<NR_OF_SAMPLES; i++)
  {
    uint32_t meanspectrum[SPECTRUM_SIZE];
    memset(meanspectrum,0,sizeof(meanspectrum));
    for (uint16_t j=0; j<NR_OF_SPECTRA; j++)
    {
      // sample microphone.
      get_audiosample(audiosample);

      // calculate sum of squares, from which later SPL can be calculated.
      // calculate sum of squared difference, from which later the rugosity can be calculated.
      for (uint16_t k=0; k<SAMPLE_SIZE; k++) 
      {
        sos += pow(audiosample[k],2);
        if (k) sosd += pow((audiosample[k]-audiosample[k-1]),2); 
      }
  
      // calculate spectrum.
      get_audiospectrum(audiosample, audiospectrum);

      // Add to array from which to later calculate average. NOTE, this neglects the shift-gain!!!
      // this is actually Welch's method for estimating power spectral density, 
      // https://ccrma.stanford.edu/~jos/sasp/Welch_s_Method.html, except for that we need to square
      // the FFT result first.
      for (uint16_t k=0; k<SPECTRUM_SIZE; k++) 
      {
        meanspectrum[k] += audiospectrum[k];
      }
    }
    for (int j=0; j<SPECTRUM_SIZE; j++) averagespectrum[j] += meanspectrum[j];
    
    average_entr += calc_spectralentropy(meanspectrum);
    
    // calc spectral code and add the histogram.
    const uint8_t code = calc_spectralcode(meanspectrum);
    s_codehistogram[code]++;

    #ifndef LORA
      blink(200); // heartbeat
    #endif
  }

  //for (int j=0; j<SPECTRUM_SIZE; j++) { Serial.print(FFT_BIN(j, SAMPLE_RATE, SAMPLE_SIZE));Serial.print("\t");Serial.println(averagespectrum[j]); }
  
  s_SPL = 100*calc_dB(sos, NR_OF_SAMPLES*SAMPLE_SIZE*NR_OF_SPECTRA);
  Serial.print(F("SPL (dB)           : ")); Serial.println(s_SPL/100.0);

  s_rugo = 255*(sosd/sos); // normalise the rugosity ... this is my definition ... .
  Serial.print(F("rugosity (%)       : ")); Serial.println(100*s_rugo/255);

  average_entr /=NR_OF_SAMPLES;
  Serial.print(F("spectr. entropy (b): ")); Serial.println(average_entr);

  // sort the codes for frequency of occurance and normalise their probability to 1 byte range.
  sort(s_code, s_codehistogram, 64);
  for (uint8_t i=0; i<24; i++) // only 24 codes are occupied.
  { 
    Serial.print(F("Code ")); if (s_code[i]<8) Serial.print(F("0")); Serial.print(s_code[i],OCT); Serial.print(F("            : ")); Serial.println(float(s_codehistogram[i])/NR_OF_SAMPLES);
    s_codehistogram[i]=255*((float)s_codehistogram[i]/NR_OF_SAMPLES); 
  }
  
  // make a condensed and normalised version of the relevant part of spectrum [STARTBIN,ENDBIN).
  uint32_t cspctrm[NR_OF_COURSE_BINS];
  memset(cspctrm,0,sizeof(cspctrm));
  uint32_t maxval=0;
  // Serial.print(F("FreeRam (audio)    : ")); Serial.println(FreeRam());
  for (uint16_t i=0; i<NR_OF_COURSE_BINS; i++) 
  {
    for (uint16_t j=0; j<NR_OF_BINS_TO_COMBINE; j++) 
    {
      cspctrm[i] += averagespectrum[(STARTBIN+i*NR_OF_BINS_TO_COMBINE+j)];
    }
    if (cspctrm[i]>maxval) maxval=cspctrm[i];
  }
  for (uint16_t i=0; i<NR_OF_COURSE_BINS; i++) 
  {
    s_spectrum[i] = 255*((float)cspctrm[i]/maxval);
    // a messy set of lines, but it displays what is needed:
    Serial.print(F("s_bin")); if (FFT_BIN((STARTBIN+i*NR_OF_BINS_TO_COMBINE), SAMPLE_RATE, SAMPLE_SIZE)<100) Serial.print(F("0")); 
      Serial.print(FFT_BIN((STARTBIN+i*NR_OF_BINS_TO_COMBINE), SAMPLE_RATE, SAMPLE_SIZE),0); Serial.print(F("_")); 
      Serial.print(FFT_BIN((STARTBIN+(i+1)*NR_OF_BINS_TO_COMBINE), SAMPLE_RATE, SAMPLE_SIZE),0); Serial.print(F("Hz     : ")); 
      Serial.println(s_spectrum[i]);
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
  byte mydata[7+NR_OF_COURSE_BINS+8];
  mydata[0] = t_i >> 8;
  mydata[1] = t_i;
  mydata[2] = p >> 8;
  mydata[3] = p;
  mydata[4] = h;
  mydata[5] = s_SPL >> 8;
  mydata[6] = s_SPL;
  mydata[7] = s_rugo;
  for (uint8_t i=0; i<NR_OF_COURSE_BINS; i++) 
  {
    mydata[8+i] = s_spectrum[i];
  }
  for (uint8_t i=0; i<4; i++)
  {
    mydata[8+NR_OF_COURSE_BINS+i] = s_code[i];
    mydata[8+NR_OF_COURSE_BINS+i+4] = s_codehistogram[i];
  }
      
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
      Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
      // Prepare upstream data transmission at the next possible time.
      // NOT SURE if the airtime is correctly calculated.
      Serial.print(F("airtime (ms)       : ")); Serial.println(1000*calcAirTime(LMIC.rps, sizeof(mydata))/OSTICKS_PER_SEC);
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
  if (!bme_status) 
  {
        Serial.println("Could not find a valid BME280 sensor"); // check the address in the library!
  }

  // start I2S with 32-bits per sample, as needed by SPH0645LM4H-B and ICS43432.
  i2s.begin(I2S_32_BIT, SAMPLE_RATE);
  i2s.enableRx();

  #ifdef LORA
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    // tell LMIC to make the receive windows bigger, in case your clock is 1% faster or slower.  
    LMIC_setClockError(MAX_CLOCK_ERROR * 1 / 100);
    // join
    LMIC_startJoining();
    // Some test today show that using the following will NOT make TTN send ADR:
    LMIC_setAdrMode(1);
    // Does TTN support LinkCheckMode?
    LMIC_setLinkCheckMode(1);
  #endif  
  blink(200); blink(200); blink(400);
  // Set data rate and transmit power
  //LMIC_setDrTxpow(DR_SF12, 14);
}

void loop() 
{
  #ifdef LORA
    os_runloop_once();
  #else
    get_audiodata();
    get_temperature();
  #endif
}
