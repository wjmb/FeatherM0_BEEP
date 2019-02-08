/**
 *
 * Basic I2S recorder
 * Based on wjmb's Feather M0 Beehive LoRa code
 *
 * Minimum example for trying to reproduce problems when
 * reading the ICS43432 using I2S without using TTN at all.
 *
**/

#include <hal/hal.h>          // https://github.com/matthijskooijman/arduino-lmic
#include <SPI.h>              // https://www.arduino.cc/en/Reference/SPI
#include <I2S.h>              // https://www.arduino.cc/en/Reference/I2S .. part of "Adafruit SAMD Boards"

// Pin mapping
#define REDLED 13

#define SAMPLE_RATE 12500 // max freq detectable is half of this according to Nyquist
#define DATA_SIZE 1024 // needs to be a multiple of 64.
uint8_t NR_OF_SPECTRA = 8; // nr of freq. spectra to add and get a less discretised spectrum

// I2S variables.
uint32_t I2Sbuffer[512];
volatile int I2Savailable = 0;


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
  for (int i=0; i<64; i++)
  {
    // If the SEL pin is low, samples will be in odd numbered positions.
    // If you connect SEL to high, data will be in even positions.
    // TODO: how much to shift the data? I guess 7 bits for the ICS43432
    // 32 bits - 1 bit - 24 bits = 7 bit shift . Not sure though.
    // For SPH0645LM4H-B, which doesn't skip the first bit, it will be:
    // 32 bits - 18 bits = 14 bit shift.
    sampledata[i] = values[(2*i) + 1] >> 14;
    // TODO: use this bitshifting as a kind of noise reduction or gain parameter.
  }
  I2Savailable = 0;
}

void get_audiodata()
{
  I2S.onReceive(onI2SReceive);
  if (!I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 32)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
  I2S.read(); // apparently this get things going.
  delay(800); // For startup/softunmute, this seems a safe minimal amount of time (at 12500 Hz sample rate)
  int16_t data[DATA_SIZE]; // TODO: is there the risk of overflow, because the SPH0645LM4H-B / ICS43432 returns 18 / 24 bits?

  // Collect microphone input and determine RMS of raw data.
  uint32_t rms = 0;
  for (int i=0; i<NR_OF_SPECTRA; i++)
  {
    // collect data .. 64 values at the time.
    for (int j=0; j<DATA_SIZE/64; j++) get_audiosample(&data[j*64]);
    for (int j=0; j<DATA_SIZE; j++) rms += data[j]*data[j];

    for (int j=0; j<DATA_SIZE; j++) Serial.println(data[j]);
  }

  // Compute and output RMS.
  rms = sqrt(rms/(NR_OF_SPECTRA*DATA_SIZE));
  Serial.print("RMS: ");
  Serial.println(rms);

  I2S.end();  // TODO: this is where is hangs sometimes !!!

}

void blink()
{
  digitalWrite(REDLED, HIGH);
  delay(200);
  digitalWrite(REDLED, LOW);
  delay(200);
}

void blink(int speed)
{
  digitalWrite(REDLED, HIGH);
  delay(speed);
  digitalWrite(REDLED, LOW);
  delay(speed);
}

void setup()
{
  // Ready
  delay(10000); // to allow me to set the serial console.

  pinMode(REDLED, OUTPUT);

  Serial.begin(9600);
  Serial.println(F("Starting"));

  blink(); blink(); blink();

  // Steady
  delay(1000);
}

void loop()
{
  blink(50);
  get_audiodata();
  blink(50); blink(50);
  delay(500);
}
