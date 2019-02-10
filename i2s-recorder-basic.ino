/**
 *
 * Basic I2S recorder
 * Based on wjmb's Feather M0 Beehive LoRa code
 *
 * Minimum example for trying to reproduce problems when
 * reading the ICS43432 using I2S without using TTN at all.
 *
**/

#include <I2S.h>              // https://www.arduino.cc/en/Reference/I2S .. part of "Adafruit SAMD Boards"

#define REDLED 13
#define SAMPLE_RATE 12500 // max freq detectable is half of this according to Nyquist
#define DATA_SIZE 1024 // needs to be a multiple of 64.

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
    sampledata[i] = values[(2*i) + 1] >> 14;
  }
  I2Savailable = 0;
}

void get_audiodata()
{
  if (!I2S.begin(I2S_PHILIPS_MODE, SAMPLE_RATE, 32)) {
    Serial.println("Failed to initialize I2S!");
    while (1); // do nothing
  }
  I2S.read(); // apparently this get things going.
  delay(800); // For startup/softunmute, this seems a safe minimal amount of time (at 12500 Hz sample rate)
  int16_t data[DATA_SIZE];
  for (int j=0; j<DATA_SIZE/64; j++) get_audiosample(&data[j*64]); // collect data .. 64 values at the time.
  //for (int j=0; j<DATA_SIZE; j++) Serial.println(data[j]);
  I2S.end();  // TODO: this is where is hangs the second time this function was called !!!
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
  I2S.onReceive(onI2SReceive);
  pinMode(REDLED, OUTPUT);
  
  delay(10000); // to allow me to set the serial console.
  Serial.begin(9600);
  Serial.println(F("Starting"));
}

void loop()
{
  blink(50);
  get_audiodata();
  blink(500);
}
