#include <arduino.h>
#include <arduinoFFT.h>
#include "arduinoFFT.h"
#include <LiquidCrystal.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


/*FFT Parameters*/
#define SAMPLES 1024              //Must be a power of 2
#define SAMPLING_FREQUENCY 1000   //Hz, must be less than 10000 due to ADC
#define LOWER_BIN_THRESHOLD 65    //ignore spectral leakage from massive DC spike but also keep space for low E tuning
#define UPPER_BIN_THRESHOLD 350   //why do we need to keep looking at bins if we passed the high E string?
#define MAGNITUDE_THRESHOLD 5000  //put this right above noise level. the lower it is the more sustain the strings get
#define ERROR -1                  //this means the string hasn't been plucked yet
#define HARMONIC_TOLERANCE 5      //harmonics are ideally multiples but there is normally a slight mismatch
#define INITAL_SPIKE_DELAY 25     //ignore initial readings of guitar spike. i quite like this cuz initial readings were always dog
#define SNR_THRESHOLD 2           //signal to noise ratio. fundamental freq at its weakest should be this ratio bigger than the strongest noise
#define ADC_THRESHOLD 2500        //DC bias of mic is 1.65V. ADC goes from 0-4095. DC bias is at 2000 with some noise.

//oled parameters
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 32     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

//set up libraries
arduinoFFT FFT = arduinoFFT();
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const int micPin = 32;

/*Initialize Variables*/
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[SAMPLES];
double vImag[SAMPLES];
uint16_t prevMaxSpike = 0;
uint8_t signalDyingFlag = 0;
static int prevFundamentalFreq = 0;

void setup() {
  Serial.begin(115200);
  sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

  //set up oled
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;  // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
}

void loop() {
  /*Sample the ADC pin in vReal*/
  uint8_t reset = 0;
  uint16_t tempSpikeHolder = 0;
  for (int i = 0; i < SAMPLES; i++) {
    microseconds = micros();  //Overflows after around 70 minutes!

    vReal[i] = analogRead(micPin);
    vImag[i] = 0;
    if (vReal[i] > tempSpikeHolder) {
      tempSpikeHolder = vReal[i];
    }

    //when u get a real signal, ignore the first 30 ms and start sampling process from scratch
    if ((vReal[i] > prevMaxSpike) && !reset) {
      i = 0;
      reset = 1;
      delay(INITAL_SPIKE_DELAY);
    }

    while (micros() < (microseconds + sampling_period_us)) {  //this ensures 1ms passes before getting next sample. defines the sampling period
    }
  }
  prevMaxSpike = tempSpikeHolder;

  /*FFT*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  //HAMMING for better precision, hanning for accuracy(peak detection)
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  double peak = findFundamentalFrequency(vReal, SAMPLES, SAMPLING_FREQUENCY);

  /*TUNER DISPLAY. First line is status, second line is frequency */
  display.clearDisplay();
  display.setCursor(0, 0);
  handleTunerDisplay(peak);
  display.display();
  display.setCursor(0, 16);
  display.println(peak);
  display.display();
  Serial.println(peak);
}

/*
Desc: Modified version of MajorPeak(). Same inputs and output, but instead of greatest peak it is the fundamental frequency.
Checks for a large magnitude spike and calls it fundamental frequency, then verifies subsequent peaks are harmonics.
if the new peak is not a harmonic, that means it is the new fundamental frequency.
*/

//definition help:
//vD[i] is the current frequency bin value
//vD[indexOfMaxY] is the most recent greatest peak
double findFundamentalFrequency(double *vD, uint16_t samples, double samplingFrequency) {
  double maxY = 0;           //store max bin magnitude value
  uint16_t IndexOfMaxY = 0;  //store bin number here
  int oldFreq = 0;
  int newFreq = 0;
  uint8_t fundamentalFreqFound = 0;

  Serial.println("Enter Function\n");

  //check all relevant fft bins
  for (uint16_t i = 1; i < ((samples >> 1) - 1); i++) {
    //identify all peaks that are above noise threshold
    if ((vD[i - 1] < vD[i]) && (vD[i] > vD[i + 1]) && (i > LOWER_BIN_THRESHOLD) && (i < UPPER_BIN_THRESHOLD) && (vD[i] > MAGNITUDE_THRESHOLD)) {  //check neighbor bins
      //debug print messages to see the relevant peaks and their magnitudes
      double Newdelta = 0.5 * ((vD[i - 1] - vD[i + 1]) / (vD[i - 1] - (2.0 * vD[i]) + vD[i + 1]));
      double NewinterpolatedX = ((i + Newdelta) * samplingFrequency) / (samples - 1);
      newFreq = (int)NewinterpolatedX;
      Serial.print("New: ");
      Serial.print(newFreq);
      Serial.print(", Mag: ");
      Serial.println(vD[i], 4);

      //if new peak is greater than old peak and not a harmonic, update the new peak
      if ((vD[i] > maxY) && (!checkHarmonic(IndexOfMaxY, i, HARMONIC_TOLERANCE))) {
        //handle edge case where the fundamental freq was below noise threshold but not the harmonic
        if (!checkHarmonic(prevFundamentalFreq, i, HARMONIC_TOLERANCE)) {
          maxY = vD[i];
          IndexOfMaxY = i;
          fundamentalFreqFound = 1;
        }
      }
    }
  }

  //nothing to calculate if we didnt find any fundamental freq
  if (!fundamentalFreqFound) {
    prevFundamentalFreq = 0;
    return (ERROR);
  }

  //calculate frequency from fft bin value
  double delta = 0.5 * ((vD[IndexOfMaxY - 1] - vD[IndexOfMaxY + 1]) / (vD[IndexOfMaxY - 1] - (2.0 * vD[IndexOfMaxY]) + vD[IndexOfMaxY + 1]));
  double interpolatedX = ((IndexOfMaxY + delta) * samplingFrequency) / (samples - 1);
  prevFundamentalFreq = round(interpolatedX);
  return (interpolatedX);
}

//a function to check if newFreq is a harmonic of oldFreq with a given tolerance.
//inputs: oldFreq, newFreq, tolerance.
//output: true being it is a harmonic.
uint8_t checkHarmonic(int oldFreq, int newFreq, int tolerance) {
  /*Definition of a harmonic: a multiple of at least two with a given tolerance.*/

  //if oldfreq is 0, this cannot be a harmonic
  if (oldFreq == 0) {
    return 0;
  }

  //firstly, to be a harmonic u need to be at least double the original value. due to the tolerance, you need to be double minus the tolerance
  //this line will disregard all values close to the fundamental frequency and only concern itself with actual harmonics
  int minValue = (oldFreq * 2) - tolerance;

  //first check upper tolerance
  int modRemainder = newFreq % oldFreq;
  //next check lower tolerance
  int negMod = oldFreq - modRemainder;
  if (((modRemainder <= tolerance) || (negMod <= tolerance)) && (newFreq >= minValue)) {
    return 1;
  } else {
    return 0;
  }
}

/*
Input: floating point value frequency
Output: return 0 if signal is dead. idk i just need to leave the function so i use return
Description: Based on input frequency, lcd display will be updated.
*/
uint8_t handleTunerDisplay(double peak) {
  double error;
  if ((signalDyingFlag) && (peak != ERROR)) {
    display.println("Pluck again      ");
    return 0;
  }
  if ((peak >= 81.28) && (peak <= 82.55)) {    //E2=82.41. need 82.2711-82.557 to be in 3 cents range. 82 is 9 cents away, 82.8 is 8 cents away.
    display.println("E2 = 82.41.    ");        //E2 is in tune
  } else if ((peak > 70) && (peak < 82.28)) {  //E2 Up
    display.println("E2: Tune Up  ");
  } else if ((peak > 82.55) && (peak < 90)) {  //E2 Down
    display.println("E2: Tune Down");
  } else if ((peak >= 109) && (peak <= 111)) {  //A2=110. need 109.8149 to 110.1962
    display.println("A2 = 110.0.    ");
  } else if ((peak > 100) && (peak < 109)) {  //A2 Up
    display.println("A2: Tune Up  ");
  } else if ((peak > 111) && (peak < 120)) {  //A2 Down
    display.println("A2: Tune Down");
  } else if ((peak >= 146) && (peak <= 148)) {  //D3=146.83
    display.println("D3 = 146.83.   ");
  } else if ((peak > 136) && (peak < 146)) {  //D3 Up
    display.println("D3: Tune Up  ");
  } else if ((peak > 148) && (peak < 158)) {  //D3 Down
    display.println("D3: Tune Down");
  } else if ((peak >= 195) && (peak <= 197)) {  //G3=196
    display.println("G3 = 196.      ");
  } else if ((peak >= 185) && (peak < 195)) {  //G3 Up
    display.println("G3: Tune Up  ");
  } else if ((peak > 197) && (peak < 207)) {  //G3 Down
    display.println("G3: Tune Down");
  } else if ((peak >= 246) && (peak <= 248)) {  //B3=246.94
    display.println("B3 = 246.94.     ");
  } else if ((peak > 236) && (peak < 246)) {  //B3 Up
    display.println("B3: Tune Up  ");
  } else if ((peak > 248) && (peak < 258)) {  //B3 Down
    display.println("B3: Tune Down");
  } else if ((peak >= 328.5) && (peak <= 330.5)) {  //E4=329.63
    display.println("E4 = 329.63      ");
  } else if ((peak > 318.5) && (peak < 328.5)) {  //E4 Up
    display.println("E4: Tune Up  ");
  } else if ((peak > 330.5) && (peak < 340.5)) {  //E4 Down
    display.println("E4: Tune Down");
  } else {
    display.println("Waiting stage");
  }
  return 1;
}
