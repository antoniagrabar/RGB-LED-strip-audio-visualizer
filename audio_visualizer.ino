
#include <Adafruit_NeoPixel.h>
#include "arduinoFFT.h"               // include Fast Fourier Transform calculations                             

#define LED_DATA 6
#define POTENTIOMETER A2          
#define MICROPHONE A1
#define BTN_PIN 3           // button on Interrupt Pin 3  

#define NUM_LEDS 60         // number of leds on a strip
#define MAX_MODE 2          // number of modes used in loop
#define BRIGHTNESS 10       // minimal brightness

#define ISR_DELAY 600
#define SAMPLES 64                        // number of samples used for fourier transform, must be a power of 2
#define SAMPLING_FREQUENCY 1000           // Hz, must be less than 10000 due to ADC

//assigning (SAMPLES/2)-1 = 31 frequency values to 3 ranges
#define Range1 10                         
#define Range2 11
#define Range3 10 

double Module1 = 0;       // average frequency value of Range1
double Module2 = 0;       // average frequency value of Range2
double Module3 = 0;       // average frequency value of Range3
  
unsigned int sampling_period_us;
unsigned long microseconds;             // current time since the Arduino board started 
double realData[SAMPLES];               // store real values after FFT
double imagData[SAMPLES];               // store imaginary values after FFT
double peak = 0;                        // highest frequency in a sampling period

volatile uint16_t time_since_isr = 0;
volatile boolean breakMode = false;
volatile uint8_t mode = 0;

int brightness = 0;
volatile uint32_t audioBuffer[NUM_LEDS];

arduinoFFT FFT = arduinoFFT();           // create FFT object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, LED_DATA, NEO_GRB + NEO_KHZ800);       // declare NeoPixel strip object


// ISR for button to switch modes 
void changeMode() {
  if (millis() - time_since_isr > ISR_DELAY) {
    time_since_isr = millis();
    mode = (mode + 1) % MAX_MODE;
    breakMode = true;
  }
}


void setup() {
  Serial.begin(115200);        // set the data rate in bits per second (baud) for serial data transmission
  analogReference(DEFAULT);    // use default 5V voltage

  sampling_period_us = round(1000000*(1.0/SAMPLING_FREQUENCY));
  brightness = BRIGHTNESS; 
  strip.setBrightness(BRIGHTNESS); 
  strip.begin();
  strip.show();

  pinMode(BTN_PIN, INPUT);            // set button pin as input
  pinMode(POTENTIOMETER, INPUT);      // set potentiometer pin as input
  
  attachInterrupt(1, changeMode, RISING);    // attach interrupt to Pin 3 for push button to switch modes
  initAudioBuffer();
}


void performFFT(){
  for(int i=0; i<SAMPLES; i++){
    microseconds = micros();    // overflows after around 70 minutes
    realData[i] = analogRead(MICROPHONE);
    imagData[i] = 0;
    
    while(micros() < (microseconds + sampling_period_us)); // wait sampling_period_us microseconds to read the next sample
  }
   
  FFT.Windowing(realData, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);      // weigh data
  FFT.Compute(realData, imagData, SAMPLES, FFT_FORWARD);                   // compute FFT
  FFT.ComplexToMagnitude(realData, imagData, SAMPLES);                     // compute magnitudes
  peak = FFT.MajorPeak(realData, SAMPLES, SAMPLING_FREQUENCY);

  Module1 = 0;
  Module2 = 0;
  Module3 = 0;
   
  for(int i=1; i < SAMPLES/2; i++){
   double a = pow(realData[i], 2);
   double b = pow(imagData[i], 2);
    
  // assign each frequency value to its corresponding range
    if(i <= Range1) Module1 += sqrt(a + b);
    else if(i >= (Range1+1) && i <= (Range1+Range2)) Module2 += sqrt(a + b);
    else Module3 += sqrt(a + b);
  }

  Module1 /= Range1; 
  Module2 /= Range2;
  Module3 /= Range3;
}


// check if potentiometer value has changed, set new brightness and return true
boolean checkBrightness() {
  uint16_t newBrightness = map(constrain(analogRead(POTENTIOMETER), 0, 1024), 0, 1024, 10, 255);
  if (abs(newBrightness - brightness) > 10) {
    brightness = newBrightness;
    strip.setBrightness(brightness);
    return true;
  }
  return false;
}


// fill all leds with one color
void fillWithColor(uint8_t red, uint8_t green, uint8_t blue) {
  for (int i = 0; i < NUM_LEDS; i++)
    strip.setPixelColor( i, strip.Color( red, green, blue ) );
  strip.show();
}


// use potentiometer to change color
void colorSpectrum() {
  int potValue = analogRead(POTENTIOMETER);
  int potRange = map(potValue, 0, 1023, 0, 255);          // map full analog value to pwm value
  int potModes = map(potRange, 0, 255, 0, 6);             // break potentiometer range into 6 modes

  int gUp = map(potRange, 0, 42, 0, 255);                 // 0 - green value rise mapped to 0-42 on pot
  int rDown = map(potRange, 43, 84, 255, 0);              // 1 - red value rise mapped to 43-84 on pot
  int bUp = map(potRange, 85, 127, 0, 255);               // 2 - blue value rise mapped to 85-127 on pot
  int gDown = map(potRange, 128, 169, 255, 0);            // 3 - green value fall mapped to 128-169 on pot
  int rUp = map(potRange, 170, 212, 0, 255);              // 4 - red value fall mapped to 170-212 on pot
  int bDown = map(potRange, 213, 255, 255, 0);            // 5 - blue value fall mapped to 213-255 on pot

  switch (potModes) {
    case 0:    
      fillWithColor(255, gUp, 0);
      break;
    case 1:    
      fillWithColor(rDown, 255, 0);
      break;
    case 2:    
      fillWithColor(0, 255, bUp);
      break;
    case 3:    
      fillWithColor(0, gDown, 255);
      break;
    case 4:
      fillWithColor(rUp, 0, 255);
      break;
    case 5:
      fillWithColor(255, 0, bDown);
      break;
  }
  delay(10);
}


void audioBrightness() { 
  performFFT();
  int avg = (Module1 + Module2 + Module3)/3;        // average frequency value from all Modules
  brightness = map(avg, 0, 2000, 20, 255);          // map avg 20-255 to fit possible brightness levels
  
  if(brightness > 255) brightness = 255;            // smooth out brightess levels
  if(brightness < 30) brightness = 20;

  strip.setBrightness(brightness);

  colorSpectrum();  
}


void initAudioBuffer() { 
  for (int i = 0; i < NUM_LEDS; i++) {
    audioBuffer[i] = 0;
  }
}


void audioRunningLeds(){
    initAudioBuffer();
   
    uint8_t r=0, g=0, b=0; 
        
    while (!breakMode) { 
      checkBrightness();
      if (breakMode)
        return;

      performFFT();
    
      // shift LED values forward
      for (volatile int i = NUM_LEDS - 1; i > 0; i--) {
        audioBuffer[i] = audioBuffer[i - 1];
      }

      if(peak > 400){
        r=255;
        g=255;
        b=255;
      }
      else if(peak > 340){
        r=0;
        g=100;
        b=200;
      }
      else if(peak > 270){
        r=0;
        g=0;
        b=255; 
      } else {
        r = 0;
        g = 0;
        b =0;
      }

      audioBuffer[0] = r; 
      audioBuffer[0] = audioBuffer[0] << 16;
      audioBuffer[0] |= ((g ) << 8); 
      audioBuffer[0] |= b;     
    
      for ( int i = 0; i < NUM_LEDS; i++) {
        strip.setPixelColor(i, strip.Color(audioBuffer[i] >> 16, (audioBuffer[i] >> 8)&B11111111, audioBuffer[i]&B11111111));
      }
      strip.show();
  } 
}


void loop() {
  breakMode = false;
  
  switch (mode) {
    case 0:                        
      audioBrightness();
      break;
    case 1:
      audioRunningLeds();
      break;
    default:
      break;
  }
}
