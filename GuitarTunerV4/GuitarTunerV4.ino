/*
Authors: Alex Shin and Jinwoo Kim
The goal of this project is to create an automatic guitar tuner. 
We use a rotary encoder to select a note to be tuned to, which is displayed on an I2C Display.
We have then connected a KY-037 High Sensitivity Sound Detection module, to our circuit, which
will listen for a note on the guitar string to be played. After listening to the note, our circuit
will automatically turn a stepper motor to tune it to the note that has been selected. 
 FFT influenced by https://clydelettsome.com/blog/2019/12/18/my-weekend-project-audio-frequency-detector-using-an-arduino/
*/

#include "arduinoFFT.h"

// ArduinoFFT definitions
#define SAMPLES 128             //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 1024 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.
// Motor definitions
#define AIN1_PIN 4
#define AIN2_PIN 5
#define PWMA_PIN 6
#define motorSpeed 255
// Sound sensor definitions
#define digitalPin 11 // KY-037 digital interface
#define analogPin A0 // KY-037 analog interface
#define ledPin 10 // Arduino LED pin
// Button
#define BUTTON_PIN 7
#define E2_PIN 3
#define A2_PIN 2
#define D3_PIN 8
#define G3_PIN 9
#define B3_PIN 12
#define E4_PIN 13
//Tuning Parameters
#define DEADZONE 1.0

// Note Definitions and Frequencies
String notes[6] = {"E2", "A2", "D3", "G3", "B3", "E4"};
double frequencies[6] = {168.5, 112, 149.85, 199.5, 252.7, 335.62};
double range[2] = {148.5, 188.5};
int listeningRange = 20;

// Note Selection
int noteSelected = 0;

// FFT Sound Sampling
arduinoFFT FFT = arduinoFFT();
unsigned int samplingPeriod;
unsigned long microSeconds;
double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values

// Motor Tuning Parameters
int initialStepDuration = 25;  // Duration of each step in milliseconds

int digitalVal;       // digital readings
int analogVal;        // analog readings

void setup() 
{
  Serial.begin(9600);
  
  // FFT
  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds

  //Button
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Motor pins
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);

  // Microphone
  pinMode(digitalPin,INPUT); 
  pinMode(analogPin, INPUT);
  pinMode(ledPin,OUTPUT);

  // LEDs
  pinMode(E2_PIN, OUTPUT);
  pinMode(A2_PIN, OUTPUT);
  pinMode(D3_PIN, OUTPUT);
  pinMode(G3_PIN, OUTPUT);
  pinMode(B3_PIN, OUTPUT);
  pinMode(E4_PIN, OUTPUT);
}
 
void loop() 
{
  // keeps ledPin low (could also use pinMode(INPUT_PULLDOWN))
  digitalWrite(ledPin, LOW);  
  handle_led();
 if (digitalRead(BUTTON_PIN) != LOW){
    // when the button is not pressed tuning mode is on
    digitalVal = digitalRead(digitalPin);
    if (digitalVal == 0) // Sound detected
    {
      // Determines the frequency and tunes the guitar
      determineFrequency();
      delay(1000);
    }
  }
  else{
    // Updates the note that has been selected when the button is pressed
    noteSelected++;
    if (noteSelected >= 6){
      noteSelected = 0;
    }
    // handle_led();
    delay(500);
  }
}

void handle_led(){
  // Controls the LEDS that show which note you are tuning to
  switch(noteSelected){
    case 0:
      digitalWrite(E2_PIN, HIGH);
      digitalWrite(A2_PIN, LOW);
      digitalWrite(D3_PIN, LOW);
      digitalWrite(G3_PIN, LOW);
      digitalWrite(B3_PIN, LOW);
      digitalWrite(E4_PIN, LOW);
      break;
    case 1:
      digitalWrite(E2_PIN, LOW);
      digitalWrite(A2_PIN, HIGH);
      digitalWrite(D3_PIN, LOW);
      digitalWrite(G3_PIN, LOW);
      digitalWrite(B3_PIN, LOW);
      digitalWrite(E4_PIN, LOW);
      break;
    case 2:
      digitalWrite(E2_PIN, LOW);
      digitalWrite(A2_PIN, LOW);
      digitalWrite(D3_PIN, HIGH);
      digitalWrite(G3_PIN, LOW);
      digitalWrite(B3_PIN, LOW);
      digitalWrite(E4_PIN, LOW);
      break;
    case 3:
      digitalWrite(E2_PIN, LOW);
      digitalWrite(A2_PIN, LOW);
      digitalWrite(D3_PIN, LOW);
      digitalWrite(G3_PIN, HIGH);
      digitalWrite(B3_PIN, LOW);
      digitalWrite(E4_PIN, LOW);
      break;
    case 4:
      digitalWrite(E2_PIN, LOW);
      digitalWrite(A2_PIN, LOW);
      digitalWrite(D3_PIN, LOW);
      digitalWrite(G3_PIN, LOW);
      digitalWrite(B3_PIN, HIGH);
      digitalWrite(E4_PIN, LOW);
      break;
    case 5:
      digitalWrite(E2_PIN, LOW);
      digitalWrite(A2_PIN, LOW);
      digitalWrite(D3_PIN, LOW);
      digitalWrite(G3_PIN, LOW);
      digitalWrite(B3_PIN, LOW);
      digitalWrite(E4_PIN, HIGH);
      break;
    default:
      digitalWrite(E2_PIN, LOW);
      digitalWrite(A2_PIN, LOW);
      digitalWrite(D3_PIN, LOW);
      digitalWrite(G3_PIN, LOW);
      digitalWrite(B3_PIN, LOW);
      digitalWrite(E4_PIN, LOW);
  }
}

void determineFrequency() {
  // Determines the frequency and calls adjustMotor which turns the DC motor
  Serial.print("Frequency: ");
  /*Sample SAMPLES times*/
  for(int i=0; i<SAMPLES; i++)
  {
      microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script. 
    
      vReal[i] = analogRead(analogPin); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
      vImag[i] = 0; //Makes imaginary term 0 always

      /*remaining wait time between samples if necessary*/
      while(micros() < (microSeconds + samplingPeriod))
      {
        //do nothing
      }
  }

  /*Perform FFT on samples*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  /*Find peak frequency and print peak*/
  double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
  double deadzone = DEADZONE;
  if (noteSelected == 4 && peak > 450){
    // For the B string, weird edge case where the frequency is calculated to be double 
    // We also increase the deadzone, which lowers the precision slightly but we use this to account for this edge case
    peak /= 2;
    deadzone = 1.5;
  }

  Serial.print("Frequency Received: ");
  Serial.println(peak);     //Print out the most dominant frequency.
  Serial.print("Target Note: ");
  Serial.println(notes[noteSelected]);
  Serial.print("Target Frequency: ");
  Serial.println(frequencies[noteSelected]);

  if (peak <= frequencies[noteSelected] + listeningRange && peak >= frequencies[noteSelected] - listeningRange){
    adjustMotor(peak, frequencies[noteSelected], deadzone);
  }
}

void adjustMotor(double freq, double targetFreq, double deadzone){
  // Turns motor to tune string to the correct note
  double freqDifference = fabs(freq - targetFreq);
  int dynamicStepDuration = initialStepDuration;

  if (freqDifference > 5) { // Threshold for large adjustments
      dynamicStepDuration *= 2; // Double the step duration for larger initial adjustments
  }

  if (freq > targetFreq + deadzone) {
    // Note is sharp so we turn counter clockwise
    spinCounterClockwise(dynamicStepDuration);
  } 
  else if (freq < targetFreq - deadzone) {
    // Note is flat so we turn clockwise
    spinClockwise(dynamicStepDuration);
  } 
  else {
    // In tune
    // Lights up Green LED indicating that the note is in tune
    Serial.println("In tune!");
    digitalWrite(ledPin, HIGH);
  }
}

// This function is not used in our final program, but we are including this to show that our program is also
// able to automatically detect the string that was played
int findNearestNote(double freq) {
  int closestNote = 0; // Index of the closest note
  double smallestDiff = 1000000; // Initialize with a large number
  double difference; // To store the absolute difference between the given frequency and each note frequency

  for (int i = 0; i < 6; i++) { // There are 6 notes in the array
    difference = fabs(frequencies[i] - freq); // Calculate the absolute difference
    if (difference < smallestDiff) { // If the current difference is smaller than the previously found
      smallestDiff = difference; // Update smallest difference
      closestNote = i; // Update the index of the closest note
    }
  }
  return closestNote; // Return the index of the closest note
}

void spinClockwise(int stepDuration) {
  // Move the motor forward for a short duration
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, LOW);
  analogWrite(PWMA_PIN, motorSpeed);  // Start the motor at the defined speed

  delay(stepDuration);  // Wait for the step to complete

  // Stop the motor
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  analogWrite(PWMA_PIN, 0);  // Stop sending PWM signal

  // Wait some time before the next step
  delay(250);  // Wait for 1 second
}

void spinCounterClockwise(int stepDuration) {
  // Move the motor forward for a short duration
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, HIGH);
  analogWrite(PWMA_PIN, motorSpeed);  // Start the motor at the defined speed

  delay(stepDuration);  // Wait for the step to complete

  // Stop the motor
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, LOW);
  analogWrite(PWMA_PIN, 0);  // Stop sending PWM signal

  // Wait some time before the next step
  delay(250);  // Wait for 1 second
}