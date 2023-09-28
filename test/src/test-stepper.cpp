#include <Arduino.h>

// Define the pins for the stepper motor
const int stepPin = 2;
const int dirPin = 5;
const int stepPinLeft = 3;
const int dirPinLeft = 6;

// Define the number of steps and speed of the motor
const int numSteps = 2000;
const int stepDelay = 100;

void setup() {
  // Set the stepper motor pins as outputs
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(stepPinLeft, OUTPUT);
  pinMode(dirPinLeft, OUTPUT);
}

void loop() {
  digitalWrite(dirPin, HIGH); // Uncomment this line for clockwise rotation
  for (int i = 0; i < numSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(1000);

  digitalWrite(dirPin, LOW); // Uncomment this line for counterclockwise rotation
  for (int i = 0; i < numSteps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(1000);

  digitalWrite(dirPinLeft, HIGH); // Uncomment this line for clockwise rotation
  for (int i = 0; i < numSteps; i++) {
    digitalWrite(stepPinLeft, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPinLeft, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(1000);

  digitalWrite(dirPinLeft, LOW); // Uncomment this line for counterclockwise rotation
  for (int i = 0; i < numSteps; i++) {
    digitalWrite(stepPinLeft, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPinLeft, LOW);
    delayMicroseconds(stepDelay);
  }
  delay(1000000);
}