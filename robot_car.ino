#include <BluetoothSerial.h>  // Library used to connect robot to bluetooth on smartphone
BluetoothSerial SerialBT;     // BluetoothSerial Object

// Motor Pins
const int motorLeftPWM  = 19; // Left PWM
const int motorRightPWM = 23; // Right PWM
const int shiftDataPin  = 5;  // DATA
const int shiftClockPin = 18; // SHCP
const int latchPin      = 17; // STCP
const int enablePin     = 16; // EN


// Ultrasonic Pins
const int triggerPin = 13;  // Trigger
const int echoPin    = 14;  // Echo

// Led Pins
const int leftLED = 2;
const int rightLED = 12;


// Motor directions
const int forward = 163;
const int backward = 92;
const int stop = 0;
const int turnLeft = 106;
const int turnRight = 149;

// Safe distance in centimeters to avoid colliding with an object
const float safeDistance = 15;

// currentState variable keeps track of the current direction the robot is moving when controlled through bluetooth
char currentState = 'S';

// initPins method to initialize Pins (Motors, Ultrasonic Sensor, Leds)
void initPins() {
  // Motors
  pinMode(motorLeftPWM, OUTPUT);
  pinMode(motorRightPWM, OUTPUT);
  pinMode(shiftClockPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  pinMode(shiftDataPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  // Ultrasonic
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // LEDs
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);
}

// Drive Function to move robot
void Drive(int direction, int speed) {
  digitalWrite(enablePin, LOW);
  analogWrite(motorLeftPWM, speed);   // Sets motor speed
  analogWrite(motorRightPWM, speed);

  digitalWrite(latchPin, LOW); // Resets latch pin
  // Shift register converts direction value to a byte. Data Pin converts value bit by bit. The shift clock 'ticks' for the next bit ↓
  shiftOut(shiftDataPin, shiftClockPin, MSBFIRST, direction);
  digitalWrite(latchPin, HIGH); // Stores final output for motor direction control
}

// Functions for movement, setting the currentState of the robot while moving
void MoveForward(int speed) {
  Drive(forward, speed);
  currentState = 'F';
}
void MoveBackward(int speed) {
  Drive(backward, speed);
  turnOnLEDs();
  currentState = 'B';
}
void TurnLeft(int speed) {
  Drive(turnLeft, speed);
  currentState = 'L';
}
void TurnRight(int speed) {
  Drive(turnRight, speed);
  currentState = 'R';
}
void Stop() {
  if (currentState == 'B') {
    turnOffLEDs();
  }
  Drive(stop, 0);
  currentState = 'S';
}

// findDistance() sends out a sound wave and measures how long it took to receive the echo
// The time is converted from microseconds to centimeters to calculate distance
float findDistance() {
  digitalWrite(triggerPin, LOW);    // Resets trigger pin
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);   // Sends out pulse
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  float distance = pulseIn(echoPin, HIGH) / 58.00;  // pulseIn() calculates time it took for the pulse to hit the object and come back to the echo pin
  delay(10);
  return distance;
}

// LED Functions
void turnOnLEDs() {
  digitalWrite(leftLED, HIGH);
  digitalWrite(rightLED, HIGH);
}

void turnOffLEDs() {
  digitalWrite(leftLED, LOW);
  digitalWrite(rightLED, LOW);
}

void setup() {
  initPins();
  SerialBT.begin("Robot");
  currentState = 'S';
}

void loop() {
  if (SerialBT.available()) {      // available() checks if the Bluetooth connection is established between both devices
    char cmd = SerialBT.read();    // Variable to store user input
 
    if (currentState != 'B') {     // If the robot is not moving backward, turn off LEDs. Meant to simulate real life reverse lights
      turnOffLEDs();
    }

    switch (cmd) {    
      case 'F':
        MoveForward(150);
        break;
      case 'B':
        MoveBackward(150);
        break;
      case 'L':
        TurnLeft(150);
        delay(750);
        Stop();
        break;
      case 'R':
        TurnRight(150);
        delay(750);
        Stop();
        break;
      case 'S':
        Stop();
        break;
    }
  }

  // If robot is moving forward and an object is too close, it will move back and stop
  float distance = findDistance();
  if (currentState == 'F' && distance <= safeDistance) {
    SerialBT.println("Something is in the way!");
    MoveBackward(150);
    delay(750);
    Stop();
  }
}