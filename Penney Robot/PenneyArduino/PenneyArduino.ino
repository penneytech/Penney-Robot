#include <Wire.h>
#include <SoftwareSerial.h>

// ROBOT PINOUT CONFIGURATIONS ///////////////////////

// IR Sensor Connections (IR1 is on the left when looking at robot from behind)
#define IR1 A3
#define IR2 A2
#define IR3 A1
#define IR4 A0

// Motor connections (using PWM pins on the arduino)
#define MOTOR_A1 3
#define MOTOR_A2 5
#define MOTOR_B1 10
#define MOTOR_B2 11

//////////////////////////////////////////////////////


// Misc Program Variables
int currentX = 0;
int currentY = 0;
int currentLine = 0;
int turnCounter = 2;
bool intersectionDetected = false;

bool isTurning = false;

// Timer Variablews
unsigned long turningStartTime = 0;   // Variable to hold the time when turning started
unsigned long lineLostStartTime = 0;  // Add a new variable to hold the time when the robot first loses sight of the line

// Software Serial Setup for ESP8266
SoftwareSerial mySerial(12, 13);

// Variables for asynchronous NodeMCU ESP messages.
unsigned long previousMillis = 0;

void setup() {

  // Setup Serial communication
  Serial.begin(115200);
  mySerial.begin(9600);

  // Setup IR sensor Pins
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);

  // Define motor control pins as outputs
  pinMode(MOTOR_A1, OUTPUT);
  pinMode(MOTOR_A2, OUTPUT);
  pinMode(MOTOR_B1, OUTPUT);
  pinMode(MOTOR_B2, OUTPUT);

  // Set the motors to initial stopped state
  digitalWrite(MOTOR_A1, LOW);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, LOW);
}

void loop() {
  // Example usage of the new functions
  followLine();  // Follow the line until an end is encountered
  stop(0);
  turn(0);        // Turn left

  followLine(1);  // Follow the line until number intersections encountered
  stop(0);
  turn(0);       // Turn left

  followLine();  // Follow the line until an end is encountered
  stop(0);
  turn(1);        // Turn right

  followLine(1);  // Follow the line until number intersections encountered
  stop(0);
  turn(1);  // Turn right
}

/********************
* Functions For Moving The Robot
********************/

void handleIntersectionDetection() {

  if (intersectionDetected == false) {
    currentLine++;  // increment currentline by 1;
    intersectionDetected = true;
    if (currentLine >= turnCounter) {
      // turnDirection = !turnDirection;
      delay(100);  // Delay for turn bump
      isTurning = true;
      currentLine = 0;
      controlMotor(90, 90);  // Bump Straight
      delay(150);
    }

    Serial.print("Current Line: ");
    Serial.println(currentLine);
  }
}

void turn(bool turnDirection) {

  // Initialize variables for IR sensor readings
  int IR2Output = 0;
  int IR3Output = 0;

  // Record the time the turn starts
  unsigned long turningStartTime = millis();

  // Initialize a boolean variable to track whether the turn is complete
  bool turnComplete = false;

  while (!turnComplete) {  // Keep looping until the turn is complete

    // Update sensor readings
    IR2Output = digitalRead(IR2);
    IR3Output = digitalRead(IR3);

    // Move in the specified turning direction
    if (turnDirection == 1) {
      controlMotor(-100, 100);  // Turn right
    } else {
      controlMotor(100, -100);  // Turn left
    }

    // Calculate the elapsed time since the turn started
    unsigned long elapsedTurningTime = millis() - turningStartTime;

    // Start trying to detect the line after 450 milliseconds
    if (elapsedTurningTime >= 450) {
      if (IR3Output == HIGH || IR2Output == HIGH) {
        // Correct the direction to stop the robot
        if (turnDirection == 1) {
          controlMotor(100, -100);  // Turn right
        } else {
          controlMotor(-100, 100);  // Turn left
        }
        delay(50);
        controlMotor(0, 0);  // Stop the robot
        delay(200);
        turnComplete = true;  // Mark the turn as complete
      }
    }

    // Handle the case where the line is not found within 2000 milliseconds
    if (elapsedTurningTime >= 2000) {
      controlMotor(0, 0);  // Stop the robot
      delay(1000);
      turnComplete = true;  // Mark the turn as complete
    }
  }
}

void stop(int delayTime) {

  controlMotor(0, 0);
  delay(delayTime);
}


void followLine(int intersections = -1) {
  int motorSpeedNormal = 160;
  int motorSpeedSlower = 60;

  // Initialize a counter for the number of intersections encountered
  int intersectionCount = 0;

  // Initialize a flag for intersection detection
  bool intersectionFlag = false;

  // Initialize a variable to store the last time an intersection was detected
  unsigned long lastIntersectionTime = 0;

  // Loop until the specified number of intersections are encountered or no line is detected
  while (intersections == -1 || intersectionCount < intersections) {
    int IR1Output = digitalRead(IR1);
    int IR2Output = digitalRead(IR2);
    int IR3Output = digitalRead(IR3);
    int IR4Output = digitalRead(IR4);

    // Check if enough time has passed since the last intersection (debounce)
    if (millis() - lastIntersectionTime >= 500) {
      if ((IR1Output == HIGH && IR4Output == HIGH) || (IR1Output == HIGH && IR4Output == LOW) || (IR1Output == LOW && IR4Output == HIGH)) {
        controlMotor(motorSpeedNormal, motorSpeedNormal);
        if (!intersectionFlag) {
          intersectionCount++;
          intersectionFlag = true;          // Set the flag to true so the count doesn't increment multiple times for a single line
          lastIntersectionTime = millis();  // Update the last intersection time
        }

        if (intersectionCount == intersections) {
          controlMotor(120, 120);  // Bump out
          delay(180);              // Hold for 50ms
          controlMotor(0, 0);      // Stop
          delay(200);              // Hold the stop for 200ms
          break;
        }
      } else {
        intersectionFlag = false;  // Reset the flag
      }
    }

    if (IR2Output == HIGH && IR3Output == HIGH) {
      controlMotor(motorSpeedNormal, motorSpeedNormal);
    } else if (IR2Output == LOW && IR3Output == HIGH) {
      controlMotor(motorSpeedSlower, motorSpeedNormal);
    } else if (IR2Output == HIGH && IR3Output == LOW) {
      controlMotor(motorSpeedNormal, motorSpeedSlower);
    } else {
      if (intersections == -1) {
        controlMotor(motorSpeedNormal, motorSpeedNormal);  // Keep going
        delay(50);  // Move for an additional 150ms
        controlMotor(0, 0);  // Stop
        delay(200);  // Hold the stop for 200ms
      }
      break;
    }
  }
}

