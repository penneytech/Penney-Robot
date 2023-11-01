#include <Wire.h>
#include <SoftwareSerial.h>

// Misc Program Variables
int currentX = 0;
int currentY = 0;
int currentLine = 0;
int turnCounter = 2;
int turnDirection = 1;  // 1 is right, 0 is left
bool intersectionDetected = false;
bool isTurning = false;
bool IR2High = false;  // Variable to keep track of IR2 state
bool IR3High = false;  // Variable to keep track of IR3 state

// Timer Variablews
unsigned long turningStartTime = 0;   // Variable to hold the time when turning started
unsigned long lineLostStartTime = 0;  // Add a new variable to hold the time when the robot first loses sight of the line

// IR Sensor Connections
#define IR1 A3
#define IR2 A2
#define IR3 A1
#define IR4 A0

// Debouncing Variables for IR sensors
unsigned long lastChangeTimeIR1 = 0;
unsigned long lastChangeTimeIR2 = 0;
unsigned long lastChangeTimeIR3 = 0;
unsigned long lastChangeTimeIR4 = 0;
int lastStateIR1 = LOW;
int lastStateIR2 = LOW;
int lastStateIR3 = LOW;
int lastStateIR4 = LOW;
unsigned long debounceThreshold = 100;  // Debounce time in milliseconds


// Motor connections
#define MOTOR_A1 3
#define MOTOR_A2 5
#define MOTOR_B1 10
#define MOTOR_B2 11

// Software Serial Setup
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

void handleIntersectionDetection() {

  if (intersectionDetected == false) {
    currentLine++;  // increment currentline by 1;
    intersectionDetected = true;
    if (currentLine >= turnCounter) {
     // turnDirection = !turnDirection;
      isTurning = true;
      currentLine = 0;
      controlMotor(90, 90);  // Bump Straight
      delay(150);
    }

    Serial.print("Current Line: ");
    Serial.println(currentLine);
  }
}

void loop() {

  /****************
  * IR PIN SYNTHESIS
  ****************/

  // Read IR pins

    // Read IR pins
  int rawIR1 = digitalRead(IR1);
  int rawIR2 = digitalRead(IR2);
  int rawIR3 = digitalRead(IR3);
  int rawIR4 = digitalRead(IR4);

  // Debounce IR1
  if (rawIR1 != lastStateIR1) {
    lastChangeTimeIR1 = millis();
  }
  int IR1Output = (millis() - lastChangeTimeIR1 > debounceThreshold) ? rawIR1 : lastStateIR1;
  lastStateIR1 = rawIR1;

  // Debounce IR2
  if (rawIR2 != lastStateIR2) {
    lastChangeTimeIR2 = millis();
  }
  int IR2Output = (millis() - lastChangeTimeIR2 > debounceThreshold) ? rawIR2 : lastStateIR2;
  lastStateIR2 = rawIR2;

  // Debounce IR3
  if (rawIR3 != lastStateIR3) {
    lastChangeTimeIR3 = millis();
  }
  int IR3Output = (millis() - lastChangeTimeIR3 > debounceThreshold) ? rawIR3 : lastStateIR3;
  lastStateIR3 = rawIR3;

  // Debounce IR4
  if (rawIR4 != lastStateIR4) {
    lastChangeTimeIR4 = millis();
  }
  int IR4Output = (millis() - lastChangeTimeIR4 > debounceThreshold) ? rawIR4 : lastStateIR4;
  lastStateIR4 = rawIR4;
  
  // int IR1Output = digitalRead(IR1);
  // int IR2Output = digitalRead(IR2);
  // int IR3Output = digitalRead(IR3);
  // int IR4Output = digitalRead(IR4);

  // Serial Print IR outputs for debugging
  // Serial.print(IR1Output);
  // Serial.print(IR2Output);
  // Serial.print(IR3Output);
  // Serial.print(IR4Output);

  /****************
 * Motor Control Logic
 ****************/

  int motorSpeedNormal = 150;
  int motorSpeedSlower = 80;

  if (isTurning == true) {

    if (turningStartTime == 0) {    // If turning just started
      turningStartTime = millis();  // Record the start time of turning
      delay(150);                    // Delay for turn bump
    }

    Serial.println("ISTURNING TRUE");

    if (turnDirection == 1) {
      controlMotor(-120, 120);  // Turn right
    } else {
      controlMotor(120, -120);  // Turn left
    }

    unsigned long elapsedTurningTime = millis() - turningStartTime;

    if (elapsedTurningTime >= 350) {  // Start trying to detect line after specified time since turn started

      IR2Output = digitalRead(IR2);
      IR3Output = digitalRead(IR3);

      if (IR3Output == HIGH) {
        //delay(30);
        controlMotor(0, 0);  // stop;
        delay(200);
        isTurning = false;
        turningStartTime = 0;  // Reset the turning start time
        currentLine = 0;
      }
    }

    // if (elapsedTurningTime >= 2000) {  // Maybe we didn't find any line.
    //   controlMotor(0, 0);              // stop;
    //   delay(1000);
    //   isTurning = false;
    //   turningStartTime = 0;  // Reset the turning start time
    //   currentLine = 0;
    // }

  } else
    // Handle Intersection Crossings
    if (IR1Output == HIGH && IR4Output == HIGH) {  // All sensors on a black line
      controlMotor(motorSpeedNormal, motorSpeedNormal);
      handleIntersectionDetection();
    } else if (IR1Output == HIGH && IR4Output == LOW) {  // All sensors on a black line except for rightmost
      controlMotor(motorSpeedNormal, motorSpeedNormal);
      handleIntersectionDetection();
    } else if (IR1Output == LOW && IR4Output == HIGH) {  // All sensors on a black line except for leftmost
      controlMotor(motorSpeedNormal, motorSpeedNormal);
      handleIntersectionDetection();
      // Keep the robot on the line
    } else if (IR2Output == HIGH && IR3Output == HIGH) {  // Both center sensors on a black line
      controlMotor(motorSpeedNormal, motorSpeedNormal);   // Both motors forward at full speed
      Serial.println("Going Straight");
      intersectionDetected = false;
    } else if (IR2Output == LOW && IR3Output == HIGH) {  // Left sensor only on a black line
      controlMotor(motorSpeedSlower, motorSpeedNormal);
      Serial.println("Turn Right");
      intersectionDetected = false;
    } else if (IR2Output == HIGH && IR3Output == LOW) {  // Right sensor only on a black line
      controlMotor(motorSpeedNormal, motorSpeedSlower);  // Motor 1 backward, Motor 2 forward (turn left)
      Serial.println("Turn Left");
      intersectionDetected = false;
      // Other cases
    } else {
      // controlMotor(0, 0);  // Stop both motors
      Serial.println("No Lines Detected");

      intersectionDetected = false;
      currentLine = 0;

      // Record the time when the line is first lost
      if (lineLostStartTime == 0) {
        lineLostStartTime = millis();
      }

      // Check if the line has been lost for a specified time
      if (millis() - lineLostStartTime >= 50) {
        Serial.print("Lost Line");
        isTurning = true;
        lineLostStartTime = 0;  // Reset the line lost start time
      }
    }

  // After processing IR sensor output, reset lineLostStartTime if a line is detected
  if (IR2Output == HIGH || IR3Output == HIGH) {
    lineLostStartTime = 0;
  }

  /****************
 * Communicate With ESP8266 Testing
 ****************/

  // unsigned long currentMillis = millis();
  // const long interval = 1000;  // 1000 milliseconds (1 second) interval for sending message

  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;

  //   //Report Location (Command, X, Y, Direction)
  //   mySerial.println("POS,2,3,N");

  //   // Report Puck Location & Color (Command, X, Y, Color)
  //   // Color is Red, Green or Blue
  //   // mySerial.print("COL,4,3,Red");
  // }
}
