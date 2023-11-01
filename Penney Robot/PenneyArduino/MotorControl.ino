/****************
 * Motor Control
 ****************/

void controlMotor(int motor1Speed, int motor2Speed) {

  int motorSpeedAMultiplier = 1;  // Adjust this multiplier for Motor A speed to correct for differences.
  int motorSpeedBMultiplier = 1;  // Adjust this multiplier for Motor B speed to correct for differences.

  //Serial.println(" Motor Control");

  // Motor A
  if (motor1Speed > 0) {
    analogWrite(MOTOR_A1, 0);                                    // Motor A, stop it (0 speed)
    analogWrite(MOTOR_A2, motor1Speed * motorSpeedAMultiplier);  // Motor A, forward speed
  } else if (motor1Speed < 0) {
    analogWrite(MOTOR_A1, -motor1Speed * motorSpeedAMultiplier);  // Motor A, backward speed
    analogWrite(MOTOR_A2, 0);                                     // Motor A, stop it (0 speed)
  } else {
    analogWrite(MOTOR_A1, 0);  // Motor A, stop it (0 speed)
    analogWrite(MOTOR_A2, 0);  // Motor A, stop it (0 speed)
  }

  // Motor B
  if (motor2Speed > 0) {
    analogWrite(MOTOR_B1, 0);                                    // Motor B, stop it (0 speed)
    analogWrite(MOTOR_B2, motor2Speed * motorSpeedBMultiplier);  // Motor B, forward speed
  } else if (motor2Speed < 0) {
    analogWrite(MOTOR_B1, -motor2Speed * motorSpeedBMultiplier);  // Motor B, backward speed
    analogWrite(MOTOR_B2, 0);                                     // Motor B, stop it (0 speed)
  } else {
    analogWrite(MOTOR_B1, 0);  // Motor B, stop it (0 speed)
    analogWrite(MOTOR_B2, 0);  // Motor B, stop it (0 speed)
  }
}
