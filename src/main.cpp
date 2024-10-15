#include <Arduino.h>
#include "AS5600.h"

constexpr int TABLE_SPEED_PIN = 6;
constexpr int TABLE_DIR_PIN = 7;
constexpr int ARM_SPEED_PIN = 5;
constexpr int ARM_DIR_PIN = 4;

constexpr int HYST_OFFSET = 10;

AS5600 as5600;   //  use default Wire

int degreesToCount(float deg);
void setMotorSpeed(int speedPin, int dirPin, int speed);

int target = 0, targetDeg = 0;  // in encoder counts, in degrees




void setup()
{
  Serial.begin(115200);
  Serial.println(__FILE__);
  Serial.print("AS5600_LIB_VERSION: ");
  Serial.println(AS5600_LIB_VERSION);

  Wire.begin();

  pinMode(TABLE_DIR_PIN, OUTPUT);
  pinMode(TABLE_SPEED_PIN, OUTPUT);
  pinMode(ARM_DIR_PIN, OUTPUT);
  pinMode(ARM_SPEED_PIN, OUTPUT);

  as5600.begin(2);  //  set direction pin.
  as5600.setDirection(AS5600_CLOCK_WISE);  //  default, just be explicit.
  int b = as5600.isConnected();
  Serial.print("Connect: ");
  Serial.println(b);
  as5600.setHysteresis(1);
  digitalWrite(ARM_DIR_PIN, HIGH);
  analogWrite(ARM_SPEED_PIN, 200);
  delay(1000);
  analogWrite(ARM_SPEED_PIN, 0);

  // Serial.println("offset:\traw:\tdeg:");
  // Serial.print(as5600.getOffset());
  // Serial.print("\t");
  // Serial.print(as5600.readAngle());
  // Serial.print("\t");
  // Serial.println(as5600.readAngle() * AS5600_RAW_TO_DEGREES);
  // delay(2000);
  // as5600.setOffset(-1 * as5600.readAngle() * HYST_OFFSET * AS5600_RAW_TO_DEGREES);    // set to 10 degrees as a 0 point for hysteresis purposes
  // Serial.print(as5600.getOffset());
  // Serial.print("\t");
  // Serial.print(as5600.readAngle());
  // Serial.print("\t");
  // Serial.println(as5600.readAngle() * AS5600_RAW_TO_DEGREES);
  // delay(10000);

  as5600.resetCumulativePosition();
  Serial.print("raw: ");
  Serial.print(as5600.getCumulativePosition());
  Serial.print("\tdeg: ");
  Serial.println(as5600.getCumulativePosition() * AS5600_RAW_TO_DEGREES);
  delay(2000);
  targetDeg = 10;
  target = degreesToCount(targetDeg);    // set initial target of 10 degrees
}


void loop()
{
  static unsigned long lastPrint = millis();
  static unsigned long targetResetTimer = 0;
  static bool targetResetTimerSet = false;
  static int dir = 1;
  int currentPos = as5600.getCumulativePosition();
  int error = target - currentPos;
  // int mSpeed = (int)(1.0 * error);
  if (error > 20) {
    setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, -200);
  } else if (error < -20) {
    setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, 200);
  } else {
    setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, 0);
    if (!targetResetTimerSet) {
      targetResetTimerSet = true;
      targetResetTimer = millis();
    }
    if (millis() - targetResetTimer > 2000) {
      targetDeg = targetDeg + (dir * 10);
      if (targetDeg > 180 || targetDeg < 0) {
        dir *= -1;
        targetDeg = targetDeg + (2 * dir * 10);
      }
      target = degreesToCount(targetDeg);
      targetResetTimerSet = false;
      targetResetTimer = millis();
    }
  }

  if (millis() - lastPrint > 100) {
    // Serial.print("raw: ");
    // Serial.print(as5600.readAngle());
    // Serial.print("\t");
    // Serial.print("deg: ");
    // Serial.println(as5600.readAngle() * AS5600_RAW_TO_DEGREES);
    
    Serial.print("raw: ");
    Serial.print(as5600.getCumulativePosition());
    Serial.print("\tdeg: ");
    Serial.print(as5600.getCumulativePosition() * AS5600_RAW_TO_DEGREES);
    Serial.print("\ttargetDeg: ");
    Serial.println(targetDeg);
    Serial.print("\terror: ");
    Serial.println(error);
    
    // Serial.print("\t");
    // Serial.println(error);
    lastPrint = millis();
  }

}



int degreesToCount(float deg) {
  return (deg) * AS5600_DEGREES_TO_RAW;
}


void setMotorSpeed(int speedPin, int dirPin, int speed) {
  if (speed < 0) {
    // Reverse direction
    digitalWrite(dirPin, LOW);
    analogWrite(speedPin, -speed); // Use absolute value for speed
  } else {
    // Forward direction
    digitalWrite(dirPin, HIGH);
    analogWrite(speedPin, speed);
  }
}