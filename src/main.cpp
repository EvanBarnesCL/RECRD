#include <Arduino.h>
#include "AS5600.h"
#include <Ramp.h>
#include <DFRobot_TCS3430.h>
#include <AnalogEncoder.h>

// proxy values for clockwise and ccw rotation of table
#define CCW   HIGH
#define CW    LOW

constexpr int TABLE_SPEED_PIN = 6;
constexpr int TABLE_DIR_PIN = 7;
constexpr int ARM_SPEED_PIN = 5;
constexpr int ARM_DIR_PIN = 4;

constexpr int HYST_OFFSET = 10;

constexpr int ARM_MOTOR_MIN_SPEED = 150;      // in testing, this was the lowest PWM value that would reliably move the arm to the demanded position without occasional stalls.

void setMotorSpeed(int speedPin, int dirPin, int speed);

int target = 0, targetDeg = 0;  // in encoder counts, in degrees


rampInt positionRamp;


DFRobot_TCS3430 TCS3430;          // object for light sensor


AnalogEncoder tableEncoder(A0);
AnalogEncoder armEncoder(A1, true);


void setup()
{
  Serial.begin(115200);

  Wire.begin();

  pinMode(TABLE_DIR_PIN, OUTPUT);
  pinMode(TABLE_SPEED_PIN, OUTPUT);
  pinMode(ARM_DIR_PIN, OUTPUT);
  pinMode(ARM_SPEED_PIN, OUTPUT);

  if (!TCS3430.begin()) {
    Serial.println("TCS3430 not connected");
  } else {
    Serial.println("TCS3430 - READY");
  }
  delay(1000);

  digitalWrite(ARM_DIR_PIN, HIGH);
  analogWrite(ARM_SPEED_PIN, 200);
  delay(1000);
  analogWrite(ARM_SPEED_PIN, 0);

  armEncoder.begin();
  tableEncoder.begin();
  armEncoder.calibrate();
  tableEncoder.calibrate();

  delay(2000);

  positionRamp.go(512, 20000, LINEAR, FORTHANDBACK);
  target = 0;


  // start the table spinning just for fun
  digitalWrite(TABLE_DIR_PIN, CW);
  analogWrite(TABLE_SPEED_PIN, 80);
}


void loop()
{
  static unsigned long lastPrint = millis();
  target = positionRamp.update();
  int currentPos = armEncoder.read();
  int error = target - currentPos;
  // int mSpeed = (int)(1.0 * error);
  if (error > 5) {
    setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, -ARM_MOTOR_MIN_SPEED);
  } else if (error < -5) {
    setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, ARM_MOTOR_MIN_SPEED);
  } else {
    setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, 0);
  }

  if (millis() - lastPrint > 100) {
    // uint16_t XData = TCS3430.getXData();
    // uint16_t YData = TCS3430.getYData();
    // uint16_t ZData = TCS3430.getZData();
    // uint16_t IR1Data = TCS3430.getIR1Data();
    // uint16_t IR2Data = TCS3430.getIR2Data();
    // String str = "X : " + String(XData) + "    Y : " + String(YData) + "    Z : " +  String(ZData) + "    IR1 : "+String(IR1Data) + "    IR2 : "+String(IR2Data);
    // // String str = "X : " + String(XData);
    // Serial.println(str);
    int tPos = tableEncoder.read();
    int aPos = armEncoder.read();
    String str = "Arm: " + String(aPos) + "    Table: " + String(tPos);
    Serial.println(str);
    lastPrint = millis();
  }

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

