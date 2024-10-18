#include <Arduino.h>
#include "AS5600.h"
#include <Ramp.h>
#include <AnalogEncoder.h>
#include <SparkFun_OPT4048.h>
#include <Wire.h>

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

AnalogEncoder tableEncoder(A0);
AnalogEncoder armEncoder(A1, true);

SparkFun_OPT4048 colorSensor;

sfe_color_t colors;

void setup()
{
  Serial.begin(115200);

  Wire.begin();

  pinMode(TABLE_DIR_PIN, OUTPUT);
  pinMode(TABLE_SPEED_PIN, OUTPUT);
  pinMode(ARM_DIR_PIN, OUTPUT);
  pinMode(ARM_SPEED_PIN, OUTPUT);

  if (!colorSensor.begin()) {
    Serial.println("color sensor not initialized.");
  } else {
    Serial.println("color sensor READY");
  }
  delay(1000);

  colorSensor.setBasicSetup();

  // home the arm motor
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
  int mSpeed = (int)(1.0 * error);
  if (error > 5) {
    setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, -ARM_MOTOR_MIN_SPEED);
  } else if (error < -5) {
    setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, ARM_MOTOR_MIN_SPEED);
  } else {
    setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, 0);
  }

  if (millis() - lastPrint > 100) {
    colors = colorSensor.getAllADC();

    // this grabs like a middle chunk of the ADC reading, so far is the most useful range. I think each channel would have to be scaled differently to be useful.
    uint8_t red = (colors.red >> 8) & 0xFF;
    uint8_t blue = (colors.blue >> 8) & 0xFF;
    uint8_t green = (colors.green >> 8) & 0xFF;
    uint8_t white = (colors.white >> 8) & 0xFF;

    // uint8_t red = (uint8_t)(colors.red * 255 / 4294967295);
    // uint8_t blue = (uint8_t)(colors.blue * 255 / 4294967295);
    // uint8_t green = (uint8_t)(colors.green * 255 / 4294967295);
    // uint8_t white = (uint8_t)(colors.white * 255 / 4294967295);

    // uint32_t red = colors.red;
    // uint32_t blue = colors.blue;
    // uint32_t green = colors.green;
    // uint32_t white = colors.white;

    String str = "R: " + String(red) + "    G: " + String(green) + "    B: " + String(blue) + "    W: " + String(white);
    Serial.print(str);
    int tPos = tableEncoder.read();
    int aPos = armEncoder.read();
    str = "    Arm: " + String(aPos) + "    Table: " + String(tPos);
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

