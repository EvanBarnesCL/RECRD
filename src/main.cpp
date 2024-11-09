#include <Arduino.h>
#include <Ramp.h>
#include <AnalogEncoder.h>
#include <digitalWriteFast.h>
#include <Crunchlabs_DRV8835.h>

constexpr uint8_t CW = 1;
constexpr uint8_t CCW = -1;

constexpr uint8_t ARM_OUT = 0;
constexpr uint8_t ARM_IN = 1;

constexpr uint8_t TABLE_SPEED_PIN = 6;
constexpr uint8_t TABLE_DIR_PIN = 7;
constexpr uint8_t ARM_SPEED_PIN = 5;
constexpr uint8_t ARM_DIR_PIN = 4;

constexpr uint8_t ARM_ENC_PIN = A6;
constexpr uint8_t TABLE_ENC_PIN = A7;


AnalogEncoder tableEncoder(TABLE_ENC_PIN);
AnalogEncoder armEncoder(ARM_ENC_PIN);


DRV8835 tableMotor(TABLE_SPEED_PIN, TABLE_DIR_PIN, 85, true);
DRV8835 armMotor(ARM_SPEED_PIN, ARM_DIR_PIN, 160, false);


void homeArm();

void setup()
{
  Serial.begin(115200);
  Serial.println("starting");
  armEncoder.begin();
  homeArm();

  tableEncoder.begin();
  armEncoder.calibrate();
  tableEncoder.calibrate();
  tableEncoder.resetCumulativePosition();
  armEncoder.resetCumulativePosition();

  delay(2000);


  tableMotor.setSpeed(255);
  armMotor.setSpeed(1);
}


void loop() {
  static unsigned long lastPrint = millis();
  static unsigned long lastColorUpdate = millis();
  uint16_t r, g, b, c;

  tableEncoder.getCumulativePosition();

  if (millis() - lastPrint > 100) {
    Serial.print(tableEncoder.update()); Serial.print("    ");
    Serial.print(tableEncoder.getRevolutions()); Serial.print("    ");
    Serial.println(tableEncoder.getAngle());
    lastPrint = millis();
  }
}



void homeArm() {
  armMotor.setSpeed(-255);
  int16_t lastPosition = 0, currentPosition = 0;
  bool homed = false;
  uint32_t lastUpdateTime = millis();
  int16_t hysteresisVal = 10;

  while (!homed) {
    armEncoder.getCumulativePosition();
    currentPosition = armEncoder.getAngle();
    if (millis() - lastUpdateTime > 100) {
      Serial.println("homing");
      if ((lastPosition > currentPosition - hysteresisVal) && (lastPosition < currentPosition + hysteresisVal)) {
        homed = true;
        armMotor.setSpeed(0);
        Serial.println("complete");
        return;
      }
      lastPosition = currentPosition;
      lastUpdateTime = millis();
    }
  }
}