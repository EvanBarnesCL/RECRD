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


void setup()
{
  Serial.begin(115200);
  Serial.println("starting");

  tableMotor.setSpeed(255);
  delay(8000);
  tableMotor.setSpeed(0);

  armEncoder.begin();
  tableEncoder.begin();
  armEncoder.calibrate();
  tableEncoder.calibrate();

  delay(2000);


  tableMotor.setSpeed(1);
}


void loop() {
  static unsigned long lastPrint = millis();
  static unsigned long lastColorUpdate = millis();
  uint16_t r, g, b, c;

  if (millis() - lastPrint > 100) {
    Serial.println(tableEncoder.read());
    lastPrint = millis();
  }
}
