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

int16_t armSpeed = 1;
int8_t armDir = 1;
int16_t armAngle, armRevolutions;


void homeArm();
int16_t getGearboxAngle();

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


  tableMotor.setSpeed(1);
  armMotor.setSpeed(armSpeed);
}


void loop() {
  static unsigned long lastPrint = millis();
  int32_t armCompletePosition = armEncoder.getCumulativePosition();
  armAngle = armEncoder.getAngle();
  armRevolutions = armEncoder.getRevolutions();
  int16_t cycloidalAngle = getGearboxAngle();

  // Add debug prints to see raw values
  if (millis() - lastPrint > 100) {
    Serial.print("Complete Position: "); Serial.print(armCompletePosition);
    Serial.print(" CycloidalAngle: "); Serial.print(cycloidalAngle);
    Serial.print(" Revs: "); Serial.print(armRevolutions);
    Serial.print(" Angle: "); Serial.println(armAngle);
  }

  bool reverseArm = (armDir > 0 && cycloidalAngle >= 1020) || 
                  (armDir < 0 && cycloidalAngle <= 5);
  
  if (reverseArm) {
    armMotor.setSpeed(0);
    Serial.print("******* Reversing: Direction="); Serial.print(armDir);
    Serial.print(" CycloidalAngle="); Serial.print(cycloidalAngle);
    Serial.print(" Revolutions="); Serial.print(armRevolutions);
    Serial.print(" RawAngle="); Serial.println(armAngle);
    delay(5000);
    armDir *= -1;
    armMotor.setSpeed(armDir * armSpeed);
  }

  if (millis() - lastPrint > 100) {
    Serial.print("Dir:"); Serial.print(armDir);
    Serial.print(" CycloidalAngle:"); Serial.print(cycloidalAngle);
    Serial.print(" Revs:"); Serial.print(armRevolutions);
    Serial.print(" Angle:"); Serial.println(armAngle);
    lastPrint = millis();
  }
}



void homeArm() {
    armMotor.setSpeed(-255);
    int16_t lastPosition = 0, currentPosition = 0;
    bool homed = false;
    uint32_t lastUpdateTime = millis();
    uint32_t startTime = millis();
    int16_t hysteresisVal = 10;
    constexpr uint32_t HOMING_TIMEOUT = 15000;  // 5 second timeout

    while (!homed) {
        if (millis() - startTime > HOMING_TIMEOUT) {
            armMotor.setSpeed(0);
            Serial.println("homing timeout!");
            return;
        }

        armEncoder.getCumulativePosition();
        currentPosition = armEncoder.getAngle();
        if (millis() - lastUpdateTime > 100) {
            Serial.println("homing");
            if ((lastPosition > currentPosition - hysteresisVal) && 
                (lastPosition < currentPosition + hysteresisVal)) {
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


int16_t getGearboxAngle() {  // Changed return type to int16_t
    // Get cumulative position from encoder
    int32_t motorPosition = armEncoder.getCumulativePosition();
    
    // Get position within one full output rotation, preserving sign
    motorPosition = (motorPosition % (30 * 1024));
    
    // Scale to -1024 to +1024 range (for ±180 degrees)
    return (int16_t)((motorPosition * 2048L) / (30 * 1024));
}