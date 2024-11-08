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

constexpr int HYST_OFFSET = 10;

constexpr int ARM_MOTOR_MIN_SPEED = 150;      // in testing, this was the lowest PWM value that would reliably move the arm to the demanded position without occasional stalls.


// temporary function for setting motor speed. make this a class later.
void setMotorSpeed(uint8_t speedPin, uint8_t dirPin, int8_t speed, uint8_t minPWM = 0);

int target = 0, targetDeg = 0;  // in encoder counts, in degrees

constexpr uint8_t MIN_ARM_PWM = 80;

// rampInt positionRamp;


AnalogEncoder tableEncoder(TABLE_ENC_PIN);
AnalogEncoder armEncoder(ARM_ENC_PIN);


DRV8835 tableMotor(TABLE_SPEED_PIN, TABLE_DIR_PIN, 85, true);


void setup()
{
  Serial.begin(115200);
  Serial.println("starting");

  

  while (true) {
    int speed = 1;
    tableMotor.setSpeed(speed);
    Serial.print(speed); Serial.print("    "); Serial.println(tableMotor.getspeedActual());
    delay(10000);
    speed = -1;
    tableMotor.setSpeed(speed);
    Serial.print(speed); Serial.print("    "); Serial.println(tableMotor.getspeedActual());
    delay(2000);
  }

  // pinMode(TABLE_DIR_PIN, OUTPUT);
  // pinMode(TABLE_SPEED_PIN, OUTPUT);
  // pinMode(ARM_DIR_PIN, OUTPUT);
  // pinMode(ARM_SPEED_PIN, OUTPUT);


  // digitalWrite(ARM_DIR_PIN, ARM_OUT);
  // analogWrite(ARM_SPEED_PIN, 200);
  // delay(3000);
  // analogWrite(ARM_SPEED_PIN, 0);

  setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, 255, 85);
  delay(8000);
  setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, 0, 85);

  armEncoder.begin();
  tableEncoder.begin();
  armEncoder.calibrate();
  tableEncoder.calibrate();

  delay(2000);

  // positionRamp.go(512, 20000, LINEAR, FORTHANDBACK);
  // target = 0;


  // start the table spinning just for fun
  // digitalWrite(TABLE_DIR_PIN, CW);
  // analogWrite(TABLE_SPEED_PIN, 80);

  setMotorSpeed(TABLE_SPEED_PIN, TABLE_DIR_PIN, 1 * CW, 90);
}


void loop() {
  static unsigned long lastPrint = millis();
  static unsigned long lastColorUpdate = millis();
  uint16_t r, g, b, c;

  if (millis() - lastPrint > 100) {
    Serial.println(tableEncoder.read());
    lastPrint = millis();
  }


  // target = positionRamp.update();
  // int currentPos = armEncoder.read();
  // int error = target - currentPos;
  // int mSpeed = (int)(1.0 * error);
  // if (error > 5) {
  //   setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, -ARM_MOTOR_MIN_SPEED);
  // } else if (error < -5) {
  //   setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, ARM_MOTOR_MIN_SPEED);
  // } else {
  //   setMotorSpeed(ARM_SPEED_PIN, ARM_DIR_PIN, 0);

  //   if (millis() - lastPrint > 100) {
  //     if (colorSensor.colorDataReady()) colorSensor.getColorData(&r, &g, &b, &c);
  //     String str = "R: " + String(r) + "    G: " + String(g) + "    B: " + String(b) + "    C: " + String(c);
  //     int tPos = tableEncoder.read();
  //     int aPos = armEncoder.read();
  //     str += "    Arm: " + String(aPos) + "    Table: " + String(tPos);
  //     Serial.println(str);
  //     lastPrint = millis();
  //   }

  // }
}

void setMotorSpeed(uint8_t speedPin, uint8_t dirPin, int8_t speed, uint8_t minPWM = 0) {
  uint8_t _speed;
  if (speed == 0) {
    analogWrite(speedPin, 0);
  } else if (speed < 0) {
    // Reverse direction
    digitalWrite(dirPin, ARM_OUT);
    _speed = abs(speed);
    _speed = map(_speed, 0, 255, minPWM, 255);
    analogWrite(speedPin, _speed); // Use absolute value for speed
  } else {
    // Forward direction
    digitalWrite(dirPin, ARM_IN);
    _speed = abs(speed);
    _speed = map(_speed, 0, 255, minPWM, 255);
    analogWrite(speedPin, _speed);
  }
  Serial.println(_speed);
}

