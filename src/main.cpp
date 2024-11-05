#include <Arduino.h>
#include <Ramp.h>
#include <AnalogEncoder.h>
#include <Adafruit_APDS9960.h>
#include <SPI.h>
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

Adafruit_APDS9960 colorSensor;


void setup()
{
  Serial.begin(115200);
  Serial.println("starting");
  Wire.begin();

  pinMode(TABLE_DIR_PIN, OUTPUT);
  pinMode(TABLE_SPEED_PIN, OUTPUT);
  pinMode(ARM_DIR_PIN, OUTPUT);
  pinMode(ARM_SPEED_PIN, OUTPUT);

  if(!colorSensor.begin()){
    Serial.println("failed to initialize device! Please check your wiring.");
  }
  else Serial.println("Device initialized!");
  delay(1000);
  colorSensor.enableColor(true);

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


void loop() {
    static unsigned long lastPrint = millis();
    static unsigned long lastColorUpdate = millis();
    uint16_t r, g, b, c;
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

    if (millis() - lastPrint > 100) {
      if (colorSensor.colorDataReady()) colorSensor.getColorData(&r, &g, &b, &c);
      String str = "R: " + String(r) + "    G: " + String(g) + "    B: " + String(b) + "    C: " + String(c);
      int tPos = tableEncoder.read();
      int aPos = armEncoder.read();
      str += "    Arm: " + String(aPos) + "    Table: " + String(tPos);
      Serial.println(str);
      lastPrint = millis();
    }

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

