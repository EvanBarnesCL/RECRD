#include <Arduino.h>
#include <AnalogEncoder.h>
#include <digitalWriteFast.h>
#include <Crunchlabs_DRV8835.h>
#include <VEML3328.h>
#include <Wire.h>

// **********************************************************************************
// Arm and Table stuff
// **********************************************************************************

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

constexpr uint8_t LED_PIN = A3;


AnalogEncoder tableEncoder(TABLE_ENC_PIN);
AnalogEncoder armEncoder(ARM_ENC_PIN);


DRV8835 tableMotor(TABLE_SPEED_PIN, TABLE_DIR_PIN, 85, true);
DRV8835 armMotor(ARM_SPEED_PIN, ARM_DIR_PIN, 160, false);

int16_t armSpeed = 1;
int8_t armDir = 1;
int16_t armAngle, armRevolutions;

void homeArm();
int16_t getGearboxAngle();
inline int16_t encoderToDistance(int16_t position);
inline int16_t distanceToEncoder(int16_t distance);


// **********************************************************************************
// Color sensor
// **********************************************************************************


VEML3328 RGBCIR;

enum class ColorChannels {
  RED,
  GREEN,
  BLUE,
  CLEAR,
  IR
};

ColorChannels currentColorChannel = ColorChannels::RED;

struct ColorValues {
  uint16_t red = 0;
  uint16_t green = 0;
  uint16_t blue = 0;
  uint16_t clear = 0;
  uint16_t IR = 0;
};

ColorValues colorData;
bool updateChannels[5] = {false, false, false, false, false}; // Flags to control which channels to update. defaults to all five off.

bool updateColorReadings(ColorValues *colorReadings);
void printColorData();


// **********************************************************************************
// Setup
// **********************************************************************************



void setup()
{
  Serial.begin(115200);
  Serial.println("starting");
  armEncoder.begin();
  homeArm();
  armEncoder.calibrate();
  armEncoder.resetCumulativePosition();
  tableEncoder.begin();
  tableEncoder.calibrate();
  tableEncoder.resetCumulativePosition();

  // set up the color sensor over i2c
  Wire.begin(); 
  if (!RGBCIR.begin()) {                                   
    Serial.println("ERROR: couldn't detect the sensor");
    while (1){}            
  }
  // IMPORTANT: Set the I2C clock to 400kHz fast mode AFTER initializing the connection to the sensor.
  // Need to use fastest I2C possible to minimize latency for Mozzi.
  Wire.setClock(400000);

  // Configure the color sensor
  RGBCIR.Enable();
  RGBCIR.setGain(3);
  RGBCIR.setSensitivity(high_sens);
  RGBCIR.setDG(4);
  RGBCIR.setIntegrationTime(IT_50MS);

  // Define which color channels to update here. Set to true to enable that color channel.
  updateChannels[static_cast<int>(ColorChannels::RED)] = true;
  updateChannels[static_cast<int>(ColorChannels::GREEN)] = true;
  updateChannels[static_cast<int>(ColorChannels::BLUE)] = true;
  updateChannels[static_cast<int>(ColorChannels::CLEAR)] = true;
  updateChannels[static_cast<int>(ColorChannels::IR)] = true;
  
  // bring the sensor above the edge of the table
  armMotor.setSpeed(200);
  while (encoderToDistance(getGearboxAngle()) < 30);
  armMotor.setSpeed(0);

  // turn on the LED to illuminate the scene for the color sensor
  digitalWrite(LED_PIN, HIGH);

  delay(1000);

  // start moving the table and arm
  tableMotor.setSpeed(1);
  // armMotor.setSpeed(armSpeed);
}



// **********************************************************************************
// Loop
// **********************************************************************************




void loop() {
  static uint16_t lastColorUpdateTime = 0;
  uint16_t currentColorUpdateTime = millis();
  bool newColorData = false;
  uint16_t updateStartTime, updateEndTime, liveTime;

  // update colors as needed
  if (currentColorUpdateTime - lastColorUpdateTime > 50) {
    updateStartTime = micros();
    newColorData = updateColorReadings(&colorData);
    updateEndTime = micros();
    lastColorUpdateTime = currentColorUpdateTime;
  }
  // if there is new color data, print it to the monitor
  if (newColorData) {
    liveTime = updateEndTime - updateStartTime; 
    Serial.print(liveTime);
    Serial.print(" ");
    printColorData();
  }
  
  // rotate to updating the next color channel - only one gets updated each loop
  currentColorChannel = static_cast<ColorChannels>((static_cast<int>(currentColorChannel) + 1) % 5);

  static unsigned long lastPrint = millis();
  // int32_t armCompletePosition = armEncoder.getCumulativePosition();
  armAngle = armEncoder.getAngle();
  armRevolutions = armEncoder.getRevolutions();
  int16_t cycloidalAngle = getGearboxAngle();
  int16_t linearPosition = encoderToDistance(cycloidalAngle);

  // bool reverseArm = (armDir > 0 && linearPosition > 100 ) || 
  //                 (armDir < 0 && linearPosition < 25);
  
  // if (reverseArm) {
  //   armMotor.setSpeed(0);
  //   delay(1000);
  //   armDir *= -1;
  //   armMotor.setSpeed(armDir * armSpeed);
  // }

  if (millis() - lastPrint > 100) {
    lastPrint = millis();
  }
}



// **********************************************************************************
// Function Definitions
// **********************************************************************************



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
                armEncoder.calibrate();
                armEncoder.resetCumulativePosition();
                // move back off the homing stop to true zero.
                armMotor.setSpeed(100);
                while (encoderToDistance(getGearboxAngle()) < 3) {
                }
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



// Convert arm encoder position (0-1024) to radial distance (0-100mm) (outer edge = 0, center of plate = 100mm)
inline int16_t encoderToDistance(int16_t position) {
    if (position < 0 || position > 1024) return 0;
    return ((int32_t)position * 26) >> 8;
}

// Convert arm radial distance (0-100mm) to encoder position (0-1024)
inline int16_t distanceToEncoder(int16_t distance) {
    if (distance < 0 || distance > 100) return 0;
    return ((int32_t)distance * 10);
}




bool updateColorReadings(ColorValues *colorReadings) {
  // check to see if the channel is enabled && if the current color channel needs to be updated
  if (updateChannels[static_cast<int>(ColorChannels::RED)] && currentColorChannel == ColorChannels::RED) {
    colorReadings->red = RGBCIR.getRed();
  } else if (updateChannels[static_cast<int>(ColorChannels::GREEN)] && currentColorChannel == ColorChannels::GREEN) {
    colorReadings->green = RGBCIR.getGreen();
  } else if (updateChannels[static_cast<int>(ColorChannels::BLUE)] && currentColorChannel == ColorChannels::BLUE) {
    colorReadings->blue = RGBCIR.getBlue();
  } else if (updateChannels[static_cast<int>(ColorChannels::CLEAR)] && currentColorChannel == ColorChannels::CLEAR) {
    colorReadings->clear = RGBCIR.getClear();
  } else if (updateChannels[static_cast<int>(ColorChannels::IR)] && currentColorChannel == ColorChannels::IR) {
    colorReadings->IR = RGBCIR.getIR();
  } else {
    return false;   // no color data was updated, so return false
  }
  return true;      // indicates that new color data is available
}


void printColorData() {
  bool first = true; // To manage commas between printed values

  if (updateChannels[static_cast<int>(ColorChannels::RED)]) {
    if (!first) Serial.print(" "); else first = false;
    // Serial.print("Red:");
    Serial.print(colorData.red);
  }
  if (updateChannels[static_cast<int>(ColorChannels::GREEN)]) {
    if (!first) Serial.print(" "); else first = false;
    // Serial.print("Green:");
    Serial.print(colorData.green);
  }
  if (updateChannels[static_cast<int>(ColorChannels::BLUE)]) {
    if (!first) Serial.print(" "); else first = false;
    // Serial.print("Blue:");
    Serial.print(colorData.blue);
  }
  if (updateChannels[static_cast<int>(ColorChannels::CLEAR)]) {
    if (!first) Serial.print(" "); else first = false;
    // Serial.print("Clear:");
    Serial.print(colorData.clear);
  }
  if (updateChannels[static_cast<int>(ColorChannels::IR)]) {
    if (!first) Serial.print(" "); else first = false;
    // Serial.print("IR:");
    Serial.print(colorData.IR);
  }
  Serial.println();
}