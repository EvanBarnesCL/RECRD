// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.
// It has a few side effects. It breaks any use of millis(). delay() also will not work, but I plan to avoid
// the use of both of those any time Mozzi is running anyway, so this might be fine! So far this hasn't affected
// my very basic Mozzi sketch.
// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.
#define USE_FAST_PWM

#define USE_MOZZI_ANALOG_READ

#include <Arduino.h>
#include <Crunchlabs_DRV8835.h>
#include <Wire.h>
#include <VEML3328.h>
#include <PWMFreak.h>
#define MOZZI_CONTROL_RATE 128
#include <Mozzi.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <IntMap.h>
#include <EventDelay.h>


// **********************************************************************************
// Analog Encoder Class
// **********************************************************************************


class AnalogEncoder {
public:
    // Constructor with pin and optional zeroCrossHysteresis parameter
    AnalogEncoder(uint8_t pin, bool zeroCrossHysteresis = false)
        : _pin(pin), _offset(0), _zeroCrossHysteresis(zeroCrossHysteresis),
          _hysteresisThreshold(1000) {}

    void begin() {
        pinMode(_pin, INPUT);
        _lastPosition = update(); // Initialize last position
    }

    // this function relies on stock analogRead and is best to call when Mozzi is not running
    void calibrate() {
      _offset = analogRead(_pin);
      _position = 0;
    }

    int update() {
        int value = readAnalog();
        // Apply the offset
        int adjustedValue = (value - _offset) & BITMASK;
        // If zero-cross hysteresis is enabled, clamp noisy values near the 0 boundary
        if (_zeroCrossHysteresis && adjustedValue > _hysteresisThreshold) {
            adjustedValue = 0;
        }
        _lastReadValue = adjustedValue;
        return adjustedValue;
    }

    void setHysteresisThreshold(int threshold) {
        _hysteresisThreshold = threshold;
    }

    // Position tracking functions
    int32_t getCumulativePosition(bool updatePosition = true) {
        if (updatePosition) {
            int16_t currentValue = update();
            // Check for rotation wrap-around
            if ((_lastPosition > 512) && (currentValue < (_lastPosition - 512))) {
                // Wrapped from high to low - clockwise rotation
                _position = _position + 1024 - _lastPosition + currentValue;
            } else if ((currentValue > 512) && (_lastPosition < (currentValue - 512))) {
                // Wrapped from low to high - counter-clockwise rotation
                _position = _position - 1024 - _lastPosition + currentValue;
            } else {
                // No wrap-around, just add the difference
                _position = _position - _lastPosition + currentValue;
            }
            _lastPosition = currentValue;
        }
        return _position;
    }

    int32_t getRevolutions() {
        // For 10-bit ADC (0-1023), one revolution is 1024 steps
        int32_t revs = _position >> 10;  // Divide by 1024
        if (revs < 0) revs++; // Correct negative values
        return revs;
    }

    int32_t resetPosition(int32_t position = 0) {
        int32_t oldPosition = _position;
        _position = position;
        return oldPosition;
    }

    int32_t resetCumulativePosition(int32_t position = 0) {
        _lastPosition = update();
        int32_t oldPosition = _position;
        _position = position;
        return oldPosition;
    }

    int16_t getAngle() {
        // Extract just the current angle from the cumulative position
        // by taking the least significant 10 bits.
        // This function was modified to return negative values when the
        // cumulative position (_position) is less than 0. At least for me,
        // this helps maintain a sense of rotation direction. Previously angle
        // was always returned as a positive (uint16_t) value, which was confusing
        // for negative direction rotations.
        return (abs(_position) & BITMASK) * (_position < 0 ? -1: 1);
    }

private:
    uint8_t _pin;             // Pin used for analogRead
    int _offset;              // Offset for calibration
    bool _zeroCrossHysteresis; // Apply hysteresis near 0 point
    int _hysteresisThreshold; // Hysteresis threshold for noisy readings
    
    // Position tracking variables
    int32_t _position = 0;    // Cumulative position counter
    int16_t _lastPosition = 0;// Last known position
    int16_t _lastReadValue = 0;// Last read value

    bool _newCalibrate = true;

    static const int BITMASK = 0x3FF;  // 10-bit bitmask (1023 in decimal)

    // helper function to make switching between analogRead and mozziAnalogRead simpler
    uint16_t readAnalog() {
      return mozziAnalogRead<10>(_pin);  // Use 16-bit version
    }
};



// **********************************************************************************
// Arm and Table stuff
// **********************************************************************************


constexpr uint8_t TABLE_SPEED_PIN = 6;
constexpr uint8_t TABLE_DIR_PIN = 7;
constexpr uint8_t ARM_SPEED_PIN = 5;
constexpr uint8_t ARM_DIR_PIN = 4;

constexpr uint8_t ARM_ENC_PIN = A7;
constexpr uint8_t TABLE_ENC_PIN = A6;

constexpr uint8_t LED_PIN = A3;


DRV8835 tableMotor(TABLE_SPEED_PIN, TABLE_DIR_PIN, 90, true);
DRV8835 armMotor(ARM_SPEED_PIN, ARM_DIR_PIN, 160, false);


void homeArm();
int16_t getGearboxAngle(bool useAnalogRead = false);
inline int16_t encoderToDistance(int16_t position);
inline int16_t distanceToEncoder(int16_t distance);


int16_t armEncVal, tableEncVal, middlePotVal, backPotVal;

AnalogEncoder tableEncoder(TABLE_ENC_PIN);
AnalogEncoder armEncoder(ARM_ENC_PIN);

int32_t tableCumulativePosition;
int32_t armCumulativePosition;


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

// delay timer for updating color data
EventDelay k_colorUpdateDelay;


// **********************************************************************************
// Potentiometers
// **********************************************************************************

constexpr uint8_t VOLUME_POT_PIN = A0;
constexpr uint8_t MIDDLE_POT_PIN = A1;
constexpr uint8_t BACK_POT_PIN = A2;

// **********************************************************************************
// Mozzi stuff
// **********************************************************************************

Oscil <2048, MOZZI_AUDIO_RATE> aSin(SIN2048_DATA);
Oscil <2048, MOZZI_CONTROL_RATE> kVib(SIN2048_DATA);
float centre_freq = 440.0;
float depth = 5.0;

// global gain controls
constexpr uint8_t MAX_GLOBAL_GAIN = 64;   // maximum global gain value
uint8_t globalGain = 12;           // global gain value for changing total volume output. non-linear changes in volume
IntMap k_GlobalGainMap(0, 255, 0, MAX_GLOBAL_GAIN);     // maps potentiometer value to be within 0-MAX_GLOBAL_GAIN so you don't blow your ears out

EventDelay k_printTimer;

// **********************************************************************************
// Setup
// **********************************************************************************



void setup()
{
  Serial.begin(115200);
  Serial.println("starting");

  // Calibrate the encoders before Mozzi starts. Calibration relies on stock analogRead(),
  // which is blocking. 
  tableEncoder.begin();
  tableEncoder.calibrate();
  armEncoder.begin();
  
  // Start the homing process for the arm. First, move away from the homing stop for half
  // a second to be sure that we actually hit the stop later on.
  armMotor.setSpeed(255);
  delay(500);
  // Now home the arm. Note that this function depends on millis(), delay(), and analogRead(),
  // so it has to be called in setup() before changing the PWM clock rate for the motor control
  // PWM pins and before starting Mozzi.
  homeArm();
  armEncoder.calibrate();

  // bring the sensor above the edge of the table
  // armMotor.setSpeed(200);
  // while (encoderToDistance(getGearboxAngle(true)) < 30);
  // armMotor.setSpeed(0);

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
  RGBCIR.setGain(2);
  RGBCIR.setSensitivity(high_sens);
  RGBCIR.setDG(2);
  RGBCIR.setIntegrationTime(IT_50MS);

  // Define which color channels to update here. Set to true to enable that color channel.
  updateChannels[static_cast<int>(ColorChannels::RED)] = true;
  updateChannels[static_cast<int>(ColorChannels::GREEN)] = true;
  updateChannels[static_cast<int>(ColorChannels::BLUE)] = true;
  updateChannels[static_cast<int>(ColorChannels::CLEAR)] = true;
  updateChannels[static_cast<int>(ColorChannels::IR)] = true;


  #ifdef USE_FAST_PWM
  // use fast PWM to remove audible noise from driving the motors. 
  // IMPORTANT: This breaks millis() and delay(), and may have other effects I haven't
  // found yet. After this line, you can't use millis() or delay() calls and have them
  // behave as expected.
  setPwmFrequency(5, 1);    // sets Timer0 clock divisor to 1 instead of 64
  #endif


  delay(1000);
  // start Mozzi
  kVib.setFreq(221.0f);
  startMozzi(MOZZI_CONTROL_RATE);


  k_printTimer.set(100);
  k_colorUpdateDelay.set(50);
  // tableMotor.setSpeed(250);
}


// **********************************************************************************
// updateControl for Mozzi
// **********************************************************************************

void updateControl() {
  bool newColorData = false;
  static bool reversed = false;
  static bool initializeControl = true;

  #pragma region controlInitializationRoutine
  // this will run for the first iteration to prime the mozziAnalogRead buffers.
  // this is necessary because of the way mozziAnalogRead asynchronously handles the ADC.
  // This has to be done at the start of updateControl() rather than during setup() because
  // mozziAnalogRead relies on interrupts generated during updateControl to actually start
  // and gather analog readings. 
  // This block also moves the color sensor to the starting position over the edge of the
  // table and primes the buffer for the 5 color sensor channels as well. This way we can
  // start the audio generation with clean sensor values, instead of values that start
  // at 0 and drastically jump to the actual value. 
  if (initializeControl) {
    tableEncoder.getCumulativePosition();
    armEncoder.getCumulativePosition();
    mozziAnalogRead<10>(VOLUME_POT_PIN);
    mozziAnalogRead<10>(MIDDLE_POT_PIN);
    mozziAnalogRead<10>(BACK_POT_PIN);
    armMotor.setSpeed(100);
    // bring the color sensor over the edge of the table
    if (encoderToDistance(getGearboxAngle()) > 30) {
      armMotor.setSpeed(0);
      initializeControl = false;
      // turn on the LED to illuminate the scene for the color sensor
      digitalWrite(LED_PIN, HIGH);
      // now prime buffer for the values for all 5 color sensor channels
      for (int i = 0; i < 10; ) {
        if (k_colorUpdateDelay.ready()) {
          newColorData = updateColorReadings(&colorData);
          k_colorUpdateDelay.start();
          // rotate to updating the next color channel - only one gets updated each loop
          currentColorChannel = static_cast<ColorChannels>((static_cast<int>(currentColorChannel) + 1) % 5);
          i++;
        }
      }
      tableMotor.setSpeed(255);
    }
    return;
  }
  #pragma endregion controlInitializationRoutine

  // update colors as needed
  if (k_colorUpdateDelay.ready()) {
    newColorData = updateColorReadings(&colorData);
    k_colorUpdateDelay.start();
  }
  // if there is new color data, print it to the monitor
  if (newColorData) {
    printColorData();
  }
  // rotate to updating the next color channel - only one gets updated each loop
  currentColorChannel = static_cast<ColorChannels>((static_cast<int>(currentColorChannel) + 1) % 5);



  static int16_t lastTableRevs = 0;
  static int8_t armTargetPos = 30;
  static int8_t armDir = 1;

  tableCumulativePosition = tableEncoder.getCumulativePosition();
  int16_t tableRevolutions = tableEncoder.getRevolutions();
  tableEncVal = tableEncoder.getAngle();


  armCumulativePosition = armEncoder.getCumulativePosition();
  int16_t armRevolutions = armEncoder.getRevolutions();
  armEncVal = armEncoder.getAngle();
  
  int16_t cycloidalAngle = getGearboxAngle();
  int16_t linearPosition = encoderToDistance(cycloidalAngle);  

  if (lastTableRevs != tableRevolutions) {
    if (armDir > 0) {
      if (linearPosition < 60) {
        armTargetPos += 5;
      } else {
        armDir = -1;
        armTargetPos -=5;
      }
    } else if (armDir < 0) {
      if (linearPosition > 30) {
        armTargetPos -=5;
      } else {
        armDir = 1;
        armTargetPos +=5;
      }
    }
    lastTableRevs = tableRevolutions;
  } 

  
  if (armTargetPos > linearPosition) {
    // Serial.println("increasing");
    armMotor.setSpeed(1);
  } else if (armTargetPos < linearPosition) {
    // Serial.println("decreasing");
    armMotor.setSpeed(-1);
  } else {
    // Serial.println("stopped");
    armMotor.setSpeed(0);
  }


  // if (tableRevolutions == 0 && reversed) reversed = false;
  
  // if (tableRevolutions > 1 && !reversed) {
  //   tableMotor.setSpeed(tableMotor.getSpeedCommand() * -1);
  //   reversed = true;
  // }

  // if (tableRevolutions < -1 && !reversed) {
  //   tableMotor.setSpeed(tableMotor.getSpeedCommand() * -1);
  //   reversed = true;
  // }
  
  
  globalGain = k_GlobalGainMap(mozziAnalogRead<8>(VOLUME_POT_PIN));
  middlePotVal = mozziAnalogRead<10>(MIDDLE_POT_PIN);
  backPotVal = mozziAnalogRead<10>(BACK_POT_PIN);

  if (k_printTimer.ready()) {
    // Combine the values into a single string with labels and print to the serial monitor
    String output;
    output += "  ";
    output += "T_REV: " + String(tableRevolutions) + "  " +
              "T_ANGLE: " + String(tableEncVal) + "  " +
              "Table cumulative position: " + String(tableCumulativePosition) + "  " +
              "A_REV: " + String(armRevolutions) + "  " +
              "A_ENC: " + String(armEncVal) + "  " +
              "Arm cumulative position: " + String(armCumulativePosition) + "  " +
              "F_PIN: " + String(globalGain) + "  " +
              "M_PIN: " + String(middlePotVal) + "  " +
              "B_PIN: " + String(backPotVal);
    // Serial.println(output);
    k_printTimer.start();
  }


  float vibrato = depth * kVib.next();
  vibrato = depth * .5 * kVib.next();
  // vibrato = depth * .33 * kVib.next();
  aSin.setFreq(centre_freq+vibrato);

}






// **********************************************************************************
// updateAudio for Mozzi
// **********************************************************************************

AudioOutput_t updateAudio() {
  return MonoOutput::from8Bit((aSin.next() * globalGain)>>8);             // 8 bit sine osc * 8 bit globalGain makes 16 bits total (2^8 * 2^8 = 2^(8+8) = 2^16)
}




// **********************************************************************************
// Loop
// **********************************************************************************

void loop() {
  // synthesize new audio for Mozzi
  audioHook();

  
  
  
}



// **********************************************************************************
// Function Definitions
// **********************************************************************************



void homeArm() {
  uint16_t lastEncoderVal, currentEncoderVal, stopTimerStart, stopTimerCurrent;
  bool timerStarted = false, homed = false;
  armMotor.setSpeed(-255);
  while (!homed) {
    currentEncoderVal = analogRead(ARM_ENC_PIN);
    bool encoderStopped = (currentEncoderVal < lastEncoderVal + 2) && (currentEncoderVal > lastEncoderVal - 2);
    Serial.println(encoderStopped ? "true" : "false");
    if (!timerStarted) {
      if (encoderStopped) {
        timerStarted = true;
        stopTimerStart = millis();
      }
    } else {
      stopTimerCurrent = millis();
      if (encoderStopped) {
        if (stopTimerCurrent - stopTimerStart > 500) {
          Serial.println("finished homing");
          homed = true;   // homing condition complete, return to main program
        }
      } else {
        timerStarted = false;     // the encoder value has changed, so reset the timer
      }
    }
    lastEncoderVal = currentEncoderVal;
    // small delay make sure the loop doesn't run so fast as to get multiple readings at the same encoder
    // position if they motor hasn't actually stopped spinning
    delay(1); 
  }
}



int16_t getGearboxAngle(bool useAnalogRead = false) {  // Changed return type to int16_t
    // Get cumulative position from encoder
    int32_t motorPosition;
    if (useAnalogRead) {
      motorPosition = analogRead(ARM_ENC_PIN);
      // Serial.print("enc: "); Serial.print(motorPosition);
    } else {
      motorPosition = armEncoder.getCumulativePosition(); 
    }
    
    // Get position within one full output rotation, preserving sign
    motorPosition = (motorPosition % (30 * 1024));
    // Serial.print("    motor pos: "); Serial.println(motorPosition);
    
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
  struct updatingColors {
    bool r = false;
    bool g = false;
    bool b = false;
    bool c = false;
    bool ir = false;
  };

  updatingColors channels;

  if (updateChannels[static_cast<int>(ColorChannels::RED)]) {
    if (!first) Serial.print(" "); else first = false;
    // Serial.print("Red:");
    channels.r = true;
    Serial.print(colorData.red);
  }
  if (updateChannels[static_cast<int>(ColorChannels::GREEN)]) {
    if (!first) Serial.print(" "); else first = false;
    // Serial.print("Green:");
    channels.g = true;
    Serial.print(colorData.green);
  }
  if (updateChannels[static_cast<int>(ColorChannels::BLUE)]) {
    if (!first) Serial.print(" "); else first = false;
    // Serial.print("Blue:");
    channels.b = true;
    Serial.print(colorData.blue);
  }
  if (updateChannels[static_cast<int>(ColorChannels::CLEAR)]) {
    if (!first) Serial.print(" "); else first = false;
    // Serial.print("Clear:");
    channels.c = true;
    Serial.print(colorData.clear);
  }
  if (updateChannels[static_cast<int>(ColorChannels::IR)]) {
    if (!first) Serial.print(" "); else first = false;
    // Serial.print("IR:");
    channels.ir = true;
    Serial.print(colorData.IR);
  }
  if (channels.r && channels.g && channels.b && channels.c) {
    int32_t combined = colorData.red + colorData.green + colorData.blue;
    int32_t diff = colorData.clear - combined;
    Serial.print("    combined (r+g+b) = "); Serial.print(combined);
    Serial.print("    clear - combined = "); Serial.print(diff);
  } 
  Serial.println();
}