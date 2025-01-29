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
#include <mozzi_utils.h>


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
    void calibrate(int calibrationValue = 0) {
      _offset = analogRead(_pin);
      _position = calibrationValue;
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


DRV8835 tableMotor(TABLE_SPEED_PIN, TABLE_DIR_PIN, 50, true);
DRV8835 armMotor(ARM_SPEED_PIN, ARM_DIR_PIN, 50, false);


void homeArm();
bool debounceSwitch();


int16_t armDistanceToAngle(int16_t distance);     // calculates arm encoder angle from linear position of color sensor
int16_t armAngleToDistance(int16_t angle);        // calculates linear position of color sensor based on arm encoder angle
bool initializationRoutine();                     // initialization routine to properly position the arm over the table and prime the color sensor
uint8_t noteNameToMIDINote(char noteName[]);          // convert note names to MIDI note numbers (e.g., F#2 -> 42)
const char* MIDINoteToNoteName(uint8_t note);     // convert MIDI note to note name (e.g., 42 -> F#2)

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
// Potentiometers and switch
// **********************************************************************************

constexpr uint8_t VOLUME_POT_PIN = A0;
constexpr uint8_t MIDDLE_POT_PIN = A1;
constexpr uint8_t BACK_POT_PIN = A2;
constexpr uint8_t HOMING_SWITCH = 2;

// **********************************************************************************
// Mozzi stuff
// **********************************************************************************

Oscil <2048, MOZZI_AUDIO_RATE> aSin(SIN2048_DATA);
Oscil <2048, MOZZI_CONTROL_RATE> kVib(SIN2048_DATA);
float centre_freq = 440.0;
float depth = 5.0;
float vibratoFreq = 221.0;

// global gain controls
constexpr uint8_t MAX_GLOBAL_GAIN = 64;   // maximum global gain value
uint8_t globalGain = 12;           // global gain value for changing total volume output. non-linear changes in volume
IntMap k_GlobalGainMap(0, 255, 0, MAX_GLOBAL_GAIN);     // maps potentiometer value to be within 0-MAX_GLOBAL_GAIN so you don't blow your ears out
IntMap k_redChannelVibMap(0, 32767, 80, 2000);       // signed int, max value of 32767
IntMap k_potToFreq(0, 1023, 80, 400);

EventDelay k_printTimer;

bool newColorData = false;      // flag to indicate that new color data is ready for use
int16_t currentArmPosition = 0; // current arm position in millimeters from center of table


// **********************************************************************************
// Setup
// **********************************************************************************

void setup()
{
  Serial.begin(115200);
  Serial.println("starting");

  // set up pin modes
  pinMode(HOMING_SWITCH, INPUT_PULLUP);

  // Calibrate the encoders before Mozzi starts. Calibration relies on stock analogRead(),
  // which is blocking. 
  tableEncoder.begin();
  tableEncoder.calibrate(0);    // calibrate to 0 as starting position for the table
  armEncoder.begin();
  
  // Now home the arm. Note that this function depends on delay() and analogRead(),
  // so it has to be called in setup() before starting Mozzi, and before changing 
  // the PWM clock rate for the motor controller's PWM pins.
  homeArm();
  // The arm is at the home position, so we can calibrate the encoder finally.
  // Adding settling delay before calibrating to be sure the motor is fully stopped.
  delay(100);
  armEncoder.calibrate(512);    // calibrate to 512 as our starting encoder measurement

  // set up the color sensor over i2c
  Wire.begin(); 
  if (!RGBCIR.begin()) {                                   
    Serial.println("ERROR: couldn't detect the sensor");
    while (1){}            
  }
  // IMPORTANT: Set the I2C clock to 400kHz fast mode AFTER initializing the connection to the sensor.
  // Need to use fastest I2C possible to minimize latency for Mozzi.
  Wire.setClock(400000);

  // Configure the color sensor.
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

  delay(1000);

  #ifdef USE_FAST_PWM
  // use fast PWM to remove audible noise from driving the motors. 
  // IMPORTANT: This breaks millis() and delay(), and may have other effects I haven't
  // found yet. After this line, you can't use millis() or delay() calls and have them
  // behave as expected.
  setPwmFrequency(5, 1);    // sets Timer0 clock divisor to 1 instead of 64
  #endif


  // start Mozzi
  kVib.setFreq(vibratoFreq);
  startMozzi(MOZZI_CONTROL_RATE);

  k_printTimer.set(100);
  k_colorUpdateDelay.set(50);
  
}





// **********************************************************************************
// updateControl for Mozzi
// **********************************************************************************

void updateControl() {
  newColorData = false;
  static bool initializeControl = true;
  
  currentArmPosition = armAngleToDistance(armEncoder.getCumulativePosition());

  // if the system has just started, we need to initialize a few things during the updateControl loop
  // prior to actually running the main sound and motor controls.
  if (initializeControl) {
    initializeControl = initializationRoutine();
    return;     // break out of the updateControl loop early if initializeControl is still true
  }
  
  
  // set the speed the arm will move at for the test pattern
  constexpr int16_t armSpeed = 0;

  // as a test pattern, move the arm in and out from edge to center and back
  static int16_t armVector = -1 * armSpeed;
  if (currentArmPosition < 5) {
    armVector = armSpeed;
  } else if (currentArmPosition > 71) {
    armVector = -1 * armSpeed;
  }
  armMotor.setSpeed(armVector);
  
  
  // update colors as needed (update interval determined by k_colorUpdateDelay)
  if (k_colorUpdateDelay.ready()) {
    newColorData = updateColorReadings(&colorData);
    k_colorUpdateDelay.start();
  }
  // if there is new color data, print it to the monitor
  if (newColorData) {
    printColorData();
  }
  // rotate to updating the next color channel - only one gets updated each loop.
  // this minimizes time spent in the updateControl loop waiting for color data. 
  // updateControl needs to run as quickly as possible, which is why I built it to do this.
  currentColorChannel = static_cast<ColorChannels>((static_cast<int>(currentColorChannel) + 1) % 5);

  // update the potentiometer values
  globalGain = k_GlobalGainMap(mozziAnalogRead<8>(VOLUME_POT_PIN));
  middlePotVal = mozziAnalogRead<10>(MIDDLE_POT_PIN);
  backPotVal = mozziAnalogRead<10>(BACK_POT_PIN);

  // finally, actually change the sound being generated based on the various controls
  aSin.setFreq(static_cast<float>(k_redChannelVibMap(colorData.red >> 1)));

}






// **********************************************************************************
// updateAudio for Mozzi
// **********************************************************************************

AudioOutput_t updateAudio() {
  // 8 bit sine osc * 8 bit globalGain makes 16 bits total (2^8 * 2^8 = 2^(8+8) = 2^16). 
  // We need to generate audio output, and Mozzi has a few functions for doing that.
  // One is MonoOutput::from8Bit. If we want to use that, we have to bring everything back
  // into 8 bit from 16 by shifting right by 8 places (divide by 2^8) to bring this back to 8 bit.
  // that would work like the following:
  // return MonoOutput::from8Bit((aSin.next() * globalGain)>>8);             

  // Or, there's MonoOutput::from16Bit, which we can just use directly without shifting:
  return MonoOutput::from16Bit((aSin.next() * globalGain));

  // there's also stereo output, which I haven't played with yet:
  // return StereoOutput::from8Bit((aSin.next() * globalGain)>>8);
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
  // first check to see if we're starting homing with the switch already pressed, move away if so
  if (digitalRead(HOMING_SWITCH) == 0) {
    armMotor.setSpeed(255);
    while (debounceSwitch());   // delay while the switch is held down
    delay(500);                 // move a bit more just for clearance
    armMotor.setSpeed(0);
    delay(200);
  }

  // start the arm motor spinning
  armMotor.setSpeed(255);
  // wait for the switch to be pressed
  while (!debounceSwitch()) {
  }

  // read the current angle on the encoder
  uint16_t switchActivatedAngle = analogRead(ARM_ENC_PIN), homeAngle = 0;
  constexpr uint16_t switchOffsetAngle = 15;            // angular distance of the switch activation position from the desired home position
  // set the target home position angle
  homeAngle = switchActivatedAngle - switchOffsetAngle;    
  homeAngle &= 1023;          // make sure this value wraps around 0 point correctly
  armMotor.setSpeed(-64);     // slow down the motor for the final approach
  while (analogRead(ARM_ENC_PIN) != homeAngle) {
    // Serial.println(analogRead(ARM_ENC_PIN));
    // just let the motor run until we hit the home angle
  }
  armMotor.setSpeed(0);
}





bool debounceSwitch() {
  static bool triggered = false, firstClose = true;
  static uint16_t triggeredTime = 0;
  triggered = digitalRead(HOMING_SWITCH) == 0 ? true : false;
  if (triggered && firstClose) {
    triggeredTime = millis();
    firstClose = false;
  } else if (triggered && millis() - triggeredTime > 5) {
    firstClose = true;
    triggered = false;
    return true;
  }
  return false;
}


int16_t armDistanceToAngle(int16_t distance) {
  // This is only accurate for input distances ranging from 0 to 100mm.
  // Beyond this point the mechanism itself is non-linear.

  // Multiply by 501 (5.01 * 100) for fixed-point arithmetic
  // Break into 501 = 512 - 11 for efficient computation using shifts
  int32_t temp = distance;  // Use 32-bit for intermediate calculation
  
  // Multiply by 512 using left shift (2^9 = 512)
  int32_t angle = temp << 9;
  
  // Subtract 11 * x
  angle -= temp * 11;
  
  // Divide by 100 to remove fixed-point scaling
  // Use combination of shifts and division for better efficiency
  // First shift right by 2 (divide by 4) then divide by 25
  angle = (angle >> 2) / 25;
  
  // found to be unneccessary - Add offset of 507
  // angle += 507;
  
  return (int16_t)angle;
}



int16_t armAngleToDistance(int16_t angle) {
  // This is only accurate for input angles ranging from 0 to 512 encoder counts.
  // Beyond this point the mechanism itself is non-linear.

  // Multiply by 20 (0.2 * 100) for fixed-point arithmetic
  // Use 16 + 4 for efficient computation using shifts
  int32_t temp = angle;  // Use 32-bit for intermediate calculation
  
  // Multiply temp by (16 + 4) using left shifts and addition
  int32_t distance = (temp << 4) + (temp << 2);

  // Divide by 100 to remove fixed-point scaling
  // Use combination of shifts and division for better efficiency
  // First shift right by 2 (divide by 4) then divide by 25
  distance = (distance >> 2) / 25;
  
  // found to be unneccessary - Add offset of 101
  // distance += 101;
  
  return (int16_t)distance;
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




// this will run for the first iteration to prime the mozziAnalogRead buffers.
// this is necessary because of the way mozziAnalogRead asynchronously handles the ADC.
// This has to be done at the start of updateControl() rather than during setup() because
// mozziAnalogRead relies on interrupts generated during updateControl to actually start
// and gather analog readings. 
// This block also moves the color sensor to the starting position over the edge of the
// table and primes the buffer for the 5 color sensor channels as well. This way we can
// start the audio generation with clean sensor values, instead of values that start
// at 0 and drastically jump to the actual value. 
bool initializationRoutine() {
  tableEncoder.getCumulativePosition();
  armEncoder.getCumulativePosition();
  mozziAnalogRead<10>(VOLUME_POT_PIN);
  mozziAnalogRead<10>(MIDDLE_POT_PIN);
  mozziAnalogRead<10>(BACK_POT_PIN);

  Serial.print(currentArmPosition);
  Serial.print("   measured: ");
  Serial.print(armEncoder.getCumulativePosition());
  Serial.print("   calculated: ");
  Serial.println(armDistanceToAngle(currentArmPosition));
  
  armMotor.setSpeed(-255);
  // bring the color sensor over the edge of the table


  if (currentArmPosition <= 70) {
    armMotor.setSpeed(0);
    // turn on the LED to illuminate the scene for the color sensor
    digitalWrite(LED_PIN, HIGH);

    // now prime buffer for the values for all 5 color sensor channels
    Serial.println("here");
    for (int i = 0; i < 10; ) {
      Serial.print("here now "); Serial.println(i);
      if (k_colorUpdateDelay.ready()) {
        newColorData = updateColorReadings(&colorData);
        k_colorUpdateDelay.start();
        // rotate to updating the next color channel - only one gets updated each loop
        currentColorChannel = static_cast<ColorChannels>((static_cast<int>(currentColorChannel) + 1) % 5);
        i++;
      }
    }
    // start the table spinning
    tableMotor.setSpeed(100);
    return false;   // initialization is finished, so we'll stop running this routine
  }
  // initialization isn't finished yet, so return true
  return true;
}



// this returns the MIDI note number for any note from C-1 to G9 (MIDI notes 0 through 127).
// Pass the note name as a string into the parameter. E.g., "D#-1" returns 3, or "F#2" returns 42.
uint8_t noteNameToMIDINote(char noteName[]) {
  // Arrays for natural and sharp notes
  const char* naturalNotes[7] = {"C", "D", "E", "F", "G", "A", "B"};
  const uint8_t naturalNoteBases[7] = {0, 2, 4, 5, 7, 9, 11};
  const char* sharpNotes[5] = {"C", "D", "F", "G", "A"};
  const uint8_t sharpNoteBases[5] = {1, 3, 6, 8, 10};

  constexpr uint8_t OCTAVE = 12;

  int8_t noteBaseIndex = -1;  // Base index for the note
  int8_t octaveNumber = 0;    // Octave number

  // Check if the note is sharp or natural
  if (noteName[1] == '#') {
    // Sharp note
    for (int i = 0; i < 5; i++) {
      if (noteName[0] == sharpNotes[i][0]) {
        noteBaseIndex = sharpNoteBases[i];
        break;
      }
    }
    if (noteBaseIndex == -1 || (noteName[2] != '-' && !isdigit(noteName[2]))) return 255; // Invalid note
    // Handle negative octave
    if (noteName[2] == '-') {
      octaveNumber = -1; // Convert char to int and make negative
    } else {
      octaveNumber = noteName[2] - '0'; // Convert char to int
    }
  } else {
    // Natural note
    for (int i = 0; i < 7; i++) {
      if (noteName[0] == naturalNotes[i][0]) {
        noteBaseIndex = naturalNoteBases[i];
        break;
      }
    }
    if (noteBaseIndex == -1 || (noteName[1] != '-' && !isdigit(noteName[1]))) return 255; // Invalid note
    // Handle negative octave
    if (noteName[1] == '-') {
      octaveNumber = -(noteName[2] - '0'); // Convert char to int and make negative
    } else {
      octaveNumber = noteName[1] - '0'; // Convert char to int
    }
  }
  // Calculate the MIDI note number
  return (octaveNumber + 1) * OCTAVE + noteBaseIndex;
}




// Converts a MIDI note number into a string (const char*) note name. E.g., 42 -> F#2
const char* MIDINoteToNoteName(uint8_t note) {
  // Note names for one octave
  const char* noteNames[] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
  };

  if (note > 127) {
    return "Invalid"; // Return an error string for invalid MIDI numbers
  }

  // Determine octave
  int8_t octave = (note / 12) - 1;
  uint8_t noteIndex = note % 12;

  // Allocate a static buffer to store the note string
  static char noteStr[5]; // Max length: "A#-1" + null terminator = 5 bytes
  snprintf(noteStr, sizeof(noteStr), "%s%d", noteNames[noteIndex], octave);

  return noteStr;
}
