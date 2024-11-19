// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.
// It has a few side effects. It breaks any use of millis(). delay() also will not work, but I plan to avoid
// the use of both of those any time Mozzi is running anyway, so this might be fine! So far this hasn't affected
// my very basic Mozzi sketch.
// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.
#define USE_FAST_PWM

#define USE_MOZZI_ANALOG_READ

#include <Arduino.h>
#include <Crunchlabs_DRV8835.h>
#include <VEML3328.h>
// #include <Wire.h>
#include <PWMFreak.h>
#define MOZZI_CONTROL_RATE 128
#include <Mozzi.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <IntMap.h>
#include <EventDelay.h>
// #include <AnalogEncoder.h>


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

    void calibrate() {
        _offset = readAnalog();
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

    uint16_t getAngle() {
        // Extract just the current angle from the cumulative position
        // by taking the least significant 10 bits
        return abs(_position) & BITMASK;
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

    static const int BITMASK = 0x3FF;  // 10-bit bitmask (1023 in decimal)

    // helper function to make switching between analogRead and mozziAnalogRead simpler
    uint16_t readAnalog() {
        #ifdef USE_MOZZI_ANALOG_READ
            return mozziAnalogRead<10>(_pin);  // Use 16-bit version
        #else
            return analogRead(_pin);
        #endif
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

// int16_t getGearboxAngle();
inline int16_t encoderToDistance(int16_t position);
inline int16_t distanceToEncoder(int16_t distance);


uint16_t armEncVal, tableEncVal, middlePotVal, backPotVal;

AnalogEncoder tableEncoder;

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


  #ifdef USE_FAST_PWM
  // use fast PWM to remove audible noise from driving the motors
  setPwmFrequency(5, 1);
  #endif

  // turn on the LED to illuminate the scene for the color sensor
  digitalWrite(LED_PIN, HIGH);

  delay(1000);

  tableEncoder.begin();
  tableEncoder.getCumulativePosition();

  // start Mozzi
  kVib.setFreq(221.0f);
  startMozzi(MOZZI_CONTROL_RATE);

  // take initial reading on all analog channels to prime the buffer
  // mozziAnalogRead<10>(ARM_ENC_PIN);
  // mozziAnalogRead<10>(TABLE_ENC_PIN);
  // mozziAnalogRead<10>(MIDDLE_POT_PIN);
  // mozziAnalogRead<10>(BACK_POT_PIN);
  // mozziAnalogRead<8>(VOLUME_POT_PIN);


  k_printTimer.set(100);

  tableMotor.setSpeed(50);
}


// **********************************************************************************
// updateControl for Mozzi
// **********************************************************************************

void updateControl() {
  float vibrato = depth * kVib.next();
  vibrato = depth * .5 * kVib.next();
  // vibrato = depth * .33 * kVib.next();
  aSin.setFreq(centre_freq+vibrato);
  globalGain = k_GlobalGainMap(mozziAnalogRead<8>(VOLUME_POT_PIN));
  armEncVal = mozziAnalogRead<10>(ARM_ENC_PIN);
  tableEncoder.getCumulativePosition();
  tableEncVal = tableEncoder.getAngle();
  middlePotVal = mozziAnalogRead<10>(MIDDLE_POT_PIN);
  backPotVal = mozziAnalogRead<10>(BACK_POT_PIN);

  if (k_printTimer.ready()) {
    // Combine the values into a single string with labels and print to the serial monitor
    String output = "T_ENC: " + String(tableEncVal) + "  " +
                    "A_ENC: " + String(armEncVal) + "  " +
                    "F_PIN: " + String(globalGain) + "  " +
                    "M_PIN: " + String(middlePotVal) + "  " +
                    "B_PIN: " + String(backPotVal);
    Serial.println(output);
    k_printTimer.start();
  }
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



// int16_t getGearboxAngle() {  // Changed return type to int16_t
//     // Get cumulative position from encoder
//     int32_t motorPosition = armEncoder.getCumulativePosition(); 
    
//     // Get position within one full output rotation, preserving sign
//     motorPosition = (motorPosition % (30 * 1024));
    
//     // Scale to -1024 to +1024 range (for ±180 degrees)
//     return (int16_t)((motorPosition * 2048L) / (30 * 1024));
// }



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