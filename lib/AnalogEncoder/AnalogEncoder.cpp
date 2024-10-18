#include "AnalogEncoder.h"

// Constructor to initialize pin and zeroCrossHysteresis
AnalogEncoder::AnalogEncoder(uint8_t pin, bool zeroCrossHysteresis)
    : _pin(pin), _offset(0), _zeroCrossHysteresis(zeroCrossHysteresis), 
      _hysteresisThreshold(1000) {}  // Default hysteresis threshold

// Initialize the analog pin (optional, for clarity)
void AnalogEncoder::begin() {
    pinMode(_pin, INPUT);
}

// Set the current reading as the offset
void AnalogEncoder::calibrate() {
    _offset = analogRead(_pin);
}

// Read the encoder value with optional zero-cross hysteresis handling
int AnalogEncoder::read() {
    int value = analogRead(_pin);

    // Apply the offset
    int adjustedValue = (value - _offset) & BITMASK;

    // If zero-cross hysteresis is enabled, clamp noisy values near the 0 boundary
    if (_zeroCrossHysteresis && adjustedValue > _hysteresisThreshold) {
        adjustedValue = 0;
    }

    return adjustedValue;
}

// Set a new hysteresis threshold
void AnalogEncoder::setHysteresisThreshold(int threshold) {
    _hysteresisThreshold = threshold;
}
