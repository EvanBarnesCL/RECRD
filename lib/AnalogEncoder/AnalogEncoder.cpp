#include "AnalogEncoder.h"
// #include "AnalogEncoderConfig.h"

AnalogEncoder::AnalogEncoder(uint8_t pin, bool zeroCrossHysteresis)
    : _pin(pin), _offset(0), _zeroCrossHysteresis(zeroCrossHysteresis), 
      _hysteresisThreshold(1000) {
      _readFunction = &analogRead; // Default to using analogRead
}

void AnalogEncoder::begin() {
    pinMode(_pin, INPUT);
    _lastPosition = update(); // Initialize last position
}

void AnalogEncoder::calibrate() {
    _offset = readAnalog();
}

int AnalogEncoder::update() {
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

void AnalogEncoder::setHysteresisThreshold(int threshold) {
    _hysteresisThreshold = threshold;
}

// New position tracking functions
int32_t AnalogEncoder::getCumulativePosition(bool updatePosition) {
    if (updatePosition) {
        int16_t currentValue = update();
        
        // Check for rotation wrap-around
        if ((_lastPosition > 512) && (currentValue < (_lastPosition - 512))) {
            // Wrapped from high to low - clockwise rotation
            _position = _position + 1024 - _lastPosition + currentValue;
        }
        else if ((currentValue > 512) && (_lastPosition < (currentValue - 512))) {
            // Wrapped from low to high - counter-clockwise rotation
            _position = _position - 1024 - _lastPosition + currentValue;
        }
        else {
            // No wrap-around, just add the difference
            _position = _position - _lastPosition + currentValue;
        }
        
        _lastPosition = currentValue;
    }
    
    return _position;
}

int32_t AnalogEncoder::getRevolutions() {
    // For 10-bit ADC (0-1023), one revolution is 1024 steps
    int32_t revs = _position >> 10;  // Divide by 1024
    if (revs < 0) revs++; // Correct negative values
    return revs;
}

int32_t AnalogEncoder::resetPosition(int32_t position) {
    int32_t oldPosition = _position;
    _position = position;
    return oldPosition;
}

int32_t AnalogEncoder::resetCumulativePosition(int32_t position) {
    _lastPosition = update();
    int32_t oldPosition = _position;
    _position = position;
    return oldPosition;
}

uint16_t AnalogEncoder::getAngle() {
    // Extract just the current angle from the cumulative position
    // by taking the least significant 10 bits
    return abs(_position) & BITMASK;
}

uint16_t AnalogEncoder::readAnalog() {
    return _readFunction(_pin);
}

// New method to set the function pointer for analog reading
void AnalogEncoder::setReadFunction(int (*readFunction)(uint8_t)) {
    _readFunction = readFunction;
}