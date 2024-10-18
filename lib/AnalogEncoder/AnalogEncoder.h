#ifndef ANALOGENCODER_H
#define ANALOGENCODER_H

#include <Arduino.h>

class AnalogEncoder {
public:
    // Constructor with pin and optional zeroCrossHysteresis parameter
    AnalogEncoder(uint8_t pin, bool zeroCrossHysteresis = false);

    void begin();     // Initialize the encoder
    void calibrate(); // Set the current position as the offset
    int read();       // Read the value with optional hysteresis handling
    void setHysteresisThreshold(int threshold);  // Set the hysteresis threshold

private:
    uint8_t _pin;             // Pin used for analogRead
    int _offset;              // Offset for calibration
    bool _zeroCrossHysteresis; // Apply hysteresis near 0 point
    int _hysteresisThreshold; // Hysteresis threshold for noisy readings

    static const int BITMASK = 0x3FF;  // 10-bit bitmask (1023 in decimal)
};

#endif  // ANALOGENCODER_H
