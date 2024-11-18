#ifndef ANALOGENCODER_H
#define ANALOGENCODER_H

#include <Arduino.h>


class AnalogEncoder {
public:
    // Constructor with pin and optional zeroCrossHysteresis parameter
    AnalogEncoder(uint8_t pin, bool zeroCrossHysteresis = false);

    void begin();     // Initialize the encoder
    void calibrate(); // Set the current position as the offset
    int update();     // Read the value with optional hysteresis handling
    void setHysteresisThreshold(int threshold);  // Set the hysteresis threshold
    
    // Position tracking functions
    int32_t getCumulativePosition(bool updatePosition = true);
    int32_t getRevolutions();
    int32_t resetPosition(int32_t position = 0);
    int32_t resetCumulativePosition(int32_t position = 0);
    uint16_t getAngle();     // Get current angle (0-1023)
    void setReadFunction(int (*readFunction)(uint8_t));

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
    uint16_t readAnalog();
    int (*_readFunction)(uint8_t);     
};

#endif  // ANALOGENCODER_H