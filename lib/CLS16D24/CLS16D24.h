/**
 * @file CLS16D24.h
 * @brief Header file for the CLS16D24 color sensor library.
 *
 * This library provides functions to interface with the CLS16D24 color sensor over I2C.
 * It includes functions for setting gain, resolution, and conversion time, as well as reading color data.
 * 
 * To do: 
 * - There the sensor has methods for setting interrupts based on high and low thresholds on the color channels. 
 *   The library currently doesn't have any functions for setting those interrupts, but they could be cool to 
 *   add for future-proofing purposes. 
 */

#ifndef CLS16D24_H
#define CLS16D24_H

#include <Arduino.h>
#include <Wire.h>

#define CLS16D24_ADDRESS 0x38  // Default I2C address

// Register addresses
#define SYSM_CTRL 0x00
#define CLS_GAIN 0x04
#define CLS_TIME 0x05
#define RCH_DATA_L 0x1C
#define RCH_DATA_H 0x1D
#define GCH_DATA_L 0x1E
#define GCH_DATA_H 0x1F
#define BCH_DATA_L 0x20
#define BCH_DATA_H 0x21
#define WCH_DATA_L 0x22
#define WCH_DATA_H 0x23
#define IRCH_DATA_L 0x24
#define IRCH_DATA_H 0x25

/**
 * @class CLS16D24
 * @brief Class for interfacing with the CLS16D24 color sensor.
 */
class CLS16D24 {
public:
    /**
     * @brief Constructor for CLS16D24 class.
     */
    CLS16D24();

    /**
     * @brief Initializes the sensor and sets up the I2C communication.
     * @param wirePort Reference to the I2C port (default: Wire).
     * @return True if initialization was successful.
     */
    bool begin(bool i2cFastMode = true, TwoWire &wirePort = Wire) {
        wire = &wirePort;
        wire->begin();
        if (i2cFastMode) wire->setClock(400000); // Set I2C to Fast Mode (400kHz)
        return true;
    }

    /**
     * @brief Resets the sensor.
     */
    void reset();

    /**
     * @brief Enables the sensor.
     */
    void enable();

    /**
     * @brief Sets the gain of the sensor.
     * @param gain Gain factor (1, 4, 8, 32, or 96).
     * @param doubleSensorArea Whether to enable double sensor area (default: true).
     */
    void setGain(uint8_t gain, bool doubleSensorArea = true);

    /**
     * @brief Sets the resolution and conversion time for the sensor.
     * @param time Resolution and conversion time setting.
     */
    void setResolutionAndConversionTime(uint8_t time);

    /**
     * @brief Reads RGBWIR color data from the sensor.
     * @param r Reference to store red channel value.
     * @param g Reference to store green channel value.
     * @param b Reference to store blue channel value.
     * @param w Reference to store white channel value.
     * @param ir Reference to store infrared channel value.
     */
    void readRGBWIR(uint16_t &r, uint16_t &g, uint16_t &b, uint16_t &w, uint16_t &ir);

    /**
     * @brief Returns the conversion time in milliseconds.
     * @return Conversion time in milliseconds.
     */
    float getConversionTimeMillis();

    /**
     * @brief Returns the resolution set for the sensor.
     * @return Resolution value.
     */
    uint16_t getResolution();

    /**
     * @brief Returns the current gain setting of the sensor.
     * @return Gain factor (1, 4, 8, 32, or 96).
     */
    uint8_t getGain();

private:
    uint8_t GAIN_BYTE = 0x10; ///< Stores the gain setting.
    uint8_t CONV_TIME = 0x10; ///< Stores the conversion time setting.
    uint8_t CLS_CONV = CONV_TIME >> 4; ///< Stores CLS_CONV value.
    uint8_t INT_TIME = intPow(4, (CONV_TIME & 0x03)); ///< Stores INT_TIME value.

    /**
     * @brief Fast integer exponentiation.
     * @param base Base value.
     * @param exp Exponent value.
     * @return Computed power value.
     */
    inline uint16_t intPow(uint16_t base, uint16_t exp);

    /**
     * @brief Writes a value to a register.
     * @param reg Register address.
     * @param value Value to write.
     */
    void writeRegister(uint8_t reg, uint8_t value);

    /**
     * @brief Reads a 16-bit value from a register.
     * @param reg Register address.
     * @return 16-bit register value.
     */
    uint16_t readRegister16(uint8_t reg);

    TwoWire *wire; ///< Pointer to I2C interface.
};

#endif  // CLS16D24_H