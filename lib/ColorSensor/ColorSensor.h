/**
 * @file ColorSensor.h
 * @brief Class wrapper around the CLS16D24 color sensor.
 *
 * Handles hardware communication, white-balance correction, and
 * conversion to the fixed-point format used by Mozzi/FixMath.
 * Raw and scaled data are stored internally and accessed via getters.
 *
 * Typical usage:
 *
 *   ColorSensor colorSensor;
 *
 *   // in setup():
 *   colorSensor.begin(false);
 *   colorSensor.reset();
 *   colorSensor.enable();
 *   colorSensor.setGain(32, false);
 *   colorSensor.setResolutionAndConversionTime(0x02);
 *   colorSensor.setChannelEnabled(ColorChannels::RED,   true);
 *   colorSensor.setChannelEnabled(ColorChannels::GREEN, true);
 *   colorSensor.setChannelEnabled(ColorChannels::BLUE,  true);
 *
 *   // in updateControl() when the I2C timer fires:
 *   colorSensor.update();
 *   mappedGreen = autoGreenMap(colorSensor.getGreenFixed().asInt());
 */

#ifndef COLORSENSOR_H
#define COLORSENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include <FixMath.h>
#include <CLS16D24.h>
#include <Configuration.h>

// ---------------------------------------------------------------------------
// Supporting types — defined at file scope so main.cpp can reference them
// without qualifying with ColorSensor::
// ---------------------------------------------------------------------------

enum class ColorChannels : uint8_t
{
    RED   = 0,
    GREEN = 1,
    BLUE  = 2,
    CLEAR = 3,
    IR    = 4
};

struct ColorValues
{
    uint16_t red   = 0;
    uint16_t green = 0;
    uint16_t blue  = 0;
    uint16_t clear = 0;
    uint16_t IR    = 0;
};

struct FixedPointColorValues
{
    UFix<16, 0> redFixed   = 0;
    UFix<16, 0> greenFixed = 0;
    UFix<16, 0> blueFixed  = 0;
    UFix<16, 0> clearFixed = 0;
    UFix<16, 0> IRFixed    = 0;
};

// ---------------------------------------------------------------------------
// ColorSensor class
// ---------------------------------------------------------------------------

class ColorSensor
{
public:
    ColorSensor();

    /**
     * Initializes the I2C bus and the underlying CLS16D24 driver.
     * Call this before reset() / enable().
     *
     * @param i2cFastMode  Use 400 kHz fast mode when true (default: false here
     *                     because the caller in main.cpp sets the clock
     *                     manually after init).
     * @param wirePort     I2C interface to use.
     * @return true on success (currently always true; kept for API parity
     *         with CLS16D24::begin()).
     */
    bool begin(bool i2cFastMode = false, TwoWire &wirePort = Wire);

    void reset();
    void enable();

    /**
     * @param gain             One of: 1, 4, 8, 32, 96.
     * @param doubleSensorArea Doubles the photodiode sensing area when true.
     */
    void setGain(uint8_t gain, bool doubleSensorArea = true);

    /**
     * Sets both resolution and conversion time from a single byte.
     * See CLS16D24.h for the full table of valid values and their effects.
     */
    void setResolutionAndConversionTime(uint8_t time);

    // -----------------------------------------------------------------------
    // Core operation
    // -----------------------------------------------------------------------

    /**
     * Reads all five channels from hardware and immediately applies the
     * white-balance correction scaling, storing both raw and scaled results
     * internally. Call this whenever the I2C update timer fires.
     */
    void update();

    // -----------------------------------------------------------------------
    // Raw data getters (uint16_t, straight from hardware)
    // -----------------------------------------------------------------------

    uint16_t getRed()   const { return _raw.red;   }
    uint16_t getGreen() const { return _raw.green; }
    uint16_t getBlue()  const { return _raw.blue;  }
    uint16_t getClear() const { return _raw.clear; }
    uint16_t getIR()    const { return _raw.IR;    }

    /**
     * Direct access to the raw struct, useful if you need all five channels
     * at once without five separate function calls.
     */
    const ColorValues &getRawData() const { return _raw; }

    // -----------------------------------------------------------------------
    // White-balance-corrected fixed-point getters (UFix<16,0>)
    //
    // Scaling applied:
    //   red   *= 1.75  (approximated as (raw * 7) >> 2)
    //   green  = raw   (reference channel; most sensitive)
    //   blue  *= 2.4   (approximated as (raw * 157286) >> 16)
    //   clear  = raw   (unscaled)
    //   IR    *= 8     (raw << 3)
    // -----------------------------------------------------------------------

    UFix<16, 0> getRedFixed()   const { return _scaled.redFixed;   }
    UFix<16, 0> getGreenFixed() const { return _scaled.greenFixed; }
    UFix<16, 0> getBlueFixed()  const { return _scaled.blueFixed;  }
    UFix<16, 0> getClearFixed() const { return _scaled.clearFixed; }
    UFix<16, 0> getIRFixed()    const { return _scaled.IRFixed;    }

    /**
     * Direct access to the scaled struct, for callers that need all channels
     * or prefer struct-style access over individual getters.
     */
    const FixedPointColorValues &getScaledData() const { return _scaled; }

    // -----------------------------------------------------------------------
    // Sensor configuration queries (delegated to CLS16D24)
    // -----------------------------------------------------------------------

    float    getConversionTimeMillis();
    uint16_t getResolution()          ;

    // -----------------------------------------------------------------------
    // Debug printing
    // -----------------------------------------------------------------------

    /**
     * Enables or disables a channel for printColorData() output.
     * All channels are disabled by default.
     */
    void setChannelEnabled(ColorChannels ch, bool enabled);

    /**
     * Prints the mapped (post-AutoMap) values for channels whose printing
     * has been enabled via setChannelEnabled(). Clear and IR channels print
     * their scaled fixed-point values since no mapped equivalent exists here.
     *
     * The mapped values are passed in rather than stored here because they
     * are produced by AutoMap objects that live in updateControl().
     *
     * Respects the USE_SERIAL flag in Configuration.h; compiles to nothing
     * when serial output is disabled.
     */
    void printColorData(uint8_t mappedRed,   uint8_t mappedGreen,
                        uint8_t mappedBlue,  uint8_t mappedWhite) const;

private:
    CLS16D24              _sensor;
    ColorValues           _raw;
    FixedPointColorValues _scaled;
    bool                  _channelEnabled[5];

    /**
     * Applies white-balance multipliers to _raw and writes results into
     * _scaled. Called automatically by update().
     */
    void _applyScaling();
};

#endif // COLORSENSOR_H
