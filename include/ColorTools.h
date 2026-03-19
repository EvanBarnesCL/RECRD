#ifndef COLORTOOLS_H
#define COLORTOOLS_H
#include <Arduino.h>
#include <Configuration.h>
#include <FixMath.h>
#include <EventDelay.h>

bool updateChannels[5] = {false, false, false, false, false}; // Flags to control which channels to update. defaults to all five off.

// this enum class provides a clear way to explicitly reference a color channel (instead of using an integer or something)
enum class ColorChannels
{
  RED,
  GREEN,
  BLUE,
  CLEAR,
  IR
};

// struct for storing the raw color value readings from the sensor as uint16_t.
// be aware that if you change the resolution of the sensor to exceed 16 bits, you're going to have a problem
// with overflow here.
struct ColorValues
{
  uint16_t red = 0;
  uint16_t green = 0;
  uint16_t blue = 0;
  uint16_t clear = 0;
  uint16_t IR = 0;
};

ColorValues colorData; // struct for the raw color data

// this stores color data as unsigned 16 bit integers in the fixed point format that Mozzi and FixMath use.
struct FixedPointColorValues
{
  UFix<16, 0> redFixed = 0;
  UFix<16, 0> greenFixed = 0;
  UFix<16, 0> blueFixed = 0;
  UFix<16, 0> clearFixed = 0;
  UFix<16, 0> IRFixed = 0;
};

// struct for storing the color data as fixed point values after they have been white balance corrected (after scaleColorDataFixedPoint() is applied to the raw colorData)
FixedPointColorValues scaledFixedColorData;

// where color data that has been mapped into control signals will be stored
uint8_t mappedGreen = 0, mappedBlue = 0, mappedRed = 0, mappedWhite = 0;

// delay timer for updating sensor data. When the sensor data is getting updated, the processor can't simultaneously update the sound it's generating.
// You can update the sensor more frequently, but that might negatively impact sound generation. This also updates all three sensors simultaneously,
// and it might work better to update each of them at staggered intervals.
EventDelay k_i2cUpdateDelay;
constexpr uint8_t I2C_UPDATE_INTERVAL = 15; // time in milliseconds

// constants and variables relating to the LEDs on the arm that illuminate the table
constexpr uint8_t NUM_BRIGHTNESS_LEVELS = 5;
uint8_t brightnessIterator = 3; // default to brightness level 3 on startup
constexpr uint16_t LEDBrightnessSequence = 0b1000011000100001;

// This function generates the sequence 0, 31, 63, 191, 255 to correspond to 5 PWM values for LED dimming. To
// understand how it works, see the section [[#how LED brightness PWM values are calculated]] in Footnotes.md
inline uint8_t getBrightness(uint8_t level = 3)
{
    return static_cast<uint8_t>(
        (level == 0) ? 0 : ((((LEDBrightnessSequence >> (4 * (level - 1))) & 0xF) << 5) - 1)
    );
}


// function prototypes
bool updateColorReadings(ColorValues *colorReadings);
void printColorData();
void scaleColorData(ColorValues *rawData);                                              // used to scale the RGB values relative to each other a bit
void scaleColorDataFixedPoint(ColorValues *rawData, FixedPointColorValues *scaledVals); // scales raw sensor values and returns them as fixed point math values instead of uint16_t









bool updateColorReadings(ColorValues *colorReadings)
{
  RGBCIR.readRGBWIR(colorReadings->red, colorReadings->green, colorReadings->blue, colorReadings->clear, colorReadings->IR);
  return true;
}

void printColorData()
{
  bool first = true; // To manage commas between printed values
  struct updatingColors
  {
    bool r = false;
    bool g = false;
    bool b = false;
    bool c = false;
    bool ir = false;
  };

  updatingColors channels;

  if (updateChannels[static_cast<int>(ColorChannels::BLUE)])
  {
    if (!first)
      SERIAL_PRINT("   ");
    else
      first = false;
    // SERIAL_PRINT("Blue:");
    channels.b = true;
    // SERIAL_PRINT(colorData.blue);
    // SERIAL_PRINT(scaledFixedColorData.blueFixed.asInt());
    SERIAL_PRINT(mappedBlue);
  }

  if (updateChannels[static_cast<int>(ColorChannels::CLEAR)])
  {
    if (!first)
      SERIAL_PRINT("   ");
    else
      first = false;
    // SERIAL_PRINT("Clear:");
    channels.c = true;
    // SERIAL_PRINT(colorData.clear);
    SERIAL_PRINT(scaledFixedColorData.clearFixed.asInt());
  }

  if (updateChannels[static_cast<int>(ColorChannels::IR)])
  {
    if (!first)
      SERIAL_PRINT("   ");
    else
      first = false;
    // SERIAL_PRINT("IR:");
    channels.ir = true;
    // SERIAL_PRINT(colorData.IR);
    SERIAL_PRINT(scaledFixedColorData.IRFixed.asInt());
  }

  if (updateChannels[static_cast<int>(ColorChannels::GREEN)])
  {
    if (!first)
      SERIAL_PRINT("   ");
    else
      first = false;
    // SERIAL_PRINT("Green:");
    channels.g = true;
    // SERIAL_PRINT(colorData.green);
    // SERIAL_PRINT(scaledFixedColorData.greenFixed.asInt());
    SERIAL_PRINT(mappedGreen);
  }
  if (updateChannels[static_cast<int>(ColorChannels::RED)])
  {
    if (!first)
      SERIAL_PRINT("   ");
    else
      first = false;
    // SERIAL_PRINT("Red:");
    channels.r = true;
    // SERIAL_PRINT(colorData.red);
    // SERIAL_PRINT(scaledFixedColorData.redFixed.asInt());
    SERIAL_PRINT(mappedRed);
  }

  // SERIAL_PRINT("  "); SERIAL_PRINT(v0);

  SERIAL_PRINTLN();
}




/**
 * used to scale RGB color data relative to each other. I did some testing with a Spyder Checkr 24 color balance
 * checking card used by photographers to figure out these values. Specifically, there is a row of 6 squares that
 * fade from pure white to pure black over several steps of grey. I put each of these under the color sensor, and
 * then looked for multipliers for the blue and red channels that would bring them up to the same level as the green
 * channel, which is generally the most sensitive and shows the strongest response. Using these scaled values has made
 * more intuitive sense to me when comparing what the sensor is looking at with what the values it reports. So now,
 * generally, when the sensor is over a strong red, the red channel with report the largest value; same for green and blue.
 * Before, the green channel was often still reporting higher values than the others, even over a strong blue color.
 * If you look in the datasheet for the sensor, you can see the response curves, and the green channel is just more sensitive.
 */
void scaleColorData(ColorValues *rawData)
{
  rawData->red = (rawData->red * 7) >> 2;                     // this is the same as multiplying the red channel by 1.75, but done with faster math operations
  rawData->blue = ((uint32_t)rawData->blue * 157286UL) >> 16; // this closely approximates multiplying blue by 2.4 (2.4 is 12/5, and you can approximate 1/5 with 51 >> 8)
  rawData->IR = rawData->IR << 3;
}

void scaleColorDataFixedPoint(ColorValues *rawData, FixedPointColorValues *scaledVals)
{
  scaledVals->redFixed = UFix<16, 0>((rawData->red * 7) >> 2);
  scaledVals->greenFixed = UFix<16, 0>(rawData->green);
  scaledVals->blueFixed = UFix<16, 0>(((uint32_t)rawData->blue * 157286UL) >> 16);
  scaledVals->clearFixed = UFix<16, 0>(rawData->clear);
  scaledVals->IRFixed = UFix<16, 0>(rawData->IR << 3);
}




#endif