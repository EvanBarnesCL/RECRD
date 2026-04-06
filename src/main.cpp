#include <Arduino.h>
#include <Crunchlabs_DRV8835.h>
#include <Wire.h>
#include <PWMFreak.h>
#include <Mozzi.h>
#include <Oscil.h>
#include <IntMap.h>
#include <EventDelay.h>
#include <mozzi_utils.h>
#include <mozzi_midi.h>
#include <AS5600.h>
#include <AutoMap.h>

#include <Configuration.h>
#include <PinAssignments.h>


EventDelay k_i2cUpdateDelay;


// **********************************************************************************
// Arm and Table
// **********************************************************************************

#include <ArmManager.h>
#include <TableManager.h>

ArmManager arm(ARM_MOTOR_SPEED_PIN, ARM_MOTOR_DIR_PIN);
TableManager table(TABLE_MOTOR_SPEED_PIN, TABLE_MOTOR_DIR_PIN);


// **********************************************************************************
// Color Sensor
// **********************************************************************************

#include <ColorSensor.h>
ColorSensor colorSensor;

uint8_t mappedGreen = 0, mappedBlue = 0, mappedRed = 0, mappedWhite = 0;

constexpr uint8_t NUM_BRIGHTNESS_LEVELS = 5;
uint8_t brightnessIterator = 3;

#define USE_BRIGHTNESS_ARRAY 1

#if USE_BRIGHTNESS_ARRAY
const uint8_t PROGMEM LEDBrightnessLevels[NUM_BRIGHTNESS_LEVELS] = {0, 31, 63, 191, 255};
inline uint8_t getBrightness(uint8_t level = 3)
{
    return pgm_read_byte(&LEDBrightnessLevels[level]);
}
#else
constexpr uint16_t LEDBrightnessSequence = 0b1000011000100001;
inline uint8_t getBrightness(uint8_t level = 3)
{
    return static_cast<uint8_t>(
        (level == 0) ? 0 : ((((LEDBrightnessSequence >> (4 * (level - 1))) & 0xF) << 5) - 1)
    );
}
#endif


// **********************************************************************************
// Wavefolder lookup table
// **********************************************************************************

/**
 * @brief Transfer function for the wavefolder, stored in PROGMEM.
 *
 * The table encodes one complete period of a triangle wave, making it a
 * natural representation of a single fold. When the FM output is used to
 * index into this table, values that would exceed the output range are
 * "folded back" rather than clipped.
 *
 * Layout:
 *   indices   0–127: linear ramp from -128 to +126 (step +2, no folding)
 *   indices 128–255: linear ramp from +127 to -127 (step -2, fold region)
 *
 * The "no folding" half (0–127) is the baseline: at minimum white, the
 * drive factor constrains the FM signal to indices 0–125, passing through
 * the table linearly. As white increases, the signal starts reaching into
 * the fold region, folding harder with each increment. At the maximum drive
 * the signal wraps through the table approximately twice, producing a
 * spectrum with two additional sets of fold-generated harmonics layered
 * on top of the FM sidebands.
 *
 * Indexing is done with a uint8_t, so the table implicitly wraps at 256.
 * Multiple folds are handled for free by the natural overflow of the
 * uint8_t index arithmetic — no modulo operation required.
 *
 * pgm_read_byte() returns uint8_t; casting to int8_t reinterprets the
 * two's-complement bit pattern correctly for negative values.
 */
const int8_t PROGMEM waveFolder[256] = {
  -128, -126, -124, -122, -120, -118, -116, -114, -112, -110, -108, -106, -104, -102, -100,  -98,
   -96,  -94,  -92,  -90,  -88,  -86,  -84,  -82,  -80,  -78,  -76,  -74,  -72,  -70,  -68,  -66,
   -64,  -62,  -60,  -58,  -56,  -54,  -52,  -50,  -48,  -46,  -44,  -42,  -40,  -38,  -36,  -34,
   -32,  -30,  -28,  -26,  -24,  -22,  -20,  -18,  -16,  -14,  -12,  -10,   -8,   -6,   -4,   -2,
     0,    2,    4,    6,    8,   10,   12,   14,   16,   18,   20,   22,   24,   26,   28,   30,
    32,   34,   36,   38,   40,   42,   44,   46,   48,   50,   52,   54,   56,   58,   60,   62,
    64,   66,   68,   70,   72,   74,   76,   78,   80,   82,   84,   86,   88,   90,   92,   94,
    96,   98,  100,  102,  104,  106,  108,  110,  112,  114,  116,  118,  120,  122,  124,  126,
   127,  125,  123,  121,  119,  117,  115,  113,  111,  109,  107,  105,  103,  101,   99,   97,
    95,   93,   91,   89,   87,   85,   83,   81,   79,   77,   75,   73,   71,   69,   67,   65,
    63,   61,   59,   57,   55,   53,   51,   49,   47,   45,   43,   41,   39,   37,   35,   33,
    31,   29,   27,   25,   23,   21,   19,   17,   15,   13,   11,    9,    7,    5,    3,    1,
    -1,   -3,   -5,   -7,   -9,  -11,  -13,  -15,  -17,  -19,  -21,  -23,  -25,  -27,  -29,  -31,
   -33,  -35,  -37,  -39,  -41,  -43,  -45,  -47,  -49,  -51,  -53,  -55,  -57,  -59,  -61,  -63,
   -65,  -67,  -69,  -71,  -73,  -75,  -77,  -79,  -81,  -83,  -85,  -87,  -89,  -91,  -93,  -95,
   -97,  -99, -101, -103, -105, -107, -109, -111, -113, -115, -117, -119, -121, -123, -125, -127
};


// **********************************************************************************
// Music Generation Control
// **********************************************************************************

#include <MusicTypes.h>

const IntMap colorToScaleNote7(0, 256, 0, 7);
const IntMap colorToScaleNote5(0, 256, 0, 5);

#include <AnalogButtons.h>

uint8_t buttonPressed = 255;


// **********************************************************************************
// FM + Wavefolder State
// **********************************************************************************

/**
 * Peak phase deviation for the modulator, written at control rate by updateFM()
 * and consumed at audio rate by updateAudio(). Same atomic-access caveat as the
 * previous FM version — a single corrupted sample is inaudible.
 */
int32_t gDeviation = 0;

/**
 * Wavefold drive factor, written at control rate by updateFM() and consumed at
 * audio rate by updateAudio(). uint8_t is written atomically on AVR, so no
 * corruption risk for this one.
 *
 * Derivation: the FM output (int8_t) is shifted to unsigned (0-255), then
 * multiplied by gDrive and right-shifted 7 to produce the fold table index.
 * At gDrive = FOLD_DRIVE_MIN (63), the index stays within 0-125 (the linear
 * ramp half of the table) — transparent, no folding. At gDrive = 128, the
 * index traverses the full 256-entry table — one complete fold. At the maximum
 * of 255, the product (before the shift) reaches ~510, wrapping the uint8_t
 * index twice — approximately two folds.
 */
uint8_t gDrive = FOLD_DRIVE_MIN;

/**
 * Inharmonic mode: when false (default), C:M ratios are integers {1..8}.
 * When true, fractional ratios in [1.0, 4.0] produce metallic/bell spectra.
 * Toggled by button B2.
 */
bool inharmonicMode = false;

void updateFM(); // forward declaration


// **********************************************************************************
// Setup
// **********************************************************************************

void setup()
{
  SERIAL_BEGIN(115200);
  SERIAL_PRINTLN("starting");

  Wire.begin();

  table.begin();
  arm.begin();
  arm.home();

  colorSensor.begin(false);
  Wire.setClock(400000);

  colorSensor.reset();
  colorSensor.enable();
  colorSensor.setGain(32, false);
  colorSensor.setResolutionAndConversionTime(0x02);

  SERIAL_PRINT("Conversion time: ");
  SERIAL_PRINTLN(colorSensor.getConversionTimeMillis());
  SERIAL_PRINT("Resolution: ");
  SERIAL_PRINTLN(colorSensor.getResolution());

  colorSensor.setChannelEnabled(ColorChannels::RED,   true);
  colorSensor.setChannelEnabled(ColorChannels::GREEN, true);
  colorSensor.setChannelEnabled(ColorChannels::BLUE,  true);
  colorSensor.setChannelEnabled(ColorChannels::CLEAR, true);
  colorSensor.setChannelEnabled(ColorChannels::IR,    false);

  if (USE_FAST_PWM)
    setPwmFrequency(5, 1);

  if (USE_LED_PWM)
  {
    setPwmFrequency(LED_PIN, 1);
    analogWrite(LED_PIN, getBrightness(brightnessIterator));
  }
  else
  {
    digitalWrite(LED_PIN, HIGH);
  }

  float baseHz = mtof(scaleContainer.selected().getNote(0));
  aCarrier.setFreq(baseHz);
  aModulator.setFreq(baseHz * 2.0f);
  kModIndex.setFreq(0.5f);

  k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);

  startMozzi(MOZZI_CONTROL_RATE);
}


// **********************************************************************************
// updateControl
// **********************************************************************************

void updateControl()
{
  static int8_t targetArmPos = 80;
  static bool initialize = true, nextButtonPressAllowed = true;
  static EventDelay buttonTimer;
  if (initialize)
  {
    buttonTimer.set(250);
    initialize = false;
  }

  static AutoMap autoGreenToUINT8_T(0, colorSensor.getResolution(), 0, 255);
  static AutoMap autoBlueToUINT8_T(0,  colorSensor.getResolution(), 0, 255);
  static AutoMap autoRedToUINT8_T(0,   colorSensor.getResolution(), 0, 255);
  static AutoMap autoWhiteToUINT8_T(0, colorSensor.getResolution(), 0, 255);

  buttonPressed = getButtonPressed(mozziAnalogRead<10>(BUTTONS_PIN));
  if (nextButtonPressAllowed && buttonPressed < 255)
  {
    buttonTimer.start();
    nextButtonPressAllowed = false;
    switch (buttonPressed)
    {
    case 0:
      brightnessIterator = (brightnessIterator + 1) % NUM_BRIGHTNESS_LEVELS;
      analogWrite(LED_PIN, getBrightness(brightnessIterator));
      break;

    case 1:
      scaleContainer.nextScale();
      SERIAL_TABS(2);
      SERIAL_PRINT("scale: ");
      SERIAL_PRINTLN(scaleContainer.scaleSelector);
      break;

    case 2:
      inharmonicMode = !inharmonicMode;
      break;

    default:
      break;
    }
  }

  if (!nextButtonPressAllowed && buttonTimer.ready())
    nextButtonPressAllowed = true;

  table.updateTargetSpeed(mozziAnalogRead<10>(POT_B_PIN));

  static uint8_t currentSensorToUpdate = 0;

  if (k_i2cUpdateDelay.ready())
  {
    switch (currentSensorToUpdate)
    {
    case 0:
      arm.updatePosition();
      currentSensorToUpdate++;
      break;
    case 1:
      table.updateAngle();
      currentSensorToUpdate++;
      break;
    case 2:
      colorSensor.update();
      colorSensor.printColorData();
      currentSensorToUpdate = 0;
      break;
    default:
      currentSensorToUpdate = 0;
      break;
    }
    k_i2cUpdateDelay.start();
  }

  targetArmPos = arm.convertPotValToRadius(mozziAnalogRead<10>(POT_A_PIN));
  arm.moveToAngle(arm.radiusToAngle(targetArmPos));

  table.applySpeed();

  mappedGreen = autoGreenToUINT8_T(colorSensor.getGreenFixed().asInt());
  mappedBlue  = autoBlueToUINT8_T(colorSensor.getBlueFixed().asInt());
  mappedRed   = autoRedToUINT8_T(colorSensor.getRedFixed().asInt());
  mappedWhite = autoWhiteToUINT8_T(colorSensor.getClearFixed().asInt());

  updateFM();
}


// **********************************************************************************
// updateAudio
// **********************************************************************************

/**
 * @brief Mozzi audio-rate callback.
 *
 * Computes one FM sample, then runs it through the wavefolder.
 *
 * Folding arithmetic:
 *   1. fm_out is int8_t (-128 to 127). Adding 128 shifts it to uint8_t (0-255)
 *      so unsigned multiplication and indexing work correctly.
 *   2. Multiplying by gDrive (uint8_t) produces a uint16_t result up to 65025.
 *      Right-shifting 7 brings the maximum to ~508.
 *   3. Casting to uint8_t discards the high byte, which is equivalent to taking
 *      the result modulo 256. This wraps the driven signal through the triangle
 *      table implicitly, handling multiple folds without any branch or division.
 *   4. pgm_read_byte() returns uint8_t; casting to int8_t reinterprets the
 *      two's-complement stored value correctly for the negative entries.
 */
AudioOutput updateAudio()
{
  int32_t modulation = (gDeviation * aModulator.next()) >> 8;
  int8_t  fm_out     = aCarrier.phMod(modulation);

  uint8_t base_index = (uint8_t)((int16_t)fm_out + 128);
  uint8_t fold_index = (uint8_t)(((uint16_t)base_index * gDrive) >> 7);
  int8_t  folded     = (int8_t)pgm_read_byte(&waveFolder[fold_index]);

  return MonoOutput::from8Bit(folded);
}


// **********************************************************************************
// Loop
// **********************************************************************************

void loop()
{
  audioHook();
}


// **********************************************************************************
// updateFM
// **********************************************************************************

/**
 * @brief Derives FM synthesis parameters from color sensor readings.
 *        Called once per updateControl() cycle (MOZZI_CONTROL_RATE Hz).
 *
 * Green  → carrier pitch, quantized to the active scale.
 *
 * Blue   → C:M ratio. Harmonic mode: integer from {1,2,3,4,5,6,7,8}.
 *           Inharmonic mode: continuous [1.0, 4.0].
 *
 * Red    → modulation index. gDeviation scales with the modulator frequency so
 *           that the same red value produces equivalent harmonic density at all
 *           pitches (a standard technique in FM design to maintain consistent
 *           timbre across the keyboard range).
 *
 * White  → two coupled parameters:
 *             LFO rate: 0.05 Hz (white=0) to ~4.7 Hz (white=255), animating
 *             the FM modulation depth so the timbre breathes continuously.
 *
 *             Fold drive (gDrive): max(FOLD_DRIVE_MIN, FM_LFO_CHANNEL).
 *             Below FOLD_DRIVE_MIN the folder is transparent. Above it, drive
 *             increases linearly with white up to 255 (~2 folds at maximum).
 *             The coupling means dim scenes are clean and slow-moving; bright
 *             scenes are both faster-animated and harmonically richer from the
 *             folding. This feels natural when sweeping the sensor over surfaces
 *             that vary from dark to bright.
 */
void updateFM()
{
  // -----------------------------------------------------------------------
  // Carrier pitch — quantize green to the active scale.
  // -----------------------------------------------------------------------
  uint8_t noteIndex = (scaleContainer.selected().numNotes == 7)
      ? colorToScaleNote7(FM_CARRIER_CHANNEL)
      : colorToScaleNote5(FM_CARRIER_CHANNEL);

  float carrier_hz = mtof(scaleContainer.selected().getNote(noteIndex));

  // -----------------------------------------------------------------------
  // C:M ratio — blue channel.
  // -----------------------------------------------------------------------
  float cm_ratio;
  if (!inharmonicMode)
  {
    static const uint8_t harmonicRatios[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    cm_ratio = (float)harmonicRatios[FM_RATIO_CHANNEL >> 5];
  }
  else
  {
    cm_ratio = 1.0f + (FM_RATIO_CHANNEL / 85.0f);
  }

  aCarrier.setFreq(carrier_hz);
  aModulator.setFreq(carrier_hz * cm_ratio);

  // -----------------------------------------------------------------------
  // LFO rate — white channel.
  // -----------------------------------------------------------------------
  kModIndex.setFreq(0.05f + (FM_LFO_CHANNEL >> 3) * 0.15f);

  // -----------------------------------------------------------------------
  // Modulation depth — red channel, LFO-modulated.
  // -----------------------------------------------------------------------
  uint16_t mod_hz_int   = (uint16_t)(carrier_hz * cm_ratio);
  int32_t baseDeviation = (int32_t)mod_hz_int * (int32_t)(FM_INDEX_CHANNEL >> 2);
  int8_t  lfo           = kModIndex.next();
  int32_t lfoMod        = (baseDeviation * (int32_t)lfo) >> 8;
  gDeviation = baseDeviation + lfoMod;
  if (gDeviation < 0) gDeviation = 0;

  // -----------------------------------------------------------------------
  // Fold drive — white channel (coupled to LFO rate).
  // -----------------------------------------------------------------------
  gDrive = max((uint8_t)FOLD_DRIVE_MIN, (uint8_t)FM_LFO_CHANNEL);
}