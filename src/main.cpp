#include <Arduino.h>
#include <Configuration.h>
#include <Crunchlabs_DRV8835.h>
#include <Wire.h>
#include <PWMFreak.h>
#include <Mozzi.h>
#include <Oscil.h>
#include <tables/saw2048_int8.h>
#include <tables/triangle_dist_cubed_2048_int8.h>
#include <tables/triangle_valve_2_2048_int8.h>
#include <IntMap.h>
#include <EventDelay.h>
#include <mozzi_utils.h>
#include <mozzi_rand.h>
#include <mozzi_midi.h>
#include <AS5600.h>
#include <AutoMap.h>
#include <Portamento.h>
#include <ADSR.h>

/**
 * @brief Timer for staggering I2C sensor updates.
 *
 * Sensor updates block the processor, interfering with audio generation.
 * Staggering updates across three sensors at I2C_UPDATE_INTERVAL intervals
 * reduces audio glitching while maintaining acceptable sensor responsiveness.
 */
EventDelay k_i2cUpdateDelay;


// **********************************************************************************
// Arm and Table
// **********************************************************************************

// ArmManager must be included AFTER <Mozzi.h> because it uses EventDelay,
// which depends on Mozzi internals. Header-only by design for this reason.
#include <ArmManager.h>
#include <TableManager.h>
#include <AnalogButtons.h>

ArmManager arm;
TableManager table;

constexpr uint8_t LED_PIN = 11;


// **********************************************************************************
// Color Sensor
// **********************************************************************************

#include <ColorSensor.h>
ColorSensor colorSensor;

uint8_t mappedGreen = 0, mappedBlue = 0, mappedRed = 0, mappedWhite = 0;

constexpr uint8_t NUM_BRIGHTNESS_LEVELS = 5;
uint8_t brightnessIterator = 3;

/**
 * @brief Compile-time selection between array-based and bit-manipulation brightness lookup.
 *
 * Set to 1 for the array method (easier modification, minor RAM cost from PROGMEM table).
 * Set to 0 for the bit-manipulation method (zero RAM cost, equivalent functionality).
 * See Footnotes.md section [[#how LED brightness PWM values are calculated]] for derivation.
 */
#define USE_BRIGHTNESS_ARRAY 1

#if USE_BRIGHTNESS_ARRAY
const uint8_t PROGMEM LEDBrightnessLevels[NUM_BRIGHTNESS_LEVELS] = {0, 31, 63, 191, 255};

/**
 * @brief Retrieves LED brightness PWM value from PROGMEM lookup table.
 * @param level Brightness level index (0-4). Default is 3.
 * @return PWM duty cycle (0-255).
 */
inline uint8_t getBrightness(uint8_t level = 3)
{
    return pgm_read_byte(&LEDBrightnessLevels[level]);
}
#else
constexpr uint16_t LEDBrightnessSequence = 0b1000011000100001;

/**
 * @brief Computes LED brightness PWM value via bit extraction from packed constant.
 * @param level Brightness level index (0-4). Default is 3.
 * @return PWM duty cycle: 0 for level 0, otherwise ((nibble << 5) - 1).
 */
inline uint8_t getBrightness(uint8_t level = 3)
{
    return static_cast<uint8_t>(
        (level == 0) ? 0 : ((((LEDBrightnessSequence >> (4 * (level - 1))) & 0xF) << 5) - 1)
    );
}
#endif


// **********************************************************************************
// Potentiometers and Buttons
// **********************************************************************************

constexpr uint8_t POT_A_PIN    = A0,
                  POT_B_PIN    = A1,
                  BUTTONS_PIN  = A3;

uint8_t buttonPressed = 255;


// **********************************************************************************
// Mozzi Configuration
// **********************************************************************************

/**
 * @brief Control rate in Hz for updateControl() callback.
 *
 * Must be a power of 2. Higher values improve responsiveness to sensor input
 * but reduce processing time available for audio synthesis. Audio glitching
 * may indicate this value should be reduced.
 */
#define MOZZI_CONTROL_RATE 128

Oscil<TRIANGLE_DIST_CUBED_2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc0(TRIANGLE_DIST_CUBED_2048_DATA);
Oscil<SAW2048_NUM_CELLS,                  MOZZI_AUDIO_RATE> osc1(SAW2048_DATA);
Oscil<TRIANGLE_VALVE_2_2048_NUM_CELLS,    MOZZI_AUDIO_RATE> osc2(TRIANGLE_VALVE_2_2048_DATA);


// **********************************************************************************
// Music Generation Control
// **********************************************************************************

#include <MusicTypes.h>
#include <OscillatorTools.h>

oscillatorParams osc0Params, osc1Params, osc2Params;

Portamento<MOZZI_CONTROL_RATE> osc2Portamento;

ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> osc0AmpEnv, osc1AmpEnv, osc2AmpEnv;

EventDelay arpNoteTimer, arpTimeout;
EventDelay osc0ButtonMode2NoteTimer, osc1ButtonMode2NoteTimer, osc2ButtonMode2NoteTimer;

const IntMap colorToScaleNote7(0, 256, 0, 7);
const IntMap colorToScaleNote5(0, 256, 0, 5);

bool enableButton2Mode = false, previousEnableButton2Mode = false;


// **********************************************************************************
// Musical Scales
// **********************************************************************************

constexpr uint8_t NUM_SCALES = 4;

#include <MusicTools.h>

DEFINE_CHORD(scale_CPentatonicMajor, N("C3"), N("D3"), N("E3"), N("G3"), N("A3"));
DEFINE_CHORD(scale_CHarmonicMajor, N("C3"), N("D3"), N("E3"), N("F3"), N("G3"), N("G#3"), N("B3"));
DEFINE_CHORD(scale_EbPentatonicMinorMIDI, N("D#3"), N("F#3"), N("G#3"), N("A#3"), N("C#4"));

constexpr MIDI_NOTE root_CLydianScale = 48;
DEFINE_CHORD(scale_CLydian, root_CLydianScale, root_CLydianScale + 2, root_CLydianScale + 4,
             root_CLydianScale + 6, root_CLydianScale + 7, root_CLydianScale + 9, root_CLydianScale + 11);

ScaleStorage scaleContainer = {
    {&scale_EbPentatonicMinorMIDI, &scale_CPentatonicMajor, &scale_CHarmonicMajor, &scale_CLydian},
    0
};

Chord currentScale = scaleContainer.selected();

void ambienceGenerator();


// **********************************************************************************
// Setup
// **********************************************************************************

/**
 * @brief Initializes hardware peripherals, sensors, and Mozzi audio engine.
 *
 * Sequence matters: I2C sensors require initialization before clock speed changes,
 * and arm homing uses delay() which becomes unreliable after PWM frequency modification.
 */
void setup()
{
  randSeed(analogRead(A7));

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
  {
    setPwmFrequency(5, 1);
  }

  if (USE_LED_PWM)
  {
    setPwmFrequency(11, 1);
    analogWrite(LED_PIN, getBrightness(brightnessIterator));
  }
  else
  {
    digitalWrite(LED_PIN, HIGH);
  }

  osc0.setFreq(mtof(noteNameToMIDINote("E2")));
  osc1.setFreq(mtof(noteNameToMIDINote("A3")));
  osc2.setFreq(mtof(noteNameToMIDINote("B4")));

  k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);

  osc0AmpEnv.setADLevels(160, 140);
  osc1AmpEnv.setADLevels(80, 60);
  osc2AmpEnv.setADLevels(160, 140);

  startMozzi(MOZZI_CONTROL_RATE);
}


// **********************************************************************************
// updateControl
// **********************************************************************************

/**
 * @brief Mozzi control-rate callback for sensor polling and state updates.
 *
 * Called at MOZZI_CONTROL_RATE Hz. Handles button debouncing, sensor updates,
 * arm/table positioning, and invokes the ambience generator. I2C reads are
 * staggered to minimize interference with audio synthesis.
 */
void updateControl()
{
  static int8_t targetArmPos = 80;
  static bool initialize = true, nextButtonPressAllowed = true;
  static EventDelay buttonTimer;
  if (initialize)
  {
    buttonTimer.set(250);
    arpTimeout.set(4000);
    arpTimeout.start();
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
      currentScale = scaleContainer.selected();
      SERIAL_TABS(2);
      SERIAL_PRINT("scale: ");
      SERIAL_PRINTLN(scaleContainer.scaleSelector);
      break;

    case 2:
      enableButton2Mode = !enableButton2Mode;
      break;

    default:
      break;
    }
  }

  if (!nextButtonPressAllowed && buttonTimer.ready())
  {
    nextButtonPressAllowed = true;
  }

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

  ambienceGenerator();

  previousEnableButton2Mode = enableButton2Mode;
}


// **********************************************************************************
// updateAudio
// **********************************************************************************

/**
 * @brief Mozzi audio-rate callback for sample generation.
 *
 * Called at audio rate (typically 16384 or 32768 Hz). Mixes the three oscillators
 * with their respective volume envelopes and returns a 17-bit scaled output.
 * @return MonoOutput containing the mixed audio sample.
 */
AudioOutput updateAudio()
{
  int32_t asig = (int32_t)
    osc0.next() * osc0Params.volume +
    osc1.next() * osc1Params.volume +
    osc2.next() * osc2Params.volume;

  return MonoOutput::fromAlmostNBit(17, asig);
}


// **********************************************************************************
// Loop
// **********************************************************************************

/**
 * @brief Main Arduino loop—delegates to Mozzi's audio hook.
 *
 * All control logic resides in updateControl(); audio synthesis occurs in
 * updateAudio(). This function merely services the Mozzi framework.
 */
void loop()
{
  audioHook();
}


// **********************************************************************************
// ambienceGenerator
// **********************************************************************************

/**
 * @brief Generates musical output from color sensor data and mode state.
 *
 * Two primary modes of operation:
 * - Normal mode: Sustained drones with notes selected from scales based on
 *   color channel values. Occasional arpeggios triggered by white channel intensity.
 * - Button mode 2: Stochastic arpeggiation with probability of note triggers
 *   modulated by color channel values.
 *
 * ADSR envelopes are reconfigured when transitioning between modes.
 */
void ambienceGenerator()
{
  static int8_t arpIndex = 0;
  static uint8_t numNotesLeftInArp = 0;
  static bool initialize = true;

  static uint16_t osc2PortTime = 20;

  const bool USE_PORTAMENTO = true;

  static uint16_t attack = 100, decay = 500, sustain = 8000, release = 3000;

  int8_t octaveShifter = (int8_t)(mappedWhite >> 6);  // mappedWhite is 8 bit, this cuts it down to 2 (range 0-3).
  static bool arpeggiate = false, arpStarted = false, arpOnTimeOut = true;

  static uint8_t baseNoteInterval = 64; // baseline amount of time that an arp note will be turned on for, gets modified by sensor readings

  if (!enableButton2Mode && previousEnableButton2Mode)
    initialize = true;

  if (initialize)
  {
    osc0AmpEnv.setADLevels(160, 140);
    osc1AmpEnv.setADLevels(60, 50);
    osc2AmpEnv.setADLevels(120, 110);
    osc0AmpEnv.setTimes(attack, decay, sustain, release);
    osc1AmpEnv.setTimes(attack, decay, sustain, release);
    osc2AmpEnv.setTimes(attack, decay, sustain, release);
    initialize = false;
  }

  if (enableButton2Mode)
  {
    if (!previousEnableButton2Mode)
    {
      osc0AmpEnv.setTimes(5, 50, 100, 200);
      osc1AmpEnv.setTimes(5, 50, 100, 200);
      osc1AmpEnv.setADLevels(60, 40);
      osc2AmpEnv.setTimes(5, 50, 100, 200);

      osc0ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval);
      osc0ButtonMode2NoteTimer.start();
      osc1ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval * 2);
      osc1ButtonMode2NoteTimer.start();
      osc2ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval);
      osc2ButtonMode2NoteTimer.start();
    }

    baseNoteInterval = max(1, (mappedWhite >> 5)) * 16;

    if (osc0ButtonMode2NoteTimer.ready())
    {
      if (rand(256) < mappedRed)
      {
        switch (scaleContainer.selected().numNotes)
        {
          case 7:
            osc0Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote7(mappedGreen)) + ((int8_t)rand(-1, 2) * 12);
            break;
          case 5:
            osc0Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote5(mappedGreen)) + ((int8_t)rand(-1, 2) * 12);
            break;
          default:
            break;
        }
      }
      osc0AmpEnv.noteOn();
      osc0Params.frequency = mtof(osc0Params.noteMIDINumber);
      osc0.setFreq(osc0Params.frequency);
      osc0ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval);
      osc0ButtonMode2NoteTimer.start();
    }

    if (osc1ButtonMode2NoteTimer.ready())
    {
      if (rand(256) < mappedGreen)
      {
        switch (scaleContainer.selected().numNotes)
        {
          case 7:
            osc1Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote7(mappedGreen)) + ((int8_t)rand(-1, 2) * 12);
            break;
          case 5:
            osc1Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote5(mappedGreen)) + ((int8_t)rand(-1, 2) * 12);
            break;
          default:
            break;
        }
      }
      osc1AmpEnv.noteOn();
      osc1Params.frequency = mtof(osc1Params.noteMIDINumber);
      osc1.setFreq(osc1Params.frequency);
      osc1ButtonMode2NoteTimer.set(max(1, mappedBlue >> 4) * baseNoteInterval * 2);
      osc1ButtonMode2NoteTimer.start();
    }

    if (osc2ButtonMode2NoteTimer.ready())
    {
      if (rand(256) < mappedBlue)
      {
        switch (scaleContainer.selected().numNotes)
        {
          case 7:
            osc2Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote7(mappedGreen)) + ((int8_t)rand(-1, 2) * 12);
            break;
          case 5:
            osc2Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote5(mappedGreen)) + ((int8_t)rand(-1, 2) * 12);
            break;
          default:
            break;
        }
      }
      osc2AmpEnv.noteOn();
      osc2Params.frequency = mtof(osc2Params.noteMIDINumber);
      osc2.setFreq(osc2Params.frequency);
      osc2ButtonMode2NoteTimer.set(max(1, mappedRed >> 4) * baseNoteInterval);
      osc2ButtonMode2NoteTimer.start();
    }

    osc0AmpEnv.update();
    osc1AmpEnv.update();
    osc2AmpEnv.update();
    osc0Params.volume = osc0AmpEnv.next();
    osc1Params.volume = osc1AmpEnv.next();
    osc2Params.volume = osc2AmpEnv.next();
  }

  else
  {
    // this constrains the amount that the octave can be shifted up or down to either -1, 0, or +1 octaves
    octaveShifter = max(-1, octaveShifter - 2);
    // this makes it so that there is a minimum amount of time between arpeggios
    if (arpTimeout.ready())
    {
      arpOnTimeOut = false;
    }

    // if an arpeggio is allowed to start, use a bit of randomness to determine if one actually will start.
    // the more white light there is, the more likely the arpeggio is.
    if (octaveShifter > 0 && !arpOnTimeOut && !arpStarted)
    {
      arpeggiate = (rand(256) <= mappedWhite) ? true : false; // could use something like rand(128) to force more arpeggios
    }

    // create an index to pick a note out of the scale
    uint8_t i = 0;
    if (scaleContainer.selected().numNotes == 7)
    {
      i = colorToScaleNote7(mappedGreen);   // this is an IntMap
    }
    else
    {
      i = colorToScaleNote5(mappedGreen);   // IntMap
    }

    // get the actual note from the scale
    osc0Params.noteMIDINumber = scaleContainer.selected().getNote(i);
    // if the note has changed, restart the note playback
    if (osc0Params.lastNoteMIDINumber != osc0Params.noteMIDINumber)
    {
      osc0AmpEnv.noteOn();
      osc0Params.lastNoteMIDINumber = osc0Params.noteMIDINumber;
    }
    // calculate the frequency of the oscillator
    osc0Params.frequency = mtof(osc0Params.noteMIDINumber);
    // set the oscillator to the calculated frequency
    osc0.setFreq((osc0Params.frequency));
    // update the amp envelope (restarts with a a new noteOn() call)
    osc0AmpEnv.update();
    osc0Params.volume = osc0AmpEnv.next();

    // 
    uint8_t j = 0;
    if (scaleContainer.selected().numNotes == 7)
    {
      j = colorToScaleNote7(mappedBlue);
    }
    else
    {
      j = colorToScaleNote5(mappedBlue);
    }

    switch (scaleContainer.scaleSelector)
    {
    case 0:
      osc1Params.noteMIDINumber = scaleContainer.selected().getNote((j + 4) % scaleContainer.selected().numNotes) - 12;
      break;
    case 1:
      osc1Params.noteMIDINumber = scaleContainer.selected().getNote((j + 2) % scaleContainer.selected().numNotes) + octaveShifter * 12;
      break;
    case 2:
      osc1Params.noteMIDINumber = scaleContainer.selected().getNote((j)) - 12;
      break;
    default:
      break;
    }

    if (osc1Params.lastNoteMIDINumber != osc1Params.noteMIDINumber)
    {
      osc1AmpEnv.noteOn();
      osc1Params.lastNoteMIDINumber = osc1Params.noteMIDINumber;
    }
    osc1Params.frequency = mtof(osc1Params.noteMIDINumber);
    osc1.setFreq(osc1Params.frequency);
    osc1AmpEnv.update();
    osc1Params.volume = osc1AmpEnv.next();

    if (!arpeggiate)
    {
      osc2Params.noteMIDINumber = scaleContainer.selected().getNote((i + 3) % scaleContainer.selected().numNotes) + (octaveShifter * 12);
      if (osc2Params.lastNoteMIDINumber != osc2Params.noteMIDINumber)
      {
        osc2AmpEnv.noteOn();
        osc2Params.lastNoteMIDINumber = osc2Params.noteMIDINumber;
      }
      osc2Params.frequency = mtof(osc2Params.noteMIDINumber);
      osc2.setFreq(osc2Params.frequency);
      osc2AmpEnv.update();
      osc2Params.volume = osc2AmpEnv.next();
    }
    else
    {
      if (!arpStarted)
      {
        numNotesLeftInArp = rand(4, 17);
        arpStarted = true;
        arpNoteTimer.set(min(mappedBlue, 192));
        arpNoteTimer.start();
        arpIndex = rand(scaleContainer.selected().numNotes);
        osc2AmpEnv.setTimes(5, 5, 100, 100);
      }

      if (arpNoteTimer.ready())
      {
        osc2Params.noteMIDINumber = scaleContainer.selected().getNote(arpIndex) + (12 * (int8_t)rand(-1, 3));
        osc2Params.frequency = mtof(osc2Params.noteMIDINumber);
        osc2Params.volume = 120;

        osc2Portamento.setTime(osc2PortTime);
        if (!USE_PORTAMENTO)
          osc2.setFreq(osc2Params.frequency);

        int8_t arpShift = rand(-5, 6);
        arpIndex += arpShift;
        arpIndex = (arpIndex < scaleContainer.selected().numNotes && arpIndex >= 0)
            ? arpIndex
            : ((arpIndex < 0)
                ? arpIndex += scaleContainer.selected().numNotes
                : arpIndex -= scaleContainer.selected().numNotes);

        arpNoteTimer.start();
        numNotesLeftInArp -= 1;
      }

      if (USE_PORTAMENTO)
      {
        osc2Portamento.start(osc2Params.noteMIDINumber);
        osc2.setFreq_Q16n16(osc2Portamento.next());
      }

      if (numNotesLeftInArp == 0)
      {
        arpeggiate = false;
        arpStarted = false;
        arpTimeout.set(mappedGreen << 4);
        arpTimeout.start();
        arpOnTimeOut = true;
        osc2AmpEnv.setTimes(attack, decay, sustain, release);
      }
    }
  }
}
