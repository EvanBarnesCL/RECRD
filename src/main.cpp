#include <Arduino.h>
#include <Configuration.h>
#include <Crunchlabs_DRV8835.h>  // motor driver
#include <Wire.h>                // I2C
#include <PWMFreak.h>            // Changes clock rate of timers (used to remove audible noise from motor driver and LED dimming)
#include <Mozzi.h>               // Main synthesizer library
#include <Oscil.h>               // Oscillator
#include <tables/saw2048_int8.h>
#include <tables/triangle_dist_cubed_2048_int8.h>
#include <tables/triangle_valve_2_2048_int8.h>
#include <IntMap.h>     // A more efficient replacement for map()
#include <EventDelay.h> // Mozzi library for performing actions at specific time intervals without delay() or millis()
#include <mozzi_utils.h>
#include <mozzi_rand.h>
#include <mozzi_midi.h> // MIDI functionality, used for mtof(), which converts MIDI note numbers to frequencies
#include <AS5600.h>     // Magnetic encoders for the arm and table positions
#include <AutoMap.h>    // A version of map() that is auto-ranging. Used for mapping color values to control signals.
#include <Portamento.h>
#include <ADSR.h>

// delay timer for updating sensor data. When the sensor data is getting updated, the processor can't simultaneously update the sound it's generating.
// You can update the sensor more frequently, but that might negatively impact sound generation. This also updates all three sensors simultaneously,
// and it might work better to update each of them at staggered intervals.
EventDelay k_i2cUpdateDelay;


// **********************************************************************************
// Arm and Table
// **********************************************************************************

// ArmManager must be included AFTER <Mozzi.h> because it uses EventDelay, which
// depends on Mozzi internals. ArmManager is header-only for exactly this reason.
#include <ArmManager.h>     // data and functions that manage the sensor arm
#include <TableManager.h>   // data and functions that manage the table
#include <AnalogButtons.h>  // data and functions that manage the three pushbuttons

ArmManager arm;
TableManager table;

// define the pin that the LED PWM control happens on. 
constexpr uint8_t LED_PIN = 11;


// **********************************************************************************
// Color sensor stuff
// **********************************************************************************

#include <ColorSensor.h>
ColorSensor colorSensor;

// where color data that has been mapped into control signals will be stored
uint8_t mappedGreen = 0, mappedBlue = 0, mappedRed = 0, mappedWhite = 0;

// constants and variables relating to the LEDs on the arm that illuminate the table
constexpr uint8_t NUM_BRIGHTNESS_LEVELS = 5;
uint8_t brightnessIterator = 3; // default to brightness level 3 on startup

/**
 * Set this to 1 to use an array to define brightness levels for the LED on the color sensor that illuminates the table.
 * The array is easier to modify, but costs a bit more RAM. Set this value to 0 to use the fancy bit manipulation method
 * instead. Both work equally well. Originally I made the bit manipulation version to replace the array version to save
 * RAM. This program was using nearly every bit of RAM available, so even though the bit manipulation method only saved
 * 6 bytes of RAM, that was worth it. To understand how it works, see the section
 * [[#how LED brightness PWM values are calculated]] in Footnotes.md
 */
#define USE_BRIGHTNESS_ARRAY 1

#if USE_BRIGHTNESS_ARRAY
  // Brightness levels stored in PROGMEM to avoid consuming SRAM. pgm_read_byte()
  // is required to fetch values from program memory on AVR — you can't dereference
  // a PROGMEM pointer directly.
  const uint8_t PROGMEM LEDBrightnessLevels[NUM_BRIGHTNESS_LEVELS] = {0, 31, 63, 191, 255};
  inline uint8_t getBrightness(uint8_t level = 3)
  {
      return pgm_read_byte(&LEDBrightnessLevels[level]);
  }
#else
  // Each nibble in LEDBrightnessSequence encodes one brightness level as n,
  // where the actual PWM value is (n << 5) - 1. Level 0 is special-cased to 0
  // since (0 << 5) - 1 would underflow. The four nibbles pack levels 1-4
  // into a single uint16_t constant with zero RAM cost.
  constexpr uint16_t LEDBrightnessSequence = 0b1000011000100001;
  inline uint8_t getBrightness(uint8_t level = 3)
  {
      return static_cast<uint8_t>(
          (level == 0) ? 0 : ((((LEDBrightnessSequence >> (4 * (level - 1))) & 0xF) << 5) - 1)
      );
  }
#endif


// **********************************************************************************
// Potentiometers and buttons
// **********************************************************************************

constexpr uint8_t POT_A_PIN    = A0,
                  POT_B_PIN    = A1,
                  AUDIO_IN_PIN = A2,
                  BUTTONS_PIN  = A3; // all 3 buttons are connected to the same analog pin via a resistor ladder

uint8_t buttonPressed = 255; // 255 = no button pressed, 0/1/2 = which button


// **********************************************************************************
// Mozzi stuff
// **********************************************************************************

// this sets the frequency that the updateControl() function gets called. Currently this sets it to be called
// at 128 times per second. Setting it faster (e.g., to 256) would improve the response speed of the program
// to changes in sensor data or to user input, but the tradeoff is that doing so reduces processing time 
// available for making sound. If you hear glitchiness in the audio you're making, try setting this to a lower
// value to create less interruptions for the sound synthesizer. Note that this number must always be a power of 2.
#define MOZZI_CONTROL_RATE 128

Oscil<TRIANGLE_DIST_CUBED_2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc0(TRIANGLE_DIST_CUBED_2048_DATA);
Oscil<SAW2048_NUM_CELLS,                  MOZZI_AUDIO_RATE> osc1(SAW2048_DATA);
Oscil<TRIANGLE_VALVE_2_2048_NUM_CELLS,    MOZZI_AUDIO_RATE> osc2(TRIANGLE_VALVE_2_2048_DATA);


// **********************************************************************************
// Music Generation Control Stuff
// **********************************************************************************

// Include the MusicTypes header, which defines MIDI_NOTE and Chord. 
#include <MusicTypes.h>
// Contains tools useful for converting music information into oscillator parameters and things
#include <OscillatorTools.h>

// create structs for storing parameters for each oscillator
oscillatorParams osc0Params, osc1Params, osc2Params;

// create portamento objects so we can glide from one note to another
Portamento<MOZZI_CONTROL_RATE> osc0Portamento, osc1Portamento, osc2Portamento;

// create ADSR envelopes to make each note fade in and out.
ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> osc0AmpEnv, osc1AmpEnv, osc2AmpEnv;

// Mozzi EventDelay timers, used instead of millis() for timing
EventDelay chordTimer, arpDurationTimer, arpNoteTimer, arpTimeout;
EventDelay osc0ButtonMode2NoteTimer, osc1ButtonMode2NoteTimer, osc2ButtonMode2NoteTimer;

// IntMaps for squashing scaled color data down to fit a musical scale.
const IntMap colorToScaleNote7(0, 256, 0, 7); // for 7-note scales
const IntMap colorToScaleNote5(0, 256, 0, 5); // for 5-note scales

// variables related to the buttons
bool enableButton2Mode = false, previousEnableButton2Mode = false;
uint8_t lastButtonMode = 0, currentButtonMode = 0;



// **********************************************************************************
// Musical Chords, Scales, and Sound Generators
// **********************************************************************************

// First, define the number of scales we're going to use
constexpr uint8_t NUM_SCALES = 4;

// Then include the MusicTools header, which includes functions that are musically useful for composition and things.
#include <MusicTools.h>

// This is a macro that makes it easy to define a new Chord object. Chords in the context of this program are just collections
// of note data. They can be up to 256 notes, and you can use them as chords, scales, arpeggios, melodies, or however you want.
// See MusicTools.h to see how this macro is constructed.
DEFINE_CHORD(scale_CPentatonicMajor, N("C3"), N("D3"), N("E3"), N("G3"), N("A3"));

DEFINE_CHORD(scale_CHarmonicMajor, N("C3"), N("D3"), N("E3"), N("F3"), N("G3"), N("G#3"), N("B3"));

DEFINE_CHORD(scale_EbPentatonicMinorMIDI, N("D#3"), N("F#3"), N("G#3"), N("A#3"), N("C#4"));

// You can also define chords as a collection of MIDI note numbers directly, instead of using the 'N("D#3")' notation from above,
// which converts note names to MIDI note numbers for you.

constexpr MIDI_NOTE root_CLydianScale = 48; // MIDI note number for C3
DEFINE_CHORD(scale_CLydian, root_CLydianScale, root_CLydianScale + 2, root_CLydianScale + 4, root_CLydianScale + 6, root_CLydianScale + 7, root_CLydianScale + 9, root_CLydianScale + 11);

// Put all of your scales into a single container so that we can iterate through them with one of the front panel buttons
ScaleStorage scaleContainer = {
    {&scale_CPentatonicMajor, &scale_CHarmonicMajor, &scale_EbPentatonicMinorMIDI, &scale_CLydian},   // this is the array of scales
    0                                                                                                 // this is the index of the currently selected scale
};

// set up a Chord object with the data for the first selected chord from the container.
Chord currentScale = scaleContainer.selected();

// function prototype for the function that will actually generate sounds from color data
void ambienceGenerator();


// **********************************************************************************
// Setup
// **********************************************************************************

void setup()
{
  randSeed(analogRead(A7));

  SERIAL_BEGIN(115200);
  SERIAL_PRINTLN("starting");

  // start I2C
  Wire.begin();

  // Initialize the table encoder and arm encoder, then home the arm.
  // home() must be called before Mozzi starts and before PWM clock divisors
  // are changed, because it uses standard delay(). table.begin() resets the
  // table encoder to 0 at the current position.
  table.begin();
  arm.begin();
  arm.home();

  // start the color sensor I2C connection
  colorSensor.begin(false);

  // IMPORTANT: Set the I2C clock to 400kHz fast mode AFTER initializing the connection to the sensors.
  // Need to use fastest I2C possible to minimize latency for Mozzi.
  Wire.setClock(400000);

  colorSensor.reset();
  colorSensor.enable();

  // possible gain settings are 1, 4, 8, 32, 96. setting the second parameter to true doubles the diode sensing area.
  colorSensor.setGain(32, false);

  // See the end of CLS16D24.h for a complete table of possible values here:
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

  // Change the PWM frequency on the motor control pins to push it above audible range.
  // After this line, millis() and delay() are unreliable. Use Mozzi EventDelay instead.
  if (USE_FAST_PWM)
  {
    setPwmFrequency(5, 1); // sets Timer0 clock divisor to 1 instead of 64
  }

  // EXPERIMENTAL: PWM dimming for the color sensor LEDs. Changing Timer 2's clock divisor
  // could affect Mozzi functions, but hasn't caused problems in testing so far.
  if (USE_LED_PWM)
  {
    setPwmFrequency(11, 1); // set Timer2 to clock divisor of 1 (31250Hz, above audible range)
    analogWrite(LED_PIN, getBrightness(brightnessIterator));
  }
  else
  {
    digitalWrite(LED_PIN, HIGH);  // just turn the LEDs on full brightness
  }

  // set some starting frequencies for the oscillators
  osc0.setFreq(mtof(noteNameToMIDINote("E2")));
  osc1.setFreq(mtof(noteNameToMIDINote("A3")));
  osc2.setFreq(mtof(noteNameToMIDINote("B4")));

  // Start the timer that controls how frequently the I2C sensors get polled. The sensors are updated
  // one at a time in updateControl() — arm encoder, table encoder, then color sensor, cycling through
  // in sequence. I2C_UPDATE_INTERVAL is defined in Configuration.h, and is currently set to 15ms.
  // That means each individual sensor gets a fresh reading every 45ms (3 sensors * 15ms). Updating
  // more frequently gives you more responsive control, but I2C reads block the processor, so polling
  // too aggressively will steal time from Mozzi's audio generation and cause audible glitching.
  // After this call, the timer is restarted at the end of each sensor read in updateControl() rather
  // than running on a fixed schedule, so the interval represents minimum time between reads, not a
  // strict period.
  k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);

  // set the initial attack and decay levels for the oscillators (basically fade in and fade out)
  osc0AmpEnv.setADLevels(160, 140);
  osc1AmpEnv.setADLevels(80, 60);
  osc2AmpEnv.setADLevels(160, 140);

  // finally, start Mozzi!
  startMozzi(MOZZI_CONTROL_RATE);
}


// **********************************************************************************
// updateControl for Mozzi
// **********************************************************************************

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

  // These are auto-ranging mappings for the color channels. They work like map() but track the
  // min/max values seen so far and update the mapping range dynamically.
  // They are static variables here (not globals) so we can use colorSensor.getResolution() to set
  // the upper input bound rather than hardcoding it.
  static AutoMap autoGreenToUINT8_T(0, colorSensor.getResolution(), 0, 255);
  static AutoMap autoBlueToUINT8_T(0,  colorSensor.getResolution(), 0, 255);
  static AutoMap autoRedToUINT8_T(0,   colorSensor.getResolution(), 0, 255);
  static AutoMap autoWhiteToUINT8_T(0, colorSensor.getResolution(), 0, 255);

  // check to see if buttons are pressed. returns 0, 1, 2, or 255 (no press)
  buttonPressed = getButtonPressed(mozziAnalogRead<10>(BUTTONS_PIN));
  if (nextButtonPressAllowed && buttonPressed < 255)
  {
    buttonTimer.start();
    nextButtonPressAllowed = false;
    switch (buttonPressed)
    {
    case 0: // left button — LED brightness levels
      brightnessIterator = (brightnessIterator + 1) % NUM_BRIGHTNESS_LEVELS;
      analogWrite(LED_PIN, getBrightness(brightnessIterator));
      break;

    case 1: // middle button — scale selector
      scaleContainer.nextScale();
      currentScale = scaleContainer.selected();
      SERIAL_TABS(2);
      SERIAL_PRINT("scale: ");
      SERIAL_PRINTLN(scaleContainer.scaleSelector);
      break;

    case 2: // right button — toggle button mode 2
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

  // Update the table's target speed from the pot. TableManager ignores this if
  // stop-detection has kicked in and resumes tracking the pot once the table
  // starts moving again.
  table.updateTargetSpeed(mozziAnalogRead<10>(POT_B_PIN));

  // Update sensors one at a time whenever the timer fires. Originally all three
  // were updated simultaneously; staggering may improve audio consistency.
  static uint8_t currentSensorToUpdate = 0;

  if (k_i2cUpdateDelay.ready())
  {
    switch (currentSensorToUpdate)
    {
    case 0: // update the arm encoder
      arm.updatePosition();
      currentSensorToUpdate++;
      break;

    case 1: // update the table encoder — stop-detection runs inside updateAngle()
      table.updateAngle();
      currentSensorToUpdate++;
      break;

    case 2: // update the color sensor
      colorSensor.update();
      colorSensor.printColorData();
      currentSensorToUpdate = 0;
      break;

    default:
      break;
    }
    k_i2cUpdateDelay.start();
  }

  // Move the arm to the position requested by the potentiometer.
  targetArmPos = arm.convertPotValToRadius(mozziAnalogRead<10>(POT_A_PIN));
  arm.moveToAngle(arm.radiusToAngle(targetArmPos));

  // Commit the table motor speed.
  table.applySpeed();

  mappedGreen = autoGreenToUINT8_T(colorSensor.getGreenFixed().asInt());
  mappedBlue  = autoBlueToUINT8_T(colorSensor.getBlueFixed().asInt());
  mappedRed   = autoRedToUINT8_T(colorSensor.getRedFixed().asInt());
  mappedWhite = autoWhiteToUINT8_T(colorSensor.getClearFixed().asInt());

  ambienceGenerator();

  previousEnableButton2Mode = enableButton2Mode;
}


// **********************************************************************************
// updateAudio for Mozzi
// **********************************************************************************

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

void loop()
{
  audioHook();
}


// **********************************************************************************
// ambienceGenerator
// **********************************************************************************

void ambienceGenerator()
{
  static int8_t arpIndex = 0;                   // used to select notes in a scale to play as an arpeggio
  static uint8_t numNotesLeftInArp = 0;         // arpeggios only play for a limited number of notes
  static bool initialize = true;                // used to initialize some things the first time this function is called

  static uint16_t osc2PortTime = 20;            // portamento value 

  const bool USE_PORTAMENTO = true;

  static uint16_t attack = 100, decay = 500, sustain = 8000, release = 3000;

  // used to move a musical note up or down by an octave. mappedWhite is the 8 bit autoMapped value
  // of the white light color channel, and the >> 6 operator performs a right shift, where it takes
  // all 8 bits in a number and literally moves them right, filling in with 0s on the left. This moves
  // 6 places to the right, so it reduces an 8 bit number to a 2 bit number, which can represent the
  // numbers 0 through 3. It's basically a way of dividing 256 by 64, but it's a much faster operation.
  // the microcontroller has hardware purpose-built for performing bit shift operations, and it only 
  // takes a single clock cycle. divisions is incredibly slow on this microcontroller, and might need
  // something like 40 clock cycles.
  int8_t octaveShifter = (int8_t)(mappedWhite >> 6);
  static bool arpeggiate = false, arpStarted = false, arpOnTimeOut = true;

  // button2mode is basically arpeggiating each oscillator, instead of holding sustained notes. 
  // baseNoteInterval is the starting point for how much time should elapse between triggering notes
  // in this mode. This gets modified by the value of the green color channel in button2mode.
  static uint8_t baseNoteInterval = 64;

  if (!enableButton2Mode && previousEnableButton2Mode)
    initialize = true; // transition out of button mode 2 requires resetting ADSRs

  // on first call, we need to set some things up
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

  // button2mode is basically arpeggiating each oscillator, instead of holding sustained notes
  if (enableButton2Mode)
  {
    // if we weren't already in button2Mode, we need to set some things up first
    if (!previousEnableButton2Mode)
    {
      // change the ADSR to make the notes sound plucky
      osc0AmpEnv.setTimes(5, 50, 100, 200);
      osc1AmpEnv.setTimes(5, 50, 100, 200);
      osc1AmpEnv.setADLevels(60, 40);
      osc2AmpEnv.setTimes(5, 50, 100, 200);

      // start the timer objects that will determine how often we should play a note
      // notes are triggered more or less frequently based on the value of the green color channel.
      // so we start with that baseNoteInterval, and then change it based on the green channel.
      // the right shift operations drop the value of the green channel to a max of 4 bits, so a
      // maximum of (2^4) - 1 = 15. The max() function call is there to add a lower. There is a chance
      // that the bit shift would create a value of 0, and then we'd be multiplying the base interval
      // by 0, which means constantly retriggering the note (which would result in a silent note because
      // the ADSR would constantly restart). So we multiply the base interval by a number between 1 and 15, 
      // so the base interval ranges from 64ms to 15*64ms = 960ms. 
      osc0ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval);
      osc0ButtonMode2NoteTimer.start();
      osc1ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval * 2);
      osc1ButtonMode2NoteTimer.start();
      osc2ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval);
      osc2ButtonMode2NoteTimer.start();
    }

    // the base note interval starts at 64, but now it gets modified by the value of the white color channel
    baseNoteInterval = max(1, (mappedWhite >> 5)) * 16; // evaluates to a range between 16 and 7*16 = 112

    // check to see if we're ready to trigger a note for osc0
    if (osc0ButtonMode2NoteTimer.ready())
    {
      // this is a fun way to add some randomness and also modulate the effect of the randomness with a sensor reading.
      // first we generate a random number between 1 and 255. If that number is less than the current value of the red
      // color channel sensor reading, then we trigger a new note. So what this means is that the more red light the sensor
      // sees, the more frequently we'll trigger a note! If there's absolutely no red light (mappedRed == 0), then we'll
      // never trigger a note because rand(256) will never return a number less than 0. I you want to tune the behavior
      // of this, change the constraints on the random number. For example, you could change to rand(128), which should
      // make notes trigger a lot more often. Or move the range up so that only extremely bright red light values trigger
      // a note by using something like rand(192, 256).
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
      // turn the note on
      osc0AmpEnv.noteOn();
      osc0Params.frequency = mtof(osc0Params.noteMIDINumber);
      osc0.setFreq(osc0Params.frequency);
      // reset the timer
      osc0ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval);
      osc0ButtonMode2NoteTimer.start();
    }

    // now do the same thing for osc1
    if (osc1ButtonMode2NoteTimer.ready())
    {
      if (rand(256) < mappedGreen)
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
      osc1AmpEnv.noteOn();
      osc1Params.frequency = mtof(osc1Params.noteMIDINumber);
      osc1.setFreq(osc1Params.frequency);
      osc1ButtonMode2NoteTimer.set(max(1, mappedBlue >> 4) * baseNoteInterval * 2);
      osc1ButtonMode2NoteTimer.start();
    }
    // and do the same for osc2
    if (osc2ButtonMode2NoteTimer.ready())
    {
      if (rand(256) < mappedBlue)
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

  else  // we're not in button 2 mode in this case
  
  {
    // octaveShifter is currently some value between 0 and 3. This moves and constrains it
    // to be either -1, 0, or 1.
    octaveShifter = max(-1, octaveShifter - 2); 

    // we'll occasionally trigger an arpeggio if the white light channel is bright enough, but to prevent
    // that from happening too frequently, we use this timer to prevent a new arpeggio triggering too soon
    // after the last one. 
    if (arpTimeout.ready())
    {
      arpOnTimeOut = false;
    }

    // this makes sure that arpeggios only get triggered if the octave has been shifted up by 1. I did this
    // to add some brightness to the sound. Basically the arpeggio triggers if the white light channel is
    // bright enough, and I wanted the arpeggio sound to mirror that brightness by being shifted up an octave.
    if (octaveShifter > 0 && !arpOnTimeOut && !arpStarted)
    {
      arpeggiate = (rand(256) <= mappedWhite) ? true : false; // add some randomness so that we don't always trigger an arp
    }

    // choose white note in the scale we'll be using based on the value of the green channel
    uint8_t i = 0;
    if (scaleContainer.selected().numNotes == 7)
    {
      i = colorToScaleNote7(mappedGreen);
    }
    else
    {
      i = colorToScaleNote5(mappedGreen);
    }

    //
    osc0Params.noteMIDINumber = scaleContainer.selected().getNote(i);
    if (osc0Params.lastNoteMIDINumber != osc0Params.noteMIDINumber)
    {
      osc0AmpEnv.noteOn();
      osc0Params.lastNoteMIDINumber = osc0Params.noteMIDINumber;
    }
    osc0Params.frequency = mtof(osc0Params.noteMIDINumber);
    osc0.setFreq((osc0Params.frequency));
    osc0AmpEnv.update();
    osc0Params.volume = osc0AmpEnv.next();

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
      osc1Params.noteMIDINumber = scaleContainer.selected().getNote((i + 2) % scaleContainer.selected().numNotes) + octaveShifter * 12;
      break;
    case 2:
      osc1Params.noteMIDINumber = scaleContainer.selected().getNote((i)) - 12;
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
