#include <Arduino.h>
#include <Configuration.h>
#include <Crunchlabs_DRV8835.h>  // motor driver
#include <Wire.h>                // I2C
#include <PWMFreak.h>            // Changes clock rate of timers (used to remove audible noise from motor driver and LED dimming)
#include <Mozzi.h>               // Main synthesizer library
#include <Oscil.h>               // Oscillator
#include <tables/saw2048_int8.h> // Saw wavetable
#include <tables/triangle_dist_cubed_2048_int8.h>
#include <tables/triangle_valve_2_2048_int8.h>
#include <IntMap.h>     // A more efficient replacement for map()
#include <EventDelay.h> // Mozzi library for performing actions at specific time intervals without delay() or millis()
#include <mozzi_utils.h>
#include <mozzi_rand.h>
#include <mozzi_midi.h> // MIDI functionality, used for mtof(), which converts MIDI note numbers to frequencies
#include <CLS16D24.h>   // Color sensor
#include <AS5600.h>     // Magnetic encoders for the arm and table positions
#include <AutoMap.h>    // A version of map() that is auto-ranging. Used for mapping color values to control signals.
#include <Portamento.h>
#include <ADSR.h>


// delay timer for updating sensor data. When the sensor data is getting updated, the processor can't simultaneously update the sound it's generating.
// You can update the sensor more frequently, but that might negatively impact sound generation. This also updates all three sensors simultaneously,
// and it might work better to update each of them at staggered intervals.
EventDelay k_i2cUpdateDelay;


// **********************************************************************************
// Arm and Table stuff
// **********************************************************************************

#include <Mechanisms.h>
#include <AnalogButtons.h>


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
 * 6 bytes of RAM, that was worth it. The trade off is that it actually costs more in flash memory space (60 bytes more).
 * But there's plenty of flash left over, and at the time, 6 bytes of RAM was a good savings. That's about 0.3% of the 
 * available 2048 bytes of RAM. I later figured out much better optimizations in the way the program stores chord data,
 * and that dropped RAM use by like 20%. But I still like this trick for generating a sequence of numbers and left it in
 * just for fun. In retrospect, it would probably make the most sense to just store the brightness array in flash.
 * 
 * so actually yes, it does make the most sense to store the array in flash. I just modified the function to do that, 
 * and now it minimizes RAM use as much as the bit manipulation, and also minimizes flash use to the same level that the 
 * array in RAM version does. So I'll use that, but I'm still leaving the bit manipulation example in because it was fun
 * to figure out how to do. Try to reverse engineer it! Or read the notes I made about it. Here's the breakdown of RAM and
 * flash use between all three methods:
 * 
 * // Array stored in RAM:
 * RAM:   [=======   ]  72.8% (used 1491 bytes from 2048 bytes)
 * Flash: [=======   ]  68.6% (used 21064 bytes from 30720 bytes)
 * 
 * // Fancy bit manipulation version:
 * RAM:   [=======   ]  72.5% (used 1485 bytes from 2048 bytes)
 * Flash: [=======   ]  68.8% (used 21124 bytes from 30720 bytes)
 * 
 * // Finally realizing I should just store the array in flash:
 * RAM:   [=======   ]  72.5% (used 1485 bytes from 2048 bytes)
 * Flash: [=======   ]  68.6% (used 21064 bytes from 30720 bytes)
 */
#define USE_BRIGHTNESS_ARRAY 1

#if USE_BRIGHTNESS_ARRAY
const uint8_t brightnessValues[NUM_BRIGHTNESS_LEVELS] PROGMEM = {0, 32, 64, 192, 255};
inline uint8_t getBrightness(uint8_t level = 3)
{
  return pgm_read_byte(&brightnessValues[level]);
}
#else
// This function generates the sequence 0, 31, 63, 191, 255 to correspond to 5 PWM values for LED dimming. To
// understand how it works, see the section [[#how LED brightness PWM values are calculated]] in Footnotes.md
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

// pin assignments
constexpr uint8_t POT_A_PIN = A0,
                  POT_B_PIN = A1,
                  AUDIO_IN_PIN = A2,
                  BUTTONS_PIN = A3; // all 3 buttons  are connected to the same analog pin. uses analog multiplexing to distinguish button presses.

uint8_t buttonPressed = 255;                     // 255 means no button pressed, 0, 1 or 2 indicates which of the three buttons was pressed
uint8_t getButtonPressed(uint16_t buttonPinVal); // pass an analog reading on BUTTON_PIN into the parameter. Returns 1, 2, or 3 if one of those buttons is pressed, otherwise returns 255.




// **********************************************************************************
// Mozzi stuff
// **********************************************************************************

// Set the frequency of updateControls() calls - by default they occur at 128Hz.
// Note that this number should always be a power of 2 (e.g., 1, 2, 4, 8, 16, 32, 64, 128, etc).
// More frequent updates here create better responsiveness for the controls, but the tradeoff is that it takes time away from generating sound,
// so if you create more frequent updateControls() calls (e.g., by setting this to 256), you might start hearing glitchy audio artifacts. In general,
// make this the smallest value you can that still feels responsive to leave as much processing power free for the audio synthesis as possible.
#define MOZZI_CONTROL_RATE 128

Oscil<TRIANGLE_DIST_CUBED_2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc0(TRIANGLE_DIST_CUBED_2048_DATA);
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc1(SAW2048_DATA);
Oscil<TRIANGLE_VALVE_2_2048_NUM_CELLS, MOZZI_AUDIO_RATE> osc2(TRIANGLE_VALVE_2_2048_DATA);




// **********************************************************************************
// Music stuff
// **********************************************************************************

#include <MusicTools.h>

oscillatorParams osc0Params, osc1Params, osc2Params;

Portamento<MOZZI_CONTROL_RATE> osc0Portamento, osc1Portamento, osc2Portamento;

ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> osc0AmpEnv, osc1AmpEnv, osc2AmpEnv; // ADSR envelopes for all three oscillators

// Mozzi EventDelay timers, used instead of millis() for timing
EventDelay chordTimer, arpDurationTimer, arpNoteTimer, arpTimeout;
EventDelay osc0ButtonMode2NoteTimer, osc1ButtonMode2NoteTimer, osc2ButtonMode2NoteTimer;

/**
 * These IntMaps are for taking in scaled color data (e.g., the 8 bit Green channel) and squashing it down to fit into a 
 * musical scale. I think that at some point the proper way to do this would be to make a templated version of this function,
 * instead of separately named functions. Or I guess not a function but some kind of templated wrapper.
 */
const IntMap colorToScaleNote7(0, 255, 0, 7); // for 7 note scales
const IntMap colorToScaleNote5(0, 255, 0, 5);

// variables related to the buttons
bool enableButton2Mode = false, previousEnableButton2Mode = false;
uint8_t lastButtonMode = 0, currentButtonMode = 0;



// **********************************************************************************
// Sound Generator Stuff
// **********************************************************************************
void ambienceGenerator(); // right now i just want to wrap all the sound control stuff in a function so I can easily separate it out from the rest of updateControl()




// **********************************************************************************
// Setup
// **********************************************************************************

void setup()
{
  randSeed(analogRead(A7)); // initialize the random number seed

  // start the serial monitor, if you want to use it (determined by #define USE_SERIAL)
  SERIAL_BEGIN(115200);
  SERIAL_PRINTLN("starting");

  // start the I2C bus
  Wire.begin();

  tableEncoder.begin();
  tableEncoder.resetCumulativePosition(); // calibrate to 0 as starting position for the table
  armEncoder.begin();

  tableEncoder.setHysteresis(3); // value has to change by +- 3 before getting reported as change by sensor
  armEncoder.setHysteresis(3);   // helps remove noise from sensor data

  // Now home the arm. This moves the arm back to the center of the table by the end of the routine,
  // and then resets the arm encoder position to 0.
  homeArm();
  currentArmPosition = armAngleToRadius(armEncoder.getCumulativePosition());

  // start the color sensor I2C connection
  colorSensor.begin(false);

  // IMPORTANT: Set the I2C clock to 400kHz fast mode AFTER initializing the connection to the sensors.
  // Need to use fastest I2C possible to minimize latency for Mozzi.
  Wire.setClock(400000);

  // initialize color sensor
  colorSensor.reset();
  colorSensor.enable();

  // possible gain settings are 1, 4, 8, 32, 96. setting the second parameter to true doubles the diode sensing area, which increases sensitivity
  colorSensor.setGain(32, false);

  // See the end of CLS16D24.h for a complete table of possible values here:
  colorSensor.setResolutionAndConversionTime(0x02);
  SERIAL_PRINT("Conversion time: ");
  SERIAL_PRINTLN(colorSensor.getConversionTimeMillis());
  SERIAL_PRINT("Resolution: ");
  SERIAL_PRINTLN(colorSensor.getResolution());

  // Define which color channels to update here. Set to true to enable that color channel.
  // This is a bit of an artifact from using the VEML3328 color sensor, instead of the CLS-16D24 that I'm using now.
  // It turned out that the VEML3328 could only reliably update one color channel at a time, with something like 50ms
  // intervals between updates, so I had to come up with a way to automatically cycle through the color channels and update
  // each one sequentially. That's originally what this was for, but I wound up using it in the printColorData() function
  // as well, and it could potentially be useful for enabling or disabling the effects of color channels on the synth.
  // However, this is probably too complex. The CLS-16D24 updates all 5 channels simultaneously and much more rapidly, so I'll
  // probably just remove this feature eventually.

  colorSensor.setChannelEnabled(ColorChannels::RED, true);
  colorSensor.setChannelEnabled(ColorChannels::GREEN, true);
  colorSensor.setChannelEnabled(ColorChannels::BLUE, true);
  colorSensor.setChannelEnabled(ColorChannels::CLEAR, true);
  colorSensor.setChannelEnabled(ColorChannels::IR, true);

  if (USE_FAST_PWM)
  {
    // use fast PWM to remove audible noise from driving the motors.
    // IMPORTANT: This breaks millis() and delay(), and may have other effects I haven't
    // found yet. After this line, you can't use millis() or delay() calls and have them
    // behave as expected. But Mozzi offers event timers that work in place of millis() anyway.
    setPwmFrequency(5, 1); // sets Timer0 clock divisor to 1 instead of 64. This moves the motor control pins above audible range
  }

  // EXPERIMENTAL:
  // I noticed that if I try to use PWM dimming for the LEDs on the color sensor I get audible noise, so this is experimental.
  // There's a chance that changing Timer 2's clock divisor will mess up Mozzi functions I'm not aware of. So far it hasn't though.
  if (USE_LED_PWM)
  {
    setPwmFrequency(11, 1);  // set Timer2 to clock divisor of 1 (clock rate 31250Hz now, above audible range)
    analogWrite(LED_PIN, getBrightness(brightnessIterator));
  }
  else
  {
    digitalWrite(LED_PIN, HIGH); // just turn the color sensor LEDs on at full brightness
  }

  // set the initial frequencies of the oscillators
  osc0.setFreq(mtof(noteNameToMIDINote("E2")));
  osc1.setFreq(mtof(noteNameToMIDINote("A3")));
  osc2.setFreq(mtof(noteNameToMIDINote("B4")));

  // start the timer that triggers sensor updates
  k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);

  // amplitude envelope settings for the oscillators (attack and decay -> basically fade in and fade out for each triggered note)
  osc0AmpEnv.setADLevels(160, 140);
  osc1AmpEnv.setADLevels(80, 60);
  osc2AmpEnv.setADLevels(160, 140);

  // start Mozzi
  startMozzi(MOZZI_CONTROL_RATE);
}

// **********************************************************************************
// updateControl for Mozzi
// **********************************************************************************

// #include <UpdateControl.h>

void updateControl()
{
  static int8_t targetArmPos = 80;
  static bool initialize = true, nextButtonPressAllowed = true;
  static EventDelay buttonTimer;
  if (initialize)
  {
    buttonTimer.set(250); // 125ms is exactly 1/16th notes for 120bpm in 4/4
    // buttonTimer.start();
    arpTimeout.set(4000);
    arpTimeout.start();
    initialize = false;
  }

  // these are auto-ranging mappings for the color channels. These work a lot like the normal map() function, except that they keep track of the
  // largest and smallest values that they have seen so far, and update the map to reflect those. So the first two parameters are the minimum and
  // maximum possible input values (if this were a standard analogRead(), that would be 0 and 1023). As you update the function, it keeps track of
  // the mapping value you provide it, and the min and max values it has seen become the new min and max values for the input map range. Note that
  // the parameters are ints, so signed 16 bit values, and you have to be sure the numbers you pass into the parameters will fit in that variable size.
  //
  // Note that this only goes one way. If a new max value is seen, that's the stored max value until the next system reset. If you shine a really bright
  // light on the sensors and max out the readings for all the color channels, this mapping will never relax back to the lower light level readings
  // that occur once you stop shining the bright light. It's like if you were used to a dark room, walked outside into bright sunlight, and your eyes
  // adapted to the bright light and then never adapted to any dim light after that. This could be changed with some kind of relaxation function.
  //
  // The reason these are static variables inside updateControl() instead of being global is that I wanted to use the RGBCIR.getResolution() function
  // to be able to set the size of the mapping, rather than hardcoding the value. That way the maps get dynamically resized based on the chosen
  // resolution of the sensor.
  static AutoMap autoGreenToUINT8_T(0, colorSensor.getResolution(), 0, 255);
  static AutoMap autoBlueToUINT8_T(0, colorSensor.getResolution(), 0, 255);
  static AutoMap autoRedToUINT8_T(0, colorSensor.getResolution(), 0, 255);
  static AutoMap autoWhiteToUINT8_T(0, colorSensor.getResolution(), 0, 255);

  // check to see if buttons are pressed. returns 0, 1, 2, 255
  // 0 = B1, 1 = B2, 2 = B3, 255 = no press
  // deal with the button presses
  buttonPressed = getButtonPressed(mozziAnalogRead<10>(BUTTONS_PIN));
  if (nextButtonPressAllowed && buttonPressed < 255)
  {
    buttonTimer.start();            // start the timer if the button is pressed to prevent new button presses being reacted to within the timer window
    nextButtonPressAllowed = false; // prevent this block from being reentered until a new button press is allowed after buttonTimer is ready.
    switch (buttonPressed)
    {
    case 0: // left button, LED brightness levels
      brightnessIterator = (++brightnessIterator) % NUM_BRIGHTNESS_LEVELS;
      analogWrite(LED_PIN, getBrightness(brightnessIterator));
      break;

    case 1: // middle button, scale selector
      scaleContainer.nextScale();
      currentScale = scaleContainer.selected();
      SERIAL_TABS(2);
      SERIAL_PRINT("scale: ");
      SERIAL_PRINTLN(scaleContainer.scaleSelector);
      break;

      case 2: // right button, all pizzicato I think, instead of sustained notes
      enableButton2Mode = !enableButton2Mode;
      break;

    default:
      break;
    }
  }

  // this timer prevents the buttons from being updated too frequently
  if (!nextButtonPressAllowed && buttonTimer.ready())
  {
    nextButtonPressAllowed = true;
  }

  // update the sensors. This updates one sensor at a time whenever the timer is ready. Originally I just updated all three
  // sensors simultaneously, but only 1/3rd as often, and got similar performance, so I'm not sure if this is actually better or not. 
  // Worth experimenting with to see if one big update or spread out updates lets you achieve more frequent updates without glitching sound output.
  static uint8_t currentSensorToUpdate = 0, tableStoppedCount = 0;

  // set the table speed based on the potentiometer position
  static int16_t targetTableSpeed = convertPotValToTableSpeed(mozziAnalogRead<10>(POT_B_PIN));
  // static int16_t savedTableSpeed = targetTableSpeed;
  static bool tableStopped = false;

  // read the target table speed, if the table hasn't been stopped
  if (!tableStopped) targetTableSpeed = convertPotValToTableSpeed(mozziAnalogRead<10>(POT_B_PIN));

  if (k_i2cUpdateDelay.ready())
  {
    switch (currentSensorToUpdate)
    {
    case 0: // update the arm encoder
      currentArmAngle = armEncoder.getCumulativePosition();
      currentArmPosition = armAngleToRadius(currentArmAngle);
      currentSensorToUpdate++;
      break;

    case 1: // update the table encoder
      lastTableAngle = currentTableAngle;
      currentTableAngle = tableEncoder.getCumulativePosition();
      if (!tableStopped)
      {
        // check to see if it looks like the the table has stopped (with a deadband to account for noise)
        if (abs(currentTableAngle - lastTableAngle) <= 5)
        {
          tableStoppedCount++;
        }
        // if we've measured the same angular position for the table two loop iterations in a row, then we can assume it is being held in place.
        // remember that this check only happens every time this sensor updates, which means this should be a window about ~90ms, depending on the
        // sensor update frequency.
        if (tableStoppedCount == 2)
        {
          targetTableSpeed = 0;
          tableStoppedCount = 0;
          tableStopped = true;
        }
      }
      else
      {
        // if there's a large enough deviation in the table position, start it spinning again.
        if (abs(currentTableAngle - lastTableAngle) > 5)
        {
          targetTableSpeed = convertPotValToTableSpeed(mozziAnalogRead<10>(POT_B_PIN));
          tableStopped = false;
        }
      }
      currentSensorToUpdate++;
      break;

    case 2: // update the color sensor
      colorSensor.update();
      colorSensor.printColorData();
      currentSensorToUpdate = 0;
      lastTableAngle = currentTableAngle;
      break;
    default:
      break;
    }
    k_i2cUpdateDelay.start(); // restart the timer immediately after new sensor data is acquired
  }

  // move the arm to the angle required by the potentiometer
  targetArmPos = convertPotValToArmRadius(mozziAnalogRead<10>(POT_A_PIN));
  int16_t targetArmAngle = armRadiusToAngle(targetArmPos);
  moveArmToAngle(targetArmAngle, currentArmAngle);

  // set the approriate table motor speed
  tableMotor.setSpeed(targetTableSpeed);

  // AutoMap instances that handle the color data mapping to control signals. These bring the values all the way down to uint8_t.
  mappedGreen = autoGreenToUINT8_T(colorSensor.getGreenFixed().asInt());
  mappedBlue = autoBlueToUINT8_T(colorSensor.getBlueFixed().asInt());
  mappedRed = autoRedToUINT8_T(colorSensor.getRedFixed().asInt());
  mappedWhite = autoWhiteToUINT8_T(colorSensor.getClearFixed().asInt());

  // call the function that will be used to convert color data to sound
  ambienceGenerator();

  previousEnableButton2Mode = enableButton2Mode;
}

// **********************************************************************************
// updateAudio for Mozzi
// **********************************************************************************

AudioOutput_t updateAudio()
{
  int32_t asig = (int32_t)
    osc0.next() *
    osc0Params.volume +
    osc1.next() * osc1Params.volume +
    osc2.next() * osc2Params.volume;

  // asig = (int32_t) mainLPFilter.next(asig);
  return MonoOutput::fromAlmostNBit(17, asig);
}

// **********************************************************************************
// Loop
// **********************************************************************************

void loop()
{
  // synthesize new audio for Mozzi
  audioHook();
}




// **********************************************************************************
// Function Definitions
// **********************************************************************************


void ambienceGenerator()
{
  static int8_t arpIndex = 0;
  static uint8_t numNotesLeftInArp = 0;
  static bool initialize = true;

  static uint16_t osc2PortTime = 20;

  const bool USE_PORTAMENTO = true;

  static uint16_t attack = 100, decay = 500, sustain = 8000, release = 3000;

  int8_t octaveShifter = (int8_t)(mappedWhite >> 6);
  static bool arpeggiate = false, arpStarted = false, arpOnTimeOut = true;

  // button mode 2 stuff
  static uint8_t baseNoteInterval = 64;

  // SERIAL_PRINTLN(currentButtonMode);

  if (!enableButton2Mode && previousEnableButton2Mode)
    initialize = true; // transition out of button mode 2 requires resetting ADSRs

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

      osc0ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval); // this will make all the notes play in interval steps of 32ms. quantize
      osc0ButtonMode2NoteTimer.start();
      osc1ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval * 2);
      osc1ButtonMode2NoteTimer.start();
      osc2ButtonMode2NoteTimer.set(max(1, mappedGreen >> 4) * baseNoteInterval);
      osc2ButtonMode2NoteTimer.start();
    }

    baseNoteInterval = (mappedWhite >> 5) * 16;

    // check the event delay to see if the note for each voice is finished playing, then set up new notes if threshold met
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
  else
  {

    octaveShifter = max(-1, octaveShifter - 2); // should set octaveShifter to -1, 0, or 1 octaves added
    // SERIAL_PRINTLN(octaveShifter);

    if (arpTimeout.ready())
    { // it's been long enough to allow an arpeggio again
      arpOnTimeOut = false;
    }

    if (octaveShifter > 0 && !arpOnTimeOut && !arpStarted)
    { // arp is allowed again and triggered by enough white light
      arpeggiate = (rand(256) <= mappedWhite) ? true : false;
    }

    uint8_t i = 0;
    if (scaleContainer.selected().numNotes == 7)
    {
      i = colorToScaleNote7(mappedGreen);
    }
    else
    {
      i = colorToScaleNote5(mappedGreen);
    }

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
        arpIndex += arpShift; // move to next note in sequence

        arpIndex = (arpIndex < scaleContainer.selected().numNotes && arpIndex >= 0) ? arpIndex : ((arpIndex < 0) ? arpIndex += scaleContainer.selected().numNotes : arpIndex -= scaleContainer.selected().numNotes);
        arpNoteTimer.start();
        numNotesLeftInArp -= 1; // we have one fewer notes left in the arp
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