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



// **********************************************************************************
// Arm and Table stuff
// **********************************************************************************

#include <mechanisms.h>



// **********************************************************************************
// Color sensor stuff
// **********************************************************************************

#include <ColorSensor.h>

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


// struct for storing parameters for each oscillator
struct oscillatorParams
{
  const char *note = 0;
  const char *lastNote = 0;
  MIDI_NOTE noteMIDINumber = 0;
  MIDI_NOTE lastNoteMIDINumber = 0;
  float frequency = 0.0;
  uint8_t volume = 0;
};

oscillatorParams osc0Params, osc1Params, osc2Params;

Portamento<MOZZI_CONTROL_RATE> osc0Portamento, osc1Portamento, osc2Portamento;

ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> osc0AmpEnv, osc1AmpEnv, osc2AmpEnv; // ADSR envelopes for all three oscillators

// Mozzi EventDelay timers, used instead of millis() for timing
EventDelay chordTimer, arpDurationTimer, arpNoteTimer, arpTimeout;
EventDelay osc0ButtonMode2NoteTimer, osc1ButtonMode2NoteTimer, osc2ButtonMode2NoteTimer;

/**
 * I just figured out that IntMap doesn't work as documented. documentation says that it's this:
 * const IntMap theMap(fromLow, fromHigh, toLow, toHigh), and that toLow and toHigh are the minimum and maximum numbers that will be ouput.
 * However, the actually maximum output is toHigh - 1. So if you want to map [0,100] (inclusive) onto [0,1000] (inclusive), you actually need
 * to set up your IntMap as
 * const IntMap correctedMap(0, 100, 0, 1001);
 *
 * or for extra clarity:
 * const IntMap correctedMap(0, 100, 0, 1000 + 1);
 *
 * At some point I should submit this as an issue to the Mozzi team to fix the documentation.
 *
 * actually I was wrong. For larger numbers my observation seems to be the case, but for the below mappings, that is incorrect.
 * */
const IntMap colorToScaleNote7(0, 255, 0, 7); // for 7 note scales
const IntMap colorToScaleNote5(0, 255, 0, 5);

// variables related to the buttons
bool enableButton2Mode = false, previousEnableButton2Mode = false;
uint8_t lastButtonMode = 0, currentButtonMode = 0;



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

  if (!RGBCIR.begin(false))
  { // set parameter to true to use i2c fast mode (400kHz instead of 100kHz)
    SERIAL_PRINTLN("ERROR: couldn't detect the sensor");
    while (1)
    {
    }
  }
  // IMPORTANT: Set the I2C clock to 400kHz fast mode AFTER initializing the connection to the sensors.
  // Need to use fastest I2C possible to minimize latency for Mozzi.
  Wire.setClock(400000);

  // Configure the color sensor.
  RGBCIR.reset();
  RGBCIR.enable();
  // possible gain settings are 1, 4, 8, 32, 96. setting the second parameter to true doubles the diode sensing area, which increases sensitivity
  RGBCIR.setGain(32, false);
  /**
   * Both the total time it takes to convert a color reading into values and the resolution of that data are set
   * by the value of a single byte, which you can set with setResolutionAndConversionTime(). The resolution and
   * conversion time are interdependent, so it's difficult to know what time and resolution you are going to get
   * by the value you set the byte to. Open CLS16D24.h, and at the bottom you will find a large comment that shows
   * all possible values for conversion time and resolution.
   *
   * The default value of 0x02 yields these results:
   * COMMAND	CLS_CONV	INT_TIME	Calculated Resolution (maximum possible value)	Calculated Resolution (in bits)	    Calculated Conversion Time (in milliseconds)
   * 0x02	    0	        2	        16383	                                          14	                                36.8942
   *
   * I chose this as a good general setting because it offers a lot of color resolution (exactly 14 bits worth) and a reasonable conversion time of about 37ms.
   * It might actually make more sense thought to use 0x00, because that reduces resolution to 10 bits, but also drops conversion time to 5.89ms.
   * I'm pretty much always squashing the values to even lower resolution later in the code, so dropping to a lower resolution straight from the sensor could be
   * a good thing to try out.
   */
  RGBCIR.setResolutionAndConversionTime(0x02);
  SERIAL_PRINT("Conversion time: ");
  SERIAL_PRINTLN(RGBCIR.getConversionTimeMillis()); // Calculates the conversion time determined by setResolutionAndConversionTime()
  SERIAL_PRINT("Resolution: ");
  SERIAL_PRINTLN(RGBCIR.getResolution()); // Calculates resolution determined by setResolutionAndConversionTime()

  // Define which color channels to update here. Set to true to enable that color channel.
  // This is a bit of an artifact from using the VEML3328 color sensor, instead of the CLS-16D24 that I'm using now.
  // It turned out that the VEML3328 could only reliably update one color channel at a time, with something like 50ms
  // intervals between updates, so I had to come up with a way to automatically cycle through the color channels and update
  // each one sequentially. That's originally what this was for, but I wound up using it in the printColorData() function
  // as well, and it could potentially be useful for enabling or disabling the effects of color channels on the synth.
  // However, this is probably too complex. The CLS-16D24 updates all 5 channels simultaneously and much more rapidly, so I'll
  // probably just remove this feature eventually.
  updateChannels[static_cast<int>(ColorChannels::RED)] = true;
  updateChannels[static_cast<int>(ColorChannels::GREEN)] = true;
  updateChannels[static_cast<int>(ColorChannels::BLUE)] = true;
  updateChannels[static_cast<int>(ColorChannels::CLEAR)] = true;
  updateChannels[static_cast<int>(ColorChannels::IR)] = true;

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
    setPwmFrequency(11, 1);                                        // set Timer2 to clock divisor of 1 (clock rate 31250Hz now, above audible range)
    // analogWrite(LED_PIN, LEDBrightnessLevels[brightnessIterator]); // set the brightness for the LEDs
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
  k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL); // control how frequently we poll the sensors on the I2C bus (color sensor, both encoders)

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
  static AutoMap autoGreenToUINT8_T(0, RGBCIR.getResolution(), 0, 255);
  static AutoMap autoBlueToUINT8_T(0, RGBCIR.getResolution(), 0, 255);
  static AutoMap autoRedToUINT8_T(0, RGBCIR.getResolution(), 0, 255);
  static AutoMap autoWhiteToUINT8_T(0, RGBCIR.getResolution(), 0, 255);

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
      // enableButton2Mode = false;
      brightnessIterator = (++brightnessIterator) % NUM_BRIGHTNESS_LEVELS;
      // analogWrite(LED_PIN, LEDBrightnessLevels[brightnessIterator]);
      analogWrite(LED_PIN, getBrightness(brightnessIterator));
      // currentButtonMode = 0;
      break;
    case 1: // middle button, scale selector
      // enableButton2Mode = false;
      scaleContainer.nextScale();
      currentScale = scaleContainer.selected();
      
      // scaleContainer.scaleSelector = (scaleContainer.scaleSelector + 1) % scaleContainer.numScales;
      // currentScale = scaleContainer.scaleArray[scaleContainer.scaleSelector];
      // numNotesInScale = scaleContainer.numNotesInSelectedScale[scaleContainer.scaleSelector];
      SERIAL_TABS(2);
      SERIAL_PRINT("scale: ");
      SERIAL_PRINTLN(scaleContainer.scaleSelector);
      // currentButtonMode = 1;
      break;
    case 2: // right button, all pizzicato I think, instead of sustained notes
      enableButton2Mode = !enableButton2Mode;
      // currentButtonMode = 2;
      break;
    default:
      break;
    }
  }

  // this timer prevents the buttons from being updated too frequently
  if (buttonTimer.ready())
  {
    nextButtonPressAllowed = true;
  }

  // update the sensors. This updates one sensor at a time whenever the timer is ready. Originally I just updated all three
  // sensors simultaneously and got similar performance, so I'm not sure if this is actually better or not. Worth experimenting
  // with to see if one big update or spread out updates lets you achieve more frequent updates without glitching sound output.
  static uint8_t currentSensorToUpdate = 0, tableStoppedCount = 0;

  // set the table speed based on the potentiometer position
  static int16_t targetTableSpeed = convertPotValToTableSpeed(mozziAnalogRead<10>(POT_B_PIN));
  // static int16_t savedTableSpeed = targetTableSpeed;
  static bool tableStopped = false;

  // read the target table speed, if the table hasn't been stopped
  if (!tableStopped)
    targetTableSpeed = convertPotValToTableSpeed(mozziAnalogRead<10>(POT_B_PIN));

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
      updateColorReadings(&colorData);
      // scaleColorData(&colorData);       // scale the blue and red channels to bring them in line with green channel (essential white balance correction)
      scaleColorDataFixedPoint(&colorData, &scaledFixedColorData);
      // printColorData();
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
  mappedGreen = autoGreenToUINT8_T(scaledFixedColorData.greenFixed.asInt());
  mappedBlue = autoBlueToUINT8_T(scaledFixedColorData.blueFixed.asInt());
  mappedRed = autoRedToUINT8_T(scaledFixedColorData.redFixed.asInt());
  mappedWhite = autoWhiteToUINT8_T(scaledFixedColorData.clearFixed.asInt());

  // call the function that will be used to convert color data to sound
  ambienceGenerator();
  // toneBeatsGenerator();

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



// Converts a MIDI note number into a string (const char*) note name. E.g., 42 -> F#2
const char *MIDINoteToNoteName(uint8_t note)
{
  // Note names for one octave
  const char *noteNames[] = {
      "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"};

  if (note > 127)
  {
    return "Invalid"; // Return an error string for invalid MIDI numbers
  }

  // Determine octave
  int8_t octave = (note / 12) - 1;
  uint8_t noteIndex = note % 12;

  // Allocate a static buffer to store the note string
  static char noteStr[6]; // Max length: "A#-1" + null terminator = 5 bytes
  snprintf(noteStr, sizeof(noteStr), "%s%d", noteNames[noteIndex], octave);

  return noteStr;
}


void setFreqsFromChord(const Chord &chord, UFix<12, 15> &f1, UFix<12, 15> &f2, UFix<12, 15> &f3, UFix<12, 15> &f4)
{
    UFix<12, 15>* freqs[] = {&f1, &f2, &f3, &f4};
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t note = chord.getNote(i);
        *freqs[i] = (note < 128) ? mtof(UFix<7, 0>(note)) : 0;
    }
}



/*
void setFreqsFromChord(const Chord &chord, UFix<12, 15> &f1, UFix<12, 15> &f2, UFix<12, 15> &f3, UFix<12, 15> &f4)
{
  uint8_t note = noteNameToMIDINote(chord.notes[0]);
  if (note < 128)
  {
    f1 = mtof(UFix<7, 0>(note));
  }
  else
  {
    f1 = 0;
  }

  note = noteNameToMIDINote(chord.notes[1]);
  if (note < 128)
  {
    f2 = mtof(UFix<7, 0>(note));
  }
  else
  {
    f2 = 0;
  }

  note = noteNameToMIDINote(chord.notes[2]);
  if (note < 128)
  {
    f3 = mtof(UFix<7, 0>(note));
  }
  else
  {
    f3 = 0;
  }

  note = noteNameToMIDINote(chord.notes[3]);
  if (note < 128)
  {
    f4 = mtof(UFix<7, 0>(note));
  }
  else
  {
    f4 = 0;
  }
}
*/

// pass in a list of notes and get back a single note, e.g.:
// const char* arpeggio[] = {"A4", "C#2", "D5"};
// const char* newNote = getNoteFromArpeggio(arpeggio, 3, 0);   // should result in newNote equaling "A4"
const char *getNoteFromArpeggio(const char *notes[], uint8_t numNotes, uint8_t selector)
{
  if (selector < numNotes)
  {
    return notes[selector];
  }
  else
  {
    return " ";
  }
}

// takes an input value (range 0 to 127) and snaps it to the nearest note in a provided scale
// pass the notes in as MIDI note numbers, not note names
uint8_t snapToNearestNote(uint8_t inputValue, const uint8_t notes[], uint8_t numNotes)
{
  uint8_t minimumDistance = 255; // The smallest distance found between inputValue and a MIDI note
  uint8_t minDistanceNote = 0;   // The corresponding nearest MIDI note

  // Iterate through the scale notes to find the nearest one
  for (uint8_t i = 0; i < numNotes; i++)
  {
    uint8_t distance = abs(inputValue - notes[i]);

    if (distance < minimumDistance)
    {
      minimumDistance = distance;
      minDistanceNote = notes[i];
    }
  }

  return minDistanceNote;
}

// convert an array of note names into an array of MIDI note numbers
void convertArray_NoteNamesToNumbers(const char *noteNames[], uint8_t numNotes, uint8_t midiNotes[])
{
  for (uint8_t i = 0; i < numNotes; i++)
  {
    midiNotes[i] = noteNameToMIDINote(noteNames[i]);
  }
}

void convertArray_NoteNumbersToNames(const uint8_t midiNotes[], uint8_t numNotes, const char *noteNames[])
{
  for (uint8_t i = 0; i < numNotes; i++)
  {
    noteNames[i] = MIDINoteToNoteName(midiNotes[i]);
  }
}

int16_t convertPotValToArmRadius(uint16_t potVal)
{
  constexpr uint8_t DEADBAND = 15;
  potVal = (potVal < 512 + DEADBAND && potVal > 512 - DEADBAND) ? 512 : potVal; // add a bit of deadband
  // SERIAL_TAB;
  // SERIAL_PRINTLN(potVal);
  return map(potVal, 1023, 0, MAX_CONSTRAINED_RADIUS, -MAX_CONSTRAINED_RADIUS);
}

int16_t convertPotValToTableSpeed(int16_t potVal)
{
  constexpr uint8_t tableSpeedDeadband = 40;
  // create a deadband in the potentiometer reading to account for noise around the detent
  potVal = (potVal > 512 + tableSpeedDeadband || potVal < 512 - tableSpeedDeadband) ? potVal : 512;
  int16_t speed = map(potVal, 0, 1023, -255, 255);
  return speed;
}



void moveArmToAngle(int16_t targetAngle, int16_t currentAngle)
{
  static EventDelay accelTimer;
  static bool initialize = true;
  constexpr uint8_t accelerationMultiplier = 1; // value added to last motor speed to cause acceleration
  constexpr uint16_t accelerationInterval = 2;  // milliseconds between speed updates
  // the first time this function is called, we need to initialize the timer to run at the appropriate interval
  if (initialize)
  {
    accelTimer.set(accelerationInterval);
    initialize = false;
  }

  static int16_t lastMotorSpeed = 0, targetMotorSpeed = 0;
  constexpr uint8_t DEADBAND = 10; // if the current position is within this distance of the target, we reached the target
  constexpr uint8_t maxMotorSpeed = 140;
  static int8_t directionVector = 1;

  int16_t displacement = targetAngle - currentAngle;

  if (abs(displacement) <= DEADBAND)
  {
    targetMotorSpeed = 0;
  }
  else
  {
    if (accelTimer.ready())
    {
      directionVector = displacement > 0 ? 1 : -1;
      targetMotorSpeed = constrain((accelerationMultiplier * directionVector) + lastMotorSpeed, -1 * maxMotorSpeed, maxMotorSpeed);
      // SERIAL_PRINT(targetRadius);
      // SERIAL_TAB;
      // SERIAL_PRINT(currentRadius);
      // SERIAL_TAB;
      // SERIAL_PRINTLN(directionVector);
      accelTimer.start();
    }
  }
  armMotor.setSpeed(targetMotorSpeed);
  lastMotorSpeed = targetMotorSpeed;
}

void moveArmToRadius(int8_t targetRadius, int16_t currentAngle = 0)
{
  // I need to add a feature that cuts motor speed to 0 when end of range of arm motion is reached. I think sometimes it exceeds the
  // range that works properly for the angle to distance calculator and causes odd behavior.

  static EventDelay accelTimer;
  static bool initialize = true;
  constexpr uint8_t accelerationMultiplier = 4; // value added to last motor speed to cause acceleration
  constexpr uint16_t accelerationInterval = 1;  // milliseconds between speed updates
  // the first time this function is called, we need to initialize the timer to run at the appropriate interval
  if (initialize)
  {
    accelTimer.set(accelerationInterval);
    initialize = false;
  }

  static int16_t lastMotorSpeed = 0, targetMotorSpeed = 0;
  static int16_t targetAngle = 0;
  constexpr uint8_t DEADBAND = 10; // if the current position is within this distance of the target, we reached the target
  constexpr uint8_t maxMotorSpeed = 128;
  static int8_t directionVector = 1;

  // handle motor acceleration
  // figure out if the target is outside the deadband range of the current position
  // int16_t displacement = targetRadius - currentRadius;   // this needs to be int16_t so that it doesn't overflow (originally I was using int8_t and had errors)

  targetAngle = armRadiusToAngle(targetRadius);
  int16_t displacement = targetAngle - currentAngle;

  if (abs(displacement) <= DEADBAND)
  {
    targetMotorSpeed = 0;
  }
  else
  {
    if (accelTimer.ready())
    {
      directionVector = displacement > 0 ? 1 : -1;
      targetMotorSpeed = constrain((accelerationMultiplier * directionVector) + lastMotorSpeed, -1 * maxMotorSpeed, maxMotorSpeed);
      // SERIAL_PRINT(targetRadius);
      // SERIAL_TAB;
      // SERIAL_PRINT(currentRadius);
      // SERIAL_TAB;
      // SERIAL_PRINTLN(directionVector);
      accelTimer.start();
    }
  }
  armMotor.setSpeed(targetMotorSpeed);
  lastMotorSpeed = targetMotorSpeed;
}

/**
 * @brief Debounces the analog multiplexed buttons using Mozzi's EventDelay class.
 *
 * @details Uses time-based debouncing based on Mozzi's EventDelay class. I originally had a fancy timeless debouncing
 * scheme that used bit shifting values as a way to wait for the button press to stabilize after bouncing. It actually
 * worked pretty well, but I was still have problems with bounce during testing. I tried layering that system on top of
 * itself, but it was getting really arcane and hard to read and understand, so I switched to time-based debouncing. It
 * instantly worked better. We have to use Mozzi's alternatives to millis() for this because the timers in the Atmega328
 * are either claimed by Mozzi, or are running faster than normal because that removes what is otherwise an audible whine
 * from the motors. This means that all normal timing functions are entirely broken and we can't use them, but Mozzi offers
 * several alternatives in the form of ticks() and EventDelay.
 *
 * @param buttonPinVal the ADC reading of the pin that the buttons are connected to. Be sure to use mozziAnalogRead() and not
 * the stock Arduino analogRead(). analogRead() is actually quite slow.
 */
uint8_t getButtonPressed(uint16_t buttonPinVal)
{
  constexpr uint8_t DEBOUNCE_INTERVAL = 10;
  static EventDelay debounceTimer;
  static bool evaluatingPress = false;
  static uint8_t triggeredVal = 255, stableVal = 255;
  /**
   * this next part is where the magic happens. The buttons are multiplexed on a single analog pin. This works by
   * creating a resistor ladder, where the buttons connect the analog pin to a different point on the ladder.
   * This creates a unique voltage divider for each button, so whenever you press a button, the analog pin sees
   * a distinct voltage that represents which button was pressed. The downside of this is that only one button can
   * ever be pressed, and lower-numbered buttons will always have precedence. If you press and hold B1 and then press
   * B0, B0 will take over from B1. But the major upside of this is that you can put a bunch of buttons on a single
   * pin! As long as you only need to detect a single button being pressed at a time, this is a worthwhile trade.
   * This next line evaluates which button was pressed by checking the value reported by the ADC. I designed the
   * resistor ladder so that there's a roughly even spacing in ADC values between each button press (~340 counts
   * between each button). This requires using different resistor values at each point in the ladder, but makes
   * the code cleaner and more reliable. Otherwise you'd get logarithmically decreasing spacing between buttons
   * when represented as ADC counts.
   */
  uint8_t rawButton = ((buttonPinVal < 172) ? 0 : ((buttonPinVal < 510) ? 1 : ((buttonPinVal < 850 ? 2 : 255))));

  // now start the debounce logic if needed. Basically this looks for a state change and starts a timer.
  if (rawButton != triggeredVal && !evaluatingPress)
  {
    evaluatingPress = true;
    debounceTimer.set(DEBOUNCE_INTERVAL);
    debounceTimer.start();
    triggeredVal = rawButton;
  }

  // if the timer is up and we're evaluating a state change for possible button press, set the new stable state
  if (evaluatingPress && debounceTimer.ready())
  {
    // if the new reading is the same as the reading that triggered evaluation, set the stable state to the new reading.
    // otherwise, leave the stable state as is.
    stableVal = (rawButton == triggeredVal) ? triggeredVal : stableVal;
    evaluatingPress = false;
  }
  return stableVal;
}



void toneBeatsGenerator()
{
}

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
  // event delay for each osc
  // thresholds for new notes and selected scales
  SERIAL_PRINTLN(currentButtonMode);

  if (!enableButton2Mode && previousEnableButton2Mode)
    initialize = true; // transition out of button mode 2 requires resetting ADSRs

  if (initialize)
  {
    // scaleContainer.scaleSelector = 2;
    // currentScale = scaleContainer.scaleArray[scaleContainer.scaleSelector];
    // numNotesInScale = scaleContainer.numNotesInSelectedScale[scaleContainer.scaleSelector];
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
    { // new isntance of button mode 2
      // set all the ADSRs to be short and plucky
      // build random chords by selecting one note from scale for each oscillator
      // set up event delay for each note to be repeated, timing determined by color channel
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
            // osc0Params.noteMIDINumber = currentScale[colorToScaleNote7(mappedGreen)] + ((int8_t)rand(-1, 2) * 12);
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
            // osc0Params.noteMIDINumber = currentScale[colorToScaleNote7(mappedGreen)] + ((int8_t)rand(-1, 2) * 12);
            osc0Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote7(mappedGreen)) + ((int8_t)rand(-1, 2) * 12);
            break;
          case 5:
            osc0Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote5(mappedGreen)) + ((int8_t)rand(-1, 2) * 12);
            break;
          default:
            break;
        }
        // if (scaleContainer.selected().numNotes == 7)
        // {
        //   osc1Params.noteMIDINumber = currentScale[colorToScaleNote7(mappedBlue)] + ((int8_t)rand(-1, 2) * 12);
        // }
        // else
        // {
        //   osc1Params.noteMIDINumber = currentScale[colorToScaleNote5(mappedBlue)] + ((int8_t)rand(-1, 2) * 12);
        // }
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
            // osc0Params.noteMIDINumber = currentScale[colorToScaleNote7(mappedGreen)] + ((int8_t)rand(-1, 2) * 12);
            osc0Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote7(mappedGreen)) + ((int8_t)rand(-1, 2) * 12);
            break;
          case 5:
            osc0Params.noteMIDINumber = scaleContainer.selected().getNote(colorToScaleNote5(mappedGreen)) + ((int8_t)rand(-1, 2) * 12);
            break;
          default:
            break;
        }
        // if (scaleContainer.selected().numNotes == 7)
        // {
        //   osc2Params.noteMIDINumber = currentScale[colorToScaleNote7(mappedRed)] + ((int8_t)rand(-1, 2) * 12);
        // }
        // else
        // {
        //   osc2Params.noteMIDINumber = currentScale[colorToScaleNote5(mappedRed)] + ((int8_t)rand(-1, 2) * 12);
        // }
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
    // octaveShifter *= 12;  //make that actual midi note values by multiplying by 12 to get movement

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
    // osc0Params.noteMIDINumber = currentScale[i];
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
    // osc1Params.noteMIDINumber = scale_CLydianMIDI[(i + 2 ) % numNotesInScale] + octaveShifter;
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
      // osc1Params.noteMIDINumber = currentScale[(j + 4) % scaleContainer.selected().numNotes] - 12;
      break;
    case 1:
      osc1Params.noteMIDINumber = scaleContainer.selected().getNote((i + 2) % scaleContainer.selected().numNotes) + octaveShifter * 12;
      // osc1Params.noteMIDINumber = currentScale[(i + 2) % scaleContainer.selected().numNotes] + octaveShifter * 12;
      break;
    case 2:
      osc1Params.noteMIDINumber = scaleContainer.selected().getNote((i)) - 12;
      // osc1Params.noteMIDINumber = currentScale[j] - 12;
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

      // osc2Params.noteMIDINumber = scale_CLydianMIDI[k] + octaveShifter;
      osc2Params.noteMIDINumber = scaleContainer.selected().getNote((i + 3) % scaleContainer.selected().numNotes) + (octaveShifter * 12);
      // osc2Params.noteMIDINumber = currentScale[(i + 3) % scaleContainer.selected().numNotes] + (octaveShifter * 12); // pretty good
      // osc2Params.noteMIDINumber = scale_CLydianMIDI[(i + 3) % numNotesInScale] - 7;
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
        // arpDurationTimer.set(max(250, mappedWhite << 3));
        // arpDurationTimer.start();

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
        // osc2Params.noteMIDINumber = currentScale[arpIndex] + (12 * (int8_t)rand(-1, 3));
        osc2Params.frequency = mtof(osc2Params.noteMIDINumber);
        osc2Params.volume = 120;

        osc2Portamento.setTime(osc2PortTime);
        if (!USE_PORTAMENTO)
          osc2.setFreq(osc2Params.frequency);

        int8_t arpShift = rand(-5, 6);
        arpIndex += arpShift; // move to next note in sequence
                              // if (arpIndex < 0) {
                              //   arpIndex += numNotesInScale;
                              // } else if (arpIndex > numNotesInScale - 1) {
        //   arpIndex -= numNotesInScale;
        // }
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



/**
 * I just realized I've been trying to use the arpeggiator function like a class, where each oscillator can call to it independently and get a unique note back.
 * But that doesn't work, because the index is a static variable inside arpeggiator! So when I have one osc set to mode 0 (arp up), that increments the index
 * to return a higher note in the scale each time it's called. But if I have another osc set to mode 1 (arp down), this decrements the index to try to create a falling
 * arpeggio. These work fine on their own, but when both are active, they fight against each other over the index! This just made a kind of cool sound by accident that
 * I might keep, but the arpeggiator does actually need to be a class in the long term that isn't a singleton like this. Although that was fun.
 *
 */

/**
 * arpMode:
 * 0 - up loop
 * 1 - down loop
 * 2 - up-down-up loop
 * 3 - random
 * 4 - manual control of the note index
 *
 * arpSpread: how many octaves to add to the span of the baseline scale. setting to 1 will expand 1 octave higher.
 *
 * manualIndex: manually control which index in the array gets returned, instead of letting the automatic feature run. Value of
 * 255 means let automatic mode take over.
 *
 * offset: signed number of steps up or down the scale to offset the output from the input, if direct input, or from the automatic progression
 */
uint8_t arpeggiator(uint8_t numNotesInScale, const uint8_t *scaleNumbers, uint8_t manualIndex = 255, int8_t offset = 0, uint8_t arpMode = 0, uint8_t arpSpread = 0)
{
  static uint8_t index = 0;
  static uint8_t outputNote = 0;
  index %= numNotesInScale; // safety feature to prevent overflowing array

  switch (arpMode)
  {
  case 0:
    outputNote = scaleNumbers[index];
    ++index %= numNotesInScale; // always wrap around at the end of the arpeggio
    break;
  case 1:
    outputNote = scaleNumbers[index];
    index = (index == 0) ? numNotesInScale - 1 : index - 1;
    // --index;
    // index %= numNotesInScale;
    break;
  case 2:
    break;
  case 3:
    break;
  case 4:
    manualIndex = (manualIndex + offset) % numNotesInScale;
    outputNote = scaleNumbers[manualIndex];
  default:
    break;
  }

  return outputNote;
}