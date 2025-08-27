/**
 * To Do: 
 * - scale the color data using FixMath operations instead of fancy bit shifting stuff. will make it easier for users to tune the scaling if they want
 * - try not using PID again for the arm controller
 * - try a version where the arm moves in 20mm wide tracks across the table, set by the knob
 * - I figured out that the volume variables were type char in the example, which is int8_t. Using uint8_t increases overall volume, but also starts
 *    the synth with obnoxious loud sounds immediately. 
 * - figure out why the arm positioning math isn't working correctly
 * - come up with more interesting synth sounds
 * - instead of just setting the frequencies of three oscs, use color channels to change chords and things
 * - trying sampling color data as a waveform
 */




// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.
// It has a few side effects. It breaks any use of millis(). delay() also will not work, but I plan to avoid
// the use of both of those any time Mozzi is running anyway, so this might be fine! So far this hasn't affected
// my very basic Mozzi sketch.
// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.


#define USE_FAST_PWM 1        // set to 1 to change the PWM clock divisor for pins 5 and 6 (timer 0) - removes motor noise from audio
#define USE_LED_PWM  1        // set to 1 to change the PWM clock divisor for pins 3 and 11 (timer 2) - removes PWM noise due to LED dimming from audio


#include <Arduino.h>
#include <Crunchlabs_DRV8835.h>     // motor driver
#include <Wire.h>                   // I2C
#include <PWMFreak.h>               // Changes clock rate of timers (used to remove audible noise from motor driver and LED dimming)
#define MOZZI_CONTROL_RATE 128      // Frequency of updateControls() calls
#include <Mozzi.h>                  // Main synthesizer library
#include <Oscil.h>                  // Oscillator
#include <tables/saw2048_int8.h>    // Saw wavetable
#include <IntMap.h>                 // A more efficient replacement for map()
#include <EventDelay.h>             // Mozzi library for performing actions at specific time intervals without delay() or millis()
#include <mozzi_utils.h>
#include <mozzi_rand.h>
#include <mozzi_midi.h>             // MIDI functionality, used for mtof(), which converts MIDI note numbers to frequencies
#include <CLS16D24.h>               // Color sensor
#include <AS5600.h>                 // Magnetic encoders for the arm and table positions
#include <DCfilter.h>               // DC filter used to detect changes in a signal (used for recognizing when arm and table stop moving)
#include <FastPID.h>                // Fast fixed point math PID implementation used for controlling arm position - might remove
#include <AutoMap.h>                // A version of map() that is auto-ranging. Used for mapping color values to control signals.


/**
 * This is a macro for easily enabling or disabling the Serial monitor print statements. 
 * 
 * You can enable serial print debugging by setting 
 *    #define USE_SERIAL 1
 * 
 * Or, you can disable serial print debugging by setting
 *    #define USE_SERIAL 0
 * 
 * In your main code, rather than using Serial.print() or Serial.println(), use their aliases defined below (e.g., SERIAL_PRINTLN()):
 *  */ 
#define USE_SERIAL 1

#define USE_SERIAL 1

#if USE_SERIAL
  #define SERIAL_PRINT(x)     Serial.print(x)
  #define SERIAL_PRINTLN(x)   Serial.println(x)
  #define SERIAL_BEGIN(baud)  Serial.begin(baud)
  #define SERIAL_TAB          Serial.print("\t")
  #define SERIAL_TABS(x)      for (uint8_t i = 0; i < x; i++) {Serial.print("\t");}
#else
  #define SERIAL_PRINT(x)     do {} while (0)
  #define SERIAL_PRINTLN(x)   do {} while (0)
  #define SERIAL_BEGIN(baud)  do {} while (0)
  #define SERIAL_TAB          do {} while (0)
  #define SERIAL_TABS         do {} while (0)
#endif



// **********************************************************************************
// Arm and Table stuff
// **********************************************************************************


constexpr uint8_t TABLE_SPEED_PIN = 5;
constexpr uint8_t TABLE_DIR_PIN = 4;
constexpr uint8_t ARM_SPEED_PIN = 6;
constexpr uint8_t ARM_DIR_PIN = 7;
constexpr uint8_t LED_PIN = 11;


DRV8835 tableMotor(TABLE_SPEED_PIN, TABLE_DIR_PIN, 50, true);
DRV8835 armMotor(ARM_SPEED_PIN, ARM_DIR_PIN, 50, true);


void homeArm(int16_t startingPosition = 0);


int16_t armRadiusToAngle(int16_t radiusMM);     // convert arm radius in millimeters to angle in encoder counts
int16_t armAngleToRadius(int16_t angleCounts);  // convert arm angle in encoder counts to radius in millimeters
int16_t convertPotValToArmRadius(uint16_t potVal);
int16_t convertPotValToTableSpeed(int16_t potVal);
void moveArmToRadius(int8_t targetRadius, int8_t currentRadius);

AS5600L tableEncoder;
AS5600 armEncoder;

int8_t currentArmPosition = 0; // current arm position in millimeters from center of table

DCfilter armDCFilter(0.95);          // DC filter detects changes in arm position (settles to 0 if the arm is not moving)
DCfilter tableDCFilter(0.6);        // DC filter for table movement
constexpr int8_t DCMovementThreshold = 30;   // If the DC filter shows a value between + and - DCMovementThreshold, we know that axis is not moving
int16_t tableDC = 0;                // the value of the DC filter for table movement.



// **********************************************************************************
// Color sensor
// **********************************************************************************

CLS16D24 RGBCIR;
uint16_t conversionTime = 0;      // time in milliseconds required for the color sensor to acquire new data - changes based on resolution settings. this variable is not currently used.

enum class ColorChannels {
  RED,
  GREEN,
  BLUE,
  CLEAR,
  IR
};

ColorChannels currentColorChannel = ColorChannels::RED;

// struct for storing the raw color value readings from the sensor as uint16_t
struct ColorValues {
  uint16_t red = 0;
  uint16_t green = 0;
  uint16_t blue = 0;
  uint16_t clear = 0;
  uint16_t IR = 0;
};

ColorValues colorData;      // struct for the raw color data


struct FixedPointColorValues {
  UFix<16, 0> redFixed = 0;
  UFix<16, 0> greenFixed = 0;  
  UFix<16, 0> blueFixed = 0;
  UFix<16, 0> clearFixed = 0;
  UFix<16, 0> IRFixed = 0;  
};

// struct for storing the color data as fixed point values after they have been white balance corrected (after scaleColorDataFixedPoint() is applied to the raw colorData)
FixedPointColorValues scaledFixedColorData;


bool updateChannels[5] = {false, false, false, false, false}; // Flags to control which channels to update. defaults to all five off.

uint8_t mappedGreen = 0, mappedBlue = 0, mappedRed = 0;   // where color data that has been mapped into control signals will be stored

bool updateColorReadings(ColorValues *colorReadings);
void printColorData();

void scaleColorData(ColorValues *rawData);      // used to scale the RGB values relative to each other a bit
void scaleColorDataFixedPoint(ColorValues *rawData, FixedPointColorValues *scaledVals);   // scales raw sensor values and returns them as fixed point math values instead of uint16_t

// delay timer for updating sensor data and PID controller for arm
EventDelay k_i2cUpdateDelay, k_PIDupdate;
constexpr uint8_t I2C_UPDATE_INTERVAL = 50;       // time in milliseconds

// used to only update PID every few cycles of updateControl(). More frequent updates lead to better movement control, but take up too much processing power
const uint8_t PID_DIVISOR = 1;                    
constexpr uint8_t PID_HZ = MOZZI_CONTROL_RATE / (float)PID_DIVISOR;

constexpr uint8_t NUM_BRIGHTNESS_LEVELS = 5;
uint8_t LEDBrightnessLevels[NUM_BRIGHTNESS_LEVELS] = {0, 32, 64, 192, 255};
uint8_t brightnessIterator = 3; // default to brightness level 3 on startup


// **********************************************************************************
// Potentiometers and switch
// **********************************************************************************

constexpr uint8_t POT_A_PIN = A0;
constexpr uint8_t POT_B_PIN = A1;
constexpr uint8_t AUDIO_IN_PIN = A2;
constexpr uint8_t BUTTONS_PIN = A3;

uint8_t buttonPressed = 255;                        // 255 means no button pressed, 0, 1 or 2 indicates which of the three buttons was pressed
uint8_t getButtonPressed(uint16_t buttonPinVal);    // pass an analog reading on BUTTON_PIN into the parameter. Returns 1, 2, or 3 if one of those buttons is pressed, otherwise returns 255.
uint8_t getDebouncedButton();                       // performs extra layer of debouncing

// **********************************************************************************
// Mozzi stuff
// **********************************************************************************

// Oscil Wash example sketch stuff

Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSaw2(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSaw3(SAW2048_DATA);
Oscil<SAW2048_NUM_CELLS, MOZZI_AUDIO_RATE> aSaw4(SAW2048_DATA);

// audio volumes updated each control interrupt and reused in audio till next control
// well this is wild. The original example sketch I built this from used char as the datatype, which is not something that
// should ever be used except for storing characters of text. A lot of older Arduino sketches use char when a more appropriate
// datatype would be byte, or even better, uint8_t. The problem with char is that it's not rigorously defined. On some 
// platforms like ARM, it's usually an unsigned 8 bit integer, but apparently, on the AVR microcontrollers in the Arduino 
// environment, it's actually a signed 8 bit integer. So for this program, I have been mapping color data into volume controls
// into a range of 0 - 255, as though volume were a uint8_t. But it's actually been overflowing and wrapping around to the range
// -127 to 127 to fit into char/int8_t. For now, I'm actually going to leave the volume variables as int8_t. It produces a nice
// effect where the synth doesn't really start producing sound until the AutoMaps have picked up enough data to start ranging
// correctly for the color data, since the volume gets centered on 0. I can set this variable to uint8_t, and one nice effect
// of that is that the volume is much louder, but it also starts making noise immediately. It's unpleasant sounding until the
// AutoMaps kick in and start ranging things correctly. I could fix this by adding some kind of volume fade that runs when the
// program boots up, but I like that with int8_t, it's adaptive to the actual color data it receives and isn't time based.
int8_t v2, v3, v4;

// **********************************************************************************
// Music stuff
// **********************************************************************************

// an array of 4 pointers to const char* strings that define up to four notes in a chord
struct Chord {
  const char* notes[4];
};

// chord progression vi-I-IV-iii
Chord Am = {"C3", "E3", "A3", " "};
Chord Am7 = {"C3", "E3", "A3", "G4"};

Chord C = {"E3," "G3", "C4", " "};
Chord Cadd9 = {"E3," "G3", "C4", "D4"};

Chord F = {"C3", "F3", "A3", " "};
Chord Fm6 = {"C4", "F3", "G#3", "D4"};

Chord Em = {"E3", "G3", "B3", " "};
Chord Em7 = {"E3", "G3", "B3", "D4"};

Chord chordProgression[4] = {Am, C, F, Em};
Chord chordProgressionVaried[4] = {Am7, Cadd9, Fm6, Em7};
Chord chordProgressionCombined[8] = {Am, Am7, C, Cadd9, F, Fm6, Em, Em7};     // might do a probabilistic version of this where it's like 75% the normal notes, 25% the augmented ones

void convertArray_NoteNumbersToNames(const uint8_t midiNotes[], uint8_t numNotes, const char* noteNames[]);
void convertArray_NoteNamesToNumbers(const char* noteNames[], uint8_t numNotes, uint8_t midiNotes[]);
uint8_t snapToNearestNote(uint8_t inputValue, const uint8_t notes[], uint8_t numNotes);
void setFreqsFromChord(const Chord& chord, UFix<12,15>& f1, UFix<12,15>& f2, UFix<12,15>& f3, UFix<12,15>& f4);
const char* getNoteFromArpeggio(const char* notes[], uint8_t numNotes, uint8_t selector);
uint8_t noteNameToMIDINote(const char* noteName);          // convert note names to MIDI note numbers (e.g., F#2 -> 42)
const char* MIDINoteToNoteName(uint8_t note);     // convert MIDI note to note name (e.g., 42 -> F#2)


const char* testArp[8] = {"C2", "E3", "A3", "G4", "C4", "F4", "G#4", "D5"};

const uint8_t numNotesInScale = 15;
const char* scale_EbPentatonicMinor[numNotesInScale] = {"D#2", "F#2", "G#2", "A#2", "C#3", "D#3", "F#3", "G#3", "A#3", "C#4", "D#4", "F#4", "G#4", "A#4", "C#5"};
uint8_t scaleNumbers_EbPentatonicMinor[numNotesInScale];



// **********************************************************************************
// Setup
// **********************************************************************************

void setup() 
{
  if (USE_SERIAL) Serial.begin(115200);
  SERIAL_PRINTLN("starting");

  // start the I2C bus
  Wire.begin(); 

  tableEncoder.begin();
  tableEncoder.resetCumulativePosition();    // calibrate to 0 as starting position for the table
  armEncoder.begin();
  
  tableEncoder.setHysteresis(11);            // value has to change by +- 3 before getting reported as change by sensor
  armEncoder.setHysteresis(11);              // helps remove noise from sensor data


  // Now home the arm. This moves the arm back to the center of the table by the end of the routine,
  // and then resets the arm encoder position to 0.
  homeArm(armRadiusToAngle(convertPotValToArmRadius(analogRead(POT_A_PIN))));

  currentArmPosition = armAngleToRadius(armEncoder.getCumulativePosition());


  if (!RGBCIR.begin(false)) {          // set parameter to true to use i2c fast mode (400kHz instead of 100kHz)                           
    SERIAL_PRINTLN("ERROR: couldn't detect the sensor");
    while (1){}            
  }
  // IMPORTANT: Set the I2C clock to 400kHz fast mode AFTER initializing the connection to the sensor.
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
  */
  RGBCIR.setResolutionAndConversionTime(0x02);
  SERIAL_PRINT("Conversion time: ");
  conversionTime = RGBCIR.getConversionTimeMillis();
  SERIAL_PRINTLN(conversionTime);  // Calculates the conversion time determined by setResolutionAndConversionTime()
  SERIAL_PRINT("Resolution: ");
  SERIAL_PRINTLN(RGBCIR.getResolution());            // Calculates resolution determined by setResolutionAndConversionTime()

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

  
  if (USE_FAST_PWM) {
    // use fast PWM to remove audible noise from driving the motors. 
    // IMPORTANT: This breaks millis() and delay(), and may have other effects I haven't
    // found yet. After this line, you can't use millis() or delay() calls and have them
    // behave as expected.
    setPwmFrequency(5, 1);    // sets Timer0 clock divisor to 1 instead of 64. This moves the motor control pins above audible range
  }
  
  // EXPERIMENTAL:
  // I noticed that if I try to use PWM dimming for the LEDs on the color sensor I get audible noise, so this is experimental.
  // There's a chance that changing Timer 2's clock divisor will mess up Mozzi functions I'm not aware of. So far it hasn't though.
  if (USE_LED_PWM) {
    setPwmFrequency(11, 1);   // set Timer2 to clock divisor of 1 (clock rate 31250Hz now, above audible range)
    analogWrite(LED_PIN, LEDBrightnessLevels[brightnessIterator]);    // set the brightness for the LEDs
  } else {
    digitalWrite(LED_PIN, HIGH);  // just turn the color sensor LEDs on at full brightness
  }

  // the scaleNumers_EbPentatonicMinor array needs to be initialized
  convertArray_NoteNamesToNumbers(scale_EbPentatonicMinor, numNotesInScale, scaleNumbers_EbPentatonicMinor);
  
  // Oscil wash example sketch stuff
  // set harmonic frequencies
  aSaw2.setFreq(mtof(noteNameToMIDINote("E2")));
  aSaw3.setFreq(mtof(noteNameToMIDINote("A3")));
  aSaw4.setFreq(mtof(noteNameToMIDINote("B4")));
  v2 = v3 = v4 = 127;
  
  // finally, start the timers
  k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);      // control how frequently we poll the sensors on the I2C bus (color sensor, both encoders)
  k_PIDupdate.set(1000 / PID_HZ);                 // update rate for the PID controller that manages the arm position
  
  // start Mozzi
  startMozzi(MOZZI_CONTROL_RATE);
}





// **********************************************************************************
// updateControl for Mozzi
// **********************************************************************************

void updateControl() {
  static int8_t targetArmPos = 80;
  static bool initialize = true;
  static EventDelay reactToDCTimer, buttonTimer;
  if (initialize) {
    reactToDCTimer.set(750);
    reactToDCTimer.start();
    buttonTimer.set(100);    //125ms is exactly 1/16th notes for 120bpm in 4/4
    buttonTimer.start();
    initialize = false;
  }

  // these are auto-ranging mappings for the color channels. These work a lot like the normal map() function, except that they keep track of the 
  // largest and smallest values that they have seen so far, and update the map to reflect those. So the first two parameters are the minimum and
  // maximum possible input values (if this were a standard analogRead(), that would be 0 and 1023). As you update the function, it keeps track of
  // the mapping value you provide it, and the min and max values it has seen become the new min and max values for the input map range. Note that
  // the parameters are ints, so signed 16 bit values, and you have to be sure the numbers you pass into the parameters will fit in that variable size.
  //
  // The reason these are static variables inside updateControl() instead of being global is that I wanted to use the RGBCIR.getResolution() function
  // to be able to set the size of the mapping, rather than hardcoding the value. That way the maps get dynamically resized based on the chosen 
  // resolution of the sensor. 
  static AutoMap autoGreenToVolume(0, RGBCIR.getResolution(), 0, 255);
  static AutoMap autoBlueToVolume(0, RGBCIR.getResolution(), 0, 255);
  static AutoMap autoRedToVolume(0, RGBCIR.getResolution(), 0, 255);

  // check to see if buttons are pressed. returns 0, 1, 2, 255
  // 0 = B1, 1 = B2, 2 = B3, 255 = no press
  // deal with the button presses
  buttonPressed = getButtonPressed(mozziAnalogRead<10>(BUTTONS_PIN));
  SERIAL_PRINTLN(buttonPressed);
  if (buttonTimer.ready()) {
    switch (buttonPressed) {
      case 0:
        brightnessIterator = (++brightnessIterator) % NUM_BRIGHTNESS_LEVELS;
        analogWrite(LED_PIN, LEDBrightnessLevels[brightnessIterator]);
        break;
      case 1:
        break;
      case 2:
        break;
        default:
        break;     
      }
    buttonTimer.start();
  }



  if (k_i2cUpdateDelay.ready()) {
    currentArmPosition = armAngleToRadius(armEncoder.getCumulativePosition());
    tableDC = tableDCFilter.next(tableEncoder.getCumulativePosition() * 4);
    updateColorReadings(&colorData);
    // scaleColorData(&colorData);       // scale the blue and red channels to bring them in line with green channel (essential white balance correction)
    k_i2cUpdateDelay.start();            // restart the timer immediately after new color data is acquired
    scaleColorDataFixedPoint(&colorData, &scaledFixedColorData);
    // printColorData();
  }
  
  // if (k_PIDupdate.ready()) {
  //   targetArmPos = convertPotValToArmRadius(mozziAnalogRead<10>(POT_A_PIN));
  //   moveArmToRadius(targetArmPos, currentArmPosition);
  //   k_PIDupdate.start();
  // }
  
  targetArmPos = convertPotValToArmRadius(mozziAnalogRead<10>(POT_A_PIN));
  moveArmToRadius(targetArmPos, currentArmPosition);


  // now deal with controlling the table motion. Eventually I'm going to use the DC filter to watch the motion of the table.
  // If the program expects it to be moving, but the DC filter shows that it has stopped, the program will stop the drive
  // motor. Basically this will let you dynamically stop the table by grabbing it. If the table then moves again, the program
  // will start the table motor up.  
  int16_t targetTableSpeed = convertPotValToTableSpeed(mozziAnalogRead<10>(POT_B_PIN));
  static int16_t savedTableSpeed = targetTableSpeed;

  tableDC = (tableDC < -DCMovementThreshold || tableDC > DCMovementThreshold) ? tableDC : 0;    // add some hysteresis
  // SERIAL_PRINT(tableDC);

  if (reactToDCTimer.ready()) {
    if (tableDC == 0) {
      savedTableSpeed = targetTableSpeed;
      targetTableSpeed = 0;
    } else {
      if (targetTableSpeed == 0) {
        targetTableSpeed = savedTableSpeed;
        reactToDCTimer.start();
      }
    } 
  }
  // SERIAL_TAB;
  // SERIAL_PRINTLN(targetTableSpeed);

  tableMotor.setSpeed(targetTableSpeed);
  // tableMotor.setSpeed(targetTableSpeed);


  // AutoMap instances that handle the color data mapping to control signals
  mappedGreen = autoGreenToVolume(scaledFixedColorData.greenFixed.asInt());
  mappedBlue = autoBlueToVolume(scaledFixedColorData.blueFixed.asInt());
  mappedRed = autoRedToVolume(scaledFixedColorData.redFixed.asInt());

  // oscil wash example sketch stuff
  v2 = mappedGreen;
  v3 = mappedBlue;
  v4 = mappedRed;

  aSaw2.setFreq(mtof(snapToNearestNote((mappedBlue >> 2) + 24, scaleNumbers_EbPentatonicMinor, numNotesInScale)));
  aSaw3.setFreq(mtof(snapToNearestNote((mappedRed >> 2) + 24, scaleNumbers_EbPentatonicMinor, numNotesInScale)));
  aSaw4.setFreq(mtof(snapToNearestNote((mappedGreen >> 2) + 24, scaleNumbers_EbPentatonicMinor, numNotesInScale)));
}






// **********************************************************************************
// updateAudio for Mozzi
// **********************************************************************************

AudioOutput_t updateAudio() {
  // oscil wash example sketch stuff
  int32_t asig = (int32_t)
    aSaw2.next() * v2 + 
    aSaw3.next() * v3 +
    aSaw4.next() * v4;

  return MonoOutput::fromAlmostNBit(17, asig);
}


// **********************************************************************************
// Loop
// **********************************************************************************

void loop() {
  // synthesize new audio for Mozzi
  audioHook();
}



// **********************************************************************************
// Function Definitions
// **********************************************************************************

void homeArm(int16_t startingPosition) {
  SERIAL_PRINTLN("starting homing");
  // start the arm moving
  armMotor.setSpeed(128);
  // depending on where the arm was, there may be backlash. wait for that to be taken up before watching for the arm to stop moving.

  delay(2000);
  SERIAL_PRINTLN("moving");
  // now we know that the backlash was taken up and the arm is moving. Or it's already against the stop, but we'll detect that next either way.

  armEncoder.getCumulativePosition();
  // I found that for the DC filter for the arm, I need to scale the encoder position by a factor of 1000. Otherwise the DC filter just hovers
  // around +- 1, and doesn't seems sensitive enough at this speed of movement. Or like it settles too fast or something. scaling sort of 
  // fixed it for now, and I also added a super rough low pass filter by implementing prevArmDC. This way two DC filter measurements in a row
  // have to indicate that the arm has stopped for it to actually stop. Could add a real LP filter on top of the DC filter if needed, but this
  // seems to work for now. Actually I wound up adding a small array to sort of average out the DC. This seems to be better. 
  int16_t armDC = armDCFilter.next(1000 * armEncoder.getCumulativePosition());
  constexpr uint8_t armDCarrSize = 5;
  int16_t armDCLP[armDCarrSize] = {armDC, 1000, 1000, 1000, 1000};
  uint8_t armDCiter = 1;

  int16_t armDCSum = 0;
  for (int i = 0; i < armDCarrSize; i++) {
    armDCSum += armDCLP[i];
  }
  while (armDCSum < -DCMovementThreshold || armDCSum > DCMovementThreshold) {     // while the arm is moving
    // SERIAL_PRINT(armEncoder.getCumulativePosition()); SERIAL_PRINT("      ");
    armDC = armDCFilter.next(1000 * armEncoder.getCumulativePosition());
    armDCLP[armDCiter] = armDC;
    armDCiter = (++armDCiter) % armDCarrSize;
    armDCSum = 0;
    for (int i = 0; i < armDCarrSize; i++) {
      armDCSum += armDCLP[i];
    }
    // SERIAL_PRINTLN(armDC);
  }
  SERIAL_PRINTLN("homing finished");
  armMotor.setSpeed(0);
  armEncoder.resetCumulativePosition(661);
  delay(1000);
}

/**
 * @brief converts an arm radius (from center of table in millimeters) to the angle in encoder counts that the arm needs to be at to reach that radius.
 * 
 * @param radiusMM the radius in millimeters
 * 
 * @return the angle in encoder counts that corresponds to the input radius
 * 
 * @paragraph The angle is defined from the line that connects the center of the table and the center of the arm gear. As the angle
 * gets more positive, the arm moves toward its vertical position. Negative angles move the other side of the center
 * line, bringing the sensor arm down toward the knobs and buttons. 
 * 
 * This function is an approximation, but a pretty good one. At worst, it will have an error of 1 degree. The actual function that
 * precisely converts radius to angle in encoder counts (4096 per revolution) is: 
 * 
 * angle = (4096 / pi) * arcsin(radiusMM / 216)
 * 
 * This radius is actually the chord length of the arc described by the sensor arm, starting from the point where that arc is
 * coincident with the table. The above function is derived from the formula for chord length. It's nice because it's exact, but it's
 * terrible for use in this context because it has floating point numbers (pi) and trig functions (arcsine). The ATMega328P is 
 * extremely slow at performing floating point operations and trigonometric functions. It only has hardware for adding, subtracting,
 * and multiplying integers, as well as hardware for performing bitwise operations directly on binary numbers, so if you need any 
 * other math operation, it gets built from those basic operations repeated a bunch of times, making it really slow. In situations
 * like this, where we need as much processing power as possible for running Mozzi, it's worth trying to find good linear approximations
 * of the actual math function that only use those basic operations. The approximation I derived is: 
 * 
 * angle = (774 * radiusMM) >> 7;
 * 
 * That's equivalent to (774 * radiusMM) / 128. The denominator of 128 is nice because that's 2^7, so instead of performing the slow
 * operation of dividing by 128, we can perform the extremely fast operation of bitshifting right by 7 places. 
 * 
 * One final note on this is that bit shift operations are not valid on signed integers (those that can contain negative values). 
 * You can perform the operation and move the bits, but it won't yield the mathematical results you expect. That's because one of the
 * bits is the sign bits, and tells the processor if the number is negative or not. Performing a bit shift moves that sign bit and messes
 * up the math. So the function implementation takes the absolute value of the radius parameter and puts that in an unsigned integer
 * so that bit shifting will work. At some point it could be interesting to test various versions of this to see which is fastest. 
 * Shifting by powers of 2 is faster than shifting by other numbers, so there's a chance that doing something like the following could be
 * faster than what I have now:
 * 
 * return ((774 * absRadius) >> 8) << 1;
 * 
 * Or maybe just forgo the intermediate step of using absolute value and unsigned integers, and just divide the signed radius value by 128.
 * 
 * If you want you can see a spreadsheet that I made to help figure all this out. It's not particularly well organized, and it's missing
 * the original math that I did by hand, as well as the Desmos graphs I made to check my reasoning, but it should help get some of the 
 * concepts clarified. https://docs.google.com/spreadsheets/d/1RaxqJCClSnBzAPjxKA0sKCvFVXpXQ3Gc1AZQWVGnURM/edit?usp=sharing
 */
int16_t armRadiusToAngle(int16_t radiusMM) {
  uint16_t absRadius = abs(radiusMM);   // bit shifting operations on signed integers create weird results, so we have to use an unsigned int
  if (radiusMM < 0) {
    return -1 * ((774 * absRadius) >> 7);
  } else {
    return (774 * absRadius) >> 7;
  }
}


/**
 * @brief converts an arm angle in encoder counts to the corresponding radius of the sensor from the center of the table in millimeters. 
 * 
 * @param angleCounts the angle in encoder counts (4096 per revolution). 
 * 
 * @return the radius of the sensor from the center of the table in millimeters. 
 * 
 * @paragraph This is similar to the function armRadiusToAngle() in which I used a linear integer approximation of a function that
 * relies on floating point numbers and trigonometry functions. The precise function that converts an angle to a radius is: 
 * 
 * radius = 216 * sin(angle * pi / 4096)
 * 
 * This can be pretty well approximated by the function
 * 
 * radius = 81 * angle >> 9
 * 
 * At worst, this approximation will produce a positional error of 2mm. This seems acceptable in this context, since backlash in the gears
 * and play in the arm height adjustment system produce more that 2mm of error, and this isn't a system where 2mm of error is going to make
 * a huge difference. The average and median error are .17mm and .42mm respectively in my sample set, which you can see in the spreadsheet 
 * linked above.
 * 
 * One important note about the linear approximations in both of these functions is that they only work because we're working with a restricted
 * domain. The sensor arm can only move through limited range of motion from about -44 degrees to 58 degrees. The functions that convert
 * angle to radius and radius to angle both look reasonably linear across that domain. However, if we were dealing with an unrestricted range
 * of motion (if the arm could spin in a full circle), these approximations wouldn't work at all, and we would need to look into other methods
 * of speeding up these math operations. If you're interested in this, you could look into piecewise linear functions and the closely related
 * concept of a lookup table with linear interpolation. You could also look into CORDIC (coordinate rotation digital computer) which is a common
 * efficient method for calculating things like trig functions. This kind of thing is used everywhere in software. Every video game you've ever
 * played has basically been a giant system of linear algebra and trig functions. 
 */
int16_t armAngleToRadius(int16_t angleCounts) {
  uint16_t absAngleCounts = abs(angleCounts);
  if (angleCounts < 0) {
    return -1 * ((81 * absAngleCounts) >> 9);
  } else {
    return (81 * absAngleCounts) >> 9;
  }
}




bool updateColorReadings(ColorValues *colorReadings) {
  RGBCIR.readRGBWIR(colorReadings->red, colorReadings->green, colorReadings->blue, colorReadings->clear, colorReadings->IR);
  return true;
}




void printColorData() {
  bool first = true; // To manage commas between printed values
  struct updatingColors {
    bool r = false;
    bool g = false;
    bool b = false;
    bool c = false;
    bool ir = false;
  };

  updatingColors channels;



  if (updateChannels[static_cast<int>(ColorChannels::BLUE)]) {
    if (!first) SERIAL_PRINT("   "); else first = false;
    // SERIAL_PRINT("Blue:");
    channels.b = true;
    // SERIAL_PRINT(colorData.blue);
    // SERIAL_PRINT(scaledFixedColorData.blueFixed.asInt());
    SERIAL_PRINT(mappedBlue);
  }
  
  if (updateChannels[static_cast<int>(ColorChannels::CLEAR)]) {
    if (!first) SERIAL_PRINT("   "); else first = false;
    // SERIAL_PRINT("Clear:");
    channels.c = true;
    // SERIAL_PRINT(colorData.clear);
    SERIAL_PRINT(scaledFixedColorData.clearFixed.asInt());
  }

  if (updateChannels[static_cast<int>(ColorChannels::IR)]) {
    if (!first) SERIAL_PRINT("   "); else first = false;
    // SERIAL_PRINT("IR:");
    channels.ir = true;
    // SERIAL_PRINT(colorData.IR);
    SERIAL_PRINT(scaledFixedColorData.IRFixed.asInt());
  }
  
  if (updateChannels[static_cast<int>(ColorChannels::GREEN)]) {
    if (!first) SERIAL_PRINT("   "); else first = false;
    // SERIAL_PRINT("Green:");
    channels.g = true;
    // SERIAL_PRINT(colorData.green);
    // SERIAL_PRINT(scaledFixedColorData.greenFixed.asInt());
    SERIAL_PRINT(mappedGreen);
  }
  if (updateChannels[static_cast<int>(ColorChannels::RED)]) {
    if (!first) SERIAL_PRINT("   "); else first = false;
    // SERIAL_PRINT("Red:");
    channels.r = true;
    // SERIAL_PRINT(colorData.red);
    // SERIAL_PRINT(scaledFixedColorData.redFixed.asInt());
    SERIAL_PRINT(mappedRed);
  }

  // SERIAL_PRINT("  "); SERIAL_PRINT(v2);

  SERIAL_PRINTLN();
}



// this returns the MIDI note number for any note from C-1 to G9 (MIDI notes 0 through 127).
// Pass the note name as a string into the parameter. E.g., "D#-1" returns 3, or "F#2" returns 42.
uint8_t noteNameToMIDINote(const char* noteName) {
  // Arrays for natural and sharp notes
  const char* naturalNotes[7] = {"C", "D", "E", "F", "G", "A", "B"};
  const uint8_t naturalNoteBases[7] = {0, 2, 4, 5, 7, 9, 11};
  const char* sharpNotes[5] = {"C", "D", "F", "G", "A"};
  const uint8_t sharpNoteBases[5] = {1, 3, 6, 8, 10};

  constexpr uint8_t OCTAVE = 12;

  int8_t noteBaseIndex = -1;  // Base index for the note
  int8_t octaveNumber = 0;    // Octave number

  // Check if the note is sharp or natural
  if (noteName[1] == '#') {
    // Sharp note
    for (int i = 0; i < 5; i++) {
      if (noteName[0] == sharpNotes[i][0]) {
        noteBaseIndex = sharpNoteBases[i];
        break;
      }
    }
    if (noteBaseIndex == -1 || (noteName[2] != '-' && !isdigit(noteName[2]))) return 255; // Invalid note
    // Handle negative octave
    if (noteName[2] == '-') {
      octaveNumber = -1; // Convert char to int and make negative
    } else {
      octaveNumber = noteName[2] - '0'; // Convert char to int
    }
  } else {
    // Natural note
    for (int i = 0; i < 7; i++) {
      if (noteName[0] == naturalNotes[i][0]) {
        noteBaseIndex = naturalNoteBases[i];
        break;
      }
    }
    if (noteBaseIndex == -1 || (noteName[1] != '-' && !isdigit(noteName[1]))) return 255; // Invalid note
    // Handle negative octave
    if (noteName[1] == '-') {
      octaveNumber = -(noteName[2] - '0'); // Convert char to int and make negative
    } else {
      octaveNumber = noteName[1] - '0'; // Convert char to int
    }
  }
  // Calculate the MIDI note number
  return (octaveNumber + 1) * OCTAVE + noteBaseIndex;
}




// Converts a MIDI note number into a string (const char*) note name. E.g., 42 -> F#2
const char* MIDINoteToNoteName(uint8_t note) {
  // Note names for one octave
  const char* noteNames[] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"
  };

  if (note > 127) {
    return "Invalid"; // Return an error string for invalid MIDI numbers
  }

  // Determine octave
  int8_t octave = (note / 12) - 1;
  uint8_t noteIndex = note % 12;

  // Allocate a static buffer to store the note string
  static char noteStr[5]; // Max length: "A#-1" + null terminator = 5 bytes
  snprintf(noteStr, sizeof(noteStr), "%s%d", noteNames[noteIndex], octave);

  return noteStr;
}



void setFreqsFromChord(const Chord& chord, UFix<12,15>& f1, UFix<12,15>& f2, UFix<12,15>& f3, UFix<12,15>& f4) {
  uint8_t note = noteNameToMIDINote(chord.notes[0]);
  if (note >= 0 && note < 128) {
    f1 = mtof(UFix<7,0>(note));
  } else {
    f1 = 0;
  }

  note = noteNameToMIDINote(chord.notes[1]);
  if (note >= 0 && note < 128) {
    f2 = mtof(UFix<7,0>(note));
  } else {
    f2 = 0;
  }

  note = noteNameToMIDINote(chord.notes[2]);
  if (note >= 0 && note < 128) {
    f3 = mtof(UFix<7,0>(note));
  } else {
    f3 = 0;
  }

  note = noteNameToMIDINote(chord.notes[3]);
  if (note >= 0 && note < 128) {
    f4 = mtof(UFix<7,0>(note));
  } else {
    f4 = 0;
  }
}


// pass in a list of notes and get back a single note, e.g.:
// const char* arpeggio[] = {"A4", "C#2", "D5"};
// const char* newNote = getNoteFromArpeggio(arpeggio, 3, 0);   // should result in newNote equaling "A4"
const char* getNoteFromArpeggio(const char* notes[], uint8_t numNotes, uint8_t selector) {
  if (selector < numNotes) {
    return notes[selector];
  } else {
    return " ";
  }
}



// takes an input value (range 0 to 127) and snaps it to the nearest note in a provided scale
// pass the notes in as MIDI note numbers, not note names
uint8_t snapToNearestNote(uint8_t inputValue, const uint8_t notes[], uint8_t numNotes) {
  uint8_t minimumDistance = 255;  // The smallest distance found between inputValue and a MIDI note
  uint8_t minDistanceNote = 0;    // The corresponding nearest MIDI note

  // Iterate through the scale notes to find the nearest one
  for (uint8_t i = 0; i < numNotes; i++) {
    uint8_t distance = abs(inputValue - notes[i]);

    if (distance < minimumDistance) {
      minimumDistance = distance;
      minDistanceNote = notes[i];
    }
  }

  return minDistanceNote;
}


// convert an array of note names into an array of MIDI note numbers
void convertArray_NoteNamesToNumbers(const char* noteNames[], uint8_t numNotes, uint8_t midiNotes[]) {
  for (uint8_t i = 0; i < numNotes; i++) {
    midiNotes[i] = noteNameToMIDINote(noteNames[i]);
  }
}

void convertArray_NoteNumbersToNames(const uint8_t midiNotes[], uint8_t numNotes, const char* noteNames[]) {
  for (uint8_t i = 0; i < numNotes; i++) {
    noteNames[i] = MIDINoteToNoteName(midiNotes[i]);
  }
}


int16_t convertPotValToArmRadius(uint16_t potVal) {
  return map(potVal, 1023, 0, 80, -80);
}

int16_t convertPotValToTableSpeed(int16_t potVal) {
  constexpr uint8_t tableSpeedDeadband = 40;
  // create a deadband in the potentiometer reading to account for noise around the detent
  potVal = (potVal > 512 + tableSpeedDeadband || potVal < 512 - tableSpeedDeadband) ? potVal : 512;
  int16_t speed = map(potVal, 0, 1023, -255, 255);
  return speed;
}


/**
 * @brief Moves the arm to a specified target radius using bang-bang control.
 * 
 * @details This function implements a closed-loop bang-bang arm position controller.
 * If the arm position is close enough to the target position, the motor is stopped.
 * Close enough is defined by the deadBand value. If the current position is outside
 * the deadband range of the target position, then the motor is set to spin full speed
 * in the appropriate direction. The one extra bit of sauce on this is the acceleration.
 * If a new target position is specified, the motor speed is not instantly set to the 
 * new maximum speed to reach that target. Instead it's ramped up quickly over the course
 * of a few milliseconds. This smooths out the current demanded by the motor. Without
 * this acceleration, the motor was drawing enough current to make the LEDs on the color
 * sensor flicker. The acceleration doesn't apply to when the arm hits the target; then
 * it just cuts to 0 speed instantly.
 * 
 * @param targetRadius The desired arm position in millimeters (radial value, not angular)
 * @param currentRadius The current arm position in millimeters (radial value, not angular)
 * @return void
 */

void moveArmToRadius(int8_t targetRadius, int8_t currentRadius) {
  // I need to add a feature that cuts motor speed to 0 when end of range of arm motion is reached. I think sometimes it exceeds the
  // range that works properly for the angle to distance calculator and causes odd behavior.

  static EventDelay accelTimer;
  static bool initialize = true;
  constexpr uint8_t accelerationMultiplier = 2; // value added to last motor speed to cause acceleration
  constexpr uint16_t accelerationInterval = 2;  // milliseconds between speed updates
  // the first time this function is called, we need to initialize the timer to run at the appropriate interval
  if (initialize) {
    accelTimer.set(accelerationInterval);
    initialize = false;
  }

  static int16_t lastMotorSpeed = 0, targetMotorSpeed = 0;
  constexpr uint8_t deadBand = 2;   // if the current position is within this distance of the target, we reached the target
  constexpr uint8_t maxMotorSpeed = 240;
  static int8_t directionVector = 1;

  // handle motor acceleration
  // figure out if the target is outside the deadband range of the current position
  int16_t displacement = targetRadius - currentRadius;   // this needs to be int16_t so that it doesn't overflow (originally I was using int8_t and had errors)
  if (abs(displacement) <= deadBand) {
    targetMotorSpeed = 0;
  } else {
    if (accelTimer.ready()) {
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
uint8_t getButtonPressed(uint16_t buttonPinVal) {
  constexpr uint8_t DEBOUNCE_INTERVAL = 10;
  static EventDelay debounceTimer;
  static bool evaluatingPress = false;
  static uint8_t triggeredVal = 255, stableVal = 255;

  // this next part is where the magic happens. The buttons are multiplexed on a single analog pin. This works by 
  // creating a resistor ladder, where the buttons connect the analog pin to a different point on the ladder.
  // This creates a unique voltage divider for each button, so whenever you press a button, the analog pin sees
  // a distinct voltage that represents which button was pressed. The downside of this is that only one button can
  // ever be pressed, and lower-numbered buttons will always have precedence. If you press and hold B1 and then pres
  // B0, B0 will take over from B1. But the major upside of this is that you can put a bunch of buttons on a single
  // pin! As long as you only need to detect a single button being pressed at a time, this is a worthwhile trade.
  // This next line evaluates which button was pressed by checking the value reported by the ADC. I designed the 
  // resistor ladder so that there's a roughly even spacing in ADC values between each button press (~340 counts
  // between each button). This requires using different resistor values at each point in the ladder, but makes
  // the code cleaner and more reliable. Otherwise you'd get logarithmically decreasing spacing between buttons 
  // when represented as ADC counts. 
  uint8_t rawButton = ((buttonPinVal < 172) ? 0 : ((buttonPinVal < 510) ? 1 : ((buttonPinVal < 850 ? 2 : 255))));

  // now start the debounce logic if needed. Basically this looks for a state change and starts a timer.
  if (rawButton != triggeredVal && !evaluatingPress) {
    evaluatingPress = true;
    debounceTimer.set(DEBOUNCE_INTERVAL);
    debounceTimer.start();
    triggeredVal = rawButton;
  }

  // if the timer is up and we're evaluating a state change for possible button press, set the new stable state
  if (evaluatingPress && debounceTimer.ready()) {
    // if the new reading is the same as the reading that triggered evaluation, set the stable state to the new reading.
    // otherwise, leave the stable state as is.
    stableVal = (rawButton == triggeredVal) ? triggeredVal : stableVal;
    evaluatingPress = false;
  }
  return stableVal;
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
void scaleColorData(ColorValues *rawData) {
  rawData->red = (rawData->red * 7) >> 2;     // this is the same as multiplying the red channel by 1.75, but done with faster math operations
  rawData->blue = ((uint32_t)rawData->blue * 157286UL) >> 16; // this closely approximates multiplying blue by 2.4 (2.4 is 12/5, and you can approximate 1/5 with 51 >> 8) 
  rawData->IR = rawData->IR << 3;
}

void scaleColorDataFixedPoint(ColorValues *rawData, FixedPointColorValues *scaledVals) {
  scaledVals->redFixed = UFix<16, 0>((rawData->red * 7) >> 2);
  scaledVals->greenFixed = UFix<16, 0>(rawData->green);
  scaledVals->blueFixed = UFix<16, 0>(((uint32_t)rawData->blue * 157286UL) >> 16);
  scaledVals->clearFixed = UFix<16, 0>(rawData->clear);
  scaledVals->IRFixed = UFix<16, 0>(rawData->IR << 3);
}