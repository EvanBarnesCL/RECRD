// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.
// It has a few side effects. It breaks any use of millis(). delay() also will not work, but I plan to avoid
// the use of both of those any time Mozzi is running anyway, so this might be fine! So far this hasn't affected
// my very basic Mozzi sketch.
// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.


#define USE_FAST_PWM 1        // set to 1 to change the PWM clock divisor for pins 5 and 6 (timer 0) - removes motor noise from audio
#define USE_LED_PWM  1        // set to 1 to change the PWM clock divisor for pins 3 and 11 (timer 2) - removes PWM noise due to LED dimming

#define USE_MOZZI_ANALOG_READ
#define MOZZI_ANALOG_READ_RESOLUTION 10

#include <Arduino.h>
#include <Crunchlabs_DRV8835.h>
#include <Wire.h>
// #include <VEML3328.h>
#include <PWMFreak.h>
#define MOZZI_CONTROL_RATE 128
#include <Mozzi.h>
#include <Oscil.h>
#include <tables/sin2048_int8.h>
#include <tables/cos8192_int8.h>
#include <IntMap.h>
#include <EventDelay.h>
#include <mozzi_utils.h>
#include <mozzi_rand.h>
#include <mozzi_midi.h>
#include <CLS16D24.h>
#include <AS5600.h>
#include <DCfilter.h>
#include <FastPID.h>

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

#if USE_SERIAL
  #define SERIAL_PRINT(x) Serial.print(x)
  #define SERIAL_PRINTLN(x) Serial.println(x)
  #define SERIAL_BEGIN(baud) Serial.begin(baud)
#else
  #define SERIAL_PRINT(x) do {} while (0)
  #define SERIAL_PRINTLN(x) do {} while (0)
  #define SERIAL_BEGIN(baud) do {} while (0)
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
bool debounceSwitch();


int16_t armRadiusToAngle(int16_t radiusMM);     // convert arm radius in millimeters to angle in encoder counts
int16_t armAngleToRadius(int16_t angleCounts);  // convert arm angle in encoder counts to radius in millimeters
int16_t convertPotValToArmRadius(uint16_t potVal);
int16_t convertPotValToTableSpeed(int16_t potVal);


// int16_t moveArmToRadius(int8_t targetRadius, int8_t currentArmPosition);
void moveArmToRadius(int8_t targetRadius, int8_t currentRadius);




// int16_t armDistanceToAngle(int16_t distance);     // calculates arm encoder angle from linear position of color sensor
// int16_t armAngleToDistance(int16_t angle);        // calculates linear position of color sensor based on arm encoder angle
// bool initializationRoutine();                     // initialization routine to properly position the arm over the table and prime the color sensor
uint8_t noteNameToMIDINote(const char* noteName);          // convert note names to MIDI note numbers (e.g., F#2 -> 42)
const char* MIDINoteToNoteName(uint8_t note);     // convert MIDI note to note name (e.g., 42 -> F#2)


int16_t armEncVal, tableEncVal, middlePotVal, backPotVal;

AS5600L tableEncoder;
AS5600 armEncoder;

DCfilter armDCFilter(0.95);          // DC filter detects changes in arm position (settles to 0 if the arm is not moving)
DCfilter tableDCFilter(0.6);        // DC filter for table movement
constexpr int8_t DCMovementThreshold = 5;   // If the DC filter shows a value between + and - DCMovementThreshold, we know that axis is not moving
int16_t tableDC = 0;                // the value of the DC filter for table movement.

int32_t tableCumulativePosition;
int32_t armCumulativePosition;


// **********************************************************************************
// Color sensor
// **********************************************************************************

CLS16D24 RGBCIR;
uint16_t conversionTime = 0;

enum class ColorChannels {
  RED,
  GREEN,
  BLUE,
  CLEAR,
  IR
};

ColorChannels currentColorChannel = ColorChannels::RED;

struct ColorValues {
  uint16_t red = 0;
  uint16_t green = 0;
  uint16_t blue = 0;
  uint16_t clear = 0;
  uint16_t IR = 0;
};

ColorValues colorData;
bool updateChannels[5] = {false, false, false, false, false}; // Flags to control which channels to update. defaults to all five off.

bool updateColorReadings(ColorValues *colorReadings);
void printColorData();

// delay timer for updating sensor data and PID controller for arm
EventDelay k_i2cUpdateDelay, k_PIDupdate;
constexpr uint8_t I2C_UPDATE_INTERVAL = 50;

const uint8_t PID_DIVISOR = 2;
constexpr uint8_t PID_HZ = MOZZI_CONTROL_RATE / (float)PID_DIVISOR;


// **********************************************************************************
// Potentiometers and switch
// **********************************************************************************

constexpr uint8_t POT_A_PIN = A0;
constexpr uint8_t POT_B_PIN = A1;
constexpr uint8_t AUDIO_IN_PIN = A2;
constexpr uint8_t BUTTONS_PIN = A3;

uint8_t buttonPressed = 255;                        // 255 means no button pressed, 0, 1 or 2 indicates which of the three buttons was pressed
uint8_t getButtonPressed(uint16_t buttonPinVal);    // pass an analog reading on BUTTON_PIN into the parameter. Returns 1, 2, or 3 if one of those buttons is pressed, otherwise returns 255.

// **********************************************************************************
// Mozzi stuff
// **********************************************************************************

// Oscil <2048, MOZZI_AUDIO_RATE> aSin(SIN2048_DATA);
// Oscil <2048, MOZZI_CONTROL_RATE> kVib(SIN2048_DATA);
// float centre_freq = 440.0;
// float depth = 5.0;
// float vibratoFreq = 221.0;

// global gain controls
constexpr uint8_t MAX_GLOBAL_GAIN = 255;   // maximum global gain value
// uint8_t globalGain = 12;           // global gain value for changing total volume output. non-linear changes in volume
// IntMap k_GlobalGainMap(0, 255, 0, MAX_GLOBAL_GAIN);     // maps potentiometer value to be within 0-MAX_GLOBAL_GAIN so you don't blow your ears out


// EventDelay k_printTimer, k_chordChangeTimer;

bool newColorData = false;      // flag to indicate that new color data is ready for use
int8_t currentArmPosition = 0; // current arm position in millimeters from center of table

IntMap map_GreenToVol(0, 4095, 0, 255), map_BlueToVol(0, 4095, 0, 255), map_RedToVol(0, 4095, 0, 255);

uint8_t mappedGreen = 0, mappedBlue = 0, mappedRed = 0;


// **********************************************************************************
// Music stuff
// **********************************************************************************

// Oscil Wash example sketch stuff

// harmonics
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos1(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos2(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos3(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos4(COS8192_DATA);
// Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos5(COS8192_DATA);
// Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos6(COS8192_DATA);
// Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos7(COS8192_DATA);
// Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos8(COS8192_DATA);

// volume controls
Oscil<COS8192_NUM_CELLS, MOZZI_CONTROL_RATE> kVol1(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_CONTROL_RATE> kVol2(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_CONTROL_RATE> kVol3(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_CONTROL_RATE> kVol4(COS8192_DATA);
// Oscil<COS8192_NUM_CELLS, MOZZI_CONTROL_RATE> kVol5(COS8192_DATA);
// Oscil<COS8192_NUM_CELLS, MOZZI_CONTROL_RATE> kVol6(COS8192_DATA);
// Oscil<COS8192_NUM_CELLS, MOZZI_CONTROL_RATE> kVol7(COS8192_DATA);
// Oscil<COS8192_NUM_CELLS, MOZZI_CONTROL_RATE> kVol8(COS8192_DATA);

// audio volumes updated each control interrupt and reused in audio till next control
char v1,v2,v3,v4,v5,v6,v7,v8;


/*/
// harmonics
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos1(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos2(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos3(COS8192_DATA);
Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos4(COS8192_DATA);
// Oscil<COS8192_NUM_CELLS, MOZZI_AUDIO_RATE> aCos5(COS8192_DATA);

// base pitch frequencies in Q16n16 fixed int format (for speed later)
UFix<12,15> f1,f2,f3,f4;
*/
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


IntMap k_redChannelVibMap(0, 32767, 80, 2000);       // signed int, max value of 32767
IntMap k_potToFreq(0, 1023, 80, 400);


void convertArray_NoteNumbersToNames(const uint8_t midiNotes[], uint8_t numNotes, const char* noteNames[]);
void convertArray_NoteNamesToNumbers(const char* noteNames[], uint8_t numNotes, uint8_t midiNotes[]);
uint8_t snapToNearestNote(uint8_t inputValue, const uint8_t notes[], uint8_t numNotes);
void setFreqsFromChord(const Chord& chord, UFix<12,15>& f1, UFix<12,15>& f2, UFix<12,15>& f3, UFix<12,15>& f4);
const char* getNoteFromArpeggio(const char* notes[], uint8_t numNotes, uint8_t selector);




const char* testArp[8] = {"C2", "E3", "A3", "G4", "C4", "F4", "G#4", "D5"};

// going to tie this into the red color channel
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

  // Calibrate the encoders before Mozzi starts. Calibration relies on stock analogRead(),
  // which is blocking. 
  tableEncoder.begin();
  // tableEncoder.setDirection(AS5600_COUNTERCLOCK_WISE);
  tableEncoder.resetCumulativePosition();    // calibrate to 0 as starting position for the table
  armEncoder.begin();
  
  tableEncoder.setHysteresis(11);
  armEncoder.setHysteresis(11);


  // Now home the arm. This moves the arm back to the center of the table by the end of the routine,
  // and then resets the arm encoder position to 0.
  homeArm(armRadiusToAngle(convertPotValToArmRadius(analogRead(POT_A_PIN))));

  currentArmPosition = armAngleToRadius(armEncoder.getCumulativePosition());

  // set up the color sensor over i2c.
  // I haven't tested this, but I think we need to set up the color sensor last, after the encoders. This is because the
  // .begin() function for the color sensor has its parameter set to true, which commands the begin function to set the
  // I2C bus to fast mode, which runs at 400kHz instead of 100kHz default. I found that setting the I2C bus to fast mode
  // only works if we do it after the sensor is connected. Fast mode reduces latency, which is essential for Mozzi.
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
  RGBCIR.setGain(32, false);     // Set gain to x96, use double diode sensing area (increases sensitivity)
  /**
   * Both the total time it takes to convert a color reading into values and the resolution of that data are set
   * by the value of a single byte, which you can set with setResolutionAndConversionTime(). The resolutoin and
   * conversion time are interdependent, so it's difficult to know what time and resolution you are going to get
   * by the value you set the byte to. Open CLS16D24.h, and at the bottom you will find a large comment that shows
   * all possible values for conversion time and resolution.
  */
  RGBCIR.setResolutionAndConversionTime(0x01);
  SERIAL_PRINT("Conversion time: ");
  conversionTime = RGBCIR.getConversionTimeMillis();
  SERIAL_PRINTLN(conversionTime);  // Calculates the conversion time determined by setResolutionAndConversionTime()
  SERIAL_PRINT("Resolution: ");
  SERIAL_PRINTLN(RGBCIR.getResolution());            // Calculates resolution determined by setResolutionAndConversionTime()

  // Define which color channels to update here. Set to true to enable that color channel.
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
  // There's a chance that changing Timer 2's clock divisor will mess up Mozzi functions I'm not aware of.
  if (USE_LED_PWM) {
    setPwmFrequency(11, 1);   // set Timer2 to clock divisor of 1 (clock rate 31250Hz now, above audible range)
    analogWrite(LED_PIN, 128);    // set the brightness for the LEDs
  } else {
    digitalWrite(LED_PIN, HIGH);  // just turn the color sensor LEDs on at full brightness
  }

  

  uint16_t tableSpeed = analogRead(POT_B_PIN);
  tableMotor.setSpeed(convertPotValToTableSpeed(tableSpeed));


  
  
  // setFreqsFromChord(chordProgression[1], f1, f2, f3, f4);
  
  // set Oscils with chosen frequencies
  // aCos1.setFreq(f1);
  // aCos2.setFreq(f2);
  // aCos3.setFreq(f3);
  // aCos4.setFreq(f4);
  // aCos5.setFreq(f5);
  
  // the scaleNumers_EbPentatonicMinor array needs to be initialized
  convertArray_NoteNamesToNumbers(scale_EbPentatonicMinor, numNotesInScale, scaleNumbers_EbPentatonicMinor);
  
  
  // finally, start the timers
  // k_printTimer.set(100);
  k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);
  k_PIDupdate.set(1000 / PID_HZ);
  // k_chordChangeTimer.set(500);
  // k_chordChangeTimer.start();
  
  
  // Oscil wash example sketch stuff
  // set harmonic frequencies
  // aCos1.setFreq(mtof(60));
  aCos2.setFreq(mtof(noteNameToMIDINote("E2")));
  aCos3.setFreq(mtof(noteNameToMIDINote("A3")));
  aCos4.setFreq(mtof(noteNameToMIDINote("B4")));
  // aCos5.setFreq(mtof(67));
  // aCos6.setFreq(mtof(81));
  // aCos7.setFreq(mtof(60));
  // aCos8.setFreq(mtof(84));
  
// set volume change frequencies
  // kVol1.setFreq(4.43f); // more of a pulse
  // kVol2.setFreq(0.0245f);
  // kVol3.setFreq(0.019f);
  // kVol4.setFreq(0.07f);
  // kVol5.setFreq(0.047f);
  // kVol6.setFreq(0.031f);
  // kVol7.setFreq(0.0717f);
  // kVol8.setFreq(0.041f);
  
  v1=v2=v3=v4=v5=v6=v7=v8=127;
  
  // start Mozzi
  // kVib.setFreq(vibratoFreq);
  startMozzi(MOZZI_CONTROL_RATE);
}





// **********************************************************************************
// updateControl for Mozzi
// **********************************************************************************

void updateControl() {
  newColorData = false;
  static uint8_t chordIterator = 0;
  static uint8_t PIDupdateIterator = 0;
  static int8_t targetArmPos = 80;

  // check to see if buttons are pressed
  buttonPressed = getButtonPressed(mozziAnalogRead<10>(BUTTONS_PIN));
  
  if (k_i2cUpdateDelay.ready()) {
    // SERIAL_PRINT(buttonPressed); SERIAL_PRINT("     ");
    currentArmPosition = armAngleToRadius(armEncoder.getCumulativePosition());
    tableDC = tableDCFilter.next(tableEncoder.getCumulativePosition() * 4);
    newColorData = updateColorReadings(&colorData);
    // SERIAL_PRINT(String((float)v1)); SERIAL_PRINT(" ");
    // SERIAL_PRINT(String((float)v2)); SERIAL_PRINT(" "); 
    // SERIAL_PRINT(String((float)v3)); SERIAL_PRINT(" ");
    // SERIAL_PRINTLN(String((float)v4));
    k_i2cUpdateDelay.start();
  }
  
  if (k_PIDupdate.ready()) {
    targetArmPos = convertPotValToArmRadius(mozziAnalogRead<10>(POT_A_PIN));
    moveArmToRadius(targetArmPos, currentArmPosition);
    k_PIDupdate.start();
  }

  
  // now deal with controlling the table motion. Eventually I'm going to use the DC filter to watch the motion of the table.
  // If the program expects it to be moving, but the DC filter shows that it has stopped, the program will stop the drive
  // motor. Basically this will let you dynamically stop the table by grabbing it. If the table then moves again, the program
  // will start the table motor up.  
  constexpr int8_t tableDChysteresis = 5;
  int16_t targetTableSpeed = convertPotValToTableSpeed(mozziAnalogRead<10>(POT_B_PIN));

  tableDC = (tableDC < -tableDChysteresis || tableDC > tableDChysteresis) ? tableDC : 0;    // add some hysteresis
  // SERIAL_PRINTLN(tableDC);

  tableMotor.setSpeed(targetTableSpeed);


  
  
  // update colors as needed (update interval determined by k_i2cUpdateDelay)
  // if (k_i2cUpdateDelay.ready()) {
  //   newColorData = updateColorReadings(&colorData);
  //   k_i2cUpdateDelay.start();
  // }
  // if there is new color data, print it to the monitor
  if (newColorData) {
    printColorData();
  }

  mappedGreen = map_GreenToVol((colorData.green - 512) * 2);
  mappedBlue = map_BlueToVol(colorData.blue * 4);
  mappedRed = map_RedToVol(colorData.red * 2);

  // oscil wash example sketch stuff
  // v1 = kVol1.next()>>1; // going at a higher freq, this creates zipper noise, so reduce the gain
  // v2 = kVol2.next();
  v2 = mappedGreen;
  // v3 = kVol3.next();
  v3 = mappedBlue;
  v4 = mappedRed;
  // v4 = kVol4.next();
  // v4 = map(colorData.green,0, 2048, -1024, 1024);
  // v5 = kVol5.next();
  // v6 = kVol6.next();
  // v7 = kVol7.next();
  // v8 = kVol8.next();



  // this is the sound stuff I had working for a bit
  // play with this bit shift number to see what value brings the color data into a good audible range
  // uint8_t downsampledRed = colorData.red >> 7;   
  // uint8_t downsampledGreen = colorData.green >> 8;
  // uint8_t downsampledBlue = colorData.blue >> 7;
  // aCos1.setFreq(mtof(snapToNearestNote(downsampledRed, scaleNumbers_EbPentatonicMinor, numNotesInScale)));
  // aCos2.setFreq(mtof(snapToNearestNote(downsampledGreen, scaleNumbers_EbPentatonicMinor, numNotesInScale)));
  // aCos3.setFreq(mtof(snapToNearestNote(downsampledBlue, scaleNumbers_EbPentatonicMinor, numNotesInScale)));
  // aCos4.setFreq(0);

}






// **********************************************************************************
// updateAudio for Mozzi
// **********************************************************************************

AudioOutput_t updateAudio() {
  // 8 bit sine osc * 8 bit globalGain makes 16 bits total (2^8 * 2^8 = 2^(8+8) = 2^16). 
  // We need to generate audio output, and Mozzi has a few functions for doing that.
  // One is MonoOutput::from8Bit. If we want to use that, we have to bring everything back
  // into 8 bit from 16 by shifting right by 8 places (divide by 2^8) to bring this back to 8 bit.
  // that would work like the following:
  // return MonoOutput::from8Bit((aSin.next() * globalGain)>>8);             

  // Or, there's MonoOutput::from16Bit, which we can just use directly without shifting:
  // return MonoOutput::from16Bit((aSin.next() * globalGain));

  /*
  This is letting Mozzi compute the number of bits for you.
  The syntax is a bit more cumbersome but FixMath will be
  clever enough to figure out the exact number of bits needed
  to create asig without any overflow, but no more.
  This number of bits will be used by Mozzi for right/left shifting
  the number to match the capability of the system.
*/
  
  /*
  auto asig = 
  (toSFraction(aCos1.next())) +
  toSFraction(aCos2.next()) + 
  toSFraction(aCos3.next());
  // toSFraction(aCos4.next());

  auto vol = UFix<8, 0>(globalGain);
  
  // toSFraction(aCos6.next()) + toSFraction(aCos6b.next()); /* +
    // toSFraction(aCos7.next()) + toSFraction(aCos7b.next()) +
  return MonoOutput::fromSFix(asig * vol);
  */


  // oscil wash example sketch stuff
  long asig = (long)
    // aCos1.next()*v1 +
    aCos2.next() * v2 + 
    aCos3.next() * v3 +
    aCos4.next() * v4;
    // aCos5.next()*v5 +
    // aCos6.next()*v6;
    // aCos7.next()*v7 +
    // aCos8.next()*v8;
  // asig = 0;
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

  
  // finally, slowly bring the sensor arm directly over the center of the table
  // armMotor.setSpeed(-128);
  // while (armEncoder.getCumulativePosition() > startingPosition) {};   // run motor toward 0 until it's reached
  // armMotor.setSpeed(0);
  // armEncoder.resetCumulativePosition(startingPosition);
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
    SERIAL_PRINT(mappedBlue);
  }
  if (updateChannels[static_cast<int>(ColorChannels::IR)]) {
    if (!first) SERIAL_PRINT("   "); else first = false;
    // SERIAL_PRINT("IR:");
    channels.ir = true;
    SERIAL_PRINT(colorData.IR);
  }
  if (updateChannels[static_cast<int>(ColorChannels::CLEAR)]) {
    if (!first) SERIAL_PRINT("   "); else first = false;
    // SERIAL_PRINT("Clear:");
    channels.c = true;
    SERIAL_PRINT(colorData.clear);
  }
  if (updateChannels[static_cast<int>(ColorChannels::GREEN)]) {
    if (!first) SERIAL_PRINT("   "); else first = false;
    // SERIAL_PRINT("Green:");
    channels.g = true;
    // SERIAL_PRINT(colorData.green);
    SERIAL_PRINT(mappedGreen);
  }
  if (updateChannels[static_cast<int>(ColorChannels::RED)]) {
    if (!first) SERIAL_PRINT("   "); else first = false;
    // SERIAL_PRINT("Red:");
    channels.r = true;
    // SERIAL_PRINT(colorData.red);
    SERIAL_PRINT(mappedRed);
  }
  // if (channels.r && channels.g && channels.b && channels.c) {
  //   int32_t combined = colorData.red + colorData.green + colorData.blue;
  //   int32_t diff = colorData.clear - combined;
  //   SERIAL_PRINT("    combined (r+g+b) = "); SERIAL_PRINT(combined);
  //   SERIAL_PRINT("    clear - combined = "); SERIAL_PRINT(diff);
  // } 
  SERIAL_PRINTLN();
}

/*/


// this will run for the first iteration to prime the mozziAnalogRead buffers.
// this is necessary because of the way mozziAnalogRead asynchronously handles the ADC.
// This has to be done at the start of updateControl() rather than during setup() because
// mozziAnalogRead relies on interrupts generated during updateControl to actually start
// and gather analog readings. 
// This block also moves the color sensor to the starting position over the edge of the
// table and primes the buffer for the 5 color sensor channels as well. This way we can
// start the audio generation with clean sensor values, instead of values that start
// at 0 and drastically jump to the actual value. 
bool initializationRoutine() {
  tableEncoder.getCumulativePosition();
  armEncoder.getCumulativePosition();
  // mozziAnalogRead<10>(POT_A_PIN);
  // mozziAnalogRead<10>(POT_B_PIN);
  // mozziAnalogRead<10>(BACK_POT_PIN);

  SERIAL_PRINT(currentArmPosition);
  SERIAL_PRINT("   measured: ");
  SERIAL_PRINT(armEncoder.getCumulativePosition());
  SERIAL_PRINT("   calculated: ");
  SERIAL_PRINTLN(armDistanceToAngle(currentArmPosition));
  
  armMotor.setSpeed(-255);
  // bring the color sensor over the edge of the table


  if (currentArmPosition <= 70) {
    armMotor.setSpeed(0);
    // turn on the LED to illuminate the scene for the color sensor
    digitalWrite(LED_PIN, HIGH);

    // now prime buffer for the values for all 5 color sensor channels
    SERIAL_PRINTLN("here");
    for (int i = 0; i < 10; ) {
      SERIAL_PRINT("here now "); SERIAL_PRINTLN(i);
      if (k_i2cUpdateDelay.ready()) {
        newColorData = updateColorReadings(&colorData);
        k_i2cUpdateDelay.start();
        // rotate to updating the next color channel - only one gets updated each loop
        currentColorChannel = static_cast<ColorChannels>((static_cast<int>(currentColorChannel) + 1) % 5);
        i++;
      }
    }
    // start the table spinning
    tableMotor.setSpeed(255);
    return false;   // initialization is finished, so we'll stop running this routine
  }
  // initialization isn't finished yet, so return true
  return true;
}

*/

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
  int16_t speed = map(potVal, 0, 1023, -255, 255);
  return (speed > 5 || speed < -5) ? speed : 0;       // add in a small deadband to account for play in the potentiometer
}


/**
 * @brief Moves the arm to a specified target radius using PID control
 * 
 * @details This function implements closed-loop arm position control using the FastPID library.
 * It calculates the appropriate motor speed to reach the target position using PID control.
 * While a simple proportional controller was almost sufficient, integral control was added
 * to eliminate steady state error. The implementation uses FastPID which handles fixed-point
 * math internally for efficient execution on microcontrollers.
 * 
 * Previous implementation:
 * @code
 * int16_t moveArmToRadius(int8_t targetRadius, int8_t currentArmPosition) {
 *   constexpr uint8_t kp = 8;
 *   int16_t error = targetRadius - currentArmPosition;
 *   // if (abs(error < 5)) error = 0;
 *   armMotor.setSpeed(constrain(kp * error, -255, 255));
 * }
 * @endcode
 * 
 * @param targetRadius The desired arm position in millimeters (radial value, not angular)
 * @param currentRadius The current arm position in millimeters (radial value, not angular)
 * @return void
 */
void moveArmToRadius(int8_t targetRadius, int8_t currentRadius) {
  static constexpr float kp = 4.0, ki = .20, kd = .50;
  static FastPID armMotorPID(kp, ki, kd, PID_HZ, 8, true);
  armMotor.setSpeed(armMotorPID.step(targetRadius, currentRadius));
}


/**
 * @brief Detects button presses with timeless debouncing using a bit shift register.
 * 
 * @details This function implements a debouncing algorithm using a 16-bit shift register
 * to track button state history. The function reads an analog value from a voltage divider
 * circuit with three buttons multiplexed on a single analog input pin. The function shifts
 * the button history left by 2 bits and stores the new button state in the 2 LSBs.
 * A button is considered stably pressed only when its unique bit pattern fills the entire
 * shift register (8 consecutive identical readings).
 * 
 * Button values:
 * - 0: First button (voltage < 172)
 * - 1: Second button (voltage < 510)
 * - 2: Third button (voltage < 850)
 * - 255: No button pressed
 * 
 * I consider this timeless because it isn't using millis() or micros() to track time, which
 * is what most software debouncing algorithms do. Partly that's just inefficient: we can
 * achieve debouncing without that overhead. But also, we can't use millis() with Mozzi. We
 * could use Mozzi's ticks() function, but again, it's really efficent to just use bitwise math, 
 * and we use every bit of processing power we can get for Mozzi. Note that I may remove this
 * debounce logic entirely and use capacitors for hardware debouncing instead. Oh, also, I'm 
 * using the fast PWM option to remove motor whine from the audio circuit, so we really can't use
 * micros() or millis(). USE_FAST_PWM changes this.
 * 
 * I tested this with 256Hz update rates for the controls, and this debouncing works well. I 
 * tried it with 1kHz updates in a test sketch, and that's too fast and we still get some bounce. 
 * This should be fine for use with MOZZI_CONTROL_RATE = 128 Hz.
 * 
 * @param buttonPinVal The analog reading from the button input pin (0-1023).
 * @return uint8_t The debounced button state (0, 1, or 2. Returns 255 if no button pressed)
 */
uint8_t getButtonPressed(uint16_t buttonPinVal) {
  static uint16_t pressedContainer = 0b1111111111111111;  // initialize to decimal 255 to indicate no buttons pressed
  pressedContainer = pressedContainer << 2;               // shift the container value left 2 places to make room for the next reading
  // This monstrosity does a few things: sort of working right to left, it uses nested ? ternary operators to check the value of 
  // buttonPinVal, and then returns a value of 0, 1, 2, or 255. That 0, 1, 2, or 255 gets a bitwise AND mask to set all the bits to 0, 
  // except for the 2 least significant bits (LSBs), which get preserved. Finally, those two LSBs get bitwise OR'd with the value in
  // pressedContainer. This essentially stashes the number of the button that was pressed in the two LSBs of pressedContainer.
  pressedContainer = pressedContainer | (((buttonPinVal < 172) ? 0 : ((buttonPinVal < 510) ? 1 : ((buttonPinVal < 850 ? 2 : 255)))) & 0b0000000000000011);
  
  // now check to see if a button has been pressed for long enough for the reading to stabilize:
  switch (pressedContainer) {
    case 0:     // if the bits in pressedContainer are all 0, this means button 0 is pressed and has stabilized
      return 0;
      break;
    
    case 0b0101010101010101:  // this pattern is what pressedContainer looks like when button 1 is stabilized
      return 1;
      break;
    
    case 0b1010101010101010:  // this pattern is what it looks like when button 2 is stabilized
      return 2;
      break;
    
    default:
      return 255;             // any other value means that no button has stabilized or is pressed
      break;
  }
}