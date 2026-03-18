interesting bits to keep note of maybe


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



/**
 * shit I've learned about Mozzi through struggle and blood:
 * 
 * - EventDelay timers have to be global or else they never update. This is almost certainly true of other things in Mozzi as well.
 *   I had wanted to use a static EventDelay timer in a function that gets called from within updateControl(), but that does not work. 
 */


// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.
// It has a few side effects. It breaks any use of millis(). delay() also will not work, but I plan to avoid
// the use of both of those any time Mozzi is running anyway, so this might be fine! So far this hasn't affected
// my very basic Mozzi sketch.
// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.


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







Chord Cmaj_I = {scale_CHarmonicMajor[0], scale_CHarmonicMajor[2], scale_CHarmonicMajor[4]};
Chord Ddim_ii = {scale_CHarmonicMajor[1], scale_CHarmonicMajor[3], scale_CHarmonicMajor[5]};
Chord Emin_iii = {scale_CHarmonicMajor[2], scale_CHarmonicMajor[4], scale_CHarmonicMajor[6]};
Chord Fmin_iv = {scale_CHarmonicMajor[3], scale_CHarmonicMajor[4], scale_CHarmonicMajor[0]};
Chord Gmaj_V = {scale_CHarmonicMajor[4], scale_CHarmonicMajor[6], scale_CHarmonicMajor[1]};
Chord GSharpAug_VI = {scale_CHarmonicMajor[5], scale_CHarmonicMajor[0], scale_CHarmonicMajor[2]};
Chord Bdim_vii = {scale_CHarmonicMajor[6], scale_CHarmonicMajor[1], scale_CHarmonicMajor[3]};

constexpr uint8_t numChordsInProgression = 7;
Chord progression[numChordsInProgression] = {Cmaj_I, Ddim_ii, Emin_iii,Fmin_iv, Gmaj_V, GSharpAug_VI, Bdim_vii};

/** 
 * mapping out what I want:
 * 
 * I think that I want to have it mostly be chords, with occasional bursts of arpeggios. So two oscillators always play long sustained notes to generate triads
 * or dyads. Then the third oscillator joins in either triads, or breaks into an arpeggio. I think that instead of having each note be selected from the scale based
 * on color value, I need to have color data select which chord is playing (with some probabilistic drift I think to keep it from repeating exactly). Osc0 will play
 * note0 from that chord, osc1 plays note1, and osc2 either plays note2, or else breaks into arp.
 * 
 * For arpeggiation, I think that the probability of arping should come from one of the color channels. So if there's a lot of red, an arp should be more likely,
 * but not guaranteed. The arpeggio should be notes from within a chord or two of the chosen progression. I can have the number of chords chosen be the spread of the arp,
 * and I could have those chosen randomly. Offset each new chord added to the arp by an octave to keep it distinct.
 * 
 * I can also have sound parameters driven by color value, but I want some randomnessa in that motion too. For example, I want to add amp envelopes to everything, but
 * I could have the ADSR partially modulated by color + randomness. 
 * 
 * 
 * Thinking I might drop into mixolydian mode
 *  key of D, so D mixolydian, diatonically from the scale
 *    I-v-ii-IV, so D, Am, Em, G
 *    
 *  chords in D mixolydian scale
 * degree       I         iim       iii         IV      v         vi        bVII
 * note         D         E         F#          G       A         B         C
 * chord        D         Em        F#dim       G       Am        Bm        C
 * chord note   D,F#,A    E,G,B     F#,A,C      G,B,D   A,C,E     B,D,F#    C,E,G
 * 
 
Chord D = {"D3", "F#3", "A3", " "}, Am = {"A3", "C4", "E4", " "}, Em = {"E3", "G3", "B3", " "}, G = {"G3", "B3", "D4", " "};
Chord progression = {D, Am, Em, G};


 * Just listening to the earliest version of this that maps Green to the probability of moving to the next chord, I think
 * I should also set some chord progression variations. That way another color channel can select the probability of moving
 * from the main progression into one of the variations. Just right now the 4 chords get boring. 
 * 
 * General algorithm:
 *  - define the chords, the scale, and the chord progression as arrays
 *  - initialize the random seed
 *  - two possible paths here: 
 *      - have the color data map to the probability of moving to the next chord in the progression;
 *        - this would be something like: uint8_t nextChord = (rand(256) < mappedGreen) ? 1 : 0; where more green makes it more likely to switch to next chord
 *        - this also needs to have update times between chord changes. So maybe mappedRed determines the time between chord changes. Chord only changes when
 *            previous chord's time has elapsed.
 *      - or, have the color data directly select the chord in the progression, probably with some random noise to keep it interesting.
 *        - uint8_t chosenChord = map(mappedGreen, 0, 256, 0, 4);   // the amount of green directly chooses a chord in the progression
 *        - time between chords is again set by another color channel
 *  - in either of those two cases, set notes for the first 2 oscs based on the chosen chord, so osc0 plays note0, osc1 plays note1. 
 *      - could add some sauce where another color channel or just a constant random probability potentially increases/decreases the value of the note played by
 *        osc1 by an octave, just to build some inversions and keep it interesting
 *  - Osc3 is either the 3rd voice in the chord, or it breaks into arpeggio
 *      - bool arpeggiate = (rand(256) < mappedBlue) ? true : false;    // the more blue their is, the more likely arpeggiation is
 *      - if (!arpeggiate) {
 *          osc2 = note2 in chosen chord
 *        } else {
 *          - choose how many chords will go into the arpeggio, so maybe random between 1 and 3. or choose random number of notes from the D mixolydian scale.
 *            the randomness for either of these could be direct or color modulated
 *          - if we're going random chords route, take the notes from each chord and put them in the arp array as their constituent notes, adding 12 to the midi
 *          - value for the notes of each subsequent chord. or else just add the first n notes from the scale.
 *          - choose the baseline time interval between arp notes: baseInterval = rand(15, max(31, 256 - mappedRed));  // more red -> less time between notes
 *          - set a flag value to show arpeggio started: bool arping = true;
 *        }
 *  - if we are arpeggiating, call the arp function for each new note until we've used all of them in the array, the set arping = false;
 * 
 */


// void toneBeatsGenerator() {
//   // uint8_t note = snapToNearestNote(mappedGreen >> 1, scale_CLydianMIDI, numNotesInScale);
//   uint8_t note = scale_CLydianMIDI[colorToScaleNote7(mappedGreen)];
//   osc0Params.noteMIDINumber = note;
//   osc0Params.frequency = mtof(osc0Params.noteMIDINumber);
//   osc0Params.volume = 140;
//   osc0.setFreq(osc0Params.frequency);
  
//   osc2Params.noteMIDINumber = osc0Params.noteMIDINumber;
//   osc2Params.frequency = mtof(osc2Params.noteMIDINumber);
//   osc2.setFreq((float)osc2Params.frequency + 0.4F);
//   osc2Params.volume = 140;
// }








/*
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



const char* testArp[8] = {"C2", "E3", "A3", "G4", "C4", "F4", "G#4", "D5"};

const uint8_t numNotesInScale = 15;
const char* scale_EbPentatonicMinor[numNotesInScale] = {"D#2", "F#2", "G#2", "A#2", "C#3", "D#3", "F#3", "G#3", "A#3", "C#4", "D#4", "F#4", "G#4", "A#4", "C#5"};
uint8_t scaleNumbers_EbPentatonicMinor[numNotesInScale];


*/


// D scale mixolydian diatonic chord progression
// Chord D = {"A2", "D3", "F#3", " "}, Am = {"A2", "C3", "E3", " "}, Em = {"G3", "B3", "E4", " "}, G = {"G3", "B3", "D4", " "};
// Chord DVar1 = {"D2", "F#2", "A2", " "}, AmVar1 = {"A2", "E3", "C4", " "}, EmVar1 = {"E3", "G3", "B3", " "}, GVar1 = {"G3", "B3", "D4", " "};
// const uint8_t numChordsInProgression = 4;
// Chord progression[numChordsInProgression] = {D, Am, Em, G};
// const uint8_t numNotesInScale = 7;
// const char* scale_DMixolydian[numNotesInScale] = {"D3", "E3", "F#3", "G3", "A3", "B3", "C4"};
// Chord progressionVar1[numChordsInProgression] = {DVar1, AmVar1, EmVar1, GVar1};









// this struct lets you define colors, creating a name and specific values of red, green, and blue for them. Can be used for defining actions when a
// specific color is seen by the color sensor, or by quantizing the color readings into specific named values, which is what findClosestColor() does.
struct ReferenceColor
{
  const char *name;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

// Define your reference colors
const ReferenceColor RED_REF =     {"RED", 255, 126, 129};
const ReferenceColor GREEN_REF =   {"GREEN", 167, 255, 174}; // example values
const ReferenceColor BLUE_REF =    {"BLUE", 123, 152, 255};   // example values
const ReferenceColor CYAN_REF =    {"CYAN", 115, 182, 255};
const ReferenceColor MAGENTA_REF = {"MAGENTA", 255, 154, 228}; // example values
const ReferenceColor YELLOW_REF =  {"YELLOW", 255, 248, 138};   // example values

// Array of reference colors
const uint8_t numReferenceColors = 6;
const ReferenceColor referenceColors[numReferenceColors] = {RED_REF, GREEN_REF, BLUE_REF, CYAN_REF, MAGENTA_REF, YELLOW_REF};






ReferenceColor findClosestColor(uint8_t redRaw, uint8_t greenRaw, uint8_t blue_raw);    // quantizes raw color readings into the nearest named color



ReferenceColor findClosestColor(uint8_t redRaw, uint8_t greenRaw, uint8_t blue_raw)
{
  uint32_t minDistance = UINT32_MAX;
  ReferenceColor closestColor = referenceColors[0];

  for (int i = 0; i < numReferenceColors; i++)
  {
    uint32_t sq_dst = (redRaw - referenceColors[i].red) * (redRaw - referenceColors[i].red) +
                      (greenRaw - referenceColors[i].green) * (greenRaw - referenceColors[i].green) +
                      (blue_raw - referenceColors[i].blue) * (blue_raw - referenceColors[i].blue);

    if (sq_dst < minDistance)
    {
      minDistance = sq_dst;
      closestColor = referenceColors[i];
    }
  }

  return closestColor;
}
