#ifndef MUSICTOOLS_H
#define MUSICTOOLS_H
#include <Arduino.h>
#include <MusicTypes.h>
#include <FixMath.h>

// const char *MIDINoteToNoteName(MIDI_NOTE note);     // convert MIDI note to note name (e.g., 42 -> F#2)
// void convertArray_NoteNumbersToNames(const uint8_t midiNotes[], uint8_t numNotes, const char *noteNames[]);   // this converts whole arrays, which I think I can actually avoid
// void convertArray_NoteNamesToNumbers(const char *noteNames[], uint8_t numNotes, uint8_t midiNotes[]);
// uint8_t snapToNearestNote(uint8_t inputValue, const uint8_t notes[], uint8_t numNotes);
void setFreqsFromChord(const Chord &chord, UFix<12, 15> &f1, UFix<12, 15> &f2, UFix<12, 15> &f3, UFix<12, 15> &f4);
// const char *getNoteFromArpeggio(const char *notes[], uint8_t numNotes, uint8_t selector);
// uint8_t arpeggiator(uint8_t numNotesInScale, const uint8_t *scaleNumbers, uint8_t manualIndex, int8_t offset, uint8_t arpMode, uint8_t arpSpread);
constexpr uint8_t noteNameToMIDINote(const char* noteName);


struct ScaleStorage
{
  const Chord *scales[NUM_SCALES];
  static constexpr uint8_t numScales = NUM_SCALES;
  uint8_t scaleSelector;

  const Chord &selected() const
  {
    return *scales[scaleSelector];
  }

  void nextScale()
  {
    scaleSelector = (scaleSelector + 1) % NUM_SCALES;
  }

  void prevScale()
  {
    scaleSelector = (scaleSelector == 0) ? NUM_SCALES - 1 : scaleSelector - 1;
  }

  void selectScale(uint8_t index = 0)
  {
    scaleSelector = index % NUM_SCALES;
  }
};





constexpr uint8_t noteNameToMIDINote(const char* noteName)
{
    constexpr uint8_t OCTAVE = 12;
    int8_t noteBaseIndex = -1;
    int8_t octaveNumber = 0;

    if (noteName[1] == '#') {
        // Sharp notes
        switch (noteName[0]) {
            case 'C': noteBaseIndex = 1;  break;  // C#
            case 'D': noteBaseIndex = 3;  break;  // D#
            case 'F': noteBaseIndex = 6;  break;  // F#
            case 'G': noteBaseIndex = 8;  break;  // G#
            case 'A': noteBaseIndex = 10; break;  // A#
            default:  return 255;
        }
        if (noteName[2] == '-')
            octaveNumber = -(noteName[3] - '0');
        else
            octaveNumber = noteName[2] - '0';
    } else if (noteName[1] == 'b') {
        // Flat notes
        switch (noteName[0]) {
            case 'D': noteBaseIndex = 1;  break;  // Db = C#
            case 'E': noteBaseIndex = 3;  break;  // Eb = D#
            case 'G': noteBaseIndex = 6;  break;  // Gb = F#
            case 'A': noteBaseIndex = 8;  break;  // Ab = G#
            case 'B': noteBaseIndex = 10; break;  // Bb = A#
            case 'C': noteBaseIndex = 11; break;  // Cb = B (special case)
            case 'F': noteBaseIndex = 4;  break;  // Fb = E
            default:  return 255;
        }
        // Cb is enharmonic with B of the previous octave
        if (noteName[0] == 'C') {
            if (noteName[2] == '-')
                octaveNumber = -(noteName[3] - '0') - 1;
            else
                octaveNumber = (noteName[2] - '0') - 1;
        } else {
            if (noteName[2] == '-')
                octaveNumber = -(noteName[3] - '0');
            else
                octaveNumber = noteName[2] - '0';
        }
    } else {
        // Natural notes
        switch (noteName[0]) {
            case 'C': noteBaseIndex = 0;  break;
            case 'D': noteBaseIndex = 2;  break;
            case 'E': noteBaseIndex = 4;  break;
            case 'F': noteBaseIndex = 5;  break;
            case 'G': noteBaseIndex = 7;  break;
            case 'A': noteBaseIndex = 9;  break;
            case 'B': noteBaseIndex = 11; break;
            default:  return 255;
        }
        if (noteName[1] == '-')
            octaveNumber = -(noteName[2] - '0');
        else
            octaveNumber = noteName[1] - '0';
    }

    return (octaveNumber + 1) * OCTAVE + noteBaseIndex;
}



// =============================================================================
// FOR_EACH preprocessor metaprogramming
// Applies a macro to each variadic argument
// =============================================================================

#define FE_0(WHAT)
#define FE_1(WHAT, X)      WHAT(X)
#define FE_2(WHAT, X, ...) WHAT(X), FE_1(WHAT, __VA_ARGS__)
#define FE_3(WHAT, X, ...) WHAT(X), FE_2(WHAT, __VA_ARGS__)
#define FE_4(WHAT, X, ...) WHAT(X), FE_3(WHAT, __VA_ARGS__)
#define FE_5(WHAT, X, ...) WHAT(X), FE_4(WHAT, __VA_ARGS__)
#define FE_6(WHAT, X, ...) WHAT(X), FE_5(WHAT, __VA_ARGS__)
#define FE_7(WHAT, X, ...) WHAT(X), FE_6(WHAT, __VA_ARGS__)
#define FE_8(WHAT, X, ...) WHAT(X), FE_7(WHAT, __VA_ARGS__)
#define FE_9(WHAT, X, ...) WHAT(X), FE_8(WHAT, __VA_ARGS__)
#define FE_10(WHAT, X, ...) WHAT(X), FE_9(WHAT, __VA_ARGS__)
#define FE_11(WHAT, X, ...) WHAT(X), FE_10(WHAT, __VA_ARGS__)
#define FE_12(WHAT, X, ...) WHAT(X), FE_11(WHAT, __VA_ARGS__)
#define FE_13(WHAT, X, ...) WHAT(X), FE_12(WHAT, __VA_ARGS__)
#define FE_14(WHAT, X, ...) WHAT(X), FE_13(WHAT, __VA_ARGS__)
#define FE_15(WHAT, X, ...) WHAT(X), FE_14(WHAT, __VA_ARGS__)
#define FE_16(WHAT, X, ...) WHAT(X), FE_15(WHAT, __VA_ARGS__)
#define FE_17(WHAT, X, ...) WHAT(X), FE_16(WHAT, __VA_ARGS__)
#define FE_18(WHAT, X, ...) WHAT(X), FE_17(WHAT, __VA_ARGS__)
#define FE_19(WHAT, X, ...) WHAT(X), FE_18(WHAT, __VA_ARGS__)
#define FE_20(WHAT, X, ...) WHAT(X), FE_19(WHAT, __VA_ARGS__)
#define FE_21(WHAT, X, ...) WHAT(X), FE_20(WHAT, __VA_ARGS__)
#define FE_22(WHAT, X, ...) WHAT(X), FE_21(WHAT, __VA_ARGS__)
#define FE_23(WHAT, X, ...) WHAT(X), FE_22(WHAT, __VA_ARGS__)
#define FE_24(WHAT, X, ...) WHAT(X), FE_23(WHAT, __VA_ARGS__)
#define FE_25(WHAT, X, ...) WHAT(X), FE_24(WHAT, __VA_ARGS__)
#define FE_26(WHAT, X, ...) WHAT(X), FE_25(WHAT, __VA_ARGS__)
#define FE_27(WHAT, X, ...) WHAT(X), FE_26(WHAT, __VA_ARGS__)
#define FE_28(WHAT, X, ...) WHAT(X), FE_27(WHAT, __VA_ARGS__)
#define FE_29(WHAT, X, ...) WHAT(X), FE_28(WHAT, __VA_ARGS__)
#define FE_30(WHAT, X, ...) WHAT(X), FE_29(WHAT, __VA_ARGS__)
#define FE_31(WHAT, X, ...) WHAT(X), FE_30(WHAT, __VA_ARGS__)
#define FE_32(WHAT, X, ...) WHAT(X), FE_31(WHAT, __VA_ARGS__)

#define GET_FE(_0, _1, _2, _3, _4, _5, _6, _7, _8, _9, _10, _11, _12, _13, _14, _15, _16, _17, _18, _19, _20, _21, _22, _23, _24, _25, _26, _27, _28, _29, _30, _31, _32, NAME, ...) NAME

#define FOR_EACH(WHAT, ...) \
    GET_FE(_0, ##__VA_ARGS__, FE_32, FE_31, FE_30, FE_29, FE_28, FE_27, FE_26, FE_25, FE_24, FE_23, FE_22, FE_21, FE_20, FE_19, FE_18, FE_17, FE_16, FE_15, FE_14, FE_13, FE_12, FE_11, FE_10, FE_9, FE_8, FE_7, FE_6, FE_5, FE_4, FE_3, FE_2, FE_1, FE_0)(WHAT, ##__VA_ARGS__)

// =============================================================================
// Simplified chord definition macro
// =============================================================================

#define DEFINE_CHORD(name, ...) \
    constexpr MIDI_NOTE name##_data[] PROGMEM = {FOR_EACH(N, __VA_ARGS__)}; \
    constexpr Chord name = {name##_data, sizeof(name##_data)}


// define a single note more easily as N("C#2")
#define N(s) noteNameToMIDINote(s)



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


#endif