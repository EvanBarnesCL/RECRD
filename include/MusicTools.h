#ifndef MUSICTOOLS_H
#define MUSICTOOLS_H
#include <Arduino.h>
#include <FixMath.h>


// typedef uint8_t MIDI_NOTE;
using MIDI_NOTE = uint8_t; // just for readability elsewhere, I'm creating an alias called MIDI_NOTE that is just uint8_t datatype.

// an array of 4 pointers to const char* strings that define up to four notes in a chord
// struct Chord
// {
//   const char *notes[4];
// };


struct Chord {
    const uint8_t* notes;   // pointer to PROGMEM array
    uint8_t numNotes;

    uint8_t getNote(uint8_t index) const {
        if (index >= numNotes) return 255;
        return pgm_read_byte(&notes[index]);
    }
};

// function prototypes
//
//original
// uint8_t noteNameToMIDINote(const char *noteName); // convert note names to MIDI note numbers (e.g., F#2 -> 42)

// new test


const char *MIDINoteToNoteName(uint8_t note);     // convert MIDI note to note name (e.g., 42 -> F#2)
void convertArray_NoteNumbersToNames(const uint8_t midiNotes[], uint8_t numNotes, const char *noteNames[]);   // this converts whole arrays, which I think I can actually avoid
void convertArray_NoteNamesToNumbers(const char *noteNames[], uint8_t numNotes, uint8_t midiNotes[]);
uint8_t snapToNearestNote(uint8_t inputValue, const uint8_t notes[], uint8_t numNotes);
void setFreqsFromChord(const Chord &chord, UFix<12, 15> &f1, UFix<12, 15> &f2, UFix<12, 15> &f3, UFix<12, 15> &f4);
const char *getNoteFromArpeggio(const char *notes[], uint8_t numNotes, uint8_t selector);
uint8_t arpeggiator(uint8_t numNotesInScale, const uint8_t *scaleNumbers, uint8_t manualIndex, int8_t offset, uint8_t arpMode, uint8_t arpSpread);
constexpr uint8_t noteNameToMIDINote(const char* noteName);



constexpr uint8_t NUM_SCALES = 4;

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
        switch (noteName[0]) {
            case 'C': noteBaseIndex = 1;  break;
            case 'D': noteBaseIndex = 3;  break;
            case 'F': noteBaseIndex = 6;  break;
            case 'G': noteBaseIndex = 8;  break;
            case 'A': noteBaseIndex = 10; break;
            default:  return 255;
        }
        if (noteName[2] == '-')
            octaveNumber = -(noteName[3] - '0');
        else
            octaveNumber = noteName[2] - '0';
    } else {
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

#define DEFINE_CHORD(name, ...)                                         \
    constexpr uint8_t name##_data[] PROGMEM = {__VA_ARGS__};               \
    constexpr Chord name = {name##_data, sizeof(name##_data)}

#define N(s) noteNameToMIDINote(s)

DEFINE_CHORD(scale_CPentatonicMajor, N("C3"), N("D3"), N("E3"), N("G3"), N("A3"));

DEFINE_CHORD(scale_CHarmonicMajor, N("C3"), N("D3"), N("E3"), N("F3"), N("G3"), N("G#3"), N("B3"));

DEFINE_CHORD(scale_EbPentatonicMinorMIDI, N("D#3"), N("F#3"), N("G#3"), N("A#3"), N("C#4"));

constexpr MIDI_NOTE root_CLydianScale = 48; // MIDI note number for C3
DEFINE_CHORD(scale_CLydian, root_CLydianScale, root_CLydianScale + 2, root_CLydianScale + 4, root_CLydianScale + 6, root_CLydianScale + 7, root_CLydianScale + 9, root_CLydianScale + 11);


ScaleStorage scaleContainer = {
    {&scale_CPentatonicMajor, &scale_CHarmonicMajor, &scale_EbPentatonicMinorMIDI, &scale_CLydian},
    0
};

Chord currentScale = scaleContainer.selected();



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



// currently this can only set the frequencies from up to 4 notes, need to revise this to be more flexible. basically,
// I want this to look at how many oscillators are active in the sketch, how many notes are in the chord, find the min()
// of those two, and then iterate through the chord, converting notes to frequencies until the min() is reached.
void setFreqsFromChord(const Chord &chord, UFix<12, 15> &f1, UFix<12, 15> &f2, UFix<12, 15> &f3, UFix<12, 15> &f4)
{
    UFix<12, 15>* freqs[] = {&f1, &f2, &f3, &f4};
    for (uint8_t i = 0; i < 4; i++)
    {
        uint8_t note = chord.getNote(i);
        *freqs[i] = (note < 128) ? mtof(UFix<7, 0>(note)) : 0;
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