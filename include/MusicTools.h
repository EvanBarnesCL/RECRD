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
void ambienceGenerator(); // right now i just want to wrap all the sound control stuff in a function so I can easily separate it out from the rest of updateControl()
void toneBeatsGenerator();







// C#maj Pentatonic
// const MIDI_NOTE scale_CPentatonicMajor[5] = {48, 50, 52, 55, 57};
// Chord<5> scale_CPentatonicMajor((const char*){"C3", "D3", "E3", "G3", "A3"});


// C harmonic major scale
// const char *scale_CHarmonicMajor[] = {"C3", "D3", "E3", "F3", "G3", "G#3", "B3"};
// const MIDI_NOTE scale_CHarmMajorMIDI[7] = {48, 50, 52, 53, 55, 56, 59};

// Eb pentatonic minor scale
// const char *scale_EbPentatonicMinor[5] = {"D#3", "F#3", "G#3", "A#3", "C#4"};
// MIDI_NOTE scale_EbPentatonicMinorMIDI[5] = {51, 54, 56, 58, 61};

// C Lydian scale
// constexpr MIDI_NOTE root_CLydianScale = 48; // MIDI note number for C3
// const MIDI_NOTE scale_CLydianMIDI[7] = {root_CLydianScale, root_CLydianScale + 2, root_CLydianScale + 4, root_CLydianScale + 6, root_CLydianScale + 7, root_CLydianScale + 9, root_CLydianScale + 11};

// number of scales we'll be storing in the container
// constexpr uint8_t NUM_SCALES = 3;

// // array for storing the scales and another array for storing the associated note numbers
// struct scaleStorage
// {
//   const uint8_t numScales = 3;
//   uint8_t scaleSelector = 0;
//   const MIDI_NOTE *scaleArray[NUM_SCALES] = {scale_EbPentatonicMinorMIDI, scale_CLydianMIDI, scale_CPentatonicMajor};
//   const uint8_t numNotesInSelectedScale[3] = { // this calculates number of notes in each scale
//       sizeof(scale_EbPentatonicMinorMIDI) / sizeof(scale_EbPentatonicMinorMIDI[0]),
//       sizeof(scale_CLydianMIDI) / sizeof(scale_CLydianMIDI[0]),
//       sizeof(scale_CPentatonicMajor) / sizeof(scale_CPentatonicMajor[0])};
// };

// scaleStorage scaleContainer;

// const MIDI_NOTE *currentScale = scaleContainer.scaleArray[scaleContainer.scaleSelector];
// uint8_t numNotesInScale = scaleContainer.numNotesInSelectedScale[scaleContainer.scaleSelector];


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




// uint8_t numNotesInScale = scaleContainer.selected().numNotes;

/*
Examples of how to use the ScaleStorage struct:

Chord current = *myScales.selected();  // doesn't copy note data, just the 3-byte struct
myScales.selectScale(2);               // changes current scale selection
uint8_t note = myScales.selected().getNote(2);
uint8_t count = myScales.selected().numNotes;

myScales.nextScale();
*/


constexpr uint8_t noteNameToMIDINote(const char* noteName);

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






#endif