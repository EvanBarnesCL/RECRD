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



// this macro makes it really easy to define a new Chord instance with any number of notes up to 256
#define DEFINE_CHORD(name, ...) \
    constexpr MIDI_NOTE name##_data[] PROGMEM = {__VA_ARGS__}; \
    constexpr Chord name = {name##_data, sizeof(name##_data)}

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