#ifndef OTHER_SCALES_H
#define OTHER_SCALES
#include <MusicTools.h>

// C Dorian - Minor mode with a raised 6th, classic jazz and folk sound
DEFINE_CHORD(scale_CDorian, "C3", "D3", "Eb3", "F3", "G3", "A3", "Bb3");

// C Whole Tone - Six-note symmetrical scale, all whole steps. Creates an
// ambiguous, dreamlike quality with no leading tone.
DEFINE_CHORD(scale_CWholeTone, "C3", "D3", "E3", "F#3", "G#3", "A#3");

// C Diminished (Half-Whole) - Eight-note octatonic scale. Alternates half-step,
// whole-step. Used extensively in classical and jazz for tension and resolution.
DEFINE_CHORD(scale_CDiminishedHalfWhole, "C3", "Db3", "Eb3", "E3", "F#3", "G3", "A3", "Bb3");

// C Blues - Essential six-note blues scale with a flatted fifth (blue note)
DEFINE_CHORD(scale_CBlues, "C3", "Eb3", "F3", "Gb3", "G3", "Bb3");

// C Melodic Minor (ascending) - Jazz minor scale with both minor and major
// character. Ascending form only, as per jazz convention.
DEFINE_CHORD(scale_CMelodicMinor, "C3", "D3", "Eb3", "F3", "G3", "A3", "B3");

// C Hirajoshi - Japanese pentatonic scale.
// Traditionally used in Japanese folk and classical music.
DEFINE_CHORD(scale_CHirajoshi, "C3", "Db3", "E3", "G3", "Ab3");

// C Hungarian Minor - Dramatic Eastern European scale.
// Features both a raised 4th and raised 7th, creating intense tension.
DEFINE_CHORD(scale_CHungarianMinor, "C3", "D3", "Eb3", "F#3", "G3", "Ab3", "B3");

// C Phrygian Dominant - Spanish and Arabic scale, the 5th mode of harmonic minor.
DEFINE_CHORD(scale_CPhrygianDominant, "C3", "Db3", "E3", "F3", "G3", "Ab3", "Bb3");

// F Major Pentatonic - Warm and pastoral
DEFINE_CHORD(scale_FMajPentatonic, "F3", "G3", "A3", "C4", "D4");

// Bb Major Pentatonic - mellow and rounded
DEFINE_CHORD(scale_BbMajPentatonic, "Bb3", "C4", "D4", "F4", "G4");

// A Minor Pentatonic - The most common minor pentatonic, very versatile
DEFINE_CHORD(scale_AMinPentatonic, "A3", "C4", "D4", "E4", "G4");

// D Hirajoshi - Japanese pentatonic with a haunting sound
DEFINE_CHORD(scale_DHirajoshi, "D3", "Eb3", "G3", "A3", "Bb3");

// G Egyptian (Suspended Pentatonic) - Neither major nor minor, creates
// an ambiguous, ancient quality. Used in traditional music across North Africa
// and the Middle East. Formula: 1, 2, 4, 5, b7
DEFINE_CHORD(scale_GEgyptian, "G3", "A3", "C4", "D4", "F4");

// E Kumoi - Japanese pentatonic often used in koto music. Has a melancholy
// but resolved quality. Formula: 1, 2, b3, 5, 6
DEFINE_CHORD(scale_EKumoi, "E3", "F#3", "G3", "B3", "C#4");

// D Iwato - Another Japanese pentatonic with an especially stark,
// unresolved character. Formula: 1, b2, 4, b5, b7
DEFINE_CHORD(scale_DIwato, "D3", "Eb3", "G3", "Ab3", "C4");








#endif