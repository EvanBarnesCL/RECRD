#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include <Arduino.h>
#include <Mozzi.h>

#define USE_SERIAL 0
#include <Debug.h>

#define USE_LED_PWM 1
#define USE_FAST_PWM 1

constexpr uint8_t I2C_UPDATE_INTERVAL = 15; // milliseconds

// **********************************************************************************
// Mozzi Configuration
// **********************************************************************************

#define MOZZI_CONTROL_RATE 128

// Two FM oscillators plus a control-rate LFO, all using cosine wavetables.
// The wavefolder is a post-processing stage in updateAudio() — it costs one
// PROGMEM read and a handful of integer operations per sample, not an oscillator.
#include <tables/cos2048_int8.h>

Oscil<COS2048_NUM_CELLS, MOZZI_AUDIO_RATE>   aCarrier(COS2048_DATA);    // FM carrier
Oscil<COS2048_NUM_CELLS, MOZZI_AUDIO_RATE>   aModulator(COS2048_DATA);  // FM modulator
Oscil<COS2048_NUM_CELLS, MOZZI_CONTROL_RATE> kModIndex(COS2048_DATA);   // mod-index LFO

// **********************************************************************************
// Music Things
// **********************************************************************************

constexpr uint8_t NUM_SCALES = 3;

#include <MusicTools.h>
#include <MusicalScales.h>

ScaleStorage scaleContainer =
{
  {
    &scale_EbPentatonicMinor,
    &scale_CLydian,
    &scale_CPentatonicMajor
  },
  0
};

// **********************************************************************************
// FM + wavefolder channel mappings
// **********************************************************************************
//
// Green  → carrier note selection within the current scale.
//
// Blue   → C:M ratio (carrier-to-modulator frequency ratio).
//           Harmonic mode:   integer ratios {1,2,3,4,5,6,7,8} — tonal spectra.
//           Inharmonic mode: fractional ratios 1.0–4.0 — bell/metallic textures.
//           Toggled by button B2.
//
// Red    → FM modulation index (depth / brightness of the FM spectrum).
//           Low red → near-pure sine. High red → harmonically dense.
//
// White  → dual purpose, naturally coupled:
//             1. Mod-index LFO rate: slow tidal breathing (low) to rapid shimmer (high).
//             2. Wavefold drive: below ~63 counts the folder is transparent (linear
//                transfer function); above that, increasing white folds the FM output
//                progressively harder — one complete fold at white=128, ~two folds at
//                white=255. This coupling is intentional: bright scenes become both
//                faster-animated and spectrally richer simultaneously.

#define FM_CARRIER_CHANNEL    mappedGreen
#define FM_RATIO_CHANNEL      mappedBlue
#define FM_INDEX_CHANNEL      mappedRed
#define FM_LFO_CHANNEL        mappedWhite

// Minimum drive value below which the wavefolder is effectively transparent.
// At this value, the FM signal covers only the first half of the fold table
// (the linear ramp), so no folding occurs. See updateFM() for the derivation.
constexpr uint8_t FOLD_DRIVE_MIN = 63;

#endif // CONFIGURATION_H