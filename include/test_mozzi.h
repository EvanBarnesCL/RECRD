// test_mozzi.h
#ifndef TEST_MOZZI_H
#define TEST_MOZZI_H

#include <MozziGuts.h>

// If this compiles, Mozzi is in the include path
#ifndef AUDIO_RATE
#error "Mozzi AUDIO_RATE not defined - MozziGuts.h not properly included"
#pragma message "Mozzi AUDIO_RATE not defined."
#endif

#endif