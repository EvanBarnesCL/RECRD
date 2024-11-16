// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.
// It has a few side effects. It breaks any use of millis(). delay() also will not work, but I plan to avoid
// the use of both of those any time Mozzi is running anyway, so this might be fine! So far this hasn't affected
// my very basic Mozzi sketch.

#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

// if this is defined, the PWM rate for Timer 0 is 62.5kHz, which removes audible PWM noise from driving motors.
#define USE_FAST_PWM

// Forward declare the mozziAnalogRead template function instead of including the whole Mozzi library
namespace MozziPrivate {
    template <uint8_t RESOLUTION> int mozziAnalogRead(uint8_t pin);
}

using MozziPrivate::mozziAnalogRead;

#endif // CONFIG_H