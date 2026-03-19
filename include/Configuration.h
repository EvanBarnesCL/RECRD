#ifndef CONFIGURATION_H
#define CONFIGURATION_H
#include <Arduino.h>

/**
 * This is a macro for easily enabling or disabling the Serial monitor print statements.
 *
 * You can enable serial print debugging by setting
 *    #define USE_SERIAL 1
 *
 * Or, you can disable serial print debugging by setting
 *    #define USE_SERIAL 0
 *
 * In your main code, rather than using Serial.print() or Serial.println(), use their aliases defined below (e.g., SERIAL_PRINTLN()).
 *
 * The Serial code takes up a lot of flash and RAM, so it really is worth disabling it if you don't need it. When I disabled it for
 * this program, I save about 12% of the RAM and 7% of the flash storage space. That's already a large savings that could make
 * the difference between a synth that sounds good and one that glitches. But printing to the serial monitor is also pretty slow,
 * so disabling saves a lot of processing time too.
 *  */

#define USE_SERIAL 0

#if USE_SERIAL
  #define SERIAL_PRINT(x) Serial.print(x)
  #define SERIAL_PRINTLN(x) Serial.println(x)
  #define SERIAL_BEGIN(baud) Serial.begin(baud)
  #define SERIAL_TAB Serial.print("\t")
  #define SERIAL_TABS(x)
  for (uint8_t i = 0; i < x; i++)
  {
    Serial.print("t");
  }
  #define SERIAL_BEGIN(x) Serial.begin(x)
#else
  #define SERIAL_PRINT(x)       do {} while (0)
  #define SERIAL_PRINTLN(x)     do {} while (0)
  #define SERIAL_BEGIN(baud)    do {} while (0)
  #define SERIAL_TAB            do {} while (0)
  #define SERIAL_TABS(x)        do {} while (0)
  #define SERIAL_BEGIN(x)       do {} while (0)
#endif


// set to 1 to change the PWM clock divisor for pins 3 and 11 (timer 2) - removes PWM noise due to LED dimming from audio
#define USE_LED_PWM 1 

// set to 1 to change the PWM clock divisor for pins 5 and 6 (timer 0) - removes motor noise from audio
#define USE_FAST_PWM 1


// defines how often to update one of the sensors. The sensors get updated one at a time, so if this is left as the default
// 15ms interval, each sensor will be updated every 45ms.
constexpr uint8_t I2C_UPDATE_INTERVAL = 15; // time in milliseconds



#endif