// AnalogEncoderConfig.h
#ifndef ANALOG_ENCODER_CONFIG_H
#define ANALOG_ENCODER_CONFIG_H

#ifdef USE_MOZZI_ANALOG_READ
    // Instead of including Mozzi.h, we'll just declare the specific function we need
    #if !defined(MOZZI_ANALOG_READ_DECLARED)
        #define MOZZI_ANALOG_READ_DECLARED
        namespace mozzi_analog {
            template<uint8_t RES> 
            extern uint16_t mozziAnalogRead(uint8_t pin);
        }
        using mozzi_analog::mozziAnalogRead;
    #endif
#endif

#endif // ANALOG_ENCODER_CONFIG_H