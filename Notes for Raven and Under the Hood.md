Notes for Raven and Under the Hood


#Sensor Notes
The sensors get polled every 50ms in stock code, or 20 times per second.
k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);      // control how frequently we poll the sensors on the I2C bus (color sensor, both encoders)
I2C_UPDATE_INTERVAL is 50




#how LED brightness PWM values are calculated

originally just started with this
```c
constexpr uint8_t NUM_BRIGHTNESS_LEVELS = 5;
uint8_t LEDBrightnessLevels[NUM_BRIGHTNESS_LEVELS] = {0, 32, 64, 192, 255};
uint8_t brightnessIterator = 3; // default to brightness level 3 on startup
```

but I switched to this:
```c
constexpr uint16_t LEDBrightnessSequence = 0b1000011000100001;
inline uint8_t getBrightness(uint8_t level = 3)
{
    return static_cast<uint8_t>(
        (level == 0) ? 0 : ((((LEDBrightnessSequence >> (4 * (level - 1))) & 0xF) << 5) - 1)
    );
}

```

This generates the sequence of brightness values 0, 31, 63, 191, 255. These make good changes in brightness
for illuminating the table. I could also just store it in an array, but this is fun. Doing it this way also
saved me about 0.3% of RAM, but cost about 0.2% more of flash storage, as compared to just making it an
array. However, RAM is the serious constraint in this program, so I think it's worthwhile.

0b000000000 --> base10: 0
0b000100000 --> base10: 32
0b001000000 --> base10: 64
0b011000000 --> base10: 192
0b100000000 --> base10: 256

shift all of those number right by 5 places to get rid of the trailing 0s they all have in common
>> 5

you're left with 4 nibbles (a nibble is 4 bits) that have the relevant bit patterns. (exclude the fifth one that is all 0s,
that gets taken care of later with a ternary operator).

0001
0010
0110
1000

Those four nibbles can be packed into a single 16 bit integer:
1000011000100001

if the requested LED brightness level is 0, just return 0.

otherwise, we can look at the packed nibbles and see how to expand them back out with basic math operations:

1000 0110 0010 0001
>>12                      shift by 4*3 = 4 * (level - 1) for level == 4
     >>8                           4*2 = 4 * (level - 1) for level == 3
          >>4                      4*1 = 4 * (level - 1) for level == 2
               >>0                 4*0 = 4 * (level - 1) for level == 1

bitwise AND the results of the above shift operation with 
0000 0000 0000 1111
to keep the first 4 bits that make up the nibble we want.

and then shift all of that left by 5 places to add back the trailing 0s and make these numbers the right size.
Finally subtract 1 to make sure that this stays in range of an 8 bit number (256 would cause overflow). 


This is hard to read and hard to change, but I switched to this because it saved 0.3% of RAM, albeit at the cost
of using 0.2% more flash. But RAM is the truly constrained resource in this program, so this was worth it. Also,
it was fun to figure out!