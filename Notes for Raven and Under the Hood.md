Notes for Raven and Under the Hood


# Sensor Notes
The sensors get polled every 50ms in stock code, or 20 times per second.
k_i2cUpdateDelay.set(I2C_UPDATE_INTERVAL);      // control how frequently we poll the sensors on the I2C bus (color sensor, both encoders)
I2C_UPDATE_INTERVAL is 50

actually now the update interval is 15ms, but I don't update all three sensors at once. Instead I rotate which one gets updated each
time an update is needed. So now there are updates every 15ms, but each sensor gets updated every 45ms. This spreads the processing load
out a bit and also makes the sensors update a little more quickly. I think this has improved performance. 



# how LED brightness PWM values are calculated

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






# from musictools.h


## understanding how memory works can help you make really important optimizations and design choices that make your projects better
The original version of this code used something like 92% of the available RAM and ~70% of the flash. A really huge chunk of that RAM
use came from the way I was storing chords and other note data in RAM. By switching to storing all of that in flash data and using tools
called pointers, I was able to drastically reduce RAM use, which means there's more memory left for hacking or adding new features.

Like let's say I had 7 notes in a scale I wanted to reference in my music generator function. Each note is represented by one byte, so that's
7 bytes. I can store all of that in RAM, or I can store it in flash. I have a lot more flash than RAM, so I put it in flash. But I still
need to pull that data out of flash and put it into RAM so the processor can do things with it. If I copy all 7 bytes over to RAM, I'm
duplicating memory use. Instead of copying the data to RAM to perform operations on, I can just give the processor an address of where to look
to find the data in flash. All 7 bytes are stored next to each other in flash, so if I know the address of the first byte in flash, and I know
how many bytes are in the scale, that's all the information I need to find any specific note within the scale.

Let's say that the array of notes in the scale gets put into flash memory starting at the 100th byte in flash memory. I can store two bytes
in RAM that hold that address. This is called a pointer. It's literally a variable that points you to an address in memory. It says, "start looking
at this exact point in memory for the thing you want." It doesn't hold the actual data you want, it just tells you where to find it. Pointers on
the ATmega328 are two bytes wide because the chip needs to be able to address up to 32,768 bytes of flash, which is way more than the 0-255 range
that a single byte can represent.
    POINTER = 100;
If I follow that pointer to address 100 in memory and then look at the contents of that specific byte in memory, I'll find the first note in my
scale. From there I can find any other note in the scale because I know exactly how many notes are in it! If I want the second note in the scale,
I just go to the address POINTER + 1 = 101. If I want the last note in the 7 note scale, I just go to address 106.

It's important to remember that C and C++ are 0-indexed. That means we start counting from 0. This case, the first element in the list of notes
resides at address 100. Another way of saying this is that I don't have to add any offset to the base address to find the first element of the list.
That means the first element in the list sits at index 0. The index is the relative position of an object in a list.
If I want the second element in the list, I just add 1 to the base address.

    Base address:   100
    
    Note number:    note1   note2   note3   note4   note5   note6   note7
    Index:          0       1       2       3       4       5       6
    Address:        100+0   100+1   100+2   100+3   100+4   100+5   100+6
                    100     101     102     103     104     105     106

So the method of storing chords or scales in flash and then using pointers is great because I only need 3 bytes in RAM to be able to find any
note from a scale with any number of notes in it: I need a pointer to the starting address, which takes two bytes, and a single byte that tells
me how many notes are in the scale. I don't have to have that second one either, but if I try to access the data from a byte that sits outside
of the scale's region in memory, I won't get what I expect. C and C++ don't have a problem with letting you do this: you'll just get whatever
value resides at that location in memory. This is called an overflow, which you might have heard of.

So to be safe, we store the number of notes in the scale along with a pointer to the memory address that contains the first note, and we can
store those things together in a struct called Chord. That's 3 bytes total: 2 for the pointer and 1 for the note count.

But hey, now you can store any number of notes in sequence (up to 256 total) and only use 3 bytes of RAM for each sequence! This is great because the
ATMega328 microcontroller has a lot more flash memory than RAM (32,768 bytes vs 2,048 bytes, respectively). This also means that even though I
named this struct Chord, you don't have to only store musical chords in there. It's really just a collection of notes. You can store chords,
scales, or even entire melodies in there!
```c
// This struct lets you store a group of notes in flash (PROGMEM) as a chord object. Because it just stores a pointer to the first
// element in the array, you can store an arbitrary number of notes in the chord, up to 255. And it should only take up 3 bytes of
// RAM per chord regardless of the number of notes in the chord, which is a huge gain in efficiency.
struct Chord {
    const uint8_t* notes;                       // pointer to the address of the first element in the PROGMEM array
    uint8_t numNotes;                           // the number of notes in the array

    uint8_t getNote(uint8_t index) const {      // retrieval function that returns the value of a note from the specified index in the array
        if (index >= numNotes) return 255;      // protection against overflows
        return pgm_read_byte(&notes[index]);    // the & in '&notes[index]' is the address-of operator: it computes the address of the element at that index, which pgm_read_byte then uses to fetch the value from flash.
    }
};

```
