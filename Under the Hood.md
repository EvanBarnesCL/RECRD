# Intro for Raven
Rather than writing this out as a script directly, I'm just going to give a walkthrough of the
program flow for you, focusing on the parts that I think will be most relevant for either the
lesson or UTH scipts. I'll call out line numbers in the various code files as references for
you to check out, and I'll also provide the code itself for you to look through. This still 
isn't on the CL IDE, so you'll have to open up the project files directly. I wrote the code in
Visual Studio Code in the PlatformIO IDE, so you could download those. That would be nice for
syntax highlighting. But you can also open the code in any text editor, and that will work. If
you want one that can also do syntax highlighting, I highly recommend Notepad Next! 

https://github.com/dail8859/NotepadNext

The code files I'm going to be talking about are mostly main.cpp, which is found in the /src
folder. There are also header files and libraries that will come up, and those are found in
/include and /lib, respectively.

Another quick note for UTH: You can (and kind of have to) program the microcontroller through
the USB port on the breadboard now! That's a new feature for all future Hack Packs that use this
same microcontroller.




# Includes and things
First the usual include directives to pull in code from libraries and header files. 

## Configuration.h
The first one to call out is Configuration.h. This is similar to Tank Plant, where I used a Configuration.h
header file to store global variables and macros that affect all of the code. There isn't actually
a lot in this one yet, so I'll probably be moving some more things into it later. But for now, there
are a couple of features that are worth talking about. 

In Configuration.h on line 31, I call
```c
#define USE_SERIAL 0
```

The comment block above this explains it all pretty well. I thought this would be interesting to talk about
because it shows just how much memory a seemingly small tool like Serial can use.

Next I have two more #define directives that I want to touch on briefly because they have dramatic effects
on the whole system. 

#define USE_LED_PWM 1           // Affects line 224 in main.cpp
#define USE_FAST_PWM 1          // Affects line 217 in main.cpp

In both of these cases, these enable or disable changes to the clock divisors. The Atmega328 microcontroller
has three timer clocks (Timer0, Timer1, and Timer2). These are used for doing things running PWM outputs or
providing the actual timekeeping mechanism for the millis() and micros() functions. The timers count clock 
cycles of the 16MHz crystal oscillator. Timers 0 and 2 are 8-bit, and Timer 1 is 16 bit. Mozzi (the synthesis
library we use for RECRD and SYNTH) completely claims Timer 1, so we can't use that. So for Timer0 or Timer2, 
if they're running, every time the 16MHz crystal oscillator ticks, the timer just increments the value it's 
counting, so they both start from 0, count up to 255, and then reset to 0 and keep counting up. You can use 
this to do things like change the PWM duty cycle of an output pin (e.g., if you want a 50% duty cycle square
wave on an output pin, set the output pin to LOW when the timer's value is less than 128, and set the output
pin high when the timer's value is between 128 and 255). That's one of the main uses of the timers in the
Arduino. Timer0 controls the PWM for pins 5 and 6, and Timer 1 controls PWM for pins 9 and 10, and Timer 2
controls PWM for pins 3 and 11. Again, Mozzi lays claim to Timer 1, so we can't use pins 9 and 10 for our
own PWM, so I had to choose from using pins 5, 6, 9, and 10 for controlling the motor speeds and the LED
brightness with PWM.

Normally this wouldn't be a problem, but this is an audio project. Timers 0 and 2 are 8 bit, so they overflow
at a rate of 62.5kHz (16MHz divided by 256). This is above the audible range for humans, but the timers use
prescalars to divide the clock signal to slow it down. By default in Arduino, Timer0 is configured to use
a divisor of 64, which brings its frequency down to 976.5625Hz. This is extremely close to 1000Hz, and this
is how millis() counts milliseconds. Every time Timer0 increments, the millis() function increments the 
elapsed time value by 1 millisecond (and then some special sauce in the background takes care of the slight
drift from this not being exactly 1000 cycles per second, so it averages out correctly). 

But this is a problem for RECRD because Timer0 is using that prescalar of 64, and is running at 976Hz. 
976Hz is right in the middle of the audible range for humans, so using this to generate PWM signals to control
the motors produces signals that we would be able to hear if this signal were somehow hooked up to a speaker
or headphones. Which in this case, unfortunately, it kind of is. Motors are electrically very noisy, and
controlling them with PWM signals feeds a bunch of that noise into the rest of the circuit, and I found that
this made a really loud whining sound in the audio output. I'm sure a real electrical engineer could design
isolation or filtering that would properly block or remove this noise, but I'm not a real electrical engineer.
I just play one on YouTube.

So instead of dealing with this electrically, I changed the clock divisors for Timer0 and Timer2 to 1 instead
of 64. That should push the noise from the motors (controlled by Timer0) and the LEDs on the color sensor
(Timer2) up to 62kHz, which is way above the audible range for humans. And it works! The noise still exists
electrically, but I can't hear it anymore, which is great. The downside of this is that you can't actually
use convenient timing functions like millis() anymore, because now they'll run way 64x too fast. But that's
fine because Mozzi provides convenient timing functions like EventDelay or the .ticks() method, so we can work
around this problem. I just want to get into this a little bit in UTH though because I want to be sure that 
people don't get really confused if they try to time something with millis().

That's a giant explanation that can probably be cut down to something like "you can't use millis() or similar
functions because I had to do some software magic to remove high pitched whining sounds from the audio output"
in the script. 

# Global variables and things, after the #include block at the top
A lot of the stuff is commented in the code, so I'll just do brief callouts for important bits:
- line 24: using Mozzi's EventDelay timer class for timekeeping functions
- line 33: I split the code apart into several smaller header files and libraries, mostly just for
    readability and organization. I have included those headers and libraries in the relevant sections.
- line 48: ColorSensor.h is a library I made that provides high level interaction with the color sensor. It
    makes it easy to update the sensor and get data from it. It's actually a wrapper around another library
    I made (CLS16D24.h) that I made to do the low level management of the color sensor.
- line 60ish: this is (I think) a really fun part for memory! I did this before I found much better ways to
    save memory, but this was still fun, and I left it in the code just to show that there are many different
    ways to do things. I wrote a longer note about it:

## how LED brightness PWM values are calculated

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
it was fun to figure out! And in the end, it actually turned out to be completely unnecessary, because I figured
out a way to save about 20% of the RAM use. In the end the best approach turned out to be storing the array
in flash memory, and you can see the effects on memory use here:

Array stored in RAM:
RAM:   [=======   ]  72.8% (used 1491 bytes from 2048 bytes)
Flash: [=======   ]  68.6% (used 21064 bytes from 30720 bytes)

Fancy bit manipulation version:
RAM:   [=======   ]  72.5% (used 1485 bytes from 2048 bytes)
Flash: [=======   ]  68.8% (used 21124 bytes from 30720 bytes)

Finally realizing I should just store the array in flash:
RAM:   [=======   ]  72.5% (used 1485 bytes from 2048 bytes)
Flash: [=======   ]  68.6% (used 21064 bytes from 30720 bytes)

## continuing through the global variables region of the main.cpp file:
- Line 108ish: MOZZI_CONTROL_RATE. The block comment says it all really.
- line 115: these are the wavetables that define the oscillator sounds. Directly related to memory and the lesson.
  Specifically, each one of these just maps out a single cycle of the waveform as a series of amplitude values. I
  made a spreadsheet that graphs one of these, the COS1024_8BIT waveform. This is a cosine wave constructed from 1024 
  samples. Each sample is an 8 bit number. So storing just this one waveform requires 1024 bytes (or exactly 1kb) of
  flash memory! There are only like 30kb of user accessible flash available, so that's a big chunk. And this isn't a
  particularly high-definition waveform. In the actual code, I use three oscillator waveforms that each use 2048
  samples at 8 bits per sample. That's 6kb of flash just spent storing the oscillator waveforms, or a full 16% of
  the available flash. And these samples are fine, but Mozzi offers much better ones, which use things like 4096 by
  16 bit samples. Just one of those takes up 8kb, or 25% of the entire flash on the microcontroller. But they sound
  way better because of the extra bit depth and increased sample resolution. 
    Here's the spreadsheet: 
    https://docs.google.com/spreadsheets/d/1Grb_DZuwk1KFbOl90frAgtAitHKfci-79SEsBOGwzho/editgid=1315902284#gid=1315902284

- Line 125, MusicTypes.h: this is a header that includes a few useful definitions. If you open MusicTypes.h:
### MusicTypes.h
First there's line 8, where I define an alias named MIDI_NOTE. I think the comments explain that well. After that
though is where I really think we could get into some memory stuff, because this is an area that helped me drastically
reduce the amount of RAM the program uses. I think it cut down total use by 20%. Specifically, I'm talking about the
Chord struct. I wrote up a pretty thorough explanation:

The original version of this code used something like 92% of the available RAM and ~70% of the flash. A really huge chunk 
of that RAM use came from the way I was storing chords and other note data in RAM. By switching to storing all of that in 
flash data and using tools called pointers, I was able to drastically reduce RAM use, which means there's more memory left
for hacking or adding new features.

Like let's say I had 7 notes in a scale I wanted to reference in my music generator function. Each note is represented by 
one byte, so that's 7 bytes. I can store all of that in RAM, or I can store it in flash. I have a lot more flash than RAM,
so I put it in flash. But I still need to pull that data out of flash and put it into RAM so the processor can do things
with it. If I copy all 7 bytes over to RAM, I'm duplicating memory use. Instead of copying the data to RAM to perform 
operations on, I can just give the processor an address of where to look to find the data in flash. All 7 bytes are stored 
next to each other in flash, so if I know the address of the first byte in flash, and I know how many bytes are in the scale, 
that's all the information I need to find any specific note within the scale.

Let's say that the array of notes in the scale gets put into flash memory starting at the 100th byte in flash memory. I can
store two bytes in RAM that hold that address. This is called a pointer. It's literally a variable that points you to an 
address in memory. It says, "start looking at this exact point in memory for the thing you want." It doesn't hold the actual 
data you want, it just tells you where to find it. Pointers on the ATmega328 are two bytes wide because the chip needs to be
able to address up to 32,768 bytes of flash, which is way more than the 0-255 range that a single byte can represent.

    POINTER = 100;

If I follow that pointer to address 100 in memory and then look at the contents of that specific byte in memory, I'll find the 
first note in my scale. From there I can find any other note in the scale because I know exactly how many notes are in it! 
If I want the second note in the scale, I just go to the address POINTER + 1 = 101. If I want the last note in the 7 note scale, 
I just go to address 106.

It's important to remember that C and C++ are 0-indexed. That means we start counting from 0. This case, the first element 
in the list of notes resides at address 100. Another way of saying this is that I don't have to add any offset to the base 
address to find the first element of the list. That means the first element in the list sits at index 0. The index is the 
relative position of an object in a list. If I want the second element in the list, I just add 1 to the base address.

    Base address:   100
    
    Note number:    note1   note2   note3   note4   note5   note6   note7
    Index:          0       1       2       3       4       5       6
    Address:        100+0   100+1   100+2   100+3   100+4   100+5   100+6
                    100     101     102     103     104     105     106

So the method of storing chords or scales in flash and then using pointers is great because I only need 3 bytes in RAM to be
able to find any note from a scale with any number of notes in it: I need a pointer to the starting address, which takes two 
bytes, and a single byte that tells me how many notes are in the scale. I don't have to have that second one either, but if 
I try to access the data from a byte that sits outside of the scale's region in memory, I won't get what I expect. C and C++ 
don't have a problem with letting you do this: you'll just get whatever value resides at that location in memory. This is 
called an overflow, which you might have heard of.

So to be safe, we store the number of notes in the scale along with a pointer to the memory address that contains the first 
note, and we can store those things together in a struct called Chord. That's 3 bytes total: 2 for the pointer and 1 for the 
note count.

But hey, now you can store any number of notes in sequence (up to 256 total) and only use 3 bytes of RAM for each sequence! 
This is great because the ATMega328 microcontroller has a lot more flash memory than RAM (32,768 bytes vs 2,048 bytes, 
respectively). This also means that even though I named this struct Chord, you don't have to only store musical chords in there. 
It's really just a collection of notes. You can store chords, scales, or even entire melodies in there!
```c
// This struct lets you store a group of notes in flash (PROGMEM) as a chord object. Because it just stores a pointer to the first
// element in the array, you can store an arbitrary number of notes in the chord, up to 255. And it should only take up 3 bytes of
// RAM per chord regardless of the number of notes in the chord, which is a huge gain in efficiency.
struct Chord {
    const uint8_t* notes;                       // pointer to the address of the first element in the PROGMEM array
    uint8_t numNotes;                           // the number of notes in the array

    uint8_t getNote(uint8_t index) const {      // retrieval function that returns the value of a note from the specified index in the array
        if (index >= numNotes) return 255;      // protection against overflows
        return pgm_read_byte(&notes[index]);    // the & in '&notes[index]' is the address-of operator: it computes the address 
                                                // of the element at that index, which pgm_read_byte then uses to fetch the value from flash.
    }
};
```

## still continuing with global variables and stuff
- line 127: include a header that provides some useful tools for setting up oscillators and things.  continue
  setting up other things related to oscillators.
- line 152: where we set up stuff related to the actual musical scales and things.
    I set this up to have 4 available scales that you can switch through using the middle button.
    Before we define those scales, include MusicTools.h, which provides a lot of useful things for music generation.
    
    ### MusicTools.h
    
    - Creates the ScaleStorage struct, which is a container into which we put all of the scales we want to use.
    - functions like noteNameToMIDINote(), which converts a named note ("C4") to it's MIDI note number.
    - line 90: macro to define chords

## back to globals and things
- line 165: define the scales the program will use! the comment block explains this well.
- line 178: put all of the scales into a container and set up a new Chord object that has the data for the first scale
    we'll use.
- line 187: function prototype for ambienceGenerator(), the function that actually turns color data into sound. that's
    defined at the end of the code.

# setup() function

- starting at line 201, the code starts the i2c peripheral, starts the connection to the table and arm encoders
  that measure angular position, and then homes the arm. This is crash homing. Could get into how it works, but it's
  just a closed loop variant of how crash homing works for SANDY, so we've covered it.
- line 213: set up the i2c connection to the color sensor.
- line 217: finally, after the i2c connections are set up, switch to I2C fast mode, which operates at 400kHz instead of
  100kHz. This makes the sensor communication take less time, and we need everything to be as fast as possible so the
  sound generation functions don't glitch. 400kHz is the max that these particular sensors can handle.
- then we set finish setting up the color sensor.
- line 240: here's where the Timer0 and Timer2 clock scalars get set to 1 instead of 64. After this point, millis()
  is broken.
- line 279: start mozzi finally!

# updateControl() function
In Mozzi, updateControl() is the function that gets called to handle all the user-defined behavior. This is the function
that will deal with button presses, potentiometer changes, encoder angle readings, and so on. This is where you put your
own code, for the most part. 

IMPORTANT NOTE: I think I want to call out the fact that you can't actually use mozziAnalogRead() until after startMozzi()
is called. mozziAnalogRead() is a much faster version of analogRead(), which matters for audio generation and being able
to read four analog pins. 

- line 304: set up some AutoMaps. The comments explain this pretty well, but they're cool because these are autoranging.
  for example, 
  
  static AutoMap autoGreenToUINT8_T(0, colorSensor.getResolution(), 0, 255);
    
  this sets up a mapping object, where later in the code you'll pass a sensor reading into it, and the map
  will squash that value down into a smaller range. In this case, the input value ranges from 0 to the 
  number returned by colorSensor.getResolution(), which 16383 for the default sensitivity settings. That's
  14 bits exactly. The output range of the map is 0..255, so the map takes in a 14 bit number and squishes
  it down into an 8 bit number, which is perfectly adequate for our uses.  

- then there's a big block that deals with buttons beinng pressed, starting at line 309. I don't think we need
  to get into this too much, other than to show that the left button changes light levels, the middle changes
  the scale being used, and the right changes into what I called button2Mode, which is currently an arpeggio thing.

- then it updates the table target speed based on potentiometer readings (line 347), updates the sensors occasionally (line 353),
  moves the arm if needed (line 380).
- Line 386: actually apply the autoMaps to the newest round of color sensor data
- 391: call ambienceGenerator()! This is the function that takes color data and makes sound.

# Make Some Noise! (that's no problem with ambienceGenerator())

- In the stock code, ambienceGenerator() does a few things. It mainly just uses