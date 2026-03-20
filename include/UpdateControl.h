#ifndef UPDATE_CONTROL_H
#define UPDATE_CONTROL_H
#include <Arduino.h>
#include <Configuration.h>
#include <EventDelay.h>
#include <AutoMap.h>



void updateControl()
{
  static int8_t targetArmPos = 80;
  static bool initialize = true, nextButtonPressAllowed = true;
  static EventDelay buttonTimer;
  if (initialize)
  {
    buttonTimer.set(250); // 125ms is exactly 1/16th notes for 120bpm in 4/4
    // buttonTimer.start();
    extern arpTimeout.set(4000);
    arpTimeout.start();
    initialize = false;
  }

  // these are auto-ranging mappings for the color channels. These work a lot like the normal map() function, except that they keep track of the
  // largest and smallest values that they have seen so far, and update the map to reflect those. So the first two parameters are the minimum and
  // maximum possible input values (if this were a standard analogRead(), that would be 0 and 1023). As you update the function, it keeps track of
  // the mapping value you provide it, and the min and max values it has seen become the new min and max values for the input map range. Note that
  // the parameters are ints, so signed 16 bit values, and you have to be sure the numbers you pass into the parameters will fit in that variable size.
  //
  // Note that this only goes one way. If a new max value is seen, that's the stored max value until the next system reset. If you shine a really bright
  // light on the sensors and max out the readings for all the color channels, this mapping will never relax back to the lower light level readings
  // that occur once you stop shining the bright light. It's like if you were used to a dark room, walked outside into bright sunlight, and your eyes
  // adapted to the bright light and then never adapted to any dim light after that. This could be changed with some kind of relaxation function.
  //
  // The reason these are static variables inside updateControl() instead of being global is that I wanted to use the RGBCIR.getResolution() function
  // to be able to set the size of the mapping, rather than hardcoding the value. That way the maps get dynamically resized based on the chosen
  // resolution of the sensor.
  static AutoMap autoGreenToUINT8_T(0, RGBCIR.getResolution(), 0, 255);
  static AutoMap autoBlueToUINT8_T(0, RGBCIR.getResolution(), 0, 255);
  static AutoMap autoRedToUINT8_T(0, RGBCIR.getResolution(), 0, 255);
  static AutoMap autoWhiteToUINT8_T(0, RGBCIR.getResolution(), 0, 255);

  // check to see if buttons are pressed. returns 0, 1, 2, 255
  // 0 = B1, 1 = B2, 2 = B3, 255 = no press
  // deal with the button presses
  buttonPressed = getButtonPressed(mozziAnalogRead<10>(BUTTONS_PIN));
  if (nextButtonPressAllowed && buttonPressed < 255)
  {
    buttonTimer.start();            // start the timer if the button is pressed to prevent new button presses being reacted to within the timer window
    nextButtonPressAllowed = false; // prevent this block from being reentered until a new button press is allowed after buttonTimer is ready.
    switch (buttonPressed)
    {
    case 0: // left button, LED brightness levels
      // enableButton2Mode = false;
      brightnessIterator = (++brightnessIterator) % NUM_BRIGHTNESS_LEVELS;
      // analogWrite(LED_PIN, LEDBrightnessLevels[brightnessIterator]);
      analogWrite(LED_PIN, getBrightness(brightnessIterator));
      // currentButtonMode = 0;
      break;
    case 1: // middle button, scale selector
      // enableButton2Mode = false;
      scaleContainer.nextScale();
      currentScale = scaleContainer.selected();
      
      // scaleContainer.scaleSelector = (scaleContainer.scaleSelector + 1) % scaleContainer.numScales;
      // currentScale = scaleContainer.scaleArray[scaleContainer.scaleSelector];
      // numNotesInScale = scaleContainer.numNotesInSelectedScale[scaleContainer.scaleSelector];
      SERIAL_TABS(2);
      SERIAL_PRINT("scale: ");
      SERIAL_PRINTLN(scaleContainer.scaleSelector);
      // currentButtonMode = 1;
      break;
    case 2: // right button, all pizzicato I think, instead of sustained notes
      enableButton2Mode = !enableButton2Mode;
      // currentButtonMode = 2;
      break;
    default:
      break;
    }
  }

  // this timer prevents the buttons from being updated too frequently
  if (buttonTimer.ready())
  {
    nextButtonPressAllowed = true;
  }

  // update the sensors. This updates one sensor at a time whenever the timer is ready. Originally I just updated all three
  // sensors simultaneously and got similar performance, so I'm not sure if this is actually better or not. Worth experimenting
  // with to see if one big update or spread out updates lets you achieve more frequent updates without glitching sound output.
  static uint8_t currentSensorToUpdate = 0, tableStoppedCount = 0;

  // set the table speed based on the potentiometer position
  static int16_t targetTableSpeed = convertPotValToTableSpeed(mozziAnalogRead<10>(POT_B_PIN));
  // static int16_t savedTableSpeed = targetTableSpeed;
  static bool tableStopped = false;

  // read the target table speed, if the table hasn't been stopped
  if (!tableStopped)
    targetTableSpeed = convertPotValToTableSpeed(mozziAnalogRead<10>(POT_B_PIN));

  if (k_i2cUpdateDelay.ready())
  {
    switch (currentSensorToUpdate)
    {
    case 0: // update the arm encoder
      currentArmAngle = armEncoder.getCumulativePosition();
      currentArmPosition = convertArmAngleToRadius(currentArmAngle);
      currentSensorToUpdate++;
      break;
    case 1: // update the table encoder
      lastTableAngle = currentTableAngle;
      currentTableAngle = tableEncoder.getCumulativePosition();
      if (!tableStopped)
      {
        // check to see if it looks like the the table has stopped (with a deadband to account for noise)
        if (abs(currentTableAngle - lastTableAngle) <= 5)
        {
          tableStoppedCount++;
        }
        // if we've measured the same angular position for the table two loop iterations in a row, then we can assume it is being held in place.
        // remember that this check only happens every time this sensor updates, which means this should be a window about ~90ms, depending on the
        // sensor update frequency.
        if (tableStoppedCount == 2)
        {
          targetTableSpeed = 0;
          tableStoppedCount = 0;
          tableStopped = true;
        }
      }
      else
      {
        // if there's a large enough deviation in the table position, start it spinning again.
        if (abs(currentTableAngle - lastTableAngle) > 5)
        {
          targetTableSpeed = convertPotValToTableSpeed(mozziAnalogRead<10>(POT_B_PIN));
          tableStopped = false;
        }
      }
      currentSensorToUpdate++;
      break;
    case 2: // update the color sensor
      updateColorReadings(&colorData);
      // scaleColorData(&colorData);       // scale the blue and red channels to bring them in line with green channel (essential white balance correction)
      scaleColorDataFixedPoint(&colorData, &scaledFixedColorData);
      // printColorData();
      currentSensorToUpdate = 0;
      lastTableAngle = currentTableAngle;
      break;
    default:
      break;
    }
    k_i2cUpdateDelay.start(); // restart the timer immediately after new sensor data is acquired
  }

  // move the arm to the angle required by the potentiometer
  targetArmPos = convertPotValToArmRadius(mozziAnalogRead<10>(POT_A_PIN));
  int16_t targetArmAngle = convertArmRadiusToAngle(targetArmPos);
  moveArmToAngle(targetArmAngle, currentArmAngle);

  // set the approriate table motor speed
  tableMotor.setSpeed(targetTableSpeed);

  // AutoMap instances that handle the color data mapping to control signals. These bring the values all the way down to uint8_t.
  mappedGreen = autoGreenToUINT8_T(scaledFixedColorData.greenFixed.asInt());
  mappedBlue = autoBlueToUINT8_T(scaledFixedColorData.blueFixed.asInt());
  mappedRed = autoRedToUINT8_T(scaledFixedColorData.redFixed.asInt());
  mappedWhite = autoWhiteToUINT8_T(scaledFixedColorData.clearFixed.asInt());

  // call the function that will be used to convert color data to sound
  ambienceGenerator();
  // toneBeatsGenerator();

  previousEnableButton2Mode = enableButton2Mode;
}



#endif