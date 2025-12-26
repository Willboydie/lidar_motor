/*------------------------------------------------------------------------------

  LIDARLite Arduino Library
  V3HP/v3HP_I2C

  This example shows methods for running the LIDAR-Lite v3 HP in various
  modes of operation. To exercise the examples open a serial terminal
  program (or the Serial Monitor in the Arduino IDE) and send ASCII
  characters to trigger the commands. See "loop" function for details.

  Connections:
  LIDAR-Lite 5 Vdc (red) to Arduino 5v
  LIDAR-Lite I2C SCL (green) to Arduino SCL
  LIDAR-Lite I2C SDA (blue) to Arduino SDA
  LIDAR-Lite Ground (black) to Arduino GND

  (Capacitor recommended to mitigate inrush current when device is enabled)
  680uF capacitor (+) to Arduino 5v
  680uF capacitor (-) to Arduino GND

  See the Operation Manual for wiring diagrams and more information:
  http://static.garmin.com/pumac/LIDAR_Lite_v3HP_Operation_Manual_and_Technical_Specifications.pdf

------------------------------------------------------------------------------*/

#include <stdint.h>
#include <Wire.h>
#include <LIDARLite_v3HP.h>
#include "Arduino.h"
#include "../../ldm/GarminV3HP.hpp"
#include "../../utils/tweakable_params.h"


// Defined in garmin_hardware_test.cpp
extern LIDARLite_v3HP myLidarLite;
// extern GarminV3HP myGarminV3HP;

#define FAST_I2C

void garmin_hardware_test();

enum rangeType_T
{
    RANGE_NONE,
    RANGE_SINGLE,
    RANGE_CONTINUOUS,
    RANGE_TIMER
};


void PrintMenu(void);

//---------------------------------------------------------------------
// Read Single Distance Measurement
//
// This is the simplest form of taking a measurement. This is a
// blocking function as it will not return until a range has been
// taken and a new distance measurement can be read.
//---------------------------------------------------------------------
uint8_t distanceSingle(uint16_t * distance);

//---------------------------------------------------------------------
// Read Continuous Distance Measurements
//
// The most recent distance measurement can always be read from
// device registers. Polling for the BUSY flag in the STATUS
// register can alert the user that the distance measurement is new
// and that the next measurement can be initiated. If the device is
// BUSY this function does nothing and returns 0. If the device is
// NOT BUSY this function triggers the next measurement, reads the
// distance data from the previous measurement, and returns 1.
//---------------------------------------------------------------------
uint8_t distanceContinuous(uint16_t * distance);

//---------------------------------------------------------------------
// Read Distance Measurement, Quickly
//
// Read distance. The approach is to poll the status register until the device goes
// idle after finishing a measurement, send a new measurement command, then read the
// previous distance data while it is performing the new command.
//---------------------------------------------------------------------
uint8_t distanceFast(uint16_t * distance);

//---------------------------------------------------------------------
// Print the correlation record for analysis
//---------------------------------------------------------------------
void dumpCorrelationRecord();

//---------------------------------------------------------------------
// Print peaks and calculated distances from the peak stack
//---------------------------------------------------------------------
void peakStackExample();