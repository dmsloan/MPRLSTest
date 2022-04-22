/*
  Basic test of the Qwiic MicroPressure Sensor
  By: Alex Wende
  SparkFun Electronics
  Date: July 2020
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).
  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/16476
  
  This example demonstrates how to get started with the Qwiic MicroPressure Sensor board, and read pressures in various units.
*/

// Include the SparkFun MicroPressure library.
// Click here to get the library: http://librarymanager/All#SparkFun_MicroPressure

#include<Wire.h>
#include <SparkFun_MicroPressure.h>

const String sketchName = "MPRLSTest";

/*
 * Initialize Constructor
 * Optional parameters:
 *  - EOC_PIN: End Of Conversion (defualt: -1)
 *  - RST_PIN: Reset (defualt: -1)
 *  - MIN_PSI: Minimum Pressure (default: 0 PSI)
 *  - MAX_PSI: Maximum Pressure (default: 25 PSI)
 */
//SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, MIN_PSI, MAX_PSI);
SparkFun_MicroPressure mpr; // Use default values with reset and EOC pins unused

double wPSI = 0;
double winH2O = 0;
double H2O = 27.707258364511;
double zeroOffset = 0.42590458737339;

// weighted average
//
// Tracks a weighted average in order to smooth out the values that it is given.
// it takes about ten readings to sabilize.
double wtgAverage (double wtgReading, double reading){
  double average;
  reading = reading*0.1;
  wtgReading = wtgReading*0.9;
  average = wtgReading + reading;
  return average;
}

void setup() {
  // Initalize UART, I2C bus, and connect to the micropressure sensor
  Serial.begin(115200);
    while (!Serial){};
  Serial.println();
  Serial.print("Sketch is called ");
  Serial.println(sketchName);

  Wire.begin();

  /* The micropressure sensor uses default settings with the address 0x18 using Wire.

     The mircropressure sensor has a fixed I2C address, if another address is used it
     can be defined here. If you need to use two micropressure sensors, and your
     microcontroller has multiple I2C buses, these parameters can be changed here.

     E.g. mpr.begin(ADDRESS, Wire1)

     Will return true on success or false on failure to communicate. */
  if(!mpr.begin())
  {
    Serial.println("Cannot connect to MicroPressure sensor.");
    while(1);
  }
}

void loop() {
  /* The micropressure sensor outputs pressure readings in pounds per square inch (PSI).
     Optionally, if you prefer pressure in another unit, the library can convert the
     pressure reading to: pascals, kilopascals, bar, torr, inches of murcury, and
     atmospheres.
   */

for (size_t i = 0; i < 10; i++)
{
  Serial.println(mpr.readPressure(), 20); // 2 decimal places by default
  wPSI = wtgAverage(wPSI, mpr.readPressure()+zeroOffset); // Calculate the time weighted average
  delay(100);
}

  Serial.printf("%4.20lf", wPSI);
  Serial.println(" Weighted average PSI corrected");
  Serial.print(mpr.readPressure()+zeroOffset, 20);
  Serial.println(" PSI corrected");
  Serial.print(wPSI*H2O, 20); // print the weighted average of H20
  Serial.println(" inH2O");
  Serial.print(((mpr.readPressure(KPA))*10)+29.3650876);
  Serial.println(" mb correct as of 7:00 PM 4/20/2022");
  Serial.print(mpr.readPressure(INHG)+0.8696,4);
  Serial.println(" inHg correct as of 7:00 PM 4/20/2022");
  Serial.println();
  delay(900);
}