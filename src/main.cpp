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

//Add BME280 temperature, humidity, and pressure support.

#include <Wire.h>
#include <Adafruit_MPRLS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//#define SEALEVELPRESSURE_HPA (1013.25) //this is the default
double SEALEVELPRESSURE_HPA (1015.8); // as reported at VNY 2023/03/18 11:00
//#define SEALEVELPRESSURE_HPA (1012.6) // as reported at BUR 2022/04/25 18:00
//#define SEALEVELPRESSURE_HPA (1016.93251660001) // calculated from main at IBE 2022/04/25

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
Adafruit_MPRLS mpr; // Use default values with reset and EOC pins unused

Adafruit_BME280 bme; // I2C

unsigned long delayTime;

double wHPa = 0;
double winH2O = 0;
double H2O = 27.707258364511;
double zeroOffset = 0.42590458737339;
double convertPaToH2O = 0.0040146307866177;
double convertHPaToH2O = 0.40146307866177;
double convertPaToInHg = 0.00029529983071445;
double convertHPaToInHg = 0.029529983071445;
double convertHPaToPsi = 0.0145038F;
double convertMtoF = 3.28084; // multiply meeters by value to obtain feet
double zeroMprPa;

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

/*
 * @brief returns temperature or relative humidity
 * @param "type" is character
 *     C = Celsius
 *     K = Keliven
 *     F = Fahrenheit
 *     H = Humidity
 *     P = Pressure
 *     A = Altitude
 * @return returns one of the values above
 * Usage: to get Fahrenheit type: getHTU('F')
 * to print it on serial monitor Serial.println(getBME('F'));
 * Written by Ahmad Shamshiri on July 13, 2019. Update july 25, 2019
 * in Ajax, Ontario, Canada
 * www.Robojax.com 
 */
float getBME(char type)
{
   // Robojax.com BME280 Code
  float value;
    float temp = bme.readTemperature();// read temperature
    float pressure = bme.readPressure() / 100.0F; // read pressure
    float rel_hum = bme.readHumidity();// read humidity
    float alt =bme.readAltitude(SEALEVELPRESSURE_HPA);// read altitude
   if(type =='F')
   {
    value = temp *9/5 + 32;//convert to Fahrenheit 
   }else if(type =='K')
   {
    value = temp + 273.15;//convert to Kelvin
   }else if(type =='H')
   {
    value = rel_hum;//return relative humidity
   }else if(type =='P')
   {
    value = pressure;//return pressure
   }else if(type =='A')
   {
    value = alt;//return approximate altitude
   }else{
    value = temp;// return Celsius
   }
   return value;
    // Robojax.com BME280 Code
}

void printValues() {
    Serial.printf("BME values\n");
    Serial.printf("Temperature = %2.1f °C.\n", bme.readTemperature());

    float tempF = getBME('F');
    Serial.printf("Temperature = %2.1f °F. \n", tempF);

    Serial.printf("Pressure = %3.4f hPa.\n", bme.readPressure() / 100.0F);

    Serial.printf("Approx. Altitude = %3.0f feet. Home is 1082. We are off by %1.0f.\n", bme.readAltitude(SEALEVELPRESSURE_HPA)*(convertMtoF), (bme.readAltitude(SEALEVELPRESSURE_HPA)*(convertMtoF))-1082);

    Serial.printf("Humidity = %2.0f%%\n", bme.readHumidity());

    Serial.printf("Pressure = %3.4f Psi.\n", (bme.readPressure() / 100.0F) * convertHPaToPsi);

    Serial.println();
}

void setup() {
  // Initalize UART, I2C bus, and connect to the micropressure sensor
  Serial.begin(115200);
    while (!Serial){};
  Serial.println();
  Serial.print("Sketch is called ");
  Serial.println(sketchName);

  /* To connect the I2C buss to different wires in the than the standard
  SDA and SCL see the following tutorial. The tutorial also addresses
  connecting two MPRLSs with the same address to two different I2C busses
  on the ESP32. 
  https://randomnerdtutorials.com/esp32-i2c-communication-arduino-ide/#7 */

  Wire.begin();

  /* The micropressure sensor uses default settings with the address 0x18 using Wire.

     The mircropressure sensor has a fixed I2C address. If you could use another address it
     can be defined here. If you need to use two micropressure sensors, and your
     microcontroller has multiple I2C buses, these parameters can be changed here.

     E.g. mpr.begin(ADDRESS, Wire1)

     Will return true on success or false on failure to communicate. */
  if(!mpr.begin())
  {
    Serial.println("Cannot connect to MicroPressure sensor, check wiring?");
    while(1);
  }
  else
  {
    Serial.println("Connected to MicroPressure sensor, MPRLS.");
  }

    unsigned status;
    
    // default settings
    status = bme.begin(0x76);  
    // You can also pass in a Wire library object like &Wire2
    // status = bme.begin(0x76, &Wire2)
    if (!status) {
        Serial.println("Could not find a valid BME280 sensor in space One, check wiring, address, sensor ID!");
        Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        delay(5000);
    }
    else
    {
      Serial.println("Connected to the BME280.");
    }
  
    Serial.print("Sensor One ID was: 0X"); Serial.print(bme.sensorID(),16); Serial.print(" with a status of ");
    Serial.println(status);

    // indoor navigation
    Serial.println("-- Indoor Navigation Scenario --");
    Serial.println("normal mode, 16x pressure / 2x temperature / 1x humidity oversampling,");
    Serial.println("0.5ms standby period, filter 16x");
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,  // temperature
                    Adafruit_BME280::SAMPLING_X16, // pressure
                    Adafruit_BME280::SAMPLING_X1,  // humidity
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_0_5 );
    // suggested rate is 1/60Hz (1m)
    delayTime = 1000; // in milliseconds
    Serial.println();

//read get the sealevel pressure from the current pressure based on given altitude.
  Serial.println(SEALEVELPRESSURE_HPA);
  Serial.printf("Standard altitude is %3.0f meters.\n", bme.readAltitude(SEALEVELPRESSURE_HPA));

  SEALEVELPRESSURE_HPA = bme.seaLevelForAltitude(330, bme.readPressure()/100);
  Serial.printf("Adjusted altitude is %3.0f meters.\n", bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(SEALEVELPRESSURE_HPA);
  Serial.println("Was that different?");

  for (int i = 0; i<30; i++){
  zeroMprPa = wtgAverage(zeroMprPa, (bme.readPressure()/100.0F) - mpr.readPressure()); // Calculate the time weighted average
  Serial.println(zeroMprPa);
  delay(100);
  }
  Serial.println();
}

void loop() {
  /* The micropressure sensor outputs pressure readings in pounds per square inch (PSI).
     Optionally, if you prefer pressure in another unit, the library can convert the
     pressure reading to: pascals, kilopascals, bar, torr, inches of murcury, and
     atmospheres.
   */

//float corrMPR = 0.8529;
//float corrBME = 0.8636;

float corrMPR = 0.0;
float corrBME = 0.0;

for (size_t i = 0; i < 10; i++)
{
//  Serial.println(mpr.readPressure(), 14); // 2 decimal places by default
  wHPa = wtgAverage(wHPa, mpr.readPressure()); // Calculate the time weighted average
  delay(100);
}

  Serial.printf("%4.20lf", wHPa);
  Serial.println(" Weighted average HPa on the MPR");
  Serial.print(mpr.readPressure(), 4);
  Serial.println("Raw uncorrected HPa read on MPR");
  Serial.print(bme.readPressure(), 4);
  serial.println("Raw uncorrected HPa read on BME)
  printValues();

float mprHPa = mpr.readPressure();
float bmeHPa = (bme.readPressure()/100.0F);

float mprV = (mprHPa * convertHPaToInHg) + corrMPR;
float bmeV = (bmeHPa * convertHPaToInHg) + corrBME;

float mprInH20 = (mprHPa + zeroMprPa) * convertHPaToH2O;
float bmeInH20 = bmeHPa * convertHPaToH2O;


  Serial.print(mprV, 4);
  Serial.println(" MPR InHg");
  Serial.print(bmeV, 4);
  Serial.println(" BME InHg");
  Serial.printf("Zero correction %4.4f \n", zeroMprPa);
  
  Serial.print(mprHPa, 4);
  Serial.println(" MPR HPa");
  Serial.print(bmeHPa, 4);
  Serial.println(" BME HPa");

  Serial.print(mprInH20, 4);
  Serial.println(" MPR H2O");
  Serial.print(bmeInH20, 4);
  Serial.println(" BME H2O");

  Serial.print(bmeHPa - mprHPa - zeroMprPa, 4);
  Serial.println(" HPa corrected difference");
  Serial.print("\033[1;32m"); //Set terminal color to green
  Serial.print((bmeHPa - mprHPa - zeroMprPa)*convertHPaToH2O, 3);
  Serial.print(" Differential pressure in inH2O\033[0m\n\n\a"); //Set terminal color back to white and insert a carrage return. \a is the Terminal bell. It works with Putty but not with the terminal built into PlatformIO
//  Serial.println("\033[1;32mbold green text\033[0m plain text\n");

  // Serial.print(mprInH20, 4);
  // Serial.println(" MPR InH2O");
  // Serial.print(bmeInH20, 4);
  // Serial.println(" BME InH2O");
  // Serial.print(mprInH20 - bmeInH20, 4);
  // Serial.println(" InH2O difference");
  delay(900);
}