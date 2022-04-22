# Repo to hold board information
### 

## Contents
- [Notes](#notes)
- [Installation Instructions](#installation-instructions)
- [Decoding Exceptions](#decoding-exceptions)
- [Issue/Bug report template](#issuebug-report-template)
- [Arduino Nano](#ARDUINO-NANO)
- [WEMOS LOLIN32 with OLED Board](#WEMOS-LOLIN32)
- [WeMos d1 Mini Pro](#d1_Mini_Pro)
- [Heltec WiFi LORA 32 V1](#Heltec-WiFi-LORA-32-V1)
- [Heltec WiFi LORA 32 V2](#Heltec-WiFi-LORA-32-V2)
- [Adafruit ADS1115](#Adafruit-ADS1115)
- [BME280/BMP280 Pressure/Temperature/Humidity sensor](#BME280/BMP280)
- [MPRLS Pressure sensor](#MPRLS)
- [LSM303DLHC e-Compass 3 Axis Accelerometer + 3 Axis Magnetometer Module Sensor](#LSM303DLHC) 

## Notes

You may have to push the button to upload to the board
ADS1115 does not seem to work with HW enabled on the OLED on the WiFi LORA
## Installation Instructions
- Use VS Code and PlatformIO

#### Decoding exceptions

I have no idea how to do this or what they are talking about.

#### Issue/Bug report template

Finally, if you're sure no one else had the issue, it's probably you.

## ARDUINO NANO
Board name ARDUINO NANO atmega328.<br/>
<ul>
</ul>

![Pin Functions](docs/arduino-nano-pinout.png)

## WEMOS LOLIN32
Board name WEMOS LOLIN32.<br/>
<ul>
    <li>OLED Connections:</li>
        <li>CLOCK 4</li>
        <li>DATA 5</li>
        <li>RESET 16</li>
</ul>

![Pin Functions](docs/WemosESP32OLEDTop.jpg)
![Pin Functions](docs/WemosESP32OLEDBottom.jpg)
![Pin Functions](docs/WemosESP32OLEDPinout.jpg)
## - #WEMOS LOLIN32V100
Board name WeMos LOLIN32 V1.0.0<br/>
![Pin Functions](docs/ESP32WeMosLOLIN32Top.jpg)
![Pin Functions](docs/ESP32WeMosLOLIN32Bottom.jpg)
![Pin Functions](docs/ESP32WeMosLOLIN32Pinout.png)
## - #d1_Mini_Pro
Board name WeMos d1 Mini Pro<br/>
![Pin Functions](docs/wemos_d1_mini_pro_pinout.png)
## Heltec WiFi LORA 32 V1
Heltec WiFi LORA 32 V1 – ESP32 with OLED and LORA Board<br/>
To program you need to hold button labeled PRG near coil antenna.<br/>
<ul>
    <li>I2C Connections:</li>
        <li>SCL 22</li>
        <li>SDA 21</li>
    <li>OLED Connections:</li>
        <li>CLOCK 15</li>
        <li>DATA 4</li>
        <li>RESET 16</li>
</ul>

![Pin Functions](docs/WiFi-LORA-32-pinout-Diagram.png)
## Heltec WiFi LORA 32 V2
Heltec WiFi LORA 32 V2 – ESP32 with OLED and LORA Board<br/>
To program you need to hold button labeled PRG near coil antenna.<br/>
I have unresolved issues using a second I2C device with the OLED<br/>
<ul>
    <li>I2C Connections:</li>
        <li>SCL 22</li>
        <li>SDA 21</li>
    <li>OLED Connections:</li>
        <li>CLOCK 15</li>
        <li>DATA 4</li>
        <li>RESET 16</li>
</ul>

![Pin Functions](docs/WIFI_LoRa_32_V2PinDiagram.png)
## Adafruit ADS1115
I2C address is 0x48<br/>
![Pin Functions](docs/AdafruitADS1015ADS1115PinDiagram.jpg)<br/>
Go to [Adafruit](https://www.adafruit.com/product/1085) for more information.<br/>
## BME280/BMP280
I2C address is 0x76 or 0x77 if you cut the trace between the left two pads and add a jumper to the right two pads.<br/>
The BME280 has Pressure, Temperature, and Humidity. The BMP only hase Pressure.<br/>
![Pin Functions](docs/BMP280.jpg)<br/>
Go to [Adafruit](https://learn.adafruit.com/adafruit-bme280-humidity-barometric-pressure-temperature-sensor-breakout) for more information.<br/>

## MPRLS
I2C address is 0x18 and cannot be changed.<br/>
The MPRLS has Pressure sensing of 0-25 PSI with a push on a tube connection.<br/>
![Pin Functions](docs/MPRLS3965-00.jpg)<br/>
Go to [Adafruit](https://www.adafruit.com/products/3965) or [Sparkfun](https://www.sparkfun.com/products/16476) for more information.<br/>

## LSM303DLHC
![Pin Functions](docs/LSM303DLHCe-Compass3AxisAccelerometerAnd3AxisMagnetometerModule.jpg)![Pin Functions](docs/LSM303DLHCe-Compass3AxisAccelerometerAnd3AxisMagnetometerModule61VO3bK8u+L._AC_SX679_.jpg) <br/>
This board is a knock off and is completely untested.<br/>
Go to [Adafruit](https://www.adafruit.com/product/1120) for more information. New information is [LSM303 Accelerometer + Compass Breakout.](https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/coding)<br/>

## Hint
Keep at it.