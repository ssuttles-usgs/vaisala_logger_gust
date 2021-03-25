/*****************************************************************************
menu_a_la_carte.ino
Written By:  Sara Damiano (sdamiano@stroudcenter.org)
Development Environment: PlatformIO
Hardware Platform: EnviroDIY Mayfly Arduino Datalogger
Software License: BSD-3.
  Copyright (c) 2017, Stroud Water Research Center (SWRC)
  and the EnviroDIY Development Team

This example sketch is written for ModularSensors library version 0.23.16

This shows most of the standard functions of the library at once.

DISCLAIMER:
THIS CODE IS PROVIDED "AS IS" - NO WARRANTY IS GIVEN.
*****************************************************************************/

// ==========================================================================
//    Defines for the Arduino IDE
//    In PlatformIO, set these build flags in your platformio.ini
// ==========================================================================
#ifndef TINY_GSM_RX_BUFFER
#define TINY_GSM_RX_BUFFER 64
#endif
#ifndef TINY_GSM_YIELD_MS
#define TINY_GSM_YIELD_MS 2
#endif
#ifndef MQTT_MAX_PACKET_SIZE
#define MQTT_MAX_PACKET_SIZE 240
#endif


// ==========================================================================
//    Include the base required libraries
// ==========================================================================
#include <Arduino.h>  // The base Arduino library
#include <EnableInterrupt.h>  // for external and pin change interrupts
#include <LoggerBase.h>  // The modular sensors library


// ==========================================================================
//    Data Logger Settings
// ==========================================================================
// The library version this example was written for
const char *libraryVersion = "0.23.16";
// The name of this file
const char *sketchName = "vaisala_logger_gust.ino";
// Logger ID, also becomes the prefix for the name of the data file on SD card
const char *LoggerID = "HoM";
// How frequently (in minutes) to log data
const uint8_t loggingInterval = 1;
// Your logger's timezone.
const int8_t timeZone = 0;  // UTC
// NOTE:  Daylight savings time will not be applied!  Please use standard time!


// ==========================================================================
//    Primary Arduino-Based Board and Processor
// ==========================================================================
#include <sensors/ProcessorStats.h>

const long serialBaud = 57600;   // Baud rate for the primary serial port for debugging
const int8_t greenLED = 8;        // MCU pin for the green LED (-1 if not applicable)
const int8_t redLED = 9;          // MCU pin for the red LED (-1 if not applicable)
const int8_t buttonPin = 21;      // MCU pin for a button to use to enter debugging mode  (-1 if not applicable)
const int8_t wakePin = A7;        // MCU interrupt/alarm pin to wake from sleep
// Set the wake pin to -1 if you do not want the main processor to sleep.
// In a SAMD system where you are using the built-in rtc, set wakePin to 1
const int8_t sdCardPwrPin = -1;    // MCU SD card power pin (-1 if not applicable)
const int8_t sdCardSSPin = 12;     // MCU SD card chip select/slave select pin (must be given!)
const int8_t sensorPowerPin = 22;  // MCU pin controlling main sensor power (-1 if not applicable)

// Create the main processor chip "sensor" - for general metadata
const char *mcuBoardVersion = "v0.5b";
ProcessorStats mcuBoard(mcuBoardVersion);

// Create sample number, battery voltage, and free RAM variable pointers for the processor
// Variable *mcuBoardBatt = new ProcessorStats_Battery(&mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *mcuBoardAvailableRAM = new ProcessorStats_FreeRam(&mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *mcuBoardSampNo = new ProcessorStats_SampleNumber(&mcuBoard, "12345678-abcd-1234-ef00-1234567890ab");


// ==========================================================================
//    Settings for Additional Serial Ports
// ==========================================================================

// The modem and a number of sensors communicate over UART/TTL - often called "serial".
// "Hardware" serial ports (automatically controlled by the MCU) are generally
// the most accurate and should be configured and used for as many peripherals
// as possible.  In some cases (ie, modbus communication) many sensors can share
// the same serial port.

#if defined(ARDUINO_ARCH_AVR) || defined(__AVR__)  // For AVR boards
// Unfortunately, most AVR boards have only one or two hardware serial ports,
// so we'll set up three types of extra software serial ports to use

// AltSoftSerial by Paul Stoffregen (https://github.com/PaulStoffregen/AltSoftSerial)
// is the most accurate software serial port for AVR boards.
// AltSoftSerial can only be used on one set of pins on each board so only one
// AltSoftSerial port can be used.
// Not all AVR boards are supported by AltSoftSerial.
#include <AltSoftSerial.h>
AltSoftSerial altSoftSerial;

// NeoSWSerial (https://github.com/SRGDamia1/NeoSWSerial) is the best software
// serial that can be used on any pin supporting interrupts.
// You can use as many instances of NeoSWSerial as you want.
// Not all AVR boards are supported by NeoSWSerial.
#include <NeoSWSerial.h>  // for the stream communication
const int8_t neoSSerial1Rx = 11;     // data in pin
const int8_t neoSSerial1Tx = -1;     // data out pin
NeoSWSerial neoSSerial1(neoSSerial1Rx, neoSSerial1Tx);
// To use NeoSWSerial in this library, we define a function to receive data
// This is just a short-cut for later
void neoSSerial1ISR()
{
    NeoSWSerial::rxISR(*portInputRegister(digitalPinToPort(neoSSerial1Rx)));
}

// The "standard" software serial library uses interrupts that conflict
// with several other libraries used within this program, we must use a
// version of software serial that has been stripped of interrupts.
// NOTE:  Only use if necessary.  This is not a very accurate serial port!
const int8_t softSerialRx = A3;     // data in pin
const int8_t softSerialTx = A4;     // data out pin

#include <SoftwareSerial_ExtInts.h>  // for the stream communication
SoftwareSerial_ExtInts softSerial1(softSerialRx, softSerialTx);
#endif  // End software serial for avr boards


// The SAMD21 has 6 "SERCOM" ports, any of which can be used for UART communication.
// The "core" code for most boards defines one or more UART (Serial) ports with
// the SERCOMs and uses others for I2C and SPI.  We can create new UART ports on
// any available SERCOM.  The table below shows definitions for select boards.

// Board =>   Arduino Zero       Adafruit Feather    Sodaq Boards
// -------    ---------------    ----------------    ----------------
// SERCOM0    Serial1 (D0/D1)    Serial1 (D0/D1)     Serial (D0/D1)
// SERCOM1    Available          Available           Serial3 (D12/D13)
// SERCOM2    Available          Available           I2C (A4/A5)
// SERCOM3    I2C (D20/D21)      I2C (D20/D21)       SPI (D11/12/13)
// SERCOM4    SPI (D21/22/23)    SPI (D21/22/23)     SPI1/Serial2
// SERCOM5    EDBG/Serial        Available           Serial1

// If using a Sodaq board, do not define the new sercoms, instead:
// #define ENABLE_SERIAL2
// #define ENABLE_SERIAL3


#if defined ARDUINO_ARCH_SAMD
#include <wiring_private.h> // Needed for SAMD pinPeripheral() function

#ifndef ENABLE_SERIAL2
// Set up a 'new' UART using SERCOM1
// The Rx will be on digital pin 11, which is SERCOM1's Pad #0
// The Tx will be on digital pin 10, which is SERCOM1's Pad #2
// NOTE:  SERCOM1 is undefinied on a "standard" Arduino Zero and many clones,
//        but not all!  Please check the variant.cpp file for you individual board!
//        Sodaq Autonomo's and Sodaq One's do NOT follow the 'standard' SERCOM definitions!
Uart Serial2(&sercom1, 11, 10, SERCOM_RX_PAD_0, UART_TX_PAD_2);
// Hand over the interrupts to the sercom port
void SERCOM1_Handler()
{
    Serial2.IrqHandler();
}
#endif

#ifndef ENABLE_SERIAL3
// Set up a 'new' UART using SERCOM2
// The Rx will be on digital pin 5, which is SERCOM2's Pad #3
// The Tx will be on digital pin 2, which is SERCOM2's Pad #2
// NOTE:  SERCOM2 is undefinied on a "standard" Arduino Zero and many clones,
//        but not all!  Please check the variant.cpp file for you individual board!
//        Sodaq Autonomo's and Sodaq One's do NOT follow the 'standard' SERCOM definitions!
Uart Serial3(&sercom2, 5, 2, SERCOM_RX_PAD_3, UART_TX_PAD_2);
// Hand over the interrupts to the sercom port
void SERCOM2_Handler()
{
    Serial3.IrqHandler();
}
#endif

#endif  // End hardware serial on SAMD21 boards


// ==========================================================================
//    Wifi/Cellular Modem Settings
// ==========================================================================

// Create a reference to the serial port for the modem
// Extra hardware and software serial ports are created in the "Settings for Additional Serial Ports" section
HardwareSerial &modemSerial = Serial1;  // Use hardware serial if possible
// AltSoftSerial &modemSerial = altSoftSerial;  // For software serial if needed
// NeoSWSerial &modemSerial = neoSSerial1;  // For software serial if needed


// Modem Pins - Describe the physical pin connection of your modem to your board
// const int8_t modemVccPin = -1;      // MCU pin controlling modem power (-1 if not applicable)
// const int8_t modemStatusPin = 19;   // MCU pin used to read modem status (-1 if not applicable)
// const int8_t modemResetPin = 20;    // MCU pin connected to modem reset pin (-1 if unconnected)
// const int8_t modemSleepRqPin = 23;  // MCU pin used for modem sleep/wake request (-1 if not applicable)
// const int8_t modemLEDPin = redLED;  // MCU pin connected an LED to show modem status (-1 if unconnected)

// ALL Sodaq uBees with Mayfly
// const int8_t modemVccPin = 23;      // MCU pin controlling modem power (UBee pin 9/Mayfly pin 23 is connected to a MIC94040 which controls the power supply to the U201)
// const int8_t modemStatusPin = 19;   // MCU pin used to read modem status (UBee pin 12/Mayfly pin 19 is connected to U201's v_int via another MIC94040)
// const int8_t modemResetPin = -1;    // MCU pin connected to modem reset pin (UBee pin 5 is connected to reset_N on the U201, but this pin is not connected to the Mayfly mcu)
// const int8_t modemSleepRqPin = 20;  // MCU pin used for modem sleep/wake request (UBee pin 16/Mayfly pin 20 is directly connected to the U201's pwr_on pin - this (and only this) pin on the U201 is 5.5V tolerant)
// const int8_t modemLEDPin = redLED;  // MCU pin connected an LED to show modem status (Optional, but I like to use it)

// GPRSbee R6 and higher
// const int8_t modemVccPin = -1;      // MCU pin controlling modem power
// const int8_t vRefPin = -1;          // MCU pin controlling voltage reference for modem communication
// const int8_t modemStatusPin = 19;   // MCU pin used to read modem status
// const int8_t modemLEDPin = redLED;  // MCU pin connected an LED to show modem status

// SIMCom SIM800 - couldn't get the GPRSbee R7 to work
const int8_t modemVccPin = -1;      // MCU pin controlling modem power
const int8_t modemStatusPin = 19;   // MCU pin used to read modem status
const int8_t modemResetPin = -1;    // MCU pin connected to modem reset pin
const int8_t modemSleepRqPin = 23;  // MCU pin used for modem sleep/wake request
const int8_t modemLEDPin = redLED;  // MCU pin connected an LED to show modem status

// Network connection information
const char *apn = "hologram";  // The APN for the gprs connection, unnecessary for WiFi
const char *wifiId = "xxxxx";  // The WiFi access point, unnecessary for gprs
const char *wifiPwd = "xxxxx";  // The password for connecting to WiFi, unnecessary for gprs


// ==========================================================================
//    The modem object
//    Note:  Don't use more than one!
// ==========================================================================

#if not defined MS_BUILD_TESTING || defined MS_BUILD_TEST_XBEE_CELLULAR
// For any Digi Cellular XBee's
// NOTE:  The u-blox based Digi XBee's (3G global and LTE-M global) can be used
// in either bypass or transparent mode, each with pros and cons
// The Telit based Digi XBees (LTE Cat1) can only use this mode.
#include <modems/DigiXBeeCellularTransparent.h>
const long modemBaud = 9600;  // All XBee's use 9600 by default
const bool useCTSforStatus = false;   // Flag to use the XBee CTS pin for status
// NOTE:  If possible, use the STATUS/SLEEP_not (XBee pin 13) for status, but
// the CTS pin can also be used if necessary
DigiXBeeCellularTransparent modemXBCT(&modemSerial,
                                      modemVccPin, modemStatusPin, useCTSforStatus,
                                      modemResetPin, modemSleepRqPin,
                                      apn);
// Create an extra reference to the modem by a generic name (not necessary)
DigiXBeeCellularTransparent modem = modemXBCT;
// ==========================================================================

#elif defined MS_BUILD_TESTING && defined MS_BUILD_TEST_S2GB
// // For the Sodaq 2GBee R6 and R7 based on the SIMCom SIM800
// // NOTE:  The Sodaq GPRSBee doesn't expose the SIM800's reset pin
// #include <modems/Sodaq2GBeeR6.h>
// const long modemBaud = 9600;  //  SIM800 does auto-bauding by default
// Sodaq2GBeeR6 modem2GB(&modemSerial,
//                       modemVccPin, modemStatusPin,
//                       apn);
// // Create an extra reference to the modem by a generic name (not necessary)
// Sodaq2GBeeR6 modem = modem2GB;

#include <modems/SIMComSIM800.h>
const long modemBaud = 9600;  //  SIM800 does auto-bauding by default
SIMComSIM800 modemS800(&modemSerial,
                       modemVccPin, modemStatusPin,
                       modemResetPin, modemSleepRqPin,
                       apn);
// Create an extra reference to the modem by a generic name (not necessary)
SIMComSIM800 modem = modemS800;
// ==========================================================================

#elif defined MS_BUILD_TESTING && defined MS_BUILD_TEST_UBEE_R410M
// For the Sodaq UBee based on the 4G LTE-M u-blox SARA R410M
#include <modems/SodaqUBeeR410M.h>
const long modemBaud = 115200;  // Default baud rate of the SARA R410M is 115200
// NOTE:  The SARA R410N DOES NOT save baud rate to non-volatile memory.  After
// every power loss, the module will return to the default baud rate of 115200.
// NOTE:  115200 is TOO FAST for an 8MHz Arduino.  This library attempts to
// compensate by sending a baud rate change command in the wake function.
// Because of this, 8MHz boards, LIKE THE MAYFLY, *MUST* use a HardwareSerial
// instance as modemSerial.
SodaqUBeeR410M modemR410(&modemSerial,
                         modemVccPin, modemStatusPin,
                         modemResetPin, modemSleepRqPin,
                         apn);
// Create an extra reference to the modem by a generic name (not necessary)
SodaqUBeeR410M modem = modemR410;
// ==========================================================================

#elif defined MS_BUILD_TESTING && defined MS_BUILD_TEST_UBEE_U201
// For the Sodaq UBee based on the 3G u-blox SARA U201
#include <modems/SodaqUBeeU201.h>
const long modemBaud = 9600;  //  SARA U2xx module does auto-bauding by default
SodaqUBeeU201 modemU201(&modemSerial,
                        modemVccPin, modemStatusPin,
                        modemResetPin, modemSleepRqPin,
                        apn);
// Create an extra reference to the modem by a generic name (not necessary)
SodaqUBeeU201 modem = modemU201;
// ==========================================================================
#endif


// Create RSSI and signal strength variable pointers for the modem
// Variable *modemRSSI = new Modem_RSSI(&modem, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *modemSignalPct = new Modem_SignalPercent(&modem, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *modemSignalPct = new Modem_BatteryState(&modem, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *modemSignalPct = new Modem_BatteryPercent(&modem, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *modemSignalPct = new Modem_BatteryVoltage(&modem, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *modemSignalPct = new Modem_Temp(&modem, "12345678-abcd-1234-ef00-1234567890ab");
// Variable *modemSignalPct = new Modem_ActivationDuration(&modem, "12345678-abcd-1234-ef00-1234567890ab");


// ==========================================================================
//    Maxim DS3231 RTC (Real Time Clock)
// ==========================================================================
#include <sensors/MaximDS3231.h>

// Create a DS3231 sensor object
MaximDS3231 ds3231(1);

// Create a temperature variable pointer for the DS3231
// Variable *ds3231Temp = new MaximDS3231_Temp(&ds3231, "12345678-abcd-1234-ef00-1234567890ab");


// ==========================================================================
//    Vaisala WXT520
// ==========================================================================
#include <sensors/VaisalaWXT536RG.h>

const char *WXTSDI12address = "0";  // The SDI-12 Address of the CTD
const uint8_t WXTNumberReadings = 1;  // The number of readings to average
const int8_t SDI12Power = -1;  // Pin to switch power on and off (-1 if unconnected)
const int8_t SDI12Data = 7;  // The SDI12 data pin

// Create a Vaisala WXT520 sensor object
VaisalaWXT536RG wxt(*WXTSDI12address, SDI12Power, SDI12Data, WXTNumberReadings);

// ==========================================================================
//    Calculated Variables
// ==========================================================================

// Create the function to give your calculated result.
// The function should take no input (void) and return a float.
// You can use any named variable pointers to access values by way of variable->getValue()

// float calculateVariableValue(void)
// {
//     float calculatedResult = -9999;  // Always safest to start with a bad value
//     // float inputVar1 = variable1->getValue();
//     // float inputVar2 = variable2->getValue();
//     // if (inputVar1 != -9999 && inputVar2 != -9999)  // make sure both inputs are good
//     // {
//     //     calculatedResult = inputVar1 + inputVar2;
//     // }
//     return calculatedResult;
// }

// Properties of the calculated variable
// const uint8_t calculatedVarResolution = 3;  // The number of digits after the decimal place
// const char *calculatedVarName = "varName";  // This must be a value from http://vocabulary.odm2.org/variablename/
// const char *calculatedVarUnit = "varUnit";  // This must be a value from http://vocabulary.odm2.org/units/
// const char *calculatedVarCode = "calcVar";  // A short code for the variable
// const char *calculatedVarUUID = "12345678-abcd-1234-ef00-1234567890ab";  // The (optional) universallly unique identifier

// // Finally, Create a calculated variable pointer and return a variable pointer to it
// Variable *calculatedVar = new Variable(calculateVariableValue, calculatedVarResolution,
//                                        calculatedVarName, calculatedVarUnit,
//                                        calculatedVarCode, calculatedVarUUID);


// ==========================================================================
//    Creating the Variable Array[s] and Filling with Variable Objects
// ==========================================================================

// FORM1: Create pointers for all of the variables from the sensors,
// at the same time putting them into an array
// NOTE:  Forms one and two can be mixed
Variable *variableList[] = {
    new ProcessorStats_SampleNumber(&mcuBoard, "sample"),
    new ProcessorStats_Battery(&mcuBoard, "boardbatt"),
    new MaximDS3231_Temp(&ds3231, "boardtemp"),
    new Modem_SignalPercent(&modem, "signalpct"),
    new VaisalaWXT536RG_Dn(&wxt, "Dn"),
    new VaisalaWXT536RG_Dm(&wxt, "Dm"),
    new VaisalaWXT536RG_Dx(&wxt, "Dx"),
    new VaisalaWXT536RG_Sn(&wxt, "Sn"),
    new VaisalaWXT536RG_Sm(&wxt, "Sm"),
    new VaisalaWXT536RG_Sx(&wxt, "Sx"),
    //new VaisalaWXT536RG_Pa(&wxt, "Pa"),
    new VaisalaWXT536RG_Ta(&wxt, "Ta"),
    new VaisalaWXT536RG_Ua(&wxt, "Ua"),
    //new VaisalaWXT536RG_Rc(&wxt, "Rc"),
};

// Count up the number of pointers in the array
int variableCount = sizeof(variableList) / sizeof(variableList[0]);

// Create the VariableArray object
VariableArray varArray(variableCount, variableList);
// VariableArray varArray(variableCount, variableList, UUIDs);


// ==========================================================================
//     The Logger Object[s]
// ==========================================================================

// Create a new logger instance
Logger dataLogger(LoggerID, loggingInterval, &varArray);


// ==========================================================================
//    A Publisher to Monitor My Watershed / EnviroDIY Data Sharing Portal
// ==========================================================================
// Device registration and sampling feature information can be obtained after
// registration at https://monitormywatershed.org or https://data.envirodiy.org
const char *registrationToken = "*Y^,[6mb";   // Make this the Hologram Device Key
const char *samplingFeature = "12345678-abcd-1234-ef00-1234567890ab";     // Sampling feature UUID

// Publisher for Hologram
// Create a data publisher for the EnviroDIY/WikiWatershed POST endpoint
#include <publishers/HologramPublisher.h>
HologramPublisher HologramAPI(dataLogger, &modem.gsmClient, registrationToken, samplingFeature);

// ==========================================================================
//    Working Functions
// ==========================================================================

// Flashes the LED's on the primary board
void greenredflash(uint8_t numFlash = 4, uint8_t rate = 75)
{
    for (uint8_t i = 0; i < numFlash; i++) {
        digitalWrite(greenLED, HIGH);
        digitalWrite(redLED, LOW);
        delay(rate);
        digitalWrite(greenLED, LOW);
        digitalWrite(redLED, HIGH);
        delay(rate);
    }
    digitalWrite(redLED, LOW);
}


// Read's the battery voltage
// NOTE: This will actually return the battery level from the previous update!
float getBatteryVoltage()
{
    if (mcuBoard.sensorValues[0] == -9999) mcuBoard.update();
    return mcuBoard.sensorValues[0];
}


// ==========================================================================
// Main setup function
// ==========================================================================
void setup()
{
    // Wait for USB connection to be established by PC
    // NOTE:  Only use this when debugging - if not connected to a PC, this
    // could prevent the script from starting
    #if defined SERIAL_PORT_USBVIRTUAL
      while (!SERIAL_PORT_USBVIRTUAL && (millis() < 10000)){}
    #endif

    // Start the primary serial connection
    Serial.begin(serialBaud);

    // Print a start-up note to the first serial port
    Serial.print(F("Now running "));
    Serial.print(sketchName);
    Serial.print(F(" on Logger "));
    Serial.println(LoggerID);
    Serial.println();

    Serial.print(F("Using ModularSensors Library version "));
    Serial.println(MODULAR_SENSORS_VERSION);

    if (String(MODULAR_SENSORS_VERSION) !=  String(libraryVersion))
        Serial.println(F(
            "WARNING: THIS EXAMPLE WAS WRITTEN FOR A DIFFERENT VERSION OF MODULAR SENSORS!!"));

    // Allow interrupts for software serial
    #if defined SoftwareSerial_ExtInts_h
        enableInterrupt(softSerialRx, SoftwareSerial_ExtInts::handle_interrupt, CHANGE);
    #endif
    #if defined NeoSWSerial_h
        enableInterrupt(neoSSerial1Rx, neoSSerial1ISR, CHANGE);
    #endif

    // Start the serial connection with the modem
    modemSerial.begin(modemBaud);

    // Start the stream for the modbus sensors; all currently supported modbus sensors use 9600 baud
    // modbusSerial.begin(9600);

    // Start the SoftwareSerial stream for the sonar; it will always be at 9600 baud
    // sonarSerial.begin(9600);

    // Assign pins SERCOM functionality for SAMD boards
    // NOTE:  This must happen *after* the various serial.begin statements
    #if defined ARDUINO_ARCH_SAMD
    #ifndef ENABLE_SERIAL2
    pinPeripheral(10, PIO_SERCOM);  // Serial2 Tx/Dout = SERCOM1 Pad #2
    pinPeripheral(11, PIO_SERCOM);  // Serial2 Rx/Din = SERCOM1 Pad #0
    #endif
    #ifndef ENABLE_SERIAL3
    pinPeripheral(2, PIO_SERCOM);  // Serial3 Tx/Dout = SERCOM2 Pad #2
    pinPeripheral(5, PIO_SERCOM);  // Serial3 Rx/Din = SERCOM2 Pad #3
    #endif
    #endif

    // Set up pins for the LED's
    pinMode(greenLED, OUTPUT);
    digitalWrite(greenLED, LOW);
    pinMode(redLED, OUTPUT);
    digitalWrite(redLED, LOW);
    // Blink the LEDs to show the board is on and starting up
    greenredflash();

    // Set the timezones for the logger/data and the RTC
    // Logging in the given time zone
    Logger::setLoggerTimeZone(timeZone);
    // It is STRONGLY RECOMMENDED that you set the RTC to be in UTC (UTC+0)
    Logger::setRTCTimeZone(0);

    // Attach the modem and information pins to the logger
    dataLogger.attachModem(modem);
    modem.setModemLED(modemLEDPin);
    dataLogger.setLoggerPins(wakePin, sdCardSSPin, sdCardPwrPin, buttonPin, greenLED);

    // Begin the logger
    dataLogger.begin();

    // Note:  Please change these battery voltages to match your battery
    // Check that the battery is OK before powering the modem
  /*   if (getBatteryVoltage() > 3.55 || !dataLogger.isRTCSane())
    {
        modem.modemPowerUp();
        modem.wake();
        modem.setup();

        // Synchronize the RTC with NIST
        Serial.println(F("Attempting to connect to the internet and synchronize RTC with NIST"));
        if (modem.connectInternet(120000L))
        {
            dataLogger.setRTClock(modem.getNISTTime());
        }
        else
        {
            Serial.println(F("Could not connect to internet for clock sync."));
        }
    }
 */
    // Set up the sensors, except at lowest battery level
    if (getBatteryVoltage() > 3.4)
    {
        Serial.println(F("Setting up sensors..."));
        varArray.setupSensors();
    }

    // Create the log file, adding the default header to it
    // Do this last so we have the best chance of getting the time correct and
    // all sensor names correct
    // Writing to the SD card can be power intensive, so if we're skipping
    // the sensor setup we'll skip this too.
    if (getBatteryVoltage() > 3.4)
    {
        Serial.println(F("Setting up file on SD card"));
        dataLogger.turnOnSDcard(true);  // true = wait for card to settle after power up
        dataLogger.createLogFile(true);  // true = write a new header
        dataLogger.turnOffSDcard(true);  // true = wait for internal housekeeping after write
    }

    // Power down the modem - but only if there will be more than 15 seconds before
    // the first logging interval - it can take the LTE modem that long to shut down
   /*  if (Logger::getNowEpoch() % (loggingInterval*60) > 15 ||
        Logger::getNowEpoch() % (loggingInterval*60) < 6)
    {
        Serial.println(F("Putting modem to sleep"));
        modem.disconnectInternet();
        modem.modemSleepPowerDown();
    }
    else
    {
        Serial.println(F("Leaving modem on until after first measurement"));
    } */

    // Call the processor sleep
    Serial.println(F("Putting processor to sleep\n"));
    dataLogger.systemSleep();
}


// ==========================================================================
// Main loop function
// ==========================================================================

// Use this short loop for simple data logging and sending
// /*
void loop()
{
    // Note:  Please change these battery voltages to match your battery
    // At very low battery, just go back to sleep
    if (getBatteryVoltage() < 3.4)
    {
        dataLogger.systemSleep();
    }
    // At moderate voltage, log data but don't send it over the modem
    else if (getBatteryVoltage() < 3.55)
    {
        dataLogger.logData();
    }
    // If the battery is good, send the data to the world
    else
    {
        //dataLogger.logDataAndPublish();
        dataLogger.logData();
    }
}
// */


// Use this long loop when you want to do something special
// Because of the way alarms work on the RTC, it will wake the processor and
// start the loop every minute exactly on the minute.
// The processor may also be woken up by another interrupt or level change on a
// pin - from a button or some other input.
// The "if" statements in the loop determine what will happen - whether the
// sensors update, testing mode starts, or it goes back to sleep.
/*
void loop()
{
    // Assuming we were woken up by the clock, check if the current time is an
    // even interval of the logging interval
    // We're only doing anything at all if the battery is above 3.4V
    if (dataLogger.checkInterval() && getBatteryVoltage() > 3.4)
    {
        // Flag to notify that we're in already awake and logging a point
        Logger::isLoggingNow = true;

        // Print a line to show new reading
        Serial.println(F("------------------------------------------"));
        // Turn on the LED to show we're taking a reading
        dataLogger.alertOn();
        // Power up the SD Card, but skip any waits after power up
        dataLogger.turnOnSDcard(false);

        // Turn on the modem to let it start searching for the network
        // Only turn the modem on if the battery at the last interval was high enough
        // NOTE:  if the modemPowerUp function is not run before the completeUpdate
        // function is run, the modem will not be powered and will not return
        // a signal strength reading.
        if (getBatteryVoltage() > 3.6)
            modem.modemPowerUp();

        // Do a complete update on the variable array.
        // This this includes powering all of the sensors, getting updated
        // values, and turing them back off.
        // NOTE:  The wake function for each sensor should force sensor setup
        // to run if the sensor was not previously set up.
        varArray.completeUpdate();

        // Create a csv data record and save it to the log file
        dataLogger.logToSD();

        // Connect to the network
        // Again, we're only doing this if the battery is doing well
        if (getBatteryVoltage() > 3.55)
        {
            if (modem.connectInternet())
            {
                // Publish data to remotes
                dataLogger.publishDataToRemotes();

                // Sync the clock at midnight
                if (Logger::markedEpochTime != 0 && Logger::markedEpochTime % 86400 == 0)
                {
                    Serial.println(F("Running a daily clock sync..."));
                    dataLogger.setRTClock(modem.getNISTTime());
                }

                // Disconnect from the network
                modem.disconnectInternet();
            }
            // Turn the modem off
            modem.modemSleepPowerDown();
        }

        // Cut power from the SD card - without additional housekeeping wait
        dataLogger.turnOffSDcard(false);
        // Turn off the LED
        dataLogger.alertOff();
        // Print a line to show reading ended
        Serial.println(F("------------------------------------------\n"));

        // Unset flag
        Logger::isLoggingNow = false;
    }

    // Check if it was instead the testing interrupt that woke us up
    if (Logger::startTesting) dataLogger.testingMode();

    // Call the processor sleep
    dataLogger.systemSleep();
}
*/
