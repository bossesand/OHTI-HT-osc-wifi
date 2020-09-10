// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

// This works
// --to be modified to support calibration if booted with a pin high.
// - save the calibration values to permanent program memory and load the old calibration values if pin is low.
/*
https://forum.arduino.cc/index.php?topic=427607.15 
Info on Pin Remapping with Wire.begin()

ESP12-F pinout
https://www.pinterest.de/pin/115193702951946398/    

Adafruit BNO055 pinout 
https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/pinouts

Pinout of Chinese module GY-BNO055:
Uses chip: BNO-055
Power supply: 3-5v (internal low voltage regulator)
Communication method: standard IIC / serial communication protocol
Communication: Module size 12mm * 20mm

Arduino config to simplify connection (for Soldering or connectors) between 8266 D1 and BNO055 modules,
when 4 straight pins soldered to bot modules BNO055 will be around 12 mm above the D1 mini. 

How to Remap the SCL and SDA to different GPIO pins depending on IMU module pinout.
For GY-BNO055 4 straight pins are used, If Adafruit bno055 is used 1 connection needs to be a wire to connect the 5 volt positions.
Wire.begin(0,2);       // for Chinese GY-BNO055      -  4 straight connection pins
Wire.begin(2,0);       // for Adafruit BNO055 module -  3 pins + one cable

GY-BNO055.                Adafruit BNO055.        Wemos D1 mini
                          -- Vin 5v             -\                 
-- Vin  5 - 3.3v          NoCon. Vin 3.3v           -- Vout   5v Powered by USB
-- GND.                   -- GND.                   -- GND
-- SCL  GPIO2.            -- SDA GPIO0.             -- D4 - Assign in Wire.begin 
-- SDA  GPIO0.            -- SCL GPIO2.             -- D3 - Assign
   ADD                                                 NoCon
   Int
   Boot
   Reset
*/

// https://platformio.org/lib/show/423/OSC/examples?file=ESP8266ReceiveMessage.ino
// D1 Wifi UDP handling https://siytek.com/esp8266-udp-send-receive/
#include <OSCMessage.h>
#include <OSCBundle.h> /// https://github.com/CNMAT/OSC 
#include <WiFiUdp.h>

#include <Wire.h>
#include "string.h"
#include <BNO055.h>

/* WiFiUDP Udp;
#include <WiFiUdp.h>
#include <ESP8266WiFi.h>
//#include <WiFiUdp.h>         //https://github.com/esp8266/Arduino/blob/master/libraries/ESP8266WiFi/src/WiFiUdp.h

WiFiUDP Udp;
*/
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>         // https://github.com/tzapu/WiFiManager

// D1 Wifi UDP handling https://siytek.com/esp8266-udp-send-receive/
// #include <OSCMessage.h>
#include <OSCMessage.h> /// https://github.com/CNMAT/OSC
#include <OSCBundle.h> /// https://github.com/CNMAT/OSC  0409

// #include <OSCBundle.h> /// https://github.com/CNMAT/OSC
#define UDP_PORT 9001

/*
UDP.remoteIP 
UDP.remotePort
outIp
outPort
*/

const IPAddress outIp(255,255,255,255);     // EditThis: The destination for OSC messages.
const unsigned int outPort = 9000; //        // EditThis: The destination port for OSC messages.

// const unsigned int UDP.remotePort = 9000; // EditThis: The destination port for OSC messages.
const unsigned int localPort = 9001;        // EditThis: The local port listening for inbound OSC.

// UDP
WiFiUDP Udp;                                // A UDP instance to let us send and receive packets over UDP

char packet[128];

//#include <ArduinoOSC.h>

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output5State = "off";
String output4State = "off";

// Assign output variables to GPIO pins
const int output5 = 5;
const int output4 = 4;

using namespace std;


#define A 0X28  // I2C address selection pin LOW
#define B 0x29  //           - " -           HIGH
BNO055 mySensor(A);

#define BNO055_ADDRESS 0x29   //  Device address of BNO055 when ADO = 1 Bosses - check bno055 voltage pin ADD

#define SerialDebug false      // set to true to get Serial output for debugging

// Set initial input parameters
enum Ascale {  // ACC Full Scale
  AFS_2G = 0,
  AFS_4G,
  AFS_8G,
  AFS_18G
};

enum Abw { // ACC Bandwidth
  ABW_7_81Hz = 0,
  ABW_15_63Hz,
  ABW_31_25Hz,
  ABW_62_5Hz,
  ABW_125Hz,
  ABW_250Hz,
  ABW_500Hz,
  ABW_1000Hz, //0x07
};

enum APwrMode { // ACC Pwr Mode
  NormalA = 0,
  SuspendA,
  LowPower1A,
  StandbyA,
  LowPower2A,
  DeepSuspendA
};

enum Gscale {  // gyro full scale
  GFS_2000DPS = 0,
  GFS_1000DPS,
  GFS_500DPS,
  GFS_250DPS,
  GFS_125DPS // 0x04
};

enum GPwrMode { // GYR Pwr Mode
  NormalG = 0,
  FastPowerUpG,
  DeepSuspendedG,
  SuspendG,
  AdvancedPowerSaveG
};

enum Gbw { // gyro bandwidth
  GBW_523Hz = 0,
  GBW_230Hz,
  GBW_116Hz,
  GBW_47Hz,
  GBW_23Hz,
  GBW_12Hz,
  GBW_64Hz,
  GBW_32Hz
};

enum OPRMode {  // BNO-55 operation modes
  CONFIGMODE = 0x00,
  // Sensor Mode
  ACCONLY,
  MAGONLY,
  GYROONLY,
  ACCMAG,
  ACCGYRO,
  MAGGYRO,
  AMG, // 0x07
  // Fusion Mode
  IMU,
  COMPASS,
  M4G,
  NDOF_FMC_OFF,
  NDOF // 0x0C
};

enum PWRMode {
  Normalpwr = 0,
  Lowpower,
  Suspendpwr
};

enum Modr { // magnetometer output data rate  
  MODR_2Hz = 0,
  MODR_6Hz,
  MODR_8Hz,
  MODR_10Hz,
  MODR_15Hz,
  MODR_20Hz,
  MODR_25Hz,
  MODR_30Hz 
};

enum MOpMode { // MAG Op Mode
  LowPower = 0,
  Regular,
  EnhancedRegular,
  HighAccuracy
};

enum MPwrMode { // MAG power mode
  Normal = 0,
  Sleep,
  Suspend,
  ForceMode
};

#define ADC_256  0x00 // define pressure and temperature conversion rates
#define ADC_512  0x02
#define ADC_1024 0x04
#define ADC_2048 0x06
#define ADC_4096 0x08
#define ADC_8192 0x0A
#define ADC_D1   0x40
#define ADC_D2   0x50

// Specify sensor configuration
uint8_t OSR = ADC_8192;           // set pressure amd temperature oversample rate
uint8_t GPwrMode = Normal;        // Gyro power mode
uint8_t Gscale = GFS_250DPS;      // Gyro full scale
//uint8_t Godr = GODR_250Hz;      // Gyro sample rate
uint8_t Gbw = GBW_23Hz;           // Gyro bandwidth
uint8_t Ascale = AFS_2G;          // Accel full scale
//uint8_t Aodr = AODR_250Hz;      // Accel sample rate
uint8_t APwrMode = Normal;        // Accel power mode
uint8_t Abw = ABW_31_25Hz;        // Accel bandwidth, accel sample rate divided by ABW_divx
//uint8_t Mscale = MFS_4Gauss;    // Select magnetometer full-scale resolution
uint8_t MOpMode = HighAccuracy;   // Select magnetometer perfomance mode
uint8_t MPwrMode = Normal;        // Select magnetometer power mode
uint8_t Modr = MODR_10Hz;         // Select magnetometer ODR when in BNO055 bypass mode
uint8_t PWRMode = Normal;         // Select BNO055 power mode
uint8_t OPRMode = NDOF;           // specify operation mode for sensors
uint8_t status;                   // BNO055 data status register
float aRes, gRes, mRes;           // scale resolutions per LSB for the sensors

uint16_t Pcal[8];         // calibration constants from MS5637 PROM registers
unsigned char nCRC;       // calculated check sum to ensure PROM integrity
// uint32_t D1 = 0;   // raw MS5637 pressure and temperature data
// uint32_t D2 = 0;  // raw MS5637 pressure and temperature data
double dT, OFFSET, SENS, T2, OFFSET2, SENS2;  // First order and second order corrections for raw S5637 temperature and pressure data
//int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output - RGR
//int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor output - RGR
//int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output - RGR
//int16_t quatCount[4];   // Stores the 16-bit signed quaternion output - RGR
//int16_t EulCount[3];    // Stores the 16-bit signed Euler angle output - RGR
//int16_t LIACount[3];    // Stores the 16-bit signed linear acceleration output - RGR
//int16_t GRVCount[3];    // Stores the 16-bit signed gravity vector output - RGR
float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};  // Bias corrections for gyro, accelerometer, and magnetometer
int16_t tempGCount, tempMCount;      // temperature raw count output of mag and gyro
float Gtemperature, Mtemperature;  // Stores the BNO055 gyro and LIS3MDL mag internal chip temperatures in degrees Celsius
double Temperature, Pressure;        // stores MS5637 pressures sensor pressure and temperature

// global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
// There is a tradeoff in the   parameter between accuracy and response speed.
// In the original Madgwick study,   of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing   (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
float GME  = sqrt(3.0f / 4.0f) * GyroMeasError;   // compute  
float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

uint32_t delt_t = 0, count = 0, sumCount = 0;  // used to control display output rate
float pitch, yaw, roll;
float Pitch, Yaw, Roll;
float LIAx, LIAy, LIAz, GRVx, GRVy, GRVz;
float deltat = 0.0f, sum = 0.0f;          // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0; // used to calculate integration interval
uint32_t Now = 0;                         // used to calculate integration interval

float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values 
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
//float quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

// ==========================================================
//       Bosses added 10-26 + 202008

//char  selva[8];
//double DD64;
// Variables for OSC bno055
/*
float qw = 0.0f;
float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;
*/
int16_t htQuatRead[4];

union combo_quat
{
  // elements of a union share the memory, i.e.
  // reside at the same address, not consecutive ones
  // 4 x 16bit = 8 bytes = 64 bit
  int16_t  quat[4];  //4x16bit = 64 bit = 8 bytes
  char hex[18]; // 4x4x8bit + NULL = 136 bit = 17 bytes
} htQuatCombo;



// ==========================================================
// Main Setup



void setup() {
  //Wire.begin();
  //TWBR = 12;  // 400 kbit/sec I2C speed for Pro Mini
  //Setup for Master mode, pins 16/17, external pullups, 400kHz for Teensy 3.1

  Wire.begin();
  // By default .begin() will set I2C SCL to Standard Speed mode of 100kHz
  Wire.setClock(400000); //Optional - set I2C SCL to High Speed Mode of 400kHz
  delay(2000);

  // Custom services and characteristics can be added as well
  //Serial.setLocalName("OHTI");
  Serial.begin(115200);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

   Serial.println("OHTI-Have booted step 0");
    delay(5000);
   Serial.println("OHTI-Have booted step 1");
     delay(5000);
   Serial.println("OHTI-Have booted step 3");
   
  // Initialize the output variables as outputs
  pinMode(output5, OUTPUT);
  pinMode(output4, OUTPUT);
  // Set outputs to LOW
  digitalWrite(output5, LOW);
  digitalWrite(output4, LOW);

  // WiFiManager
  // Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  
  // Uncomment and run it once, if you want to erase all the stored information
  //wifiManager.resetSettings();
  
  // set custom ip for portal
  //wifiManager.setAPConfig(IPAddress(10,0,1,1), IPAddress(10,0,1,1), IPAddress(255,255,255,0));

  // fetches ssid and pass from eeprom and tries to connect
  // if it does not connect it starts an access point with the specified name
  // here  "OHTI-AP"
  // and goes into a blocking loop awaiting configuration
  wifiManager.autoConnect("OHTI-AP");
  // or use this for auto generated name ESP + ChipID
  //wifiManager.autoConnect();
  
  // if you get here you have connected to the WiFi
  Serial.println("Connected.");
  
  server.begin();

  //202008
    // Begin listening to UDP port
    
  Udp.begin(localPort);
  Serial.print("Listening on UDP port ");
  Serial.println(UDP_PORT);
  
  
  // ==========================================================
  //          Calibration and comm test started here !! - Bosse
  
  // Serial.println("Please use Adafruit Bluefruit LE app to connect in UART mode");
  // Serial.println("Then Enter characters to send");
  // Read the WHO_AM_I register, this is a good test of communication
  
  Serial.println("BNO055 ReadByte starts");
  byte c = readByte(BNO055_ADDRESS, BNO055_CHIP_ID);  // Read WHO_AM_I register for BNO055
  //Serial.print("BNO055 Address = 0x"); Serial.println(BNO055_ADDRESS, HEX);
  //Serial.print("BNO055 WHO_AM_I = 0x"); Serial.println(BNO055_CHIP_ID, HEX);
  //Serial.print("BNO055 "); Serial.print("I AM "); Serial.print(c, HEX); Serial.println(" I should be 0xA0");  
  delay(1000); 

  // Read the WHO_AM_I register of the accelerometer, this is a good test of communication
  // byte d = readByte(BNO055_ADDRESS, BNO055_ACC_ID);  // Read WHO_AM_I register for accelerometer
  //Serial.print("BNO055 ACC "); Serial.print("I AM "); Serial.print(d, HEX); Serial.println(" I should be 0xFB");  
  delay(1000); 

  // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
  // byte e = readByte(BNO055_ADDRESS, BNO055_MAG_ID);  // Read WHO_AM_I register for magnetometer
  //Serial.print("BNO055 MAG "); Serial.print("I AM "); Serial.print(e, HEX); Serial.println(" I should be 0x32");
  delay(1000);   

  // Read the WHO_AM_I register of the gyroscope, this is a good test of communication
  // byte f = readByte(BNO055_ADDRESS, BNO055_GYRO_ID);  // Read WHO_AM_I register for LIS3MDL
  //Serial.print("BNO055 GYRO "); Serial.print("I AM "); Serial.print(f, HEX); Serial.println(" I should be 0x0F");
  delay(1000); 

  if (c == 0xA0) { // BNO055 WHO_AM_I should always be 0xA0  
    Serial.println("BNO055 is online.");

    // Check software revision ID
    byte swlsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_LSB);
    byte swmsb = readByte(BNO055_ADDRESS, BNO055_SW_REV_ID_MSB);
    Serial.print("BNO055 SW Revision ID: "); Serial.print(swmsb, HEX); Serial.print("."); Serial.println(swlsb, HEX); Serial.println("Should be 03.04");

    // Check bootloader version
    byte blid = readByte(BNO055_ADDRESS, BNO055_BL_REV_ID);
    Serial.print("BNO055 bootloader Version: "); Serial.println(blid); 

    // Check self-test results
    byte selftest = readByte(BNO055_ADDRESS, BNO055_ST_RESULT);
    if(selftest & 0x01) {
      Serial.println("accelerometer passed selftest"); 
    } else {
      Serial.println("accelerometer failed selftest"); 
    }
    if(selftest & 0x02) {
      Serial.println("magnetometer passed selftest"); 
    } else {
      Serial.println("magnetometer failed selftest"); 
    }  
    if(selftest & 0x04) {
      Serial.println("gyroscope passed selftest"); 
    } else {
      Serial.println("gyroscope failed selftest"); 
    }      
    if(selftest & 0x08) {
      Serial.println("MCU passed selftest"); 
    } else {
      Serial.println("MCU failed selftest"); 
    }
    delay(1000);

    accelgyroCalBNO055(accelBias, gyroBias);
    // Serial.println("Average accelerometer bias (mg) = "); Serial.println(accelBias[0]); Serial.println(accelBias[1]); Serial.println(accelBias[2]);
    // Serial.println("Average gyro bias (dps) = "); Serial.println(gyroBias[0]); Serial.println(gyroBias[1]); Serial.println(gyroBias[2]);
    delay(1000); 

    magCalBNO055(magBias);
    //Serial.println("Average magnetometer bias (mG) = "); Serial.println(magBias[0]); Serial.println(magBias[1]); Serial.println(magBias[2]);
    delay(1000); 

    // Check calibration status of the sensors
    uint8_t calstat = readByte(BNO055_ADDRESS, BNO055_CALIB_STAT);
    Serial.print("System calibration status (Not calibrated = 0, fully calibrated = 3) "); Serial.println( (0xC0 & calstat) >> 6);
    //Serial.print("Gyro   calibration status "); Serial.println( (0x30 & calstat) >> 4);
    //Serial.print("Accel  calibration status "); Serial.println( (0x0C & calstat) >> 2);
    //Serial.print("Mag    calibration status "); Serial.println( (0x03 & calstat) >> 0);



    // ==========================================================
    //                Initialize the BNO055
    
    initBNO055(); 
    Serial.println("BNO055 initialized for sensor read mode.");
  }
  else
  {
    Serial.print("Could not connect to BNO055: 0x");
    Serial.println(c, HEX);
    while(1);    // Loop forever if communication doesn't happen
  }
}
String Qhex;
char buf[18];
void setQuatComboHex() {
  int quat[4];

for (int i=0; i<=3; i++) {
    if (htQuatCombo.quat[i] & 0x8000)
      quat[i] = (0x8000 - htQuatCombo.quat[i]) & 0xffff;
    else
      quat[i] = htQuatCombo.quat[i];
  }

 sprintf(htQuatCombo.hex, "%04x%04x%04x%04x",
    quat[0], quat[1], quat[2], quat[3]);
}



// ==========================================================
//              Calibration procedure moved here

void calibrateBNO055()
{
}

// ==========================================================
//    Set of useful function to access acceleration. 
//    gyroscope, magnetometer, and temperature data


void readAccelData(int16_t * destination)
{
  uint8_t rawData[6];                                                // x/y/z accel register data stored here
  readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;         // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readGyroData(int16_t * destination)
{
  uint8_t rawData[6];                                                // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;         // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ; 
}

int8_t readGyroTempData()
{
  return readByte(BNO055_ADDRESS, BNO055_TEMP);                      // Read the two raw data registers sequentially into data array 
}

void readMagData(int16_t * destination)
{
  uint8_t rawData[6];                                                // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;         // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readQuatData(int16_t * destination)
{
  uint8_t rawData[8];                                                // w/x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_QUA_DATA_W_LSB, 8, &rawData[0]);  // Read the 8 (boss4) raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;         // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
  destination[3] = ((int16_t)rawData[7] << 8) | rawData[6] ;
}

void readEulData(int16_t * destination)
{
  uint8_t rawData[6];                                                // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_EUL_HEADING_LSB, 6, &rawData[0]); // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;         // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readLIAData(int16_t * destination)
{
  uint8_t rawData[6];                                                // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_LIA_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;         // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void readGRVData(int16_t * destination)
{
  uint8_t rawData[6];                                                // x/y/z gyro register data stored here
  readBytes(BNO055_ADDRESS, BNO055_GRV_DATA_X_LSB, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;         // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;  
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}

void initBNO055() {
  // Select page 1 to configure sensors
  writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x01);
  // Configure ACC
  writeByte(BNO055_ADDRESS, BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 3 | Ascale );
  // Configure GYR
  writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale );
  writeByte(BNO055_ADDRESS, BNO055_GYRO_CONFIG_1, GPwrMode);
  // Configure MAG
  writeByte(BNO055_ADDRESS, BNO055_MAG_CONFIG, MPwrMode << 4 | MOpMode << 2 | Modr );

  // Select page 0 to read sensors
  writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);

  // Select BNO055 gyro temperature source 
  writeByte(BNO055_ADDRESS, BNO055_TEMP_SOURCE, 0x01 );

  // Select BNO055 sensor units (temperature in degrees C, rate in dps, accel in mg)
  writeByte(BNO055_ADDRESS, BNO055_UNIT_SEL, 0x01 );

  // Select BNO055 system power mode
  writeByte(BNO055_ADDRESS, BNO055_PWR_MODE, PWRMode );

  // Select BNO055 system operation mode
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );
}

void accelgyroCalBNO055(float * dest1, float * dest2) 
{
  uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  Serial.println("Accel/Gyro Calibration: Put device on a level surface and keep motionless! Wait.");
  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(4000);

  // Select page 0 to read sensors
  writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
  // Select BNO055 system operation mode as NDOF for calibration
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
  delay(25);
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, NDOF );

  // In NDF fusion mode, accel full scale is at +/- 4g, ODR is 62.5 Hz
  sample_count = 256;
  for(ii = 0; ii < sample_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0};
    readBytes(BNO055_ADDRESS, BNO055_ACC_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
    accel_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ; // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    accel_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    accel_bias[0]  += (int32_t) accel_temp[0];
    accel_bias[1]  += (int32_t) accel_temp[1];
    accel_bias[2]  += (int32_t) accel_temp[2];
    delay(20);  // at 62.5 Hz ODR, new accel data is available every 16 ms
  }
  accel_bias[0]  /= (int32_t) sample_count;  // get average accel bias in mg
  accel_bias[1]  /= (int32_t) sample_count;
  accel_bias[2]  /= (int32_t) sample_count;

  if(accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t) 1000; // Remove gravity from the z-axis accelerometer bias calculation
  }  
  else 
  {
    accel_bias[2] += (int32_t) 1000;
  }

  dest1[0] = (float) accel_bias[0];  // save accel biases in mg for use in main program
  dest1[1] = (float) accel_bias[1];  // accel data is 1 LSB/mg
  dest1[2] = (float) accel_bias[2];          

  // In NDF fusion mode, gyro full scale is at +/- 2000 dps, ODR is 32 Hz
  for(ii = 0; ii < sample_count; ii++) {
    int16_t gyro_temp[3] = {0, 0, 0};
    readBytes(BNO055_ADDRESS, BNO055_GYR_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
    gyro_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;  // Form signed 16-bit integer for each sample in FIFO
    gyro_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    gyro_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];
    delay(35);  // at 32 Hz ODR, new gyro data available every 31 ms
  }
  gyro_bias[0]  /= (int32_t) sample_count;  // get average gyro bias in counts
  gyro_bias[1]  /= (int32_t) sample_count;
  gyro_bias[2]  /= (int32_t) sample_count;

  dest2[0] = (float) gyro_bias[0]/16.;      // save gyro biases in dps for use in main program
  dest2[1] = (float) gyro_bias[1]/16.;      // gyro data is 16 LSB/dps
  dest2[2] = (float) gyro_bias[2]/16.;          

  // Return to config mode to write accelerometer biases in offset register
  // This offset register is only used while in fusion mode when accelerometer full-scale is +/- 4g
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
  delay(25);

  // Write biases to accelerometer offset registers ad 16 LSB/dps
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB, (int16_t)accel_bias[0] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB, ((int16_t)accel_bias[0] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB, (int16_t)accel_bias[1] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB, ((int16_t)accel_bias[1] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB, (int16_t)accel_bias[2] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB, ((int16_t)accel_bias[2] >> 8) & 0xFF);

  // Check that offsets were properly written to offset registers
  //Serial.println("Average accelerometer bias = "); 
  //Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_X_LSB))); 
  //Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Y_LSB))); 
  //Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_ACC_OFFSET_Z_LSB)));

  // Write biases to gyro offset registers
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB, (int16_t)gyro_bias[0] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB, ((int16_t)gyro_bias[0] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB, (int16_t)gyro_bias[1] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB, ((int16_t)gyro_bias[1] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB, (int16_t)gyro_bias[2] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB, ((int16_t)gyro_bias[2] >> 8) & 0xFF);

  // Select BNO055 system operation mode
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );

  // Check that offsets were properly written to offset registers
  //Serial.println("Average gyro bias = "); 
  //Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_X_LSB))); 
  //Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Y_LSB))); 
  //Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_GYR_OFFSET_Z_LSB)));

  digitalWrite(LED_BUILTIN, LOW); // turn the LED off by making the voltage LOW
  Serial.println("Accel/Gyro Calibration done!");
}

void magCalBNO055(float * dest1) 
{
  uint8_t data[6]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii = 0, sample_count = 0;
  int32_t mag_bias[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

  Serial.println("MagCal8"); // Mag Calibration: Wave device in a figure eight until done!
  Serial.println("Wave device in a figure eight until done!");
  
  // Start our for loop 10 second, Toggle both LEDs every second
  for (int j=1; j<=10; j=j+1) { 
    // digitalToggle(LED_BUILTIN);
    delay(1000);
  }

  // Select page 0 to read sensors
  writeByte(BNO055_ADDRESS, BNO055_PAGE_ID, 0x00);
  // Select BNO055 system operation mode as NDOF for calibration
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
  delay(25);
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, NDOF );

 //     In NDF fusion mode, mag data is in 16 LSB/microTesla, ODR is 20 Hz in forced mode
 //
  sample_count = 256;
  for(ii = 0; ii < sample_count; ii++) {
    int16_t mag_temp[3] = {0, 0, 0};
    readBytes(BNO055_ADDRESS, BNO055_MAG_DATA_X_LSB, 6, &data[0]);  // Read the six raw data registers into data array
    mag_temp[0] = (int16_t) (((int16_t)data[1] << 8) | data[0]) ;   // Form signed 16-bit integer for each sample in FIFO
    mag_temp[1] = (int16_t) (((int16_t)data[3] << 8) | data[2]) ;
    mag_temp[2] = (int16_t) (((int16_t)data[5] << 8) | data[4]) ;
    for (int jj = 0; jj < 3; jj++) {
      if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
      if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(55);  // at 20 Hz ODR, new mag data is available every 50 ms
  }

  //Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
  //Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
  //Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0] / 1.6;  // save mag biases in mG for use in main program
  dest1[1] = (float) mag_bias[1] / 1.6;  // mag data is 1.6 LSB/mg
  dest1[2] = (float) mag_bias[2] / 1.6;          

  // Return to config mode to write mag biases in offset register
  // This offset register is only used while in fusion mode when magnetometer sensitivity is 16 LSB/microTesla
  
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, CONFIGMODE );
  delay(25);

  // Write biases to accelerometer offset registers as 16 LSB/microTesla
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB, (int16_t)mag_bias[0] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB, ((int16_t)mag_bias[0] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB, (int16_t)mag_bias[1] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB, ((int16_t)mag_bias[1] >> 8) & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB, (int16_t)mag_bias[2] & 0xFF);
  writeByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB, ((int16_t)mag_bias[2] >> 8) & 0xFF);

  // Check that offsets were properly written to offset registers
  //Serial.println("Average magnetometer bias = "); 
  //Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_X_LSB))); 
  //Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Y_LSB))); 
  //Serial.println((int16_t)((int16_t)readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_MSB) << 8 | readByte(BNO055_ADDRESS, BNO055_MAG_OFFSET_Z_LSB)));
  
  // Select BNO055 system operation mode
  writeByte(BNO055_ADDRESS, BNO055_OPR_MODE, OPRMode );

  Serial.println("Mag Calibration done!");
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off (LOW is the voltage level)
}


// ==========================================================
//      I2C read/write functions for the BNO055 sensor

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  Wire.beginTransmission(address);        // Initialize the Tx buffer
  Wire.write(subAddress);                 // Put slave register address in Tx buffer
  Wire.write(data);                       // Put data in Tx buffer
  Wire.endTransmission();                 // Send the Tx buffer
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
  uint8_t data;                           // `data` will store the register data     
  Wire.beginTransmission(address);        // Initialize the Tx buffer
  Wire.write(subAddress);                 // Put slave register address in Tx buffer
  //Wire.endTransmission(I2C_NOSTOP);     // Send the Tx buffer, but send a restart to keep connection alive
  Wire.endTransmission(false);            // Send the Tx buffer, but send a restart to keep connection alive
  //Wire.requestFrom(address, 1);         // Read one byte from slave register address 
  Wire.requestFrom(address, (size_t) 1);  // Read one byte from slave register address 
  data = Wire.read();                     // Fill Rx buffer with result
  return data;                            // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
  Wire.beginTransmission(address);        // Initialize the Tx buffer
  Wire.write(subAddress);                 // Put slave register address in Tx buffer
  //Wire.endTransmission(I2C_NOSTOP);     // Send the Tx buffer, but send a restart to keep connection alive
  Wire.endTransmission(false);            // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  //Wire.requestFrom(address, count);     // Read bytes from slave register address 
  Wire.requestFrom(address, (size_t) count); // Read bytes from slave register address 
  while (Wire.available()) {
  dest[i++] = Wire.read(); }              // Put read results in the Rx buffer
}
//======================================
// CNMAT OSC send
 /* void oscMessage() {
 //Serial.print("oscMessage()");
    OSCMessage msg("/SceneRotator/quaternions");
    msg.add(qw);
    msg.add(qx);
    msg.add(qy);
    msg.add(qy);
    Udp.beginPacket(outIp,outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    // sendCount = 0;
   
    Udp.beginPacket(outIp, 9000);  // Hardcode UDP port
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();
    
    delay(50);
  //Serial.println("  is done.");
}
    */
//
// ==========================================================
//                      Main Loop

void loop() {
  delay(14);
  mySensor.readQuat(); 
  // Read sensor

  readQuatData(htQuatRead);  // Read the w/x/y/z adc values, quatCount = int16_t (signed)
  // To Calculate the quaternion values - divide when receiving with 16384

  htQuatCombo.quat[0] = (int16_t)(htQuatRead[0]);    
  htQuatCombo.quat[1] = (int16_t)(htQuatRead[1]);  
  htQuatCombo.quat[2] = (int16_t)(htQuatRead[2]);   
  htQuatCombo.quat[3] = (int16_t)(htQuatRead[3]);  

  float fqw = (int16_t)htQuatCombo.quat[0];
  float fqx = (int16_t)htQuatCombo.quat[1];
  float fqy = (int16_t)htQuatCombo.quat[2];
  float fqz = (int16_t)htQuatCombo.quat[3];

  
  float qw = fqw / 32767;
  float qx = fqx / 32767;
  float qy = fqy / 32767;
  float qz = fqz / 32767;
 /* 
 //Debug quaternion values
  Serial. println(qw, 3);
  Serial. println(qx, 3);
  Serial. println(qy, 3);
  Serial. println(qz, 3);
*/
    setQuatComboHex();
    Serial.println(htQuatCombo.hex);
      //OSCMessage msg("/address");
      
/*
    OSCBundle bndl;
    bndl.add("/SceneRotator/qw").add(qw);
    bndl.add("/SceneRotator/qx").add(qx);
    bndl.add("/SceneRotator/qy").add(qy);
    bndl.add("/SceneRotator/qz").add(qz);
    Udp.beginPacket(outIp, outPort);
    bndl.send(Udp); // send the bytes to the OUT stream
    Udp.endPacket(); // mark the end of the OSC Packet
    bndl.empty(); // empty the bundle to free room for a new one
    //delay(100);  // 1/10 sec delay for slower message rate -- slow test
    */ // Send UDP packet And Delay
    OSCMessage msg("/SceneRotator/quaternions");
    msg.add(qw);
    msg.add(qx);
    msg.add(qy);
    msg.add(qz);
    Udp.beginPacket(outIp, outPort);
    msg.send(Udp);
    Udp.endPacket();
    msg.empty();   

    // delay(100);  // 1/10 sec delay for slower message rate -- slow test
}
