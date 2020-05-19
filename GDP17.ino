// Initializes arduino and loads libraries
#include <SPI.h> 
#include <Adafruit_GPS.h> 
#include <SD.h> 
#include <SoftwareSerial.h>
#include <avr/sleep.h> 
////SoftwareSerial mySerial(8, 7); 
#define mySerial Serial1 
Adafruit_GPS GPS(&mySerial);
#define LOG_FIXONLY false  
boolean usingInterrupt = false;
#define chipSelect 10 
#define ledPin 13 
#define GPSECHO true
#define LOG_FIXONLY false
File logfile;
#include <Pixy2.h>
Pixy2 pixy;
#include "MPU9250.h"
MPU9250 IMU(Wire,0x68);
int status;
#include "Wire.h"
#include "I2Cdev.h"
#define LED_PIN 13 
bool blinkState = false;



void setup() {
      Serial.begin(115200); 
      Serial1.begin(115200);  // starts communication with GPS
      Serial.println("\r\nUltimate GPSlogger Shield");
      pinMode(ledPin, OUTPUT);
      pinMode(10, OUTPUT);
      pixy.init();
      Wire.begin(); // Initialize I2C comms
      Serial.println("Initializing I2C devices...");
      pinMode(LED_PIN, OUTPUT);
      status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.print("Status: ");
    Serial.println(status);
  }
  // IMU stuf
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  IMU.setSrd(19);
  //sets up the SD card
      if (!SD.begin(chipSelect, 11, 12, 13))
      {
        Serial.println("SD Card init. failed!");
    }
      char filename[15];
      strcpy(filename, "GPSLOG00.CSV");
      for (uint8_t i = 0; i < 100; i++)
     {
      filename[6] = '0' + i/10;
      filename[7] = '0' + i%10;
      // create if does not exist, do not open existing, write, sync after write
        if (! SD.exists(filename))
    {
            break;
        }
        }
      logfile = SD.open(filename, FILE_WRITE);
      logfile.println("Orange, Time, Date, Latitude, Longitude, Elevation, Speed (Knots), Angle, Satellites, AcX, AcY, AcZ, GyX, GyY, GyZ");
      logfile.flush();

      GPS.begin(9600); 
      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate
      GPS.sendCommand(PGCMD_NOANTENNA);  
}
void loop() {
  int i; 
  // Red colour detection // Pixy camera stuff
    pixy.ccc.getBlocks();
    IMU.readSensor();
    char c = GPS.read(); //CH "to read characters coming from the GPS
    if (GPS.newNMEAreceived()) {
    GPS.parse(GPS.lastNMEA()); //CH Parse the new GPS data available   
      if (pixy.ccc.numBlocks)
  {
     logfile.print("Orange Detected");
     Serial.println("Orange Detected");
        logfile.print(",");
  }
  else{
      logfile.print("No Orange");
      Serial.println(" Orange Not Detected");
        logfile.print(",");
    }  
        logfile.print(GPS.hour, DEC);
        logfile.print(':');
        logfile.print(GPS.minute, DEC);
        logfile.print(':');
        logfile.print(GPS.seconds, DEC);
        logfile.print('.');
        logfile.print(GPS.milliseconds);
        logfile.print(",");

       
        logfile.print(GPS.day, DEC); 
        logfile.print('/');
        logfile.print(GPS.month, DEC);
        logfile.print("/20");
        logfile.print(GPS.year, DEC);
        logfile.print(",");

        // GPS Data (Latitude, Longitude, Altitude, Speed, Angle, Satellite Count)
        logfile.print(GPS.latitude, 5);
        logfile.print(GPS.lat);
        logfile.print(", ");
        logfile.print(GPS.longitude, 5);
        logfile.print(GPS.lon);
        logfile.print(",");
        logfile.print(GPS.altitude);
        logfile.print(",");
        logfile.print(GPS.speed);
        logfile.print(",");
        logfile.print(GPS.angle);
        logfile.print(",");
        logfile.print((int)GPS.satellites);
        logfile.print(",");
        logfile.print(IMU.getAccelX_mss(),6);
        logfile.print(",");
        logfile.print(IMU.getAccelY_mss(),6);
        logfile.print(", ");
        logfile.print(IMU.getAccelZ_mss(),6);
        logfile.print(",");
        logfile.print(IMU.getGyroX_rads());
        logfile.print(",");
        logfile.print(IMU.getGyroY_rads());
        logfile.print(",");
        logfile.println(IMU.getGyroZ_rads());
        logfile.flush();       
        }
    }
