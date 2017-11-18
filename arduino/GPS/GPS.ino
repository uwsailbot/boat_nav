/*  UW Sailbot ROS Publisher for GPS data 
 *  Benjamin Wilkins
 *  
 *  Requires Adafruit GPS Library
 *  can be downloaded here https://github.com/adafruit/Adafruit_GPS
 *  
 *  Uses custom GPS message. To generate arduino libraries for message
 *  $ cd <sketchbook>/libraries
 *  $ rm -rf ros_lib
 *  $ rosrun rosserial_arduino make_libraries.py .
 */

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <ros.h>
#include <boat_nav/GPS.h>


//Pins the gps is attached to
#define GPS_TX_PIN 3
#define GPS_RX_PIN 2

SoftwareSerial mySerial(GPS_TX_PIN, GPS_RX_PIN);
Adafruit_GPS GPS(&mySerial);

//TODO: Need to change this when using mega with mutltiple subscribers and publishers
//<HardwareType, MAX_PUBLISHERS, MAX_SUBSCRIBERS, IN_BUFFER_SIZE, OUT_BUFFER_SIZE>
ros::NodeHandle_<ArduinoHardware, 1, 1, 0, 340> nh;
boat_nav::GPS gpsData;
ros::Publisher gps("gps_raw", &gpsData);


void setup() {
  GPS.begin(9600);
  // Tell gps to send RMC (recommended minimum) and GGA (fix data) data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  
  nh.initNode();
  nh.advertise(gps);
}


uint32_t GPS_timer = millis();
float last_lat = 0, last_long = 0;
int last_status = 0;

void loop() {
  GPS.read();
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

    // approximately every second publish the current stats
  if (millis() - GPS_timer > 1000) { 
    GPS_timer = millis(); // reset the timer
    
    if (GPS.fix) {
      gpsData.status = 0;
      gpsData.satellites_used = GPS.satellites;
      gpsData.latitude = GPS.latitudeDegrees;
      gpsData.longitude = GPS.longitudeDegrees;
      gpsData.altitude = GPS.altitude;
      gpsData.track = GPS.angle;
      gpsData.speed = GPS.speed;
      gpsData.hdop = GPS.HDOP; // horizontal diltuion of precision
    }else{
      gpsData.status = -1;
    }

    //only publish is readings have changes
   if ((last_lat != gpsData.latitude) ||
        (last_long != gpsData.longitude) ||
        (last_status != gpsData.status)){
      gps.publish(&gpsData);
      
      last_lat = gpsData.latitude;
      last_long = gpsData.longitude;
      last_status = gpsData.status;
    }
    
    gps.publish(&gpsData);
  }

  nh.spinOnce();
}
