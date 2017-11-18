#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <boat_nav/GPS.h>
#include <math.h>

//Pins the gps is attached to
#define GPS_TX_PIN 3
#define GPS_RX_PIN 2
#define WindVanePin (A4)       // The pin the wind vane sensor is connected to

ros::NodeHandle  nh;
std_msgs::Int32 winddir;
boat_nav::GPS gpsData;

SoftwareSerial mySerial(GPS_TX_PIN, GPS_RX_PIN);
Adafruit_GPS GPS(&mySerial);
Servo servo_rudder;
Servo servo_winch;
int VaneValue;       // raw analog value from wind vane
int CalDirection;    // calibrated wind value
int LastValue;
uint32_t GPS_timer = millis();
float last_lat = 0, last_long = 0;
int last_status = 0;

void servo_cb( const std_msgs::Float32& cmd_msg){
    //Write the received data directly to the rudder servo
    servo_rudder.write(cmd_msg.data);  
}

void winch_cb( const std_msgs::Int32& pos_msg){
    //map the rotation (0-2160 degrees) to a motor value between 1000 and 2000
    int position_msg = map(pos_msg.data, 0, 2160, 1000, 2000);
    servo_winch.writeMicroseconds(position_msg);
}

ros::Subscriber<std_msgs::Float32> sub_rudder("rudder", servo_cb);
ros::Subscriber<std_msgs::Int32> sub_winch("winch",winch_cb);
ros::Publisher anemometer("anemometer", &winddir);
ros::Publisher gps("gps_raw", &gpsData);

void setup(){
    // setup subscribers 
    nh.initNode();
    nh.subscribe(sub_rudder);
    nh.subscribe(sub_winch);
    nh.advertise(anemometer);
    nh.advertise(gps);
  
    servo_rudder.attach(3); // attach rudder to pin 3
    servo_winch.attach(4); // attach winch to pin 4
    LastValue = 0;

    GPS.begin(9600);
    // Tell gps to send RMC (recommended minimum) and GGA (fix data) data
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // Set the update rate
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  

    pinMode(WindVanePin, INPUT);
}

void loop(){
     VaneValue = analogRead(WindVanePin);
     GPS.read();

     // Map 0-1023 ADC value to 0-360
     CalDirection = map(VaneValue, 0, 1023, 0, 360);
   
     if(CalDirection >= 360)
         CalDirection = CalDirection - 360;
     
     if(CalDirection < 0)
         CalDirection = CalDirection + 360;
  

      // Only update the display if change greater than 5 degrees. 
    if(abs(CalDirection - LastValue) > 5)
    { 
         LastValue = CalDirection;
         winddir.data = CalDirection;
         anemometer.publish(&winddir);
       
    }

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

