#include <ros/ros.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
#include <boat_nav/GPS.h>

ros::Publisher pub;

void subscriber_callback(const boat_nav::GPS::ConstPtr& receivedMsg){
  sensor_msgs::NavSatFix msg;
  msg.header=receivedMsg->header;
  msg.status.status = receivedMsg->status;
  msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS; //?
  msg.latitude = receivedMsg->latitude;
  msg.longitude = receivedMsg->longitude;
  msg.altitude = receivedMsg->altitude;
  //covariance?
  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "navsatfix_msg_adapter_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("gps_raw",1,subscriber_callback);
  pub = nh.advertise<sensor_msgs::NavSatFix>("gps/TODO_gps_raw",1);
  ros::spin();
  return 0;
}
