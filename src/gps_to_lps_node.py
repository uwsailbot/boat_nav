#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32
from boat_nav.msg import Point
from boat_nav.msg import PointArray
from boat_nav.msg import GPS
import math

# Declare global variables needed for the node
origin_lps = Point()
RADIUS = 6378137 # Radius of earth, in meters

# Declare the publishers for the node
lps_pub = rospy.Publisher('lps', Point, queue_size=10)
waypoints_pub = rospy.Publisher('waypoints', PointArray, queue_size=10)

# Convert the current boat location from gps to lps
def gps_callback(gps):
	local = gpsToLps(getCoords(gps))
	
	# Commenting this out so that we don't spam the output
	rospy.loginfo(rospy.get_caller_id() + " Long: %f, Lat: %f --- X: %f, Y: %f", getCoords(gps).x, getCoords(gps).y, local.x, local.y)
	lps_pub.publish(local)

# Convert the list of waypoints from gps to lps
def waypoints_callback(waypoints_raw):
	waypoints = PointArray()

	for curpoint in waypoints_raw.points:
		waypoints.points.append(gpsToLps(curpoint))
		
	rospy.loginfo(rospy.get_caller_id() + " Converted waypoints to local positioning system")
	waypoints_pub.publish(waypoints)
	
# Convert from gps to lps
def gpsToLps(coords):
	global RADIUS
	global origin_lps
	local = Point()
	local.x = RADIUS * cosd(coords.y) * math.radians(coords.x) - origin_lps.x
	local.y = RADIUS * math.radians(coords.y) - origin_lps.y
	return local
	
# Extract longitude and latitude from boat_nav.msg.GPS
def getCoords(gps):
	coords = Point()
	coords.x = gps.longitude
	coords.y = gps.latitude
	return coords

def cosd(angle):
	return math.cos(math.radians(angle))

def sind(angle):
	return math.sin(math.radians(angle))

# Initialize the node
def listener():
    global origin_lps
    
    rospy.init_node('gps_to_lps')
    
    # setup the origin
    origin_coords = rospy.wait_for_message('gps_raw', GPS)
    origin_lps = gpsToLps(getCoords(origin_coords))
    rospy.loginfo("Got origin: x:%f y:%f", origin_lps.x, origin_lps.y)
    
    
    rospy.Subscriber('gps_raw', GPS, gps_callback)
    rospy.Subscriber('waypoints_raw', PointArray, waypoints_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
