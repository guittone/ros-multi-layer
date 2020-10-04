#! /usr/bin/env python          
 
import rospy
 
 #import the messages, from the sensor_msgs we import the LaserScan
from sensor_msgs.msg import LaserScan

#define the call back method
def clbk_laser(msg):
#rospy.loginfo(msg)    it prints the messages,it is used only yo test if it works, then the whole range is splitted

    # the range of 720 points is divided in 5 regions
    # we work with the minimum value of the region, so the closest object is catched
    # then it is considered as a maximum value the max distant reachebale by the laser, 10m
    # 720 / 5 = 144
    regions = [
        min(min(msg.ranges[0:143]), 10),
        min(min(msg.ranges[144:287]), 10),
        min(min(msg.ranges[288:431]), 10),
        min(min(msg.ranges[432:575]), 10),
        min(min(msg.ranges[576:719]), 10),
    ]
    rospy.loginfo(regions)
 
def main():
    #initialization of the node
    rospy.init_node('reading_laser')
    
    # it subscribes to the laser (the name of the topic can be found with rostopic list, if the robot is spawned in gaebo)
    # then there is the kind of message and a call back
    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)
 
    #to keep the node running
    rospy.spin()
 
if __name__ == '__main__':
    main()
