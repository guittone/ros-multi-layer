#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include "sensor_msgs/LaserScan.h"
#include <iomanip>
#include <string>
#include <iostream>


void LaserCallBack (const sensor_msgs::LaserScan::ConstPtr& msg)
{
    std::string s;
    std::string a;
    int i;
    float myFloat = 0;
    std::stringstream ss;
    s.clear();
    a.clear();

/*  to print all the data from laserscanner range

    myFloat = msg->ranges[0];
    ss << myFloat;
    s = ss.str();
    ss.str("");
*/    

// Divides the whole range in 6 ranges and calculates the minimum value of each range

    float min1 = msg->ranges[0];
    for( i = 0; i < 120; i++)
    {   
        if(msg->ranges[i] < min1)
            min1 = msg->ranges[i];
    }

    float min2 = msg->ranges[120];
    for( i = 120; i < 240; i++)
    {   
        if(msg->ranges[i] < min2)
            min2 = msg->ranges[i];
    }

    float min3 = msg->ranges[240];
    for( i = 240; i < 360; i++)
    {   
        if(msg->ranges[i] < min3)
            min3 = msg->ranges[i];
    }

    float min4 = msg->ranges[360];
    for( i = 360; i < 480; i++)
    {   
        if(msg->ranges[i] < min4)
            min4 = msg->ranges[i];
    }

    float min5 = msg->ranges[480];
    for( i = 480; i < 600; i++)
    {   
        if(msg->ranges[i] < min5)
            min5 = msg->ranges[i];
    }

    float min6 = msg->ranges[600];
    for( i = 600; i < 720; i++)
    {   
        if(msg->ranges[i] < min6)
            min6 = msg->ranges[i];
    }
    
    // Set the value of each range to 10 if the object is not within 10 m

    if(min1 > 10)
        {  
            min1 = 10;
        }
    
    if(min2 > 10)
        {  
            min2 = 10;
        }

    if(min3 > 10)
        {  
            min3 = 10;
        }

    if(min4 > 10)
        {  
            min4 = 10;
        }

    if(min5 > 10)
        {  
            min5 = 10;
        }

    if(min6 > 10)
        {  
            min6 = 10;
        }

    
    ROS_INFO("\n[The minimum value of the ranges is:]\n[%f]\n[%f]\n[%f]\n[%f]\n[%f]\n[%f]\n",min1,min2,min3,min4,min5,min6);
}

int main(int argc, char **argv)
{

/* initialization of the node which is placed in the third argument, the others are required*/
ros::init(argc, argv, "reading_laser"); 
/*reprents the node and give information about how to publish and subscribe*/
ros::NodeHandle n;
/*subscibe to the given topic, find it with rostopic list, number of messages to maintain, third argument is a call back function*/
ros::Subscriber sub = n.subscribe("/m2wr/laser/scan", 720, LaserCallBack);
/* it enters a loop, exit if ctrl-c is pressed*/
ros::spin();

return 0;
}