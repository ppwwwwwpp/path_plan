#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <vector>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Polygon.h>
#include "std_msgs/Float64.h"
#include "bp_std_msgs/VirtualWall.h"
#include <ros/ros.h>

using namespace std;

int main(int argc, char** argv)
{
    ros::init(argc,argv,"test_Virtual_Wall");
    ros::NodeHandle n;
    ros::Publisher pub=n.advertise<bp_std_msgs::VirtualWall>("bp_virtual_wall",1,true);

    bp_std_msgs::VirtualWall test;
    geometry_msgs::Polygon poly;
    geometry_msgs::Point32 point;

    point.x=150;
    point.y=100;
    poly.points.push_back(point);

    point.x=100;
    point.y=200;
    poly.points.push_back(point);

    point.x=200;
    point.y=200;
    poly.points.push_back(point);

    test.PolyArray.push_back(poly);
    poly.points.clear();

    point.x = 500;
    point.y = 500;
    poly.points.push_back(point);

    point.x = 400;
    point.y = 400;
    poly.points.push_back(point);
    test.PolyArray.push_back(poly);

    test.name_map="testmap";

    pub.publish(test);
    ros::spin();
    return 0;
    
}