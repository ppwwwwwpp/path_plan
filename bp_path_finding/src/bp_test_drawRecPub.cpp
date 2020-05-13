#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include "std_msgs/Float64.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"test_DrawRec");
    ros::NodeHandle n;
    ros::Publisher pub=n.advertise<geometry_msgs::PoseArray>("bp_area_draw",1,true);

    geometry_msgs::PoseArray msg;
    geometry_msgs::Pose temp1,temp2;

    std_msgs::Float64 x1, y1, x2, y2;

    x1.data=450;
    y1.data=360;
    x2.data=650;
    y2.data=500;

    temp1.position.x=x1.data;
	temp1.position.y=y1.data;
	temp2.position.x=x2.data;
	temp2.position.y=y2.data;

    msg.poses.push_back(temp1);
    msg.poses.push_back(temp2);


    ROS_INFO("x1:%f,y1:%f,x2:%f,y2:%f",temp1.position.x,msg.poses[0].position.y,msg.poses[1].position.x,msg.poses[1].position.y);
    pub.publish(msg);
    ros::spin();

    return 0;
}