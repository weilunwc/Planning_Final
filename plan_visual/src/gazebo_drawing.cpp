/*
 *  Gazebo interface of drawing
 *  Reference - https://bitbucket.org/osrf/gazebo/src/default/examples/stand_alone/marker/marker.cc
 *  Maintainer : Wei Lun William Chen, Date: 2018/12/1
 */

#include <ros/ros.h>
#include "GazeboDrawing.h"
#include <vector>

int main(int argc, char **argv){
    ROS_INFO("Start gazebo drawing");
    ros::init(argc, argv, "gazebo_drawing");
    GazeboDrawing gazebo_drawing;
    ros::Rate rate(1000);
    while(ros::ok()){
        rate.sleep();
        ros::spinOnce();
    }
    
    ros::spin();
    return 0;
}
