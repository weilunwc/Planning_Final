/*
 *  Class: Gazebo Drawing
 *  Reference - https://bitbucket.org/osrf/gazebo/src/default/examples/stand_alone/marker/marker.cc
 *  Author: Wei Lun William Chen, 
 *  Date: 2018/12/1 
 *  Description: ROS service of drawing within gazebo
 */

#ifndef GAZEBODRAWING_H
#define GAZEBODRAWING_H

#include <iostream>
#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <gazebo/common/Time.hh>

#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "plan_msgs/DrawPoints.h"
#include "plan_msgs/DrawPath.h"
#include "std_srvs/Empty.h"


#include <vector>

class GazeboDrawing{
    private:
        // ros interface
        ros::NodeHandle global;

        // ros service 
        ros::ServiceServer draw_target_service, draw_depot_service, draw_path_service;

        // gazebo interface
        ignition::transport::Node node;
        
        // markers 
        ignition::msgs::Marker targetMarker;
        ignition::msgs::Marker depotMarker;
        ignition::msgs::Marker pathMarker;


        // server callback function
        bool draw_target_cb(plan_msgs::DrawPoints::Request &req, plan_msgs::DrawPoints::Response &res);
        bool draw_depot_cb(plan_msgs::DrawPoints::Request &req, plan_msgs::DrawPoints::Response &res);
        bool draw_path_cb(plan_msgs::DrawPath::Request &req, plan_msgs::DrawPath::Response &res);

        // drawing functions
        void DrawPoints(std::vector<geometry_msgs::Point> &points, ignition::msgs::Marker &marker);
        void DrawPath(nav_msgs::Path path);

        
    public:
        GazeboDrawing();
};



#endif
