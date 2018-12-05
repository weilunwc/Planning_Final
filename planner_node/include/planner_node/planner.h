#ifndef PLANNER_H
#define PLANNER_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <planner_node/planner_pub.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <geometry_msgs/Pose.h>


namespace planner_node
{
class planner_rrt
{
 public:
  explicit planner_rrt(ros::NodeHandle nh);
  void publish_pos(float x, float y, float z);
  void subscribe_pos();
  void current_pos_cb(geometry_msgs::Pose msg_pos);
  octomap::point3d get_current_pos();
 private:
  //! Turn on publisher.
  void start();
  octomap::point3d current_pos;
  //! Turn off publisher.
  void stop();
  bool enable_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::NodeHandle nh_;

};
}

#endif  // NODE_EXAMPLE_LISTENER_H
