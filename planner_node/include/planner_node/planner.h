#ifndef PLANNER_H
#define PLANNER_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <planner_node/planner_pub.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <std_msgs/Bool.h>


namespace planner_node
{
class planner_rrt
{
 public:
  explicit planner_rrt(ros::NodeHandle nh);
  void publish_pos(float x, float y, float z);
  void subscribe_reached();
  void reached_cb(const std_msgs::Bool::ConstPtr& Reached);
  bool get_reach();
 private:
  //! Turn on publisher.
  void start();
  // octomap::point3d current_pos;
  bool reached;
  //! Turn off publisher.
  void stop();
  bool enable_;
  ros::Publisher pub_;
  ros::Subscriber sub_;
  ros::NodeHandle nh_;

};
}

#endif  // NODE_EXAMPLE_LISTENER_H
