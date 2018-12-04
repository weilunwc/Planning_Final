#ifndef PLANNER_H
#define PLANNER_H

// ROS includes.
#include <ros/ros.h>
#include <ros/time.h>
#include <planner_node/planner_pub.h>

namespace planner_node
{
class planner_rrt
{
 public:
  explicit planner_rrt(ros::NodeHandle nh);
  void publish_pos(float x, float y, float z);
 private:
  //! Turn on publisher.
  void start();

  //! Turn off publisher.
  void stop();
  bool enable_;
  ros::Publisher pub_;
  ros::NodeHandle nh_;
  

};
}

#endif  // NODE_EXAMPLE_LISTENER_H
