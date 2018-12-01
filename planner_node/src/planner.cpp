#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <planner_node/planner.h>
namespace planner_node
{
using namespace octomap;
using namespace std;
planner_rrt::planner_rrt(ros::NodeHandle nh): enable_(true)
{
	ROS_INFO("starting node");
  if (enable_)
  {
    start();
  }

}

void planner_rrt::start()
{
  	pub_ = nh_.advertise<planner_node::planner_pub>("drone_pose", 10);
}

void planner_rrt::stop()
{
  pub_.shutdown();
}

void planner_rrt::publish_pos(float x, float y, float z){
  planner_node::planner_pub msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "/world";
  msg.message = "i am groot";
  msg.x = x;
  msg.y = y;
  msg.z = z;

  pub_.publish(msg);

}

}



int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  planner_node::planner_rrt node(nh);

  // Let ROS handle all callbacks.
  
  ros::Rate loop_rate(10);   

  float z_height = 3.0;
  while (ros::ok())
  {

	node.publish_pos(0.0, 0.0, z_height);
    ros::spinOnce();

    loop_rate.sleep();
    z_height += 0.01;
    // std::cout << count << std::endl;
  }
  ros::spin();


  return 0;
}  // end main()
