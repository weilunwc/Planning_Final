#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <planner_node/planner.h>
#include <vector>
#include <algorithm>
using namespace octomap;
using namespace std;

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

bool check_result(OcTreeNode* node) {
	if (node != NULL) {
		if (node->getOccupancy() > 0.5) {
			return true;
		}
		else {
			return false;
		}
	}
	else {
		return false;
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


  // Octotree
  OcTree* tree = new OcTree("willow_large_octomap.bt");
  OcTreeNode* result;
  point3d query;

  std::vector<double> x(100, 0), y(100, 0);
  for (int i = 0; i < 100; i++) {
  	x[i] = -0.1 * (double)i;
  	y[i] = 0.1 * (double)i;
  }


  float z_height = 1.0;
  int counter = 0;
  bool flag = true;
  float old_pos_x = 0.0;
  float old_pos_y = 0.0;
  float old_pos_z = z_height;
  while (ros::ok())
  {
  	// counter = min(99, counter);
  	if(counter > 99) break;
  	counter++;

    query = point3d(x[counter], y[counter], 1.);
    result = tree->search(query);
    bool is_collide = check_result(result);
    if(is_collide) {
    	break;
    	// if(flag){
    	// 	flag = false;
    	// 	old_pos_x = (float)x[counter];
    	// 	old_pos_y = (float)y[counter];
    	// 	old_pos_z = z_height;
    		
    	// }
    }
    //check_result(result);
    cout << (is_collide? "Collision detected" : "Not occupied") << endl;
    // if(flag){
		node.publish_pos((float)x[counter], (float)y[counter], z_height);
    	cout << "counter: " << (float)x[counter] << " " << (float)y[counter] << endl;
    // }
  //   else{
		// node.publish_pos(old_pos_x, old_pos_y, old_pos_z);
  //   	cout << "counter: " << old_pos_x << " " << old_pos_y << endl;

  //   }
    // z_height += 0.01;
    ros::spinOnce();

    loop_rate.sleep();
    // std::cout << count << std::endl;
  }
  ros::spin();


  return 0;
}  // end main()
