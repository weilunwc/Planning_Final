#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <planner_node/planner.h>
#include <vector>
#include <algorithm>
#include "RRT.h"

using namespace octomap;
using namespace std;

namespace planner_node
{

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

// Check with ray_casting
bool is_box_collided(octomap::OcTree* tree, std::vector<float> x, std::vector<float> y, std::vector<float> z, octomap::point3d center, float x_sam, float y_sam, float z_sam) {
        octomap::point3d d(x_sam - center.x(), y_sam - center.y(), z_sam - center.z()); // direction
        
        octomap::point3d end;
        // std::cout << "d = "<< d << std::endl; 
    for (int i = 0; i < x.size(); i++) {
        octomap::point3d oi(center.x()+x[i], center.y()+y[i], center.z()+z[i]);
        
        // std::cout << "origin" << i << "= "<< oi << std::endl; 
        bool is_occupied = tree->castRay(oi,d,end,false,-1);
        // std::cout << (is_occupied ? "The ray hit something" : "Nothing hit...") << std::endl;
        // std::cout << end << std::endl; 
    }

    return false;
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  planner_node::planner_rrt node(nh);

  // Octotree
  OcTree* tree = new OcTree("willow_large_octomap.bt");

  double xSize = 10;
  double ySize = 10;
  double zSize = 3;
  // double start[3] = {0,0,1.0};
  // double goal[3] = {4.2,3.5,1.0};
  // double *start =  new double[3];
  // double *goal = new double[3];
  // start[0] = 0;
  // start[1] = 0;
  // start[2] = 1.0;
  // goal[0] = 4.2;
  // goal[1] = 3.5;
  // goal[2] = 1.0;

  vector<double> start;
  vector<double> goal;
  start.push_back(0);
  start.push_back(0);
  start.push_back(1.0);
  goal.push_back(4.2);
  goal.push_back(3.5);
  goal.push_back(1.0);


  RRT rrtNode(xSize,ySize,zSize,start,goal,tree);

  // Let ROS handle all callbacks.
  
  ros::Rate loop_rate(10);   

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


  // define vertices of the bounding box
  float h_x = 0.4, h_y = 0.4, h_z = 0.1;
  std::vector<float> x_v = {h_x, -h_x, h_x, -h_x, h_x, -h_x, h_x, -h_x};
  std::vector<float> y_v = {h_y, h_y, -h_y, -h_y, h_y, h_y, -h_y, -h_y};
  std::vector<float> z_v = {h_z, h_z, h_z, h_z, -h_z, -h_z, -h_z, -h_z};

  float x_sam = 1., y_sam = 0., z_sam = 1.;
  // Test the raycasting function
  octomap::point3d center(0., 0., 1.); 
  is_box_collided(tree, x_v, y_v, z_v, center, x_sam, y_sam, z_sam);

  rrtNode.planner();

  // while (ros::ok())
  // {
  	// if(counter > 99) break;
  	// counter++;

    // validNewConf(point,tree);
    // query = point3d(x[counter], y[counter], 1.);
    // result = tree->search(query);
    // bool is_collide = check_result(result);
    // if(is_collide) {
    // 	break;
    // 	// if(flag){
    // 	// 	flag = false;
    // 	// 	old_pos_x = (float)x[counter];
    // 	// 	old_pos_y = (float)y[counter];
    // 	// 	old_pos_z = z_height;
    		
    // 	// }
    // }
    //check_result(result);
    // cout << (is_collide? "Collision detected" : "Not occupied") << endl;
    // if(flag){
		// node.publish_pos((float)x[counter], (float)y[counter], z_height);
    // 	cout << "counter: " << (float)x[counter] << " " << (float)y[counter] << endl;
    // }
  //   else{
		// node.publish_pos(old_pos_x, old_pos_y, old_pos_z);
  //   	cout << "counter: " << old_pos_x << " " << old_pos_y << endl;

  //   }
    // z_height += 0.01;
  //   ros::spinOnce();

  //   loop_rate.sleep();
  //   // std::cout << count << std::endl;
  // }
  ros::spin();


  return 0;
}  // end main()
