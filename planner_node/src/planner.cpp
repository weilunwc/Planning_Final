#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <planner_node/planner.h>
#include <vector>
#include <algorithm>
#include <Eigen/Eigen>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
// #include <fcl/fcl.h>
// #include "fcl/shape/geometric_shapes.h"
// #include "fcl/narrowphase/narrowphase.h"
// #include <fcl/collision.h>
// #include "fcl/ccd/motion.h"
// #include <fcl/geometry/octree/octree.h>
// #include "fcl/traversal/traversal_node_octree.h"
// #include "fcl/broadphase/broadphase.h"
// #include "fcl/shape/geometric_shape_to_BVH_model.h"
// #include "fcl/math/transform.h"
// #include "fcl/BV/AABB.h"
// #include "fcl/collision_object.h"

// using namespace octomap;
// using namespace std;
// using namespace fcl;
namespace planner_node
{
// using namespace octomap;
// using namespace std;


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

bool check_result(octomap::OcTreeNode* node) {
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

bool rayCast_collision(octomap::point3d origin, octomap::point3d des_pos, octomap::OcTree* tree){
	double dr = 0.9; // drone radius
  	octomap::point3d ray_end;
  	octomap::point3d dir = des_pos - origin;
  	octomap::point3d dir_normed = dir.normalize();
  	octomap::point3d extend_vec = octomap::point3d(dir_normed.x()*dr, dir_normed.y()*dr, dir_normed.z()*dr);
	octomap::point3d collision_point = origin + extend_vec;
	tree->castRay(collision_point, direction, ray_end);
	return collision_point.distance(des_pos) >= collision_point.distance(ray_end); 
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
  octomap::OcTree* tree = new octomap::OcTree("willow_large_octomap.bt");
  octomap::OcTreeNode* result;
  octomap::point3d query;

  // OcTree* fcl_tree = new OcTree(tree);

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
  bool is_collide;
  
  while (ros::ok())
  {
  	// counter = min(99, counter);
  	if(counter > 99) break;
  	counter++;

    octomap::point3d origin(0.0, 0.0, z_height);
  	octomap::point3d target_pos(x[counter], y[counter], z_height);
  	is_collide = rayCast_collision(origin, target_pos, tree);

    std::cout << (is_collide? "Collision detected" : "Not occupied") << std::endl;
	node.publish_pos((float)x[counter], (float)y[counter], z_height);
    std::cout << "counter: " << (float)x[counter] << " " << (float)y[counter] << std::endl;
    if(is_collide){
    	break;
    }
    ros::spinOnce();

    loop_rate.sleep();

  }
  ros::spin();


  return 0;
}  // end main()
