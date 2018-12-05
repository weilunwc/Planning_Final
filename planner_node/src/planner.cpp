#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <planner_node/planner.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/Pose.h>
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
  this->nh_ = nh;

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


void planner_rrt::subscribe_pos(){
  // subscriber for current position
  // geometry_msgs::Pose msg_pos;
  sub_ = nh_.subscribe("curr_pos", 10000, &planner_rrt::current_pos_cb, this);
  
}

void planner_rrt::current_pos_cb(geometry_msgs::Pose msg_pos){
  octomap::point3d curr_pos((float)msg_pos.position.x , (float)msg_pos.position.y, (float)msg_pos.position.z); 
  std::cout << "in the subscriber callbacks: " << curr_pos << std::endl;
  this->current_pos = curr_pos;
}

octomap::point3d planner_rrt::get_current_pos(){
  return this->current_pos;
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
bool rayCast_collision(octomap::point3d origin, octomap::point3d des_pos, octomap::OcTree* tree){
	double dr = 0.9; // drone radius
  octomap::point3d ray_end;
  octomap::point3d dir = origin - des_pos;
  octomap::point3d dir_normed = dir;
  dir_normed.normalize();
  octomap::point3d extend_vec = octomap::point3d(dir_normed.x()*dr, dir_normed.y()*dr, dir_normed.z()*dr);
	octomap::point3d new_des_pos = des_pos - extend_vec;
	bool success = tree->castRay(origin, -dir, ray_end);
	// std::cout << dir << -dir << dir_normed << extend_vec
	//  << des_pos << new_des_pos << ray_end  << " "
	//  << origin.distance(new_des_pos) << " " << origin.distance(ray_end) << std::endl;
	return success ? origin.distance(new_des_pos) >= origin.distance(ray_end) : false;
}

octomap::point3d rayCast_collision(octomap::point3d origin, octomap::point3d des_pos, octomap::OcTree* tree, 
  octomap::point3d ray_end){
  double dr = 0.9; // drone radius
  octomap::point3d dir = origin - des_pos;
  octomap::point3d dir_normed = dir;
  dir_normed.normalize();
  octomap::point3d extend_vec = octomap::point3d(dir_normed.x()*dr, dir_normed.y()*dr, dir_normed.z()*dr);
  octomap::point3d new_des_pos = des_pos - extend_vec;
  bool success = tree->castRay(origin, -dir, ray_end);
  // std::cout << dir << -dir << dir_normed << extend_vec
  //  << des_pos << new_des_pos << ray_end  << " "
  //  << origin.distance(new_des_pos) << " " << origin.distance(ray_end) << std::endl;
  bool is_collide = success ? origin.distance(new_des_pos) >= origin.distance(ray_end) : false;
  return is_collide ? ray_end : new_des_pos;

}



// Check with ray_casting
bool is_box_collided(octomap::OcTree* tree, std::vector<float> x, std::vector<float> y, std::vector<float> z, octomap::point3d center, float x_sam, float y_sam, float z_sam) {
        octomap::point3d d(x_sam - center.x(), y_sam - center.y(), z_sam - center.z()); // direction

        octomap::point3d end;
        std::cout << "d = "<< d << std::endl;
    for (int i = 0; i < x.size(); i++) {
        octomap::point3d oi(center.x()+x[i], center.y()+y[i], center.z()+z[i]);

        std::cout << "origin" << i << "= "<< oi << std::endl;
        bool is_occupied = tree->castRay(oi,d,end,false,-1);
        std::cout << (is_occupied ? "The ray hit something" : "Nothing hit...") << std::endl;
        std::cout << end << std::endl;
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

  // Let ROS handle all callbacks.

  ros::Rate loop_rate(10);


  // Octotree
  OcTree* tree = new OcTree("willow_large_octomap.bt");
  OcTreeNode* result;
  point3d query;

  std::vector<double> x(100, 0), y(100, 0);
  for (int i = 0; i < 100; i++) {
  	x[i] = -0.1 * (double)i;
  }

  for (int j = 0; j < 100; j++){
  	y[j] = 0.1 * (double)j;
    
  }


float z_height = 1.0;
int counter = 0;
bool flag = true;
float old_pos_x = 0.0;
float old_pos_y = 0.0;
float old_pos_z = z_height;
bool is_collide = false;


// // define vertices of the bounding box
// float h_x = 0.4, h_y = 0.4, h_z = 0.1;
// std::vector<float> x_v = {h_x, -h_x, h_x, -h_x, h_x, -h_x, h_x, -h_x};
// std::vector<float> y_v = {h_y, h_y, -h_y, -h_y, h_y, h_y, -h_y, -h_y};
// std::vector<float> z_v = {h_z, h_z, h_z, h_z, -h_z, -h_z, -h_z, -h_z};

// float x_sam = 1., y_sam = 0., z_sam = 1.;
// // Test the raycasting function
// octomap::point3d center(0., 0., 1.);
// is_collide = is_box_collided(tree, x_v, y_v, z_v, center, x_sam, y_sam, z_sam);

std::vector<float> way_x{0.0, -2.0, -4.0, -5.0, -6.0, -7.0, -8.0};
std::vector<float> way_y{0.0, -2.0, -4.0, -4.0, -4.0, -5.0, -6.0};
counter = 0;
while (ros::ok())
{
	// if(counter ==  way_x.size() - 1) break;
  octomap::point3d origin((float)0.0, (float)0.0, z_height);
  // octomap::point3d origin((float)way_x[counter-1], (float)way_y[counter-1], z_height);
  octomap::point3d target_pos(x[counter], y[counter], z_height);
  // octomap::point3d target_pos((float)way_x[counter], (float)way_y[counter], z_height);
  is_collide = rayCast_collision(origin, target_pos, tree);

  std::cout << (is_collide? "Collision detected" : "No collision") << std::endl;
  // std::cout << "counter: " << (float)way_x[counter] << " " << (float)way_y[counter] << std::endl;
  std::cout << "counter: " << (float)x[counter] << " " << (float)y[counter] << std::endl;
  // node.publish_pos((float)way_x[counter], (float)way_y[counter], z_height);
  
  counter++;
  if (is_collide || counter > 99) {

    break;
  }

  octomap::point3d cur_pos = node.get_current_pos();
  std::cout << "current node:   " << cur_pos << std::endl;
  node.publish_pos((float)x[counter], (float)y[counter], z_height);
  ros::spinOnce();

  loop_rate.sleep();
  node.subscribe_pos();
  loop_rate.sleep();

	// query = point3d(x[counter], y[counter], 1.);
	// result = tree->search(query);
	// bool is_collide = check_result(result);

}
ros::spin();


return 0;
}  // end main()
