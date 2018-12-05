#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <planner_node/planner.h>
#include <vector>
#include <algorithm>
#include "RRT.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
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
  this->nh_ = nh;
  this->reached = false;
}

void planner_rrt::start()
{
  	// pub_ = nh_.advertise<planner_node::planner_pub>("drone_pose", 10);
    pub_ = nh_.advertise<nav_msgs::Path>("drone_path", 10);
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

void planner_rrt::publish_path(std::vector<double> x, std::vector<double> y, std::vector<double> z) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "/world";
  for (int i = 0; i < x.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "/pose";
    pose.pose.position.x = (float)x[i];
    pose.pose.position.y = (float)y[i];
    pose.pose.position.z = (float)z[i];
    
    path.poses.push_back(pose);
  }
  pub_.publish(path);
}


void planner_rrt::subscribe_reached(){
  // subscriber for current position
  // geometry_msgs::Pose msg_pos;
  sub_ = nh_.subscribe("is_reached", 10000, &planner_rrt::reached_cb, this);
  
}

void planner_rrt::reached_cb(const std_msgs::Bool::ConstPtr& Reached){
  // octomap::point3d curr_pos((float)msg_pos.position.x , (float)msg_pos.position.y, (float)msg_pos.position.z); 
  std::cout << "in the subscriber callbacks: " << std::endl;
  this->reached = Reached->data;
}

bool planner_rrt::get_reach(){
  return this->reached;
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
  // bool rayCast_collision(octomap::point3d origin, octomap::point3d des_pos, octomap::OcTree* tree){
  // 	double dr = 0.9; // drone radius
  //   octomap::point3d ray_end;
  //   octomap::point3d dir = origin - des_pos;
  //   octomap::point3d dir_normed = dir;
  //   dir_normed.normalize();
  //   octomap::point3d extend_vec = octomap::point3d(dir_normed.x()*dr, dir_normed.y()*dr, dir_normed.z()*dr);
  // 	octomap::point3d new_des_pos = des_pos - extend_vec;
  // 	bool success = tree->castRay(origin, -dir, ray_end);
  // 	// std::cout << dir << -dir << dir_normed << extend_vec
  // 	//  << des_pos << new_des_pos << ray_end  << " "
  // 	//  << origin.distance(new_des_pos) << " " << origin.distance(ray_end) << std::endl;
  // 	return success ? origin.distance(new_des_pos) >= origin.distance(ray_end) : false;
  // }

  //   octomap::point3d rayCast_collision(octomap::point3d origin, octomap::point3d des_pos, octomap::OcTree* tree, 
  //   octomap::point3d ray_end){
  //   double dr = 0.9; // drone radius
  //   octomap::point3d dir = origin - des_pos;
  //   octomap::point3d dir_normed = dir;
  //   dir_normed.normalize();
  //   octomap::point3d extend_vec = octomap::point3d(dir_normed.x()*dr, dir_normed.y()*dr, dir_normed.z()*dr);
  //   octomap::point3d new_des_pos = des_pos - extend_vec;
  //   bool success = tree->castRay(origin, -dir, ray_end);
  //   // std::cout << dir << -dir << dir_normed << extend_vec
  //   //  << des_pos << new_des_pos << ray_end  << " "
  //   //  << origin.distance(new_des_pos) << " " << origin.distance(ray_end) << std::endl;
  //   bool is_collide = success ? origin.distance(new_des_pos) >= origin.distance(ray_end) : false;
  //   return is_collide ? ray_end : new_des_pos;

  // }



// Check with ray_casting
// bool is_box_collided(octomap::OcTree* tree, std::vector<float> x, std::vector<float> y, std::vector<float> z, octomap::point3d center, float x_sam, float y_sam, float z_sam) {
//         octomap::point3d d(x_sam - center.x(), y_sam - center.y(), z_sam - center.z()); // direction

//         octomap::point3d end;

//         // std::cout << "d = "<< d << std::endl; 
//     for (int i = 0; i < x.size(); i++) {
//         octomap::point3d oi(center.x()+x[i], center.y()+y[i], center.z()+z[i]);
        
//         // std::cout << "origin" << i << "= "<< oi << std::endl; 
//         bool is_occupied = tree->castRay(oi,d,end,false,-1);
//         // std::cout << (is_occupied ? "The ray hit something" : "Nothing hit...") << std::endl;
//         // std::cout << end << std::endl; 

//         std::cout << "d = "<< d << std::endl;
//     for (int i = 0; i < x.size(); i++) {
//         octomap::point3d oi(center.x()+x[i], center.y()+y[i], center.z()+z[i]);

//         std::cout << "origin" << i << "= "<< oi << std::endl;
//         bool is_occupied = tree->castRay(oi,d,end,false,-1);
//         std::cout << (is_occupied ? "The ray hit something" : "Nothing hit...") << std::endl;
//         std::cout << end << std::endl;

//     }

//     return false;
// }

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "planner");
  ros::NodeHandle nh;

  // Create a new node_example::Talker object.
  planner_node::planner_rrt node(nh);

  // Octotree
  // OcTree* tree = new OcTree("willow_large_octomap.bt");
  OcTree* tree = new OcTree("test_city2.bt");

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
  start.push_back(0.);
  start.push_back(0.);
  start.push_back(1.0);
  // goal.push_back(-9.5);
  // goal.push_back(-6.5);
  // goal.push_back(1.2);

  // start.push_back(21.86);
  // start.push_back(2.05);
  // start.push_back(2.0);
  goal.push_back(14);
  goal.push_back(-21);
  goal.push_back(5.);


  RRT rrtNode(xSize,ySize,zSize,start,goal,tree);

  // Let ROS handle all callbacks.

  ros::Rate loop_rate(10);

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


  while (ros::ok())
  {
    rrtNode.plan();
    vector< vector<double> > plan = rrtNode.exportPlan();

    for(auto p:plan)
    {
      cout << "x: " << p[0] << " y: " << p[1] << " z: " << p[2] <<endl;
    }

    vector<double> X;
    vector<double> Y;
    vector<double> Z;

    for (auto p : plan){
      X.push_back(p[0]);
      Y.push_back(p[1]);
      Z.push_back(p[2]);
    }

    std::reverse(X.begin(), X.end());
    std::reverse(Y.begin(), Y.end());
    std::reverse(Z.begin(), Z.end());

    // if(counter > 99) break;
    // counter++;

    // validNewConf(point,tree);
    // query = point3d(x[counter], y[counter], 1.);
    // result = tree->search(query);
    // bool is_collide = check_result(result);
    // if(is_collide) {
    //  break;
    //  // if(flag){
    //  //  flag = false;
    //  //  old_pos_x = (float)x[counter];
    //  //  old_pos_y = (float)y[counter];
    //  //  old_pos_z = z_height;
        
    //  // }
    // }
    //check_result(result);
    // cout << (is_collide? "Collision detected" : "Not occupied") << endl;
    // if(flag){
    // node.publish_pos((float)x[counter], (float)y[counter], z_height);
    //  cout << "counter: " << (float)x[counter] << " " << (float)y[counter] << endl;
    // }
  //   else{
    // node.publish_pos(old_pos_x, old_pos_y, old_pos_z);
  //    cout << "counter: " << old_pos_x << " " << old_pos_y << endl;

  //   }
    // z_height += 0.01;
    node.publish_path( X, Y, Z);
    // rrtNode.start_ = goal;
    // vector<double> new_goal{-4,-21,1.5};
    // rrtNode.goal_ = new_goal;
    ros::spinOnce();

    loop_rate.sleep();
  //   // std::cout << count << std::endl;
  }
  ros::spin();



return 0;
}  // end main()
