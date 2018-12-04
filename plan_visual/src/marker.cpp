/*
 *  Gazebo interface of adding visual markers example
 *  Reference - https://bitbucket.org/osrf/gazebo/src/default/examples/stand_alone/marker/marker.cc
 */

#include <iostream>
#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>
#include <gazebo/common/Time.hh>

int main(){
    int delay_time = 1;
    
    // interface with gazebo
    ignition::transport::Node node;

    // create marker msg
    ignition::msgs::Marker markerMsg;
    markerMsg.set_ns("default"); 
    markerMsg.set_id(0);
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::SPHERE);
    

    // set up marker material
    ignition::msgs::Material *matMsg = markerMsg.mutable_material();
    matMsg->mutable_script()->set_name("Gazebo/BlueLaser");
    
    // spawn markers
    std::cout << "Spawning a sphere at origin\n";
    gazebo::common::Time::Sleep(delay_time);
    node.Request("/marker", markerMsg);
    
    std::cout << "Moving the sphere to x=0, y=0, z=1\n"; 
    gazebo::common::Time::Sleep(delay_time);
    ignition::msgs::Set(markerMsg.mutable_pose(), ignition::math::Pose3d(0,0,1,0,0,0));
    node.Request("/marker", markerMsg);
    
    std::cout << "Shrinking the sphere\n";
    gazebo::common::Time::Sleep(delay_time);
    ignition::msgs::Set(markerMsg.mutable_scale(), ignition::math::Vector3d(0.2,0.2,0.2));
    node.Request("/marker", markerMsg);
    
    std::cout << "Changing the sphere to red\n";
    gazebo::common::Time::Sleep(delay_time);
    matMsg->mutable_script()->set_name("Gazebo/Red");
    node.Request("/marker", markerMsg);
    
    std::cout << "Adding a green box\n";
    gazebo::common::Time::Sleep(delay_time);
    markerMsg.set_id(1);
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::BOX);
    matMsg->mutable_script()->set_name("Gazebo/Green");
    ignition::msgs::Set(markerMsg.mutable_scale(), ignition::math::Vector3d(1.0,1.0,1.0));
    ignition::msgs::Set(markerMsg.mutable_pose(), ignition::math::Pose3d(2,0,.5,0,0,0));
    node.Request("/marker", markerMsg);

    std::cout << "Changing the green box to a cylinder\n";
    gazebo::common::Time::Sleep(delay_time);
    markerMsg.set_type(ignition::msgs::Marker::CYLINDER);
    node.Request("/marker", markerMsg);
    
    std::cout << "Adding a line between the sphere and cylinder";
    gazebo::common::Time::Sleep(delay_time);
    markerMsg.set_id(2);
    ignition::msgs::Set(markerMsg.mutable_pose(), ignition::math::Pose3d(0,0,0,0,0,0));
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::LINE_LIST);
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(0,0,1.1));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(2,0,0.5));
    node.Request("/marker", markerMsg);
    
    std::cout << "Adding a square around the origin\n";
    gazebo::common::Time::Sleep(delay_time);
    matMsg->mutable_script()->set_name("Gazebo/Green");
    markerMsg.set_id(3);
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
    markerMsg.clear_point();
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d( 0.5, 0.5,0.005));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d( 0.5,-0.5,0.005));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(-0.5,-0.5,0.005));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(-0.5, 0.5,0.005));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d( 0.5, 0.5,0.005));
    node.Request("/marker", markerMsg);
    
    std::cout << "Adding 100 points inside the square\n";
    gazebo::common::Time::Sleep(delay_time);
    markerMsg.set_id(4);
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::POINTS);
    ignition::msgs::Set(markerMsg.mutable_scale(), ignition::math::Vector3d(1.0,1.0,1.0));
    markerMsg.clear_point(); 
    for(int i = 0;i < 100;i++){
        ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(ignition::math::Rand::DblUniform(-0.49, 0.49),
                                                                            ignition::math::Rand::DblUniform(-0.49, 0.49),
                                                                            0.05));
    }
    node.Request("/marker", markerMsg);

    std::cout << "Adding Hello at (0,0,2)\n";
    gazebo::common::Time::Sleep(delay_time);
    markerMsg.set_id(5);
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::TEXT);
    markerMsg.set_text("HELLO");
    ignition::msgs::Set(markerMsg.mutable_scale(), ignition::math::Vector3d(.2,.2,.2));
    ignition::msgs::Set(markerMsg.mutable_pose(), ignition::math::Pose3d(0,0,2,0,0,0));
    node.Request("/marker", markerMsg);

    std::cout << "Adding a semi-circular triangle fan\n";
    gazebo::common::Time::Sleep(delay_time);
    markerMsg.set_id(6);
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_FAN);
    markerMsg.clear_point();
    // mutable pose sets an offet
    ignition::msgs::Set(markerMsg.mutable_pose(), ignition::math::Pose3d(0,1.5,0,0,0,0));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(0,0,0.05));
    double radius = 2;
    for(double t = 0; t <= M_PI; t+=0.01){
        ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(radius*cos(t), radius*sin(t), 0.05));
    }
    node.Request("/marker", markerMsg);
    
    std::cout << "Adding two triangles using a triangle list\n";
    gazebo::common::Time::Sleep(delay_time);
    markerMsg.set_id(7);
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_LIST);
    markerMsg.clear_point();
    ignition::msgs::Set(markerMsg.mutable_pose(), ignition::math::Pose3d(0,-1.5,0,0,0,0));
    // add traingle fan    
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(0,0,0.05));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(1,0,0.05));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(1,1,0.05));

    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(1,1,0.05));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(2,1,0.05));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(2,2,0.05));
    node.Request("/marker", markerMsg);
    std::cout << "Adding a rectangular triangle strip\n";
    gazebo::common::Time::Sleep(delay_time);
    markerMsg.set_id(8);
    markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
    markerMsg.set_type(ignition::msgs::Marker::TRIANGLE_STRIP);
    markerMsg.clear_point();
    // mutable pose sets an offet
    ignition::msgs::Set(markerMsg.mutable_pose(), ignition::math::Pose3d(-2,-2,0,0,0,0));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(0,0,0.05));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(1,0,0.05));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(0,1,0.05));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(1,1,0.05));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(0,2,0.05));
    ignition::msgs::Set(markerMsg.add_point(), ignition::math::Vector3d(1,2,0.05));
    node.Request("/marker", markerMsg);

    



    std::cout << "Delete all markers\n";
    gazebo::common::Time::Sleep(delay_time);
    markerMsg.set_action(ignition::msgs::Marker::DELETE_ALL);
    node.Request("/marker", markerMsg);
    return 0;
}
