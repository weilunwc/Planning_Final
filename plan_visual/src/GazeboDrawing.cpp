#include "GazeboDrawing.h"

GazeboDrawing::GazeboDrawing(){
    ROS_INFO("Initializing gazebo drawing");
    // set up drawing server 
    this->draw_target_service = this->global.advertiseService("/draw_target", &GazeboDrawing::draw_target_cb, this);
    this->draw_depot_service = this->global.advertiseService("/draw_depot", &GazeboDrawing::draw_depot_cb, this);
    this->draw_path_service = this->global.advertiseService("/draw_path", &GazeboDrawing::draw_path_cb, this);
    
    // set up markers 
    this->targetMarker.set_ns("targets");
    ignition::msgs::Material *targetMaterial = this->targetMarker.mutable_material();
    targetMaterial->mutable_script()->set_name("Gazebo/BlueLaser"); 
    
    this->depotMarker.set_ns("depots");
    ignition::msgs::Material *depotMaterial = this->depotMarker.mutable_material();
    depotMaterial->mutable_script()->set_name("Gazebo/Red"); 
    
    this->pathMarker.set_ns("path");
    this->pathMarker.set_action(ignition::msgs::Marker::ADD_MODIFY);
    this->pathMarker.set_type(ignition::msgs::Marker::LINE_STRIP);
    ignition::msgs::Material *pathMaterial = this->pathMarker.mutable_material();
    pathMaterial->mutable_script()->set_name("Gazebo/Green"); 
    
}

bool GazeboDrawing::draw_target_cb(plan_msgs::DrawPoints::Request &req, plan_msgs::DrawPoints::Response &res){
    
    // delete previous points first
    this->targetMarker.set_action(ignition::msgs::Marker::DELETE_ALL);
    this->node.Request("/marker", this->targetMarker);
    this->DrawPoints(req.points, this->targetMarker);
    return true;
}

bool GazeboDrawing::draw_depot_cb(plan_msgs::DrawPoints::Request &req, plan_msgs::DrawPoints::Response &res){
    
    // delete previous points first
    this->depotMarker.set_action(ignition::msgs::Marker::DELETE_ALL);
    this->node.Request("/marker", this->depotMarker);
    this->DrawPoints(req.points, this->depotMarker);
    return true;
}

bool GazeboDrawing::draw_path_cb(plan_msgs::DrawPath::Request &req, plan_msgs::DrawPath::Response &res){
    
    this->DrawPath(req.path);
    return true;
}


// Drawing Funcions

void GazeboDrawing::DrawPoints(std::vector<geometry_msgs::Point> &points, ignition::msgs::Marker &marker){
    // draw points using a circular representation
    int n = points.size();
    for(int i = 0;i < n;i++){
        marker.set_id(i);
        marker.set_action(ignition::msgs::Marker::ADD_MODIFY);
        marker.set_type(ignition::msgs::Marker::SPHERE);
        double x = points[i].x;
        double y = points[i].y;
        double z = points[i].z;
        ignition::msgs::Set(marker.mutable_pose(), ignition::math::Pose3d(x,y,z,0,0,0));
        ignition::msgs::Set(marker.mutable_scale(), ignition::math::Vector3d(0.2, 0.2, 0.2));
        
        this->node.Request("/marker", marker);
    }
}

void GazeboDrawing::DrawPath(nav_msgs::Path path)
{
    this->pathMarker.set_id(0);
    this->pathMarker.clear_point();
    int n = path.poses.size();
    for(int i = 0;i < n;i++)
    {
        double x = path.poses[i].pose.position.x;
        double y = path.poses[i].pose.position.y;
        ignition::msgs::Set(this->pathMarker.add_point(), ignition::math::Vector3d(x, y, 0.05));
    }
    this->node.Request("/marker", this->pathMarker);

}
