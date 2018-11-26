### 16-782 Planning and Decision-making in Robotics Final Project ###

The goal of this project is to develop a plannner that is aware of the energy depletion during its mission.

The robotic platform we are going to work with is the hector quadrotor package in Gazebo developed by the Team Hector Darmstadt of Technische Universität Darmstadt.

Make sure you set up the Gazebo version in Gazebo8, since starting from this version we are able to provide visual aids inside the simulation environment.

### UAV Usage

To set up the controller for this quadrotor, just install any package cmake wants. To make life easier, you could try to install everything here if you don't have them:
```
sudo apt-get install ros-kinetic-ros-control
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-unique-identifier
sudo apt-get install ros-kinetic-geographic-info
sudo apt-get install ros-kinetic-laser-geometry
sudo apt-get install ros-kinetic-tf-conversions
sudo apt-get install ros-kinetic-tf2-geometry-msgs
sudo apt-get install ros-kinetic-joy
``` 

The drone package source code is all inside the hector_package folder, you should be able to catkin_make and compile.

However, you might run into some other issue if you have `gazebo8`. Just install all packages with `gazebo8` and remove all those with `gazebo7` (I suppose you are using ROS Kinetic).  

If you want to play around the UAV, please refer to [this (though a bit outdated)](http://wiki.ros.org/hector_quadrotor/Tutorials/Quadrotor%20indoor%20SLAM%20demo)  

__UAV Usage__  

The control code of the UAV is inside the drone_control package, currently is implemented using a simple position P controller (which I think should be enough for this project)

The controller subscribes to the "target_position" topic, which is a geometry_msgs/Pose message type. To test its behavior, launch the demo_controller.launch
Default target position is (0,0,3), you can change it by publishing the topic through command line
rostopic pub /target_position (keep pressing tab to fill out the rest)

### Generate Map ###

Starting from a 2D map, we will need to use the map server package
```
sudp apt-get install ros-kinetic-map-server
```

