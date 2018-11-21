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

I've made things easier in the `scripts` folder with two runnable `run.sh` and `call_srv.sh`.  

You can just do `./run.sh` in the `scripts` folder to launch everything you need (I think).  

To enable the motor controllers of the UAVs do 
```
./call_srv.sh $NUM_ROBOT
``` 
where `$NUM_ROBOT` is the number of robots you are currently launching in your config file.  

To control the height of the UAVs, do
```
python ./altitude_control.py -$ROBOT_NUM -$MIN_HEIGHT
```
Here `MIN_HEIGHT` is the threshold when the z-direction velocity stops publishing, so they might go beyond the height a bit (or a lot depent on the setting of twist).  


### Generate Map ###

Starting from a 2D map, we will need to use the map server package
```
sudp apt-get install ros-kinetic-map-server
```
