#!/usr/bin/env python
import rospy
import sys

from geometry_msgs.msg import Twist, Pose, Point
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
from planner_node.msg import planner_pub 
from std_msgs.msg import Bool
class UavRobotControl:
    def __init__(self):
        # initialize variables
        self.robot_name = "robot"
        self.curr_pos = Pose()
        self.received_pose_info = False
        self.diff_pos = Pose()
        self.target_pos = Pose()

        # Initialize its targe pose to be at height 3
        self.target_pos.position.z = 1
        self.received_target_info = False
        
        self.Kpz = 1.5
        self.Kpx = 1.5
        self.Kpy = 1.5

        # Subscriber of gazebo states to extract robot state info
        self.robot_curr_pos_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.curr_pos_cb)
        
        # Subscriber to target position
        self.target_position_sub = rospy.Subscriber("/target_position", Pose, self.target_pos_cb)
        self.diff_pos = rospy.Subscriber("/drone_pose", planner_pub, self.drone_pos_cb)
        # Publisher to gazebo sending commands 
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
        
        # Publisher sending messages of current position for easier debug
        self.is_reached_pub = rospy.Publisher("is_reached", Bool, queue_size=100)

    def curr_pos_cb(self, msg = ModelStates()):
        # Extract the drone position from the gazebo model states 
        for index in range(0, len(msg.name)):
            curr_name = msg.name[index]
            #keep updating goal pose since we only care about turning... robot should be in same position though.
            if curr_name == self.robot_name:
                self.received_pose_info = True
                self.curr_pos = msg.pose[index]
    
    def target_pos_cb(self, target):
        self.target_pos = target

    def drone_pos_cb(self, planner_pub):
        self.target_pos.position.x = planner_pub.x
        self.target_pos.position.y = planner_pub.y
        self.target_pos.position.z = planner_pub.z
        # rospy.loginfo("dron/e_pose.z: ", planner_pub.z)

    # def test_add_height(self, coeff=0.01):
    #     self.target_pos.position.z = min(5.0, target_pos.position.z+coeff)
    #     rospy.loginfo("test_add_height: ", self.target_pos.position.z)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Calculate command using simple P control
            curr_cmd_vel = Twist()
            curr_cmd_vel.linear.x = self.Kpx * (self.target_pos.position.x - self.curr_pos.position.x)
            curr_cmd_vel.linear.y = self.Kpy * (self.target_pos.position.y - self.curr_pos.position.y)
            curr_cmd_vel.linear.z = self.Kpz * (self.target_pos.position.z - self.curr_pos.position.z)

            # Send command
            self.cmd_vel_pub.publish(curr_cmd_vel)
            self.is_reached_pub.publish(True)
            # self.test_add_height()
            rate.sleep()
            

if __name__=="__main__":

    rospy.init_node("uav_robot_controller")
    
    robot_controller = UavRobotControl()

    rospy.sleep(2)
    robot_controller.run()
    rospy.spin()

