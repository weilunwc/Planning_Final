#!/usr/bin/env python
import rospy
import sys
from plan_msgs.srv import DrawPoints, DrawPointsRequest, DrawPath, DrawPathRequest
from geometry_msgs.msg import Point

if __name__=="__main__":
    # kinda wait for the urdf model to swapn
    rospy.sleep(5)


    rospy.init_node("setup_environment")
    rospy.logwarn("Set up environment") 
    targets = [[0,1,2],[2,4,5]]

    try:
        rospy.logwarn("waiting for draw target service")
        rospy.wait_for_service("/draw_target", 2.0)    
        draw_target = rospy.ServiceProxy("draw_target", DrawPoints)
        rospy.logwarn("got draw target service")
    except ROSException:
        rospy.logwarn("failed to get draw target service")
    
    if draw_target != None:
        draw_target_req = DrawPointsRequest()
        for i in range(len(targets)):
            p = Point()
            p.x = targets[i][0]
            p.y = targets[i][1]
            p.z = targets[i][2]
            draw_target_req.points.append(p)
        draw_target(draw_target_req)
    

    # Do the same thing for depots
    depots = [[1,1,1], [0,0,0]]
    try:
        rospy.logwarn("waiting for draw depot service")
        rospy.wait_for_service("/draw_depot", 2.0)    
        draw_depot = rospy.ServiceProxy("draw_depot", DrawPoints)
        rospy.logwarn("got draw depot service")
    except ROSException:
        rospy.logwarn("failed to get draw depot service")

    if draw_depot != None:
        draw_depot_req = DrawPointsRequest()
        for i in range(len(depots)):
            p = Point()
            p.x = depots[i][0]
            p.y = depots[i][1]
            p.z = depots[i][2]
            draw_depot_req.points.append(p)
        rospy.loginfo("{0}".format(draw_depot_req))
        draw_depot(draw_depot_req)
    
    


    
    rospy.spin()

