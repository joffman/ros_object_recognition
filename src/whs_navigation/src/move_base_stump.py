#!/usr/bin/env python

## move_base_stump:
#   SimpleActionServer for emulating the move_base Action Server.
#   This can be used for testing programs that use the move_base action.

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction


def goal_cb(goal):
    rospy.loginfo("I received:")
    rospy.loginfo("\tHeader:")
    rospy.loginfo("\t\tframe_id: %s" %goal.target_pose.header.frame_id)
    rospy.loginfo("\t\tstamp: %s" %goal.target_pose.header.stamp)
    rospy.loginfo("\t\tseq: %s" %goal.target_pose.header.seq)
    rospy.loginfo("\tPose:")
    rospy.loginfo("\t\tposition.x: %s" %goal.target_pose.pose.position.x)
    rospy.loginfo("\t\tposition.y: %s" %goal.target_pose.pose.position.y)
    rospy.loginfo("\t\tposition.z: %s" %goal.target_pose.pose.position.z)
    rospy.loginfo("\t\torientation.x: %s" %goal.target_pose.pose.orientation.x)
    rospy.loginfo("\t\torientation.y: %s" %goal.target_pose.pose.orientation.y)
    rospy.loginfo("\t\torientation.z: %s" %goal.target_pose.pose.orientation.z)
    rospy.loginfo("\t\torientation.w: %s" %goal.target_pose.pose.orientation.w)

    rospy.sleep(1.0)

    server.set_succeeded()


if __name__ == "__main__":
    ## Setup                    ==============================================
    rospy.init_node("move_base_stump")

    server = actionlib.SimpleActionServer("move_base", MoveBaseAction, goal_cb,
            False)
    server.start()

    ## Event loop               ==============================================
    rospy.spin()
