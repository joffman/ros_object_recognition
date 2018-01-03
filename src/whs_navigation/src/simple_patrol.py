#!/usr/bin/env python

## simple_patrol.py
#   This program lets the Turtlebot patrol, using a given file of goal-poses.

import sys

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction

from move_base_goals_from_file import get_move_base_goals


client = actionlib.SimpleActionClient("move_base", MoveBaseAction)


def onShutdown():
    client.cancel_goal()


def main():
    ## Setup            =============================================
    rospy.init_node("patrol")

    # Try to open given file.
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) != 2:
        rospy.loginfo("Usage: %s <poses_file>" %argv[0])
        return 0
    try:
        poses_file = open(argv[1], "r")
    except IOError as e:
        rospy.logfatal("Cannot open file '%s': %s" %(argv[1], e))
        return 1

    rospy.on_shutdown(onShutdown)

    rospy.loginfo("Waiting for action server ...")
    client.wait_for_server()

    # Create goals for move_base.
    rospy.logdebug("Creating move_base goals ...")
    try:
        move_base_goals = get_move_base_goals(poses_file)
    except IOError as e:
        rospy.logfatal("Format of file '%s' is invalid: %s" %(argv[1], e))
        rospy.logfatal("File has to contain poses with format: x,y,z, x,y,z,w")
        return 1

    ## Event loop       =============================================
    rospy.loginfo("Beginning to send action goals ...")
    while not rospy.is_shutdown():
        for goal in move_base_goals:
            client.send_goal(goal)
            client.wait_for_result()

    ## Clean up and return  =========================================
    return 0


if __name__ == "__main__":
    main()
