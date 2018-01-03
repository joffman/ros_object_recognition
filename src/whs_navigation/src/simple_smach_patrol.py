#!/usr/bin/env python

# TODO: Bug: The node doesn't stop on ^C until the current goal is reached.
#   Change that.

## simple_patrol.py
#   This program lets the Turtlebot patrol, using a given file of goal-poses.

import sys

import rospy
import smach, smach_ros
import actionlib
from move_base_msgs.msg import MoveBaseAction

from move_base_goals_from_file import get_move_base_goals


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

    # Setup the action client.
    client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
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

    # Create StateMachine for patroling.
    patrol_sm = \
            smach.StateMachine(outcomes=["succeeded", "aborted", "preempted"])
    with patrol_sm:
        for i, goal_pose in enumerate(move_base_goals):
            state_name = "POSE_" + str(i)
            next_state_name = "POSE_" + str((i+1) % len(move_base_goals))
            smach.StateMachine.add(state_name,
                    smach_ros.SimpleActionState("move_base", MoveBaseAction,
                        goal=goal_pose),
                    transitions={"succeeded": next_state_name})
            # TODO: Handling of other outcomes than 'succeeded'.

    ## Event loop       =============================================
    patrol_sm.execute()

    ## Clean up and return  =========================================
    return 0


if __name__ == "__main__":
    main()
