#!/usr/bin/env python

import sys

import rospy

from read_move_base_goals import readMoveBaseGoalsFromFile
from smach_patrol import SmachPatrol


def printUsage(program_name):
    rospy.loginfo("Usage: %s <poses-file>", program_name)


def getPosesFilenameFromCmdLine():
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) != 2:
        printUsage(argv[0])
        raise IOError("2 command line arguments expected, " + \
                str(len(argv)) + " given")
    return argv[1]


def main():
    rospy.init_node("patrol")

    poses_filename = getPosesFilenameFromCmdLine()
    poses_file = open(poses_filename, 'r')
    patrol_goals, docking_goal = readMoveBaseGoalsFromFile(poses_file)

    patrol = SmachPatrol(patrol_goals, docking_goal)
    patrol.run()


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        rospy.logfatal("Exception: %s" %e)
