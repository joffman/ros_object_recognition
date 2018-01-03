#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseGoal


def _removeComment(line):
    comment_char = "#"  # Comments start with this char.
    comment_start_idx = line.find(comment_char)
    if comment_start_idx != -1:
        line = line[0:comment_start_idx]
    return line


def _assertNumPoseComponents(components):
    num_positions = 3       # x, y, z
    num_orientations = 4    # x, y, z, w

    if len(components) != num_positions + num_orientations:
        raise IOError("wrong number of pose-components; " +
                str(num_positions + num_orientations) + "expected, " +
                str(len(components)) + " provided")


def _convertComponentStringsToFloats(components):
    try:
        floats = [float(x) for x in components]
    except ValueError:
        raise IOError("one given pose contains non-numeric values")
    return floats


def _extractPoseComponentsFromLine(line):
    seperator = ","     # seperates elements of pose

    components = line.split(seperator)
    _assertNumPoseComponents(components)
    components = _convertComponentStringsToFloats(components)

    return components


def _createPoseFromLine(line):
    components = _extractPoseComponentsFromLine(line)

    pose = Pose()

    pose.position.x = components[0]
    pose.position.y = components[1]
    pose.position.z = components[2]

    pose.orientation.x = components[3]
    pose.orientation.y = components[4]
    pose.orientation.z = components[5]
    pose.orientation.w = components[6]

    return pose


def _readPosesFromFile(poses_file):
    """Read poses for the robot-station and the patrol-points from file.
    The file should contain poses in the form 'x,y,z,x,y,z,w'. Comments are
    allowed and start with the comment-character('#').
    The pose of the robot-station is preceded by the station-pose-tag
    ('station_pose:').
    Violations of this format lead to an IOError.
    """
    station_pose_tag = "station_pose:"
    empty_str = ""

    goal_poses = []
    station_pose = None
    for line in poses_file:
        line = _removeComment(line)
        line = line.strip()

        if line.startswith(station_pose_tag):
            line = line.lstrip(station_pose_tag)
            line = line.lstrip()
            if line != empty_str:
                station_pose = _createPoseFromLine(line)
        elif line != empty_str:
            goal_poses.append(_createPoseFromLine(line))

    return goal_poses, station_pose


def _assertNumPatrolPoses(patrol_poses):
    if len(patrol_poses) == 0:
        raise IOError("no patrol poses could be read from file")


def _createMoveBaseGoalFromPose(pose):
    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose = pose

    return goal


def readMoveBaseGoalsFromFile(poses_file):
    """Read and return MoveBaseGoals for the robot-station and patrol-poses.
    If the contents of the file do not obey the syntax rules of
    _readPosesFromFile(), or if no patrol-poses were found, an IOError
    exception is raised.
    """
    patrol_poses, station_pose = _readPosesFromFile(poses_file)

    _assertNumPatrolPoses(patrol_poses)

    patrol_goals = [_createMoveBaseGoalFromPose(x) for x in patrol_poses]
    station_goal = None if station_pose==None else _createMoveBaseGoalFromPose(station_pose)

    return patrol_goals, station_goal

    
if __name__ == "__main__":
    print "This module can be imported for reading MoveBaseGoals from a file \
            whose name is given on the command line."
