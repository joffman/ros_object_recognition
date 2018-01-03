#!/usr/bin/env python

# TODO: Shut down robot in SHUTDOWN state. Is this possible?
# TODO: Transition to SHUTDOWN state when recharging is interrupted.

import threading

import rospy
import smach, smach_ros
import actionlib

from sensor_msgs.msg import BatteryState
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction

from kobuki_msgs.msg import Sound
from kobuki_msgs.msg import PowerSystemEvent
from kobuki_msgs.msg import AutoDockingAction

from dynamic_reconfigure.server import Server
from whs_navigation.cfg import ParametersConfig


class _ResultSoundsPublisher(object):
    def __init__(self):
        self.__sounds_pub = rospy.Publisher("sound_command", Sound,
                queue_size=10)

    def playResultSound(self, userdata=None, status=GoalStatus.SUCCEEDED,
            result=None):
        if status == GoalStatus.SUCCEEDED:
            self.__sounds_pub.publish(Sound(value=Sound.BUTTON))
        elif status == GoalStatus.ABORTED:
            self.__sounds_pub.publish(Sound(value=Sound.ERROR))
        elif status == GoalStatus.PREEMPTED:
            self.__sounds_pub.publish(Sound(value=Sound.ERROR))
       
_sounds_publisher = _ResultSoundsPublisher()
    # That's my python-equivalent of a function with a static variable.


class _BatteryMonitor(object):
    def __init__(self):
        self.__laptop_battery_limit = 20.
        self.__laptop_battery_percentage = 100.
        self.__robot_battery_low = False

        self.__laptop_charge_sub = rospy.Subscriber("laptop_charge",
                BatteryState, self.__laptopBatteryCB, queue_size=10)
        self.__robot_battery_sub = rospy.Subscriber("power_system_event",
                PowerSystemEvent, self.__robotBatteryCB, queue_size=10)

        self.__reconfigure_server = Server(ParametersConfig,
                self.__dynamicReconfigureCB)


    def __laptopBatteryCB(self, msg):
        self.__laptop_battery_percentage = msg.percentage


    def __robotBatteryCB(self, msg):
        if msg.event == PowerSystemEvent.BATTERY_LOW or \
                msg.event == PowerSystemEvent.BATTERY_CRITICAL:
            self.__robot_battery_low = True 
        elif msg.event == PowerSystemEvent.CHARGE_COMPLETED:
            self.__robot_battery_low = False


    def __dynamicReconfigureCB(self, cfg, level):
        self.__laptop_battery_limit = cfg.laptop_battery_limit
        self.__robot_battery_low = cfg.robot_battery_low
        return cfg


    def laptopBatteryLow(self):
        return self.__laptop_battery_percentage < self.__laptop_battery_limit


    def robotBatteryLow(self):
        return self.__robot_battery_low


def _createPatrolSM(patrol_goals):
    patrol_sm = smach.StateMachine(outcomes=["preempted"])
    with patrol_sm:
        for i, goal in enumerate(patrol_goals):
            state_name = "POSE_" + str(i)
            next_state_name = "POSE_" + str((i+1) % len(patrol_goals))
            smach.StateMachine.add(state_name,
                    smach_ros.SimpleActionState("move_base", MoveBaseAction,
                        goal=goal, result_cb=_sounds_publisher.playResultSound),
                    transitions={"succeeded": next_state_name,
                        "aborted": next_state_name, "preempted": "preempted"})
    return patrol_sm


def _monitoredPatrolChildTerminationCB(outcome_map):
    # Preempt all remaining states of the concurrence as soon as one
    # child-state terminates.
    return True


class _LaptopBatteryMonitorState(smach.State):
    def __init__(self, battery_monitor):
        smach.State.__init__(self, outcomes=["laptop_battery_low", "preempted"])
        self.__battery_monitor = battery_monitor

    def execute(self, userdata):
        while not self.__preemptedWhileSleeping():
            if self.__battery_monitor.laptopBatteryLow():
                return "laptop_battery_low"
        return "preempted"

    def __preemptedWhileSleeping(self):
        """Try to sleep. Return True if sleep is preempted, False otherwise."""
        if self.preempt_requested():
            return True
        try:
            rospy.sleep(5.)
            return False
        except rospy.exceptions.ROSInterruptException:
            return True


class _RobotBatteryMonitorState(smach.State):
    def __init__(self, battery_monitor):
        smach.State.__init__(self, outcomes=["robot_battery_low", "preempted"])
        self.__battery_monitor = battery_monitor

    def execute(self, userdata):
        while not self.__preemptedWhileSleeping():
            if self.__battery_monitor.robotBatteryLow():
                return "robot_battery_low"
        return "preempted"

    def __preemptedWhileSleeping(self):
        """Try to sleep. Return True if sleep is preempted, False otherwise."""
        if self.preempt_requested():
            return True
        try:
            rospy.sleep(5.)
            return False
        except rospy.exceptions.ROSInterruptException:
            return True


def _createMonitoredPatrolSM(patrol_goals, battery_monitor):
    patrol_sm = _createPatrolSM(patrol_goals)

    monitored_patrol_sm = smach.Concurrence(
            outcomes=["preempted", "laptop_battery_low", "robot_battery_low"],
            default_outcome="preempted",
            outcome_map={"laptop_battery_low":
                {"LAPTOP_BATTERY_MONITOR": "laptop_battery_low"},
                "robot_battery_low":
                {"ROBOT_BATTERY_MONITOR": "robot_battery_low"}},
            child_termination_cb=_monitoredPatrolChildTerminationCB)

    with monitored_patrol_sm:
        smach.Concurrence.add("PATROL", patrol_sm)
        smach.Concurrence.add("LAPTOP_BATTERY_MONITOR",
                _LaptopBatteryMonitorState(battery_monitor))
        smach.Concurrence.add("ROBOT_BATTERY_MONITOR",
                _RobotBatteryMonitorState(battery_monitor))

    return monitored_patrol_sm


def _createDockingSM(docking_goal):
    dock_sm = smach.StateMachine(outcomes=["succeeded", "failed"])
    with dock_sm:
        smach.StateMachine.add("MOVE_TO_STATION",
                smach_ros.SimpleActionState("move_base", MoveBaseAction,
                    goal=docking_goal,
                    result_cb=_sounds_publisher.playResultSound),
                transitions={"succeeded": "DOCK", "aborted": "failed",
                    "preempted": "failed"})
        smach.StateMachine.add("DOCK",
                smach_ros.SimpleActionState("dock_drive_action",
                    AutoDockingAction),
                transitions={"succeeded": "succeeded", "aborted": "failed",
                    "preempted": "failed"})
    return dock_sm


class _WaitUntilChargedState(smach.State):
    def __init__(self, battery_monitor):
        smach.State.__init__(self, outcomes=["robot_battery_full", "preempted"])
        self.__battery_monitor = battery_monitor

    def execute(self, userdata):
        try:
            self.__waitUntilFullyCharged()
            return "robot_battery_full"
        except rospy.exceptions.ROSInterruptException:
            return "preempted"

    def __waitUntilFullyCharged(self):
        while self.__battery_monitor.robotBatteryLow():
            # TODO: Check that we are still charging.
            rospy.sleep(5.)


class _ShutdownState(smach.State):   # TODO: Shut down robot. Possible?
    def __init__(self):
        smach.State.__init__(self, outcomes=["shutdown"])

    def execute(self, userdata):
        rospy.loginfo("Shutting down node.")
        rospy.signal_shutdown("SHUTDOWN state executed")
        return "shutdown"


def _createTopLevelSM(patrol_goals, docking_goal):
    battery_monitor = _BatteryMonitor()

    monitored_patrol_sm = _createMonitoredPatrolSM(patrol_goals,
            battery_monitor)
    if docking_goal != None:
        dock_sm = _createDockingSM(docking_goal)

    top_sm = smach.StateMachine(outcomes=["shutdown"])
    with top_sm:
        if docking_goal != None:
            smach.StateMachine.add("MONITORED_PATROL", monitored_patrol_sm,
                    transitions={"preempted": "SHUTDOWN",
                        "laptop_battery_low": "SHUTDOWN",
                        "robot_battery_low": "DOCK_AT_STATION"})
            smach.StateMachine.add("DOCK_AT_STATION", dock_sm,
                    transitions={"succeeded": "WAIT_UNTIL_CHARGED",
                        "failed": "SHUTDOWN"})
            smach.StateMachine.add("WAIT_UNTIL_CHARGED",
                    _WaitUntilChargedState(battery_monitor),
                    transitions={"robot_battery_full": "SHUTDOWN",
                        "preempted": "SHUTDOWN"})
        else:
            smach.StateMachine.add("MONITORED_PATROL", monitored_patrol_sm,
                    transitions={"preempted": "SHUTDOWN",
                        "laptop_battery_low": "SHUTDOWN",
                        "robot_battery_low": "SHUTDOWN"})

        smach.StateMachine.add("SHUTDOWN", _ShutdownState(),
                transitions={"shutdown": "shutdown"})
    return top_sm


class SmachPatrol(object):
    def __init__(self, patrol_goals, docking_goal=None):
        self.__move_base_client = actionlib.SimpleActionClient("move_base",
                MoveBaseAction)
        self.__top_sm = _createTopLevelSM(patrol_goals, docking_goal)

        self.__move_base_client.wait_for_server()
        rospy.on_shutdown(self.__stopRobot)
            # I just haven't found a better way to stop the robot on ^C.


    def __stopRobot(self):
        rospy.loginfo("Stopping robot.")
        self.__move_base_client.cancel_all_goals()


    def run(self):
        # We use multi-threading to preempt the state-machine on ^C.
        # I'm not sure if this is really useful.
        introspection_srv = smach_ros.IntrospectionServer(
                "patrol_introspection_server", self.__top_sm, "/PATROL_SM")
        introspection_srv.start()

        sm_thread = threading.Thread(target=self.__top_sm.execute)
        sm_thread.start()
        rospy.spin()
        self.__top_sm.request_preempt()
        sm_thread.join()

        introspection_srv.stop() 


if __name__ == "__main__":
    print "This module is supposed to be imported."
