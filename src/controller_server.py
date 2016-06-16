#!/usr/bin/env python

'''
Created on 12 Mar 2013

@author: nathan
'''
import sys
import math
from threading import Thread

import actionlib
import rospy

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from p2os_msgs.msg import MotorState
from PyKDL import Rotation
from trajectory_msgs.msg import JointTrajectoryPoint

from sf_controller_msgs.msg import SunflowerAction, SunflowerFeedback, SunflowerResult
from sf_lights_msgs.msg import LightsAction, LightsGoal


class SunflowerController(object):

    def __init__(self, topic, joints):
        self._as = actionlib.ActionServer(
            topic,
            SunflowerAction,
            goal_cb=self._goalCB,
            cancel_cb=self._cancelCB,
            auto_start=False)
        self._as.start()

        rospy.loginfo("Started Sunflower Controller on %s", topic)
        global ros_param_lock
        with ros_param_lock:
            joint_config = rospy.get_param(joints, {})

        if 'base' in joint_config:
            # TODO: These should be in a config file
            self._maxTrans = joint_config['base'].get('max_translation', 1.0)
            self._linear_rate = joint_config['base'].get('linear_rate', 0.1)
            self._maxRot = joint_config['base'].get('max_rotation', math.pi)
            self._rotation_rate = joint_config['base'].get('rotation_rate', 0.1)
        else:
            rospy.logerr("Limits for base movement not specified, exiting")
            exit(1)

        self._goals = {}
        self._joints = joints
        self._jointsControllers, self._jointUpdates = self._connectToJoints(joint_config)
        self._cmdVel, self._motorState = self._connectToWheels()
        self._actions = {
            'init': self.init,
            'move': self.move,
            'park': self.park,
            'stop': self.stop}

    def __del__(self):
        map(lambda h: h.unregister(), self._joints)
        map(lambda h: h.unregister(), self._jointUpdates)

    def run(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass

    def _connectToWheels(self):
        return (rospy.Publisher('cmd_vel', Twist),
                rospy.Publisher('cmd_motor_state', MotorState))

    def _connectToJoints(self, joints):
        controllers = {}
        state_subscribers = {}
        rospy.loginfo("Connecting to controllers...")
        for name, config in joints.iteritems():
            controller = config.get('controller', None)
            typeName = config.get('type', None)
            if controller and typeName:
                if typeName == 'MoveBaseAction':
                    ActionClass = MoveBaseAction
                elif typeName == 'FollowJointTrajectoryAction':
                    ActionClass = FollowJointTrajectoryAction
                elif typeName == 'LightsAction':
                    ActionClass = LightsAction
                else:
                    rospy.logwarn("Unknown action type: %s" % (typeName, ))
                a = ActionClass()
                ActionFeedback = type(a.action_feedback)
                controllers[name] = actionlib.SimpleActionClient(controller, ActionClass)
                rospy.loginfo("Waiting on %s...", controller)
                controllers[name].wait_for_server()
                rospy.loginfo("Connected to %s!", controller)
                state_subscribers[name] = rospy.Subscriber('%s/status' % (controller, ),
                                                           ActionFeedback,
                                                           callback=self._updateJointState,
                                                           callback_args=(topic, ),
                                                           queue_size=2)

        return controllers, state_subscribers

    def _updateJointState(self, msg):
        pass

    def _cancelCB(self, goalHandle):
        def statusCB(status, msg='', *args, **kwargs):
            feedback = SunflowerFeedback()
            feedback.status = status
            feedback.msg = msg if type(msg) == str else ''
            goalHandle.publish_feedback(feedback)

        def doneCB(result, msg=""):
            res = SunflowerResult(result, msg)
            goalHandle.set_canceled(res)

        self.stop(goalHandle, statusCB, doneCB)

    def _goalCB(self, goalHandle):
        goalHandle.set_accepted()
        goal = goalHandle.get_goal()

        def updateStatus(status, msg='', *args, **kwargs):
            rospy.loginfo("Status: %s", msg)
            feedback = SunflowerFeedback()
            feedback.status = status
            feedback.msg = msg if type(msg) == str else ''
            goalHandle.publish_feedback(feedback)

        def completed(result, msg=''):
            status = goalHandle.get_goal_status().status
            updateStatus(status, msg)

            if status == actionlib.GoalStatus.ACTIVE:
                goalHandle.set_succeeded(result)
            elif status == actionlib.GoalStatus.PREEMPTED:
                goalHandle.set_preempted(result)
            else:
                goalHandle.set_canceled(result)

        if goal.action in self._actions:
            updateStatus(goalHandle.get_goal_status().status, "Starting action %s" % (goal.action, ))
            Thread(target=self._actions[goal.action], args=(goalHandle, updateStatus, completed)).start()
        else:
            rospy.logwarn("Unknown action %s", goal.action)
            goalHandle.set_rejected()

    def park(self, goalHandle, statusCB, doneCB):
        pass

    def init(self, goalHandle, statusCB, doneCB):
        name = goalHandle.get_goal().component
        if name[:4] == 'base':
            statusCB(actionlib.GoalStatus.ACTIVE, 'Initialising base')
            self._motorState.publish(MotorState(1))

        doneCB()

    def stop(self, goalHandle, statusCB, doneCB):
        name = goalHandle.get_goal().component
        if name in self._jointsControllers:
            rospy.loginfo("%s: Stopping %s", rospy.get_name(), name)
            client = self._jointsControllers[name]
            client.cancel_all_calls()
        else:
            goalHandle.set_cancel_requested()

        doneCB(client.get_result() if client else None)

    def setlight(self, goalHandle, statusCB, doneCB):
        goal = goalHandle.get_goal()
        client = self._jointControllers.get('light', None)
        if client:
            client.wait_for_server()

            lightGoal = LightsGoal(rgb=goal.jointPositions)
            client.send_goal(lightGoal, done_cb=doneCB)
        else:
            goalHandle.set_rejected()

    def move(self, goalHandle, statusCB, doneCB):
        goal = goalHandle.get_goal()
        joints = goal.jointPositions

        def done(result=None, msg=None):
            rospy.logdebug("%s: '%s to %s' Result:%s",
                           rospy.get_name(),
                           goal.component,
                           goal.namedPosition or joints,
                           result)
            doneCB(result, msg)

        if goal.namedPosition:
            global ros_param_lock
            with ros_param_lock:
                goal.jointPositions = rospy.get_param('%s/%s/positions/%s' % (self._joints, goal.component, goal.namedPosition), [None])[0]

        rospy.loginfo("%s: Setting %s to %s",
                      rospy.get_name(),
                      goal.component,
                      goal.namedPosition or joints)

        if goal.component == 'base':
            self.navigate(goalHandle, statusCB, done)
        elif goal.component == 'base_direct':
            self.moveBase(goalHandle, statusCB, done)
        elif goal.component == 'light':
            self.setlight(goalHandle, statusCB, done)
        else:
            self.moveJoints(goalHandle, statusCB, done)

    def moveBase(self, goalHandle, statusCB, doneCB):
        goal = goalHandle.get_goal()
        positions = goal.jointPositions
        rotation = positions[0]
        linear = positions[1]

        rospy.loginfo("Moving base %s rad and %s meters", rotation, linear)

        # step 0: check validity of parameters:
        if not isinstance(rotation, (int, float)) or not isinstance(linear, (int, float)):
            rospy.logerr("Non-numeric rotation list, aborting moveBase")
            doneCB()
        if abs(linear) >= self._maxTrans:
            rospy.logerr(
                "Maximal relative translation step exceeded(max: %sm), aborting moveBase" % self._maxTrans)
            doneCB()
        if abs(rotation) >= self._maxRot:
            rospy.logerr(
                "Maximal relative rotation step exceeded(max: %srad), aborting moveBase" % self._maxRot)
            doneCB()

        # step 1: determine duration of motion so that upper thresholds for
        # both translational as well as rotational velocity are not exceeded
        duration_trans_sec = abs(linear) / self._linear_rate
        duration_rot_sec = abs(rotation) / self._rotation_rate
        duration_sec = max(duration_trans_sec, duration_rot_sec)

        # step 2: determine actual velocities based on calculated duration
        x_vel = linear / duration_sec
        rot_vel = rotation / duration_sec

        # step 3: send constant velocity command to base_controller for the calculated duration of motion
        # pub = rospy.Publisher('cmd_vel', Twist)
        twist = Twist()
        twist.linear.x = x_vel
        twist.angular.z = rot_vel
        # duration of motion in ROS time
        duration_ros = rospy.Duration.from_sec(duration_sec)

        rate = rospy.Rate(50)
        end_time = rospy.Time.now() + duration_ros
        self._cmdVel.publish(twist)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            # pub.publish(twist)
            # p2os has issues if you republish,
            # but seems to continue using last received cmd_vel
            if goalHandle.get_goal_status() != actionlib.GoalStatus.ACTIVE:
                rospy.loginfo('%s: Preempted move' % rospy.get_name())
                goalHandle.set_preempted()
                break
            rate.sleep()

        self._cmdVel.publish(Twist())  # send a stop command, see above comment
        doneCB()

    def navigate(self, goalHandle, statusCB, doneCB):
        positions = goalHandle.get_goal().jointPositions
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = positions[0]
        pose.pose.position.y = positions[1]
        pose.pose.position.z = 0.0
        q = Rotation.RotZ(positions[2]).GetQuaternion()
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        client_goal = MoveBaseGoal()
        client_goal.target_pose = pose

        client = self._jointControllers.get('base', None)
        if client:
            client.wait_for_server()
            rospy.loginfo("%s: Navigating to %s",
                          rospy.get_name(),
                          pose)

            client.send_goal(client_goal, done_cb=doneCB)
        else:
            goalHandle.set_rejected()

    def moveJoints(self, goalHandle, statusCB, doneCB):
        name = goalHandle.get_goal().component
        positions = goalHandle.get_goal().jointPositions

        controller = self._jointControllers.get(name, None)
        if controller:
            with ros_param_lock:
                joint_names = rospy.get_param('%s/%s/joint_names' % (self._joints, name), [name, ])
            goal = FollowJointTrajectoryGoal()
            goal.trajectory.joint_names = joint_names
            point = JointTrajectoryPoint()
            point.positions = positions
            point.time_from_start = rospy.Duration(3)
            goal.trajectory.points.append(point)
            controller.send_goal(goal, doneCB)
        else:
            rospy.logerr('Undefined joint: %s', name)
            goalHandle.set_rejected('Undefined joint: %s', name)

        # TODO: How to detect done?

if __name__ == '__main__':
    rospy.init_node('sf_controller')

    global ros_param_lock
    with ros_param_lock:
        if len(sys.argv) == 1:
            rospy.set_param('~joints', 'sunflower1_1/joints')

        try:
            topic = rospy.get_param('~topic')
            joints = rospy.get_param('~joints')
        except KeyError as e:
            rospy.logfatal("sf_controller Missing param: %s" % (e.message,))
            exit(0)

    SunflowerController(topic, joints)

    rospy.spin()
