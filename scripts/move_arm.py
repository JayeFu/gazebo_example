#! /usr/bin/env python

from __future__ import print_function
import rospy

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the joint trajectory control action, including the
# goal message and the result message.
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
# import actionlib_tutorials.msg

def joint_trajectory_client():
    # Creates the SimpleActionClient, passing the type of the action
    # (FollowJointTrajectoryAction) to the constructor.
    client = actionlib.SimpleActionClient('/mbx/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

    # Waits until the action server has started up and started
    # listening for goals.
    rospy.loginfo("start waiting for the action server")
    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = FollowJointTrajectoryGoal()

    # Set the parameters of the goal
    goal.trajectory.joint_names = ["/mbx/shoulder_pan_joint", "/mbx/shoulder_lift_joint", "/mbx/elbow_joint", \
        "/mbx/wrist_1_joint", "/mbx/wrist_2_joint", "/mbx/wrist_3_joint"]
    trajPt = JointTrajectoryPoint()
    trajPt.positions = [0.0, -0.8, 0.0, 0.0, 0.0, 0.0]
    trajPt.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    trajPt.time_from_start = rospy.Duration(secs=2.0)
    goal.trajectory.points.append(trajPt)

    goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)

    # Sends the goal to the action server.
    client.send_goal(goal)

    # Waits for the server to finish performing the action.
    rospy.loginfo("start waiting for the result")
    client.wait_for_result()

    # Prints out the result of executing the action
    return client.get_result()  # A JointTrajectoryResult

if __name__ == '__main__':
    try:
        SUCCESS_CODE = 0
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.
        rospy.init_node('fibonacci_client_py')
        result = joint_trajectory_client()
        # print("Result:", result.error_code)
        if result.error_code == SUCCESS_CODE:
            rospy.loginfo("Successfully move the joint")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
