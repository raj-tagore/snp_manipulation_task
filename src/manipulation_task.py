#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


if __name__ == '__main__':
    
    # Initialize the moveit_commander and rospy
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "panda_arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )
    
    # add a box to the scene
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = robot.get_planning_frame()
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.x = 0.4
    box_pose.pose.position.y = 0.1
    box_pose.pose.position.z = 0.4
    
    # size of the box is analogous to the observed size of the object
    scene.add_box("box", box_pose, size=(0.15, 0.075, 0.15))
    rospy.sleep(2)
    rospy.loginfo("Box added to the scene")
    
    # move the robot to the object
    rospy.loginfo("Move to the object")
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = -1.0
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    
    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    
    grasping_group = "panda_hand"
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(move_group.get_end_effector_link(), "box", touch_links=touch_links)
    
    rospy.loginfo("Carry Object to other pose")
    
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.x = -1.0
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = -0.1
    pose_goal.position.z = 0.4
    
    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)
    move_group.stop()
    move_group.clear_pose_targets()
    
    rospy.loginfo("Release Object")
    scene.remove_attached_object(move_group.get_end_effector_link(), name="box")
    
