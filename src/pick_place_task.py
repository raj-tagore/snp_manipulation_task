#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_trajectory_command(positions):
    pub = rospy.Publisher('/pos_joint_traj_controller/command', JointTrajectory, queue_size=10)
    
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
    point = JointTrajectoryPoint()
    point.positions = positions  # Example positions
    point.time_from_start = rospy.Duration(2.0)  # Move to the target positions in 2 seconds
    
    trajectory_msg.points.append(point)
    
    rospy.sleep(1)
    
    rospy.loginfo("Sending trajectory command...")
    pub.publish(trajectory_msg)
    
def send_gripper_command(position):
    pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)
    
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = ['robotiq_85_left_knuckle_joint']
    
    point = JointTrajectoryPoint()
    # Close the gripper
    point.positions = [position]
    point.time_from_start = rospy.Duration(2.0)  # Close the gripper in 2 seconds
    
    trajectory_msg.points.append(point)
    
    rospy.sleep(1)
    
    rospy.loginfo("Sending gripper command...")
    pub.publish(trajectory_msg)

if __name__ == '__main__':
    rospy.init_node('send_trajectory_command', anonymous=True)
    try:
        pre_grasp_position = [0.0, 0.5, 1.5, -2.0, 0, 0.0]
        send_trajectory_command(pre_grasp_position)
        rospy.sleep(5)
        
        gripper_open = 0.0
        send_gripper_command(gripper_open)
        rospy.sleep(5)
        
        grasp_pos = [0.0, 0.6, 1.6, -2.2, 0, 0.0]
        send_trajectory_command(grasp_pos)
        rospy.sleep(5)
        
        gripper_close = 0.3
        send_gripper_command(gripper_close)
        rospy.sleep(5)
        
        send_trajectory_command(pre_grasp_position)
        rospy.sleep(5)
        
        post_grasp_position = [0.1, 0.5, 1.5, -2.0, 0, 0.0]
        send_trajectory_command(post_grasp_position)
        rospy.sleep(5)
        
        place_position = [-0.5, 0.6, 1.6, -2.2, 0, 0.0]
        send_trajectory_command(place_position)
        rospy.sleep(5)
        
        send_gripper_command(gripper_open)
        rospy.sleep(5)
        
        send_trajectory_command(grasp_pos)
        rospy.sleep(5)
        
        
        
    except rospy.ROSInterruptException:
        pass
