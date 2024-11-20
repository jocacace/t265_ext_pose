#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix

class OdometryTransformer:
    def __init__(self):
        rospy.init_node('odometry_transformer', anonymous=True)

        # Transformation matrix (4x4) to apply
        self.rotation_z_neg_90 = np.array([
            [0,  1, 0, 0],  
            [-1, 0, 0, 0],  
            [0,  0, 1, 0],  
            [0, 0, 0,  1]
        ])
        self.rotation_x_180 = np.array([
            [1,  0,  0, 0],
            [0, -1,  0, 0],
            [0,  0, -1, 0],
            [0,  0,  0, 1]
        ])
        
        self.transformation_matrix = np.dot(self.rotation_x_180, self.rotation_z_neg_90)
        # Subscriber and Publisher
        rospy.Subscriber('/camera/odom/sample', Odometry, self.odom_callback)
        self.pose_pub = rospy.Publisher('/transformed_pose', PoseStamped, queue_size=10)

        rospy.loginfo("Node initialized. Waiting for Odometry messages...")

    def odom_callback(self, msg):
        # Extract position
        position = np.array([msg.pose.pose.position.x, 
                             msg.pose.pose.position.y, 
                             msg.pose.pose.position.z])
        
        # Extract orientation (quaternion)
        orientation = [msg.pose.pose.orientation.x, 
                       msg.pose.pose.orientation.y, 
                       msg.pose.pose.orientation.z, 
                       msg.pose.pose.orientation.w]

        # Create homogeneous transformation matrix from Odometry pose
        pose_matrix = quaternion_matrix(orientation)
        pose_matrix[:3, 3] = position

        #rospy.loginfo("Original Pose Matrix:\n{}".format(pose_matrix))

        # Apply transformation
        #transformed_matrix = np.dot(self.transformation_matrix, pose_matrix)
        transformed_matrix = np.dot(self.transformation_matrix, pose_matrix)
        
        #rospy.loginfo("Transformed Matrix:\n{}".format(transformed_matrix))

        # Extract transformed position and orientation
        transformed_position = transformed_matrix[:3, 3]
        transformed_orientation = quaternion_from_matrix(transformed_matrix)

        # Publish transformed pose
        '''
        transformed_pose = PoseStamped()
        transformed_pose.header = msg.header  # Use the same header (timestamp, frame_id)
        transformed_pose.pose.position.x = transformed_position[0]
        transformed_pose.pose.position.y = transformed_position[1]
        transformed_pose.pose.position.z = transformed_position[2]
        transformed_pose.pose.orientation.x = transformed_orientation[0]
        transformed_pose.pose.orientation.y = transformed_orientation[1]
        transformed_pose.pose.orientation.z = transformed_orientation[2]
        transformed_pose.pose.orientation.w = transformed_orientation[3]
        '''
        print("Original x: ", msg.pose.pose.position.x, " / ", transformed_position[0])
        print("Original y: ", msg.pose.pose.position.y, " / ", transformed_position[1])
        print("Original z: ", msg.pose.pose.position.z, " / ", transformed_position[2])
        print("\n")
        #self.pose_pub.publish(transformed_pose)
        #rospy.loginfo("Published transformed PoseStamped message.")

if __name__ == '__main__':
    try:
        OdometryTransformer()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node terminated.")

