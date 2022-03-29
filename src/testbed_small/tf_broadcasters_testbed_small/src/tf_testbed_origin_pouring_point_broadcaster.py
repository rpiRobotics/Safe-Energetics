#!/usr/bin/env python  

"""
Author: Burak Aksoy
Node: tf_testbed_origin_pouring_point_broadcaster
Description:
    Broadcasts to tf2 the trasformation between the testbed_origin frame
    and pouring point

Parameters:
    - TODO
Publishes to:
    - NONE
Broadcasts to:
    - tf2
"""

import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg
# import kinova_msgs.msg
import math
import numpy as np

class TestbedOrigin2Pouring_point_Tf():
    def __init__(self):
        rospy.init_node('tf_testbed_origin_pouring_point_broadcaster', anonymous=True)

        self.tf_parent_frame_id = rospy.get_param("~tf_parent_frame_id", "testbed_origin")
        self.tf_child_frame_id = rospy.get_param("~tf_child_frame_id", "pouring_point")
    
        self.plate_tip_position = rospy.get_param("~plate_tip_position") 
        self.plate_orientation = rospy.get_param("~plate_orientation") 

        self.pouring_offset_position = rospy.get_param("~pouring_offset_position") 

        self.plate_angles = rospy.get_param("~plate_angles") 
        self.plate_tip_heights = rospy.get_param("~plate_tip_heights") 

        self.desired_pouring_height = rospy.get_param("~desired_pouring_height") 
        self.plate_selection_index = rospy.get_param("~plate_selection_index") 

        self.position = [None,None,None]

        self.calculate_position()

        # self.tf_broadcaster = tf2_ros.TransformBroadcaster() # Create a tf broadcaster
        self.tf_broadcaster_static = tf2_ros.StaticTransformBroadcaster() # Create a static tf broadcster for rbg camera and the world frame
        self.publish_tf_static()

    def calculate_position(self):
        self.position[0] = self.plate_tip_position['x'] + self.pouring_offset_position['x']
        self.position[1] = self.plate_tip_position['y'] + self.pouring_offset_position['y']

        offset = ((self.pouring_offset_position['x'])**2 + (self.pouring_offset_position['y'])**2)**0.5
        theta = np.deg2rad(self.plate_angles[self.plate_selection_index])
        h_plate = self.plate_tip_heights[self.plate_selection_index]
        self.position[2] = h_plate - offset*math.tan(theta) + self.desired_pouring_height

    def publish_tf_static(self):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()

        t.header.frame_id = self.tf_parent_frame_id
        t.child_frame_id = self.tf_child_frame_id

        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]
        
        t.transform.rotation.x = self.plate_orientation['x']
        t.transform.rotation.y = self.plate_orientation['y']
        t.transform.rotation.z = self.plate_orientation['z']
        t.transform.rotation.w = self.plate_orientation['w']

        self.tf_broadcaster_static.sendTransform(t)

if __name__ == '__main__':
    testbedOrigin2Pouring_point_Tf = TestbedOrigin2Pouring_point_Tf()
    rospy.spin()