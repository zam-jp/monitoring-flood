#!/usr/bin/env python3
'''
@version: 4.0
@author: Shaiful Nizam
@since: May 23, 2022
@organization: RoboLab (Ritsumeikan University, College of Information Science and Engineering)
@note: This version will use 16 drones.
'''

import rospy
import os 
import math
from std_msgs.msg import String, Int32
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Vector3
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import configparser
import sys
import tempfile
import csv
import tf
import message_filters
import cv2, cv_bridge

config = configparser.ConfigParser()
config.read('config.ini')

class Follower:
    def __init__(self):
        
        self.bridge = cv_bridge.CvBridge()
        rospy.Subscriber('monitor/downward_cam/camera/image', Image, self.image_callback)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        self.cmd_vel_pub = rospy.Publisher('monitor/cmd_vel',Twist, queue_size=10)
        self.twist = Twist()
        self.file_counter = 1  # Counter for naming of saved image file.
        self.frame_counter = 5
     
    def image_callback(self, msg):

        # Get image from drone camera.        
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8') #bgr8
        # Slicing the image to remove vertical red lines
        image = image[:, 30:image.shape[1]-30, :]        
                
        # Convert BGR to HSV color format.
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
          
        #IMPORTANT NOTE: Color is in HSV format
        lower_boundary = np.array([int(config['color_range']['lower_H']),
                                   int(config['color_range']['lower_S']),
                                   int(config['color_range']['lower_V'])])
        upper_boundary = np.array([int(config['color_range']['upper_H']),
                                   int(config['color_range']['upper_S']),
                                   int(config['color_range']['upper_V'])])
        # print("[drone_camera] Lower boundary:", lower_boundary)
        # print("[drone_camera] Upper boundary:", upper_boundary)
        
        # Color range for drones
        lower_red = np.array([0,96,97])
        upper_red = np.array([0,255,255])

        # cv2.inRange is used to render an image in white and black.
        # All the pixels that fall inside the interval [lower, upper] will be white
        # All the pixels that do not fall inside the interval will black, for all three channels:
        mask_flood = cv2.inRange(hsv_image, lower_boundary, upper_boundary)
        
        # All the pixels that are white in the mask will survive the AND operation, 
        # all the black pixels will remain black
        masked_image = cv2.bitwise_and(image, image, mask=mask_flood)        

        mask_drone = cv2.inRange(hsv_image, lower_red, upper_red)   # Needed for drawing contour of drone.
        
        # Find contours for the water.
        flood_contours, hierarchy = cv2.findContours(mask_flood, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Find contours for the drones.
        drone_contours, hierarchy = cv2.findContours(mask_drone, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Drawing the contours
        cv2.drawContours(image, flood_contours, -1, (0, 0, 255), 1)     # Flood outline in red.
        cv2.drawContours(image, drone_contours, -1, (0, 255, 255), 8)   # Drone outline in yellow.
            
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE) # Rotate image clockwise by 90deg
        #===========================================================================================================
        # Save image to file. PLEASE COMMENT OUT WHEN NOT NEEDED.
        #===========================================================================================================
        # if (self.frame_counter == 5):
        #     if self.file_counter < 10:
        #         cv2.imwrite('images/monitor/FIG_000' + str(self.file_counter) + '.png', image)
        #     elif self.file_counter >= 10 and self.file_counter < 100:
        #         cv2.imwrite('images/monitor/FIG_00' + str(self.file_counter) + '.png', image)
        #     elif self.file_counter >= 100 and self.file_counter < 1000:
        #         cv2.imwrite('images/monitor/FIG_0' + str(self.file_counter) + '.png', image)
        #     else:
        #         cv2.imwrite('images/monitor/FIG_' + str(self.file_counter) + '.png', image)
        #     self.frame_counter = 0            
        #     self.file_counter += 1
        # self.frame_counter += 1
        #===========================================================================================================

        # Display drone camera image
        cv2.imshow("monitor", image)
        cv2.waitKey(3)
    

    def callback(self, data):
        drone_monitor = data.pose[int(config['g_index']['monitor'])]
        ceiling_height = float(config['position']['ceiling_height'])
        drone_monitor_posX = drone_monitor.position.x
        drone_monitor_posY = drone_monitor.position.y
        drone_monitor_posZ = drone_monitor.position.z

        v_vel_scale = float(config['velocity']['v_vel_scale'])       # The velocity in the vertical plane
        drone_height = float(config['position']['monitor_height'])
        if drone_monitor_posZ < ceiling_height:
            # Move the drone straight up vertically when drone is not yet at ceiling height.
            calc_odom = (Vector3(0,0,v_vel_scale*(drone_monitor_posZ-drone_height)))
        else:
            # When drone is at ceiling height then move it to the center of the map.
            posX = 0
            posY = 0
            h_vel_scale = float(config['velocity']['h_vel_scale'])       # The velocity in the horizontal plane
            calc_odom = (Vector3(h_vel_scale*(drone_monitor_posX-posX), 
                                 h_vel_scale*(drone_monitor_posY-posY), 
                                 v_vel_scale*(drone_monitor_posZ-drone_height)))        
        pub = self.cmd_vel_pub
        pub.publish(Twist(calc_odom, Vector3(0,0,0)))
        rospy.Rate(1).sleep

def listener():
    rospy.init_node('monitor')
    follower = Follower()
    rospy.spin()

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
# END ALL
