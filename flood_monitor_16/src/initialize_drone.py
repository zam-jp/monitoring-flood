#!/usr/bin/env python3
'''
@version: 4.0
@author: Shaiful Nizam
@since: May 23, 2022
@organization: RoboLab (Ritsumeikan University, College of Information Science and Engineering)
@note: This version will use 16 drones.
'''

import rospy

from gazebo_msgs.srv import GetModelState, GetModelStateRequest
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import sys

# import math
# import time
# from scipy.integrate import odeint
# import matplotlib.pyplot as plt
# from matplotlib.pyplot import MultipleLocator
# import numpy as np
# from shapely.geometry import Polygon
# from shapely.geometry import Point
# from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
# from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area
# import mpl_scatter_density
# from astropy.visualization import LogStretch
# from astropy.visualization.mpl_normalize import ImageNormalize
# from PIL import Image
import configparser

config = configparser.ConfigParser()
config.read('config.ini')
boundary_limit = float(config['distance']['aoi_boundary'])

def get_pose(drone_name):
    """
    Get the position of the drone.
    
    Args:
        drone_name - The name of the drone the user wants to control.
    Returns:
        response.pose - (position: {x, y, z}, velocity: {x, y, z}). Extract the position and the
                        velocity of the robot.
    """
    sets = GetModelStateRequest()
    sets.model_name = drone_name
    response = call(sets)
    return response.pose


class Initialize:
    def __init__(self):        
        #Rospy's publisher to /controller that has message of type Float64MultiArray
        self.pub = rospy.Publisher('/controller', Float64MultiArray, queue_size=10)
        
    def update(self, init_pos):
        """
        Updates initial position of drones by publishing to ROS Topics with message name 'controller'.

        Args:
            init_pos - Initial position of drones.
        Returns:
            None
        """
        data2send = Float64MultiArray(data = init_pos)
        self.pub.publish(data2send)
        


if __name__ == '__main__':

    rospy.init_node('view_node')

    call = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    rate = rospy.Rate(1)
    
    # Coordinate system for 16 drones. Drones are arrange in a 4x4 formation.
    p = boundary_limit/4
    
    # These positions are the drones's distribution coordinates.
    final_pos = [-3*p, 3*p, -p, 3*p, p, 3*p, 3*p, 3*p, 
                -3*p, p, -p, p, p, p, 3*p, p, 
                -3*p, -p, -p, -p, p, -p, 3*p, -p, 
                -3*p, -3*p, -p, -3*p, p, -3*p, 3*p, -3*p]

    num_of_drones = 16
    max_count = num_of_drones * 3
    init = Initialize()
    count =0
    # To give enough time for the drones to receive their initial position as they are called from flood_monitor.sh
    while count < max_count:
        init.update(final_pos)
        rate.sleep()
        count = count + 1
        
