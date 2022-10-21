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
from std_msgs.msg import Float64MultiArray, Int32, Float32MultiArray
from gazebo_msgs.msg import ModelStates

import math
import matplotlib.pyplot as plt
import numpy as np
import mpl_scatter_density
import csv
import atexit
import configparser
from scipy.integrate import odeint
from shapely.geometry import Polygon
from shapely.geometry import Point
from geovoronoi import coords_to_points, points_to_coords, voronoi_regions_from_coords
from geovoronoi.plotting import plot_voronoi_polys_with_points_in_area
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

drone_pos_list = []

config = configparser.ConfigParser()
config.read('config.ini')
boundary_limit = float(config['distance']['aoi_boundary'])
num_of_drones = 16

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
    response = service_call(sets)
    return response.pose

class Voronoi:
    def __init__(self):
        """
        Initialize the variables from drone01 to drone09.
        The drone variables is the status of that drone that see water [1] or not [0] - default value = 0.
        Including the following variables:
            pub - (Publisher) Rospy's publisher to topic "/controller" that has message of Float64MultiArray
            pub_centroid - (Publisher) Rospy's publisher to topic "/centroid" that has message of Float64MultiArray
            boundary_limits - (list of set) Give the vertical and horizontal limits for the voronoi diagram drawn (i.e dimension)
            targets - (list) Give the centroid of the density plot drawn on the right
            weights - (list) Give the weight for each centroid for the area of importance
            standard_deviation - (int) Give a standard_deviation for the random normal distribution
            numbers - (int) Size of the random variable for the plot.
            hasNotUpdated - (bool) for updating the flood status.
            
        Args:
            None
        Returns:
            None
        """
        self.pub = rospy.Publisher('/controller', Float64MultiArray, queue_size=10)
        self.pub_centroid = rospy.Publisher('/centroid', Float64MultiArray, queue_size=10)
        
        rospy.Subscriber('drone01/flood_status', Int32, self.get_drone01_flood_status)
        rospy.Subscriber('drone02/flood_status', Int32, self.get_drone02_flood_status)
        rospy.Subscriber('drone03/flood_status', Int32, self.get_drone03_flood_status)
        rospy.Subscriber('drone04/flood_status', Int32, self.get_drone04_flood_status)
        rospy.Subscriber('drone05/flood_status', Int32, self.get_drone05_flood_status)
        rospy.Subscriber('drone06/flood_status', Int32, self.get_drone06_flood_status)
        rospy.Subscriber('drone07/flood_status', Int32, self.get_drone07_flood_status)
        rospy.Subscriber('drone08/flood_status', Int32, self.get_drone08_flood_status)
        rospy.Subscriber('drone09/flood_status', Int32, self.get_drone09_flood_status)
        rospy.Subscriber('drone10/flood_status', Int32, self.get_drone10_flood_status)
        rospy.Subscriber('drone11/flood_status', Int32, self.get_drone11_flood_status)
        rospy.Subscriber('drone12/flood_status', Int32, self.get_drone12_flood_status)
        rospy.Subscriber('drone13/flood_status', Int32, self.get_drone13_flood_status)
        rospy.Subscriber('drone14/flood_status', Int32, self.get_drone14_flood_status)
        rospy.Subscriber('drone15/flood_status', Int32, self.get_drone15_flood_status)
        rospy.Subscriber('drone16/flood_status', Int32, self.get_drone16_flood_status)
        
        self.drone01 = 0
        self.drone02 = 0
        self.drone03 = 0
        self.drone04 = 0
        self.drone05 = 0
        self.drone06 = 0
        self.drone07 = 0
        self.drone08 = 0
        self.drone09 = 0
        self.drone10 = 0
        self.drone11 = 0
        self.drone12 = 0
        self.drone13 = 0
        self.drone14 = 0
        self.drone15 = 0
        self.drone16 = 0

        rospy.Subscriber('drone01/moments', Float32MultiArray, self.drone01_moments_callback) 
        rospy.Subscriber('drone02/moments', Float32MultiArray, self.drone02_moments_callback) 
        rospy.Subscriber('drone03/moments', Float32MultiArray, self.drone03_moments_callback) 
        rospy.Subscriber('drone04/moments', Float32MultiArray, self.drone04_moments_callback) 
        rospy.Subscriber('drone05/moments', Float32MultiArray, self.drone05_moments_callback) 
        rospy.Subscriber('drone06/moments', Float32MultiArray, self.drone06_moments_callback) 
        rospy.Subscriber('drone07/moments', Float32MultiArray, self.drone07_moments_callback) 
        rospy.Subscriber('drone08/moments', Float32MultiArray, self.drone08_moments_callback) 
        rospy.Subscriber('drone09/moments', Float32MultiArray, self.drone09_moments_callback) 
        rospy.Subscriber('drone10/moments', Float32MultiArray, self.drone10_moments_callback) 
        rospy.Subscriber('drone11/moments', Float32MultiArray, self.drone11_moments_callback) 
        rospy.Subscriber('drone12/moments', Float32MultiArray, self.drone12_moments_callback) 
        rospy.Subscriber('drone13/moments', Float32MultiArray, self.drone13_moments_callback) 
        rospy.Subscriber('drone14/moments', Float32MultiArray, self.drone14_moments_callback) 
        rospy.Subscriber('drone15/moments', Float32MultiArray, self.drone15_moments_callback) 
        rospy.Subscriber('drone16/moments', Float32MultiArray, self.drone16_moments_callback) 

        self.drone01_moments = []
        self.drone02_moments = []
        self.drone03_moments = []
        self.drone04_moments = []
        self.drone05_moments = []
        self.drone06_moments = []
        self.drone07_moments = []
        self.drone08_moments = []
        self.drone09_moments = []
        self.drone10_moments = []
        self.drone11_moments = []
        self.drone12_moments = []
        self.drone13_moments = []
        self.drone14_moments = []
        self.drone15_moments = []
        self.drone16_moments = []
         
        # Four corners of the bounding box representing the area of interest.
        self.boundary_limits = [(-boundary_limit, boundary_limit), 
                                (-boundary_limit, -boundary_limit), 
                                (boundary_limit, -boundary_limit), 
                                (boundary_limit, boundary_limit)]
        self.weights = []
        self.targets = []   # target refers to drones that sees water. This list will store drone position.
        self.target_moments = []    # This list will store moments of the target.
        self.target_centroids = []  # This list will store the x-y coordinates of centroids of the target.
        self.standard_deviation = 7 # This will determine the effective radius of attraction between the drones.
        self.numbers = 1000
        self.hasNotUpdated = True 
        self.plot_initialization()
        self.count = 0
        self.drone_list = ['drone01', 'drone02', 'drone03', 'drone04', 
                           'drone05', 'drone06', 'drone07', 'drone08', 
                           'drone09', 'drone10', 'drone11', 'drone12',
                           'drone13', 'drone14', 'drone15', 'drone16']
   
    def drone01_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone01_moments = data.data
        
    def drone02_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone02_moments = data.data
        
    def drone03_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone03_moments = data.data
        
    def drone04_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone04_moments = data.data
        
    def drone05_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone05_moments = data.data
        
    def drone06_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone06_moments = data.data
        
    def drone07_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone07_moments = data.data
        
    def drone08_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone08_moments = data.data
        
    def drone09_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone09_moments = data.data
        
    def drone10_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone10_moments = data.data
        
    def drone11_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone11_moments = data.data
        
    def drone12_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone12_moments = data.data
        
    def drone13_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone13_moments = data.data
        
    def drone14_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone14_moments = data.data
        
    def drone15_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone15_moments = data.data
        
    def drone16_moments_callback(self, data):
        """
        Args:
            data - (Float32MultiArray) moments from OpenCV.
        """
        self.drone16_moments = data.data
        
    def get_drone01_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone01 = data.data
    
    def get_drone02_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone02 = data.data
    
    def get_drone03_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone03 = data.data
    
    def get_drone04_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone04 = data.data
        
    def get_drone05_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone05 = data.data
        
    def get_drone06_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone06 = data.data
    
    def get_drone07_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone07 = data.data
        
    def get_drone08_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone08 = data.data
        
    def get_drone09_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone09 = data.data
    
    def get_drone10_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone10 = data.data
    
    def get_drone11_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone11 = data.data
    
    def get_drone12_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone12 = data.data
    
    def get_drone13_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone13 = data.data
    
    def get_drone14_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone14 = data.data
    
    def get_drone15_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone15 = data.data
    
    def get_drone16_flood_status(self, data):
        """
        Args:
            data - (Int32) flood status.
        """
        self.drone16 = data.data
    
    def plot_initialization(self):
        """
        Initialize for both plot using the matplotlib.
        """      
        # Initialize the figure for matplotlib
        self.fig = plt.figure(figsize = [13, 5])
        self.fig.subplots_adjust(wspace = 0.3, hspace = 0, top = 0.9, bottom = 0.2)
        
        self.ax1 = self.fig.add_subplot(121, projection = 'scatter_density') # for plotting the Voronoi cells.
        self.ax2 = self.fig.add_subplot(122, projection = '3d') # For 3D plotting of density function.
        
        # Set equal scaling (i.e., make circles circular) by changing dimensions of the plot box. 
        self.ax1.axis('scaled')
        
    def update_status(self, drone_pos):
        """
        Update status of the flood.

        Args:
            drone_pos - (List) List of drone position.
        """
        
        # List of drone moments.
        drone_moments = [self.drone01_moments,
                         self.drone02_moments,
                         self.drone03_moments,
                         self.drone04_moments,
                         self.drone05_moments,
                         self.drone06_moments,
                         self.drone07_moments,
                         self.drone08_moments,
                         self.drone09_moments,
                         self.drone10_moments,
                         self.drone11_moments,
                         self.drone12_moments,
                         self.drone13_moments,
                         self.drone14_moments,
                         self.drone15_moments,
                         self.drone16_moments]
        
        # Array of flood status
        flood_state = np.array([self.drone01,
                                self.drone02,
                                self.drone03,
                                self.drone04,
                                self.drone05,
                                self.drone06,
                                self.drone07,
                                self.drone08,
                                self.drone09,
                                self.drone10,
                                self.drone11,
                                self.drone12,
                                self.drone13,
                                self.drone14,
                                self.drone15,
                                self.drone16], dtype=object)
        
        print("Flood state:", flood_state)
        print("hasNotUpdated:", self.hasNotUpdated)
        print()
        
        if np.all(flood_state == 0) and self.hasNotUpdated: # If all flood_state = 0. np.all will compare the condition to all elements in the list.
            # print("Do nothing...")
            pass
        elif 1 in flood_state and not np.all(flood_state == 1) and self.hasNotUpdated: # If flood_state contains 1, but not all flood_state = 1.
            self.weights = []
            self.targets = []
            self.target_moments = []
            numOfDroneThatSeesWater = 0
            for index, state in enumerate(flood_state):
                if state == 1:
                    self.targets.append(list(drone_pos[index]))
                    self.target_moments.append(drone_moments[index])
                    numOfDroneThatSeesWater += 1
            
            for i in range(numOfDroneThatSeesWater):
                self.weights.append([1/numOfDroneThatSeesWater])
                
            self.update(drone_pos, flood_state)

        elif np.all(flood_state == 1) or not self.hasNotUpdated: # If all flood_state = 1.
            if self.hasNotUpdated:
                self.weights = []
                self.targets = []
                self.target_moments = []
                for index, state in enumerate(flood_state):
                    if state == 1:
                        self.targets.append(list(drone_pos[index]))
                        self.target_moments.append(drone_moments[index])
                self.weights = [[1/num_of_drones] for i in range(num_of_drones)]
                self.hasNotUpdated = False 
                
            self.update(drone_pos, flood_state)
        
    def update(self, drone_pos, flood_state):
        """
        Updates: centroid coordinate, drone_pos_list, voronoi plot, density plot.
        Publish: new coordinates for drones, new centroid coordinates.

        Args:
            drone_pos - List containing all drone current positions, in x-y coordinate.
            flood_state - List of integers, 1's and 0's. 1 = drone detect flood, 0 = drone did not detect enough flood.
        """
        # print("drone_pos:", drone_pos)
        pos_list = []
        for i in range(len(drone_pos)):
            pos_list.append(drone_pos[i][0])
            pos_list.append(drone_pos[i][1])
        # print("pos_list:", pos_list)
        drone_pos_list.append(pos_list) # To be used for plotting trajectory path.
        # print("drone_pos_list", drone_pos_list)
                
        ################Initialization#############
        outer_pos = self.boundary_limits

        N = len(drone_pos)
        self.area_shape = Polygon(outer_pos)

        # centroid_poly = np.array(self.area_shape.centroid.xy).astype(np.float64)
        pts = [p for p in coords_to_points(drone_pos) if p.within(self.area_shape)]  # converts to shapely Point
        pts = self.compensate(pts, N)
        
        self.coords = points_to_coords(pts)   # convert back to simple NumPy coordinate array
        # self.poly_shapes, pts, self.poly_to_pt_assignments = voronoi_regions_from_coords(self.coords, self.area_shape, accept_n_coord_duplicates = 0)
        self.poly_shapes, pts, self.poly_to_pt_assignments = voronoi_regions_from_coords(self.coords, self.area_shape, return_unassigned_points=True)        
        self.x_unit, self.y_unit = self.sampling(self.targets, self.standard_deviation, self.numbers)
        
        ############ Pair Points and Centroids ######
        self.region_value = self.value_contribution(self.x_unit, self.y_unit, self.poly_shapes)
        self.poly_centroids = self.centroid_calculation(self.region_value, self.poly_shapes)
        self.new_centroids = self.match_pair(self.poly_shapes, list(self.coords), list(self.poly_centroids))
        
        new_coords = self.cal_tra(self.coords, self.new_centroids)        
        coords_to_points(new_coords)

        if self.hasNotUpdated:
            self.target_centroids = []
            for index, state in enumerate(flood_state):
                if state == 1:
                    self.target_centroids.append(self.new_centroids[index])

        ########### Visualize ##########
        
        self.plot_voronoi(self.ax1, outer_pos, self.poly_shapes, self.coords, new_coords)
        self.plot_multivariate_gaussian(self.ax2)
        
        ######################### PLEASE COMMENT WHEN NOT NEEDED ###########################
        ### Uncomment the following line to save plot as an image file. ###
        # if self.count < 10:
        #     plt.savefig('images/plot/FIG_000' + str(self.count) + '.png')
        # elif self.count >= 10 and self.count < 100:
        #     plt.savefig('images/plot/FIG_00' + str(self.count) + '.png')
        # elif self.count >= 100 and self.count < 1000:
        #     plt.savefig('images/plot/FIG_0' + str(self.count) + '.png')
        # else:
        #     plt.savefig('images/plot/FIG_' + str(self.count) + '.png')
        # self.count += 1
        ####################################################################################

        ########### Trajectory Calculation ##########

        new_coords = np.array(new_coords).reshape(-1)
        data2send = Float64MultiArray(data = new_coords)
        self.pub.publish(data2send)
        centroid2send = Float64MultiArray(data = self.new_centroids)
        self.pub_centroid.publish(centroid2send)
        

        plt.pause(0.000001)
        self.ax1.clear()
        self.ax2.clear()

    def match_pair(self, poly_shapes, coords, centroids):
        """
        Sort order of centroid list to find pair of each region and its centroid.
        Matching the pair of each drone to the centroids

        Args:
            poly_shapes - (Points) The polygon shapes of the boundary
            coords - (Points) The coordinates of each point in the boundary
            centroids - (Array) The centroids of each drone
        Returns:
            sorted_centroids - (Array) The sorted centroids from the coordinated
        """
        sorted_centroids = []
        points = coords_to_points(coords)
        for p in points:
            for j in range(len(poly_shapes)):
                if p.within(poly_shapes[j]): 
                    pair = centroids[j]
                    sorted_centroids.append(pair)
        return sorted_centroids

    def value_contribution(self, x_unit, y_unit, poly_shapes):
        """
        Give a value contribution to the x and y coord according to the poly shapes

        Args:
            x_unit
            y_unit 
            poly_shapes 
        Returns:
            region_value 
        """
        point_value = np.vstack((np.array(x_unit), np.array(y_unit))).T # make pairs of samples
        poly_nums = len(poly_shapes)
        region_value =[[] for _ in range(poly_nums)] #this stores coordinates of samples in a region
        for p in point_value:
            for j in range(poly_nums):
                point = Point(p) # convert sample coordinate to Point
                if point.within(poly_shapes[j]): # if the sample locates in the region polyshapes[j] then append this p to region_value[j]
                    region_value[j].append(p)
        return np.array(region_value, dtype=object)

    def centroid_calculation(self, region_value, poly_shapes):
        """
        Calculate centroids. 

        Args:
            region_value
            poly_shapes
        Returns:
            poly_centroids
        """
        sum_value = []
        for i in range(len(poly_shapes)):
            init = [0,0]
            for j in region_value[i]:
                init += j
            sum_value.append(init)
            
        poly_centroids = []
        for i in range(len(poly_shapes)):
            poly_size = len(region_value[i])
            if poly_size == 0: # If number of sample in a region is 0 then use its centroid as the next target
                poly_centroids.append(poly_shapes[i].centroid.coords[0])
            else: # next target is the CM of samples in a region
                poly_dense_x = sum_value[i][0]/poly_size
                poly_dense_y = sum_value[i][1]/poly_size
                poly_centroids.append([poly_dense_x,poly_dense_y])
        return poly_centroids
        
    def plot_voronoi(self, ax, outer, poly_shapes, coords, new_coords):
        """
        Graph on the left of the plot showing the Voronoi distribution among
        the drone.

        Args:
            ax - (plt) To initialize the axis plot for the matplotlib
            outer
            poly_shapes 
            coords
            new_coords 
        Returns:
            None
        """

        self.area_shape = Polygon(outer)
        plot_voronoi_polys_with_points_in_area(ax, self.area_shape, poly_shapes, coords,
                                               self.poly_to_pt_assignments, points_color='black', # plot centroids as black circles
                                               points_markersize=40, voronoi_and_points_cmap=None,
                                               plot_voronoi_opts={'alpha': 0.5},
                                               plot_points_opts={'zorder': 5}) # set zorder more than 1 to render the black circles above the red squares
        
        for centroid in self.new_centroids:
            c1 = centroid
            ax.plot(c1[0],c1[1], 'rs', markersize = 8) # new centroids as red square

        ax.set_xlim([-boundary_limit, boundary_limit]) # set range of x-axis
        ax.set_ylim([-boundary_limit, boundary_limit]) # set range of y-axis
        
        # set grid
        ax.tick_params(labelsize=8)
        ax.xaxis.set_ticks(np.arange(-boundary_limit, boundary_limit + 1, boundary_limit/3))
        ax.yaxis.set_ticks(np.arange(-boundary_limit, boundary_limit + 1, boundary_limit/3))
        
        font = {'size':10}
        ax.set_xlabel('x (m)', font) # set x-axis label
        ax.set_ylabel('y (m)', font) # set y-axis label

    def plot_multivariate_gaussian(self, ax):
        """
        This is to plot a multivariate Gaussian graph on the second sub-plot.

        Args:
            ax - matplotlib.pyplot subplot axis
        Returns:
            None
        """
        
        # We assume weight is uniform across all drones that see water.
        # Let the sum of weight for all drones that see water = 1,
        # Then, the distributed weight = 1 / number of drones that see water.        
        weight = 1 / len(self.targets)
        # print("weight =", weight)

        # Sampling according to gaussian distribution
        variance = 0.25
        X = np.arange(-boundary_limit, boundary_limit, variance)    # X range
        # print("X size:", X.size)
        Y = np.arange(-boundary_limit, boundary_limit, variance)    # Y range
        # print("Y size:", Y.size)
        X, Y = np.meshgrid(X, Y, sparse=True)   # The 2-dimension XY range 
        
        # Initialize Z with the right shape and fill with 0's.
        Z = np.zeros((int((boundary_limit * 2)/variance), int((boundary_limit * 2)/variance)))
        # print("Z shape:", Z.shape)
        
        # for index in range(len(self.targets)):
        for index in range(len(self.target_centroids)):
            # xy = self.targets[index]
            xy = self.target_centroids[index]
            # print("Drone",index," =",xy)
            
            # note: moments list = [M["m00"],M["m10"],M["m01"],M["m11"],M["m20"],M["m02"]]
            moments = self.target_moments[index] 
            
            # deltaX and deltaY are standard deviation of x and y coordinates respectively. 
            deltaX = np.sqrt(moments[4])
            deltaY = np.sqrt(moments[5])
            # print("deltaX:", deltaX)
            # print("deltaY:", deltaY)

            rho = moments[3]/(deltaX * deltaY)
            # print("rho:", rho)
            
            # Note: Sigma need to be small
            # sigma = 1 / (2 * (1 - rho**2)) # big positive sigma
            sigma = (1 / (2 * (1 - rho**2))) * (10**-2) # very small positive sigma, gives a smoother 3D bell shape.
            # print("sigma:", sigma)
            
            # compute Z domain.
            R = ((X - xy[0])**2) + ((Y - xy[1])**2)

            # applying Gaussian function to Z domain, to obtain the Z range.
            # Z = Z + (1024**3.7) * (1 / (2 * np.pi * deltaX * deltaY * np.sqrt(1 - (rho**2)))) * np.exp((-0.5 / (np.sqrt(1 - (rho**2)))) * R) # Original
            Z = Z + ((1024**3.65) * (1 / (2 * np.pi * deltaX * deltaY * np.sqrt(1 - rho**2))) * np.exp(-sigma * R))
            # print("Z:", Z)
        
        # Plot the surface.
        surf = ax.plot_surface(X, Y, Z, cmap=cm.Spectral, edgecolor='none', linewidth=0, antialiased=False)

        # Customize the z axis.
        ax.set_xlim([-boundary_limit, boundary_limit])
        ax.set_ylim([-boundary_limit, boundary_limit])
        ax.set_zlim([0, 1])

        ax.tick_params(labelsize=8)
        ax.xaxis.set_ticks(np.arange(-boundary_limit, boundary_limit + 1, boundary_limit/3))
        ax.yaxis.set_ticks(np.arange(-boundary_limit, boundary_limit + 1, boundary_limit/3))
        ax.zaxis.set_ticks(np.arange(0, 1.1, 0.1))
        
        font = {'size':9}
        ax.set_xlabel('x (m)',font)
        ax.set_ylabel('y (m)',font)
        # ax.set_zlabel('\u03C6',font, labelpad=10) # Small phi
        ax.set_zlabel('\u03A6',font) # Capital phi
        
    def diff_equation_x(self, y_list, t, e, omega):
        """
        Calculate the differential equation of x for calculating the
        trajectory of the drone movement

        Args:
            y_list - 
            t - 
            e - 
            omega - 
        Returns:
            result - 
        """
        ki = 200
        sum_phi = y_list[1] + e
        coef = 0
        for i in range(2,len(y_list)):
            if i%2 == 0:
                coef = int(i/2)
                sum_phi += (y_list[i] + e*math.cos(coef*omega*t)) * math.cos(coef*omega*t)
            else:
                coef = int((i-1)/2)
                sum_phi += (y_list[i] + e*math.sin(coef*omega*t)) * math.sin(coef*omega*t)
        
        result = []
        result.append(-ki*e-sum_phi + 20*math.cos(math.pi*t))
        result.append(-e)
        for i in range(2,len(y_list)):
            if i%2 == 0:
                coef = int(i/2)
                result.append(coef*e*omega*math.sin(coef*omega*t) + ki*e*math.cos(coef*omega*t))
            else:
                coef = int((i-1)/2)
                result.append((-e)*coef*omega*math.cos(coef*omega*t) + ki*e*math.sin(coef*omega*t))
        return np.array(result)
                         
    def diff_equation_y(self, y_list, t, e, omega):
        """
        Calculate the differential equation of y for calculating the
        trajectory of the drone movement

        Args:
            y_list - 
            t - 
            e - 
            omega - 
        Returns:
            result - 
        """
        ki = 200
        sum_fu = y_list[1] + e
        coef = 0
        for i in range(2,len(y_list)):
            if i%2 == 0:
                coef = int(i/2)
                sum_fu += (y_list[i] + e*math.cos(coef*omega*t)) * math.cos(coef*omega*t)
            else:
                coef = int((i-1)/2)
                sum_fu += (y_list[i] + e*math.sin(coef*omega*t)) * math.sin(coef*omega*t)
        result = []
        result.append(-ki*e-sum_fu + 20*math.sin(math.pi*t))
        result.append(-e)
        for i in range(2,len(y_list)):
            if i%2 == 0:
                coef = int(i/2)
                result.append(coef*e*omega*math.sin(coef*omega*t) + ki*e*math.cos(coef*omega*t))
            else:
                coef = int((i-1)/2)
                result.append((-e)*coef*omega*math.cos(coef*omega*t) + ki*e*math.sin(coef*omega*t))
        return np.array(result)

    def cal_tra(self, new_coords,new_centroids):
        """
        To calculate the trajectory of the drone to the centroid
        According to the Voronoi.

        Args:
            new_coords - 
            new_centroids - 
        Returns:
            point_lists - 
        """
        T=2
        t=np.linspace(0,T,num=150) # in 2time step update 50 times  
        omega = math.pi*2/T
        point_lists = []
        for i in range(len(new_coords)):
            y_list_x = [new_coords[i][0],0,0,0,0,0,0,0,0,0,0]
            y_list_y = [new_coords[i][1],0,0,0,0,0,0,0,0,0,0]
            result_x = odeint(self.diff_equation_x, y_list_x, t, args=(new_coords[i][0]-new_centroids[i][0],omega))
            result_y = odeint(self.diff_equation_y, y_list_y, t, args=(new_coords[i][1]-new_centroids[i][1],omega))
            result_xt = result_x[:,0]
            result_yt = result_y[:,0]
            new_result = np.vstack((np.array(result_xt), np.array(result_yt))).T
            point_lists.append(list(new_result)[1])
        return point_lists

    def compensate(self, pts, N_POINTS):
        """
        To compensate the position of the drone and the Voronoi algorithm

        Args:
            pts - 
            N_POINTS - 
        Returns:
            pts - 
        """
        while len(pts)<N_POINTS:
            drone_pos = points_to_coords(pts)
            randx = np.random.uniform(min(self.boundary_limits[:])[0], max(self.boundary_limits[:])[0], N_POINTS-len(pts))
            randy = np.random.uniform(min(self.boundary_limits[:])[1], max(self.boundary_limits[:])[1], N_POINTS-len(pts))
            compensated = np.vstack((randx, randy)).T
            if len(drone_pos):
                drone_pos = np.append(drone_pos, compensated, axis=0)
            else:
                drone_pos = compensated
                
                
            pts = [p for p in coords_to_points(drone_pos) if p.within(self.area_shape)]  # converts to shapely Point
            print('[controller] %d of %d drone pos is available' % (len(pts), N_POINTS))
        return pts

    # Taking sample for the density and centroid using normal density distribution.
    def sampling(self, targets, standard_deviation=20, numbers=1000):
        """
        Sampling according to gaussian distribution with x, y and sigma(x),
        sigma(y) the same value. Therefore, there is no multi-dimensional distribution.

        Args:
            targets - (List of List) The target centroid of the gaussian distribution
            standard_deviation - (int) The standard deviation for the gaussian distribution
            numbers - The number of point that will be deployed for the distribution
        Returns:
            x_unit - (Array) Give the gaussian sample for the x coordinate 
            y_unit - (Array) Give the gaussian sample for the y coordinate
        """
        ########### Density Distribution and Centroids Calculation #############
        if len(targets)==1:
            x_unit = np.random.normal(targets[0][0], standard_deviation, int(numbers * self.weights[0][0]))
            y_unit = np.random.normal(targets[0][1], standard_deviation, int(numbers * self.weights[0][0]))
        elif len(targets)>1:
            x_unit = np.array([])
            y_unit = np.array([])
            for i in range(len(targets)):
                x_unit = np.concatenate((x_unit,np.random.normal(targets[i][0], standard_deviation, int(numbers * self.weights[i][0]))))
                y_unit = np.concatenate((y_unit,np.random.normal(targets[i][1], standard_deviation, int(numbers * self.weights[i][0]))))

        return x_unit, y_unit

def exit_handler():
    # Write list of drone position into CSV file on exiting the program.
    # CSV file will be created if it does not exist.
    
    # creating header row
    header = []
    for i in range(num_of_drones):
        header.append('x'+str(i))
        header.append('y'+str(i))

    # Open CSV file for writing. CSV file will be created if it does not exist.
    with open('drone_position.csv', 'w', newline='') as file:
        csvwriter = csv.writer(file)    # create a CSV writer
        csvwriter.writerow(header)      # write the header row
        
        # write coordinates of drones from the list.
        for i in range(len(drone_pos_list)):            
            csvwriter.writerow(drone_pos_list[i])   
        

if __name__ == '__main__':
    ### Register function 'exit handler' to execute when this program (controller.py) is terminated using CTRL-C.
    ### Please comment when not required.
    # atexit.register(exit_handler)
    
    rospy.init_node('view_node')
    service_call = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    rate = rospy.Rate(1)
    voronoi = Voronoi()
    while not rospy.is_shutdown():
        pos_x = []
        pos_y = []
        for drone in voronoi.drone_list:
            # Get position of drones
            pose = get_pose(drone)
            x = pose.position.x
            y = pose.position.y
            pos_x.append(x)
            pos_y.append(y)
            
        drone_pos = np.vstack((np.array(pos_x).astype(np.float64), np.array(pos_y).astype(np.float64))).T
        voronoi.update_status(drone_pos)
        rate.sleep()
        
        
