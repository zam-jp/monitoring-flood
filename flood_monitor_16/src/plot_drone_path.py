#!/usr/bin/env python3
'''
@author: Shaiful Nizam
@since: February 15, 2022
@organization: RoboLab (Ritsumeikan University, College of Information Science and Engineering)
@summary: To plot the path taken by the drones.
'''

import matplotlib.pyplot as plt
import pandas as pd
import configparser

config = configparser.ConfigParser()
config.read('config.ini')
boundary_limit = float(config['distance']['aoi_boundary'])
num_of_drones = 16

df = pd.read_csv('drone_position.csv')

plt.figure(figsize=[8,8])
for i in range(num_of_drones):
    plt.scatter(df['x'+str(i)].values[0], df['y'+str(i)].values[0], marker = "v") # Use an upside-down triangle to mark the starting point        
    plt.plot(df['x'+str(i)].values, df['y'+str(i)].values, label = "Drone "+str(i+1))

plt.legend(bbox_to_anchor=(0.5, 1.12), ncol=5, loc="upper center", fontsize=12)
plt.xticks(fontsize=18)
plt.yticks(fontsize=18)
plt.xlim([-boundary_limit, boundary_limit])
plt.ylim([-boundary_limit, boundary_limit])
font = {'size':18}
plt.xlabel('x (m)', font)
plt.ylabel('y (m)', font)
plt.show()