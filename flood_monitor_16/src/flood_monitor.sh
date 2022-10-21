#!/bin/sh
rosservice call /monitor/enable_motors true
rosservice call /drone01/enable_motors true
rosservice call /drone02/enable_motors true
rosservice call /drone03/enable_motors true
rosservice call /drone04/enable_motors true
rosservice call /drone05/enable_motors true
rosservice call /drone06/enable_motors true
rosservice call /drone07/enable_motors true
rosservice call /drone08/enable_motors true
rosservice call /drone09/enable_motors true
rosservice call /drone10/enable_motors true
rosservice call /drone11/enable_motors true
rosservice call /drone12/enable_motors true
rosservice call /drone13/enable_motors true
rosservice call /drone14/enable_motors true
rosservice call /drone15/enable_motors true
rosservice call /drone16/enable_motors true
sleep 1
gnome-terminal --tab -- rosrun flood_monitor_16 drone_monitor.py 
sleep 10
gnome-terminal --tab -- rosrun flood_monitor_16 initialize_drone.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone04.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone08.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone12.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone16.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone03.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone07.py 
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone11.py 
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone15.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone02.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone06.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone10.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone14.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone01.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone05.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone09.py
sleep 2
gnome-terminal --tab -- rosrun flood_monitor_16 drone13.py
sleep 20
gnome-terminal --tab -- rosrun flood_monitor_16 controller.py