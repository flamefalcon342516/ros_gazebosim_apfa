#ros_gazebosim_apfa
ROS 2-based Car Simulation in Gazebo

This repository features a ROS 2 project for simulating and controlling a car in Gazebo. The simulation uses a custom car model built with .xacro and .gazebo files, with a launch file to initialize the environment. Car movements—forward, backward, left, and right—are controlled via Python by manipulating wheel rotations through Gazebo messages.

Key Features:
Custom Car Model: Designed with .xacro and .gazebo files for realistic dynamics.
Python Control: Scripts to manage car movement and control.
ROS 2 Integration: Communicate with the simulation using ROS 2 topics and services.
Visualization: Use matplotlib to plot movement paths, obstacles, and targets.
Planned Enhancements:
Path planning and obstacle avoidance.
Integration of LIDAR, GPS, and camera sensors.
Support for multi-vehicle simulation.

