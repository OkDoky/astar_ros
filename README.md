# A* for ROS
This repo is an A* planner configured with ROS service.

## Subscribed Topic
- map (nav_msgs/OccupancyGrid)

## Service Request
- start (geometry_msgs/Point)

- goal (geometry_msgs/Point)

## Service Response
- path (nav_msgs/Path)

## Prerequisites
- Ubuntu
- ROS

Tested on Ubuntu 20.04, ROS Noetic.
## Getting Started
- Clone this repo in your src directory of ROS workspace.
```
git clone https://github.com/rohgal/astar_ros.git
```
- Build
```
catkin_make
```
- Launch A* ROS server
```
roslaunch astar_ros astar_server.launch
```
