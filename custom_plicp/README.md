# Custom implementation of Canonical Scan Matcher (CSM)
This repo is the implementation of Point-to-Line Iterative Closest Point (PL-ICP) algorithm proposed by Censi [1]. The original code can be found in [2]. The current package uses open karto library [3] for scan matching and is the direct work of Jian Wen [4]. The current implementation has the drawback of incremental error which can be offset by GMSAT (a smoothing algorithm) as done by Frank Kung [5]. The demo bag file has been taken from the same project [6].

## How to use on Ubuntu?
1. This package has been tested well in Ubuntu 18.04 with ROS Melodic.

2. To use the package do the following
```python
# Change directory to src of your catkin workspace
$ cd ~/catkin_ws/src/
# Copy the two packages namely `csm` and `custom_plicp` there
$ cd ..
# The below command should build both the packages as CSM is a requirement
$ catkin build custom_plicp
$ source devel/setup.bash
```

3. Test if everything is working fine
```python
$ roslaunch custom_plicp demo.launch
$ unzip demo.bag.zip
$ rosbag play demo.bag --clock
```

## Topics

### Subscribed topics
- `/scan` ([sensor_msgs/LaserScan](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/LaserScan.html))
- `/tf` ([tf2_msgs/TFMessage](http://docs.ros.org/melodic/api/tf2_msgs/html/msg/TFMessage.html))

### Published topics
- `/map` ([nav_msgs/OccupancyGrid](http://docs.ros.org/melodic/api/nav_msgs/html/msg/OccupancyGrid.html))
- `/map_metadata` ([nav_msgs/MapMetaData](http://docs.ros.org/melodic/api/nav_msgs/html/msg/MapMetaData.html))
- `/tf` ([tf2_msgs/TFMessage](http://docs.ros.org/melodic/api/tf2_msgs/html/msg/TFMessage.html))

## Nodes
  [Work in Progress]

## References

[1] A. Censi, "An ICP variant using a point-to-line metric," in *Proceedings of the IEEE International Conference on Robotics and Automation*, 2008, pp. 19-25.

[2] https://github.com/ccny-ros-pkg/scan_tools

[3] [Open Karto](https://github.com/ros-perception/open_karto)

[4] [Laser Scan Tools for ROS](https://github.com/nkuwenjian/laser_scan_matcher)

[5] [Robot Localization using Laser Scanner and Pose-graph Optimization](https://medium.com/@k3083518729/robot-localization-using-laser-scanner-and-pose-graph-optimization-fc40605bf5bc)

[6] [PLICP + GSTAM](https://github.com/kungfrank/icp_gtsam_localization)
