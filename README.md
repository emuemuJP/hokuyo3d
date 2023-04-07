# hokuyo3d

ROS driver node for HOKUYO 3D LIDARs
====================

This package provides a ROS driver node for 3D LIDARs with VSSP protocol.
VSSP protocol 1.0, 1.1 (for HOKUYO YVT-X002) and 2.1 (for HOKUYO YVT-35LX) are supported.

![Screen capture of the data](http://openspur.org/~atsushi.w/files/ros_hokuyo3d_cap.png)

## Installation
### Cloning this repository
```
cd ~/catkin_ws/src
git clone -b topic_class https://github.com/emuemuJP/hokuyo3d.git
```
### Installation of dependencies
```
```

## Running

### detect area distance of laserscan
```
rosrun hokuyo3d hokuyo3d
```

## Input/Output topic
### Input

None

### Output
- hokuyo_cloud [PointCloud](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud.html)

- hokuyo_cloud2_frame [PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)

- hokuyo_cloud2_field [PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html) 

- hokuyo_cloud2_line [PointCloud2](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html)

- imu [Imu](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html)

- mag [MagneticField](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/MagneticField.html)

## Parameters
- ip(string): IP address of the sensor. Default value is "192.168.0.10".
- port(int) : Port number of the sensor. Default value is 10940.
- frame_id(string): Frame id of the topic. Default value is "hokuyo3d".
- imu_frame_id(string): Imu frame id of the topic. Default value is frame_id + "_imu".
- mag_frame_id(string): Mag frame id of the topic. Default value is frame_id + "_mag".
- range_min(float) : Default value is 0.0.
- auto_reset(bool) : Default value is false.
- allow_jump_back(bool) : Default value is false.
- enable_frame_topic(bool) : Enable/disable sending of frame topic. Default value is false.
- enable_field_topic(bool) : Enable/disable sending of field topic. Default value is false.
- enable_line_topic(bool) : Enable/disable sending of line topic. Default value is false.
