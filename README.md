# System Health Monitor

This package contains a system_health_node to monitor status of running nodes and restarts them (by calling service).

[ROS node sample graph](rosgraph.png)

## Using the package

### 1. Dependencies

Install `rosmon`

For ROS Melodic:

```
sudo apt-get install ros-melodic-rosmon

source /opt/ros/${ROS_DISTRO}/setup.bash # Needed to use the 'mon launch' shortcut
```

For more information please refer to the respective [github](https://github.com/xqms/rosmon) and [tutorial](http://wiki.ros.org/rosmon)

### 2. Installation

Build the `system_health_monitor` package using `catkin_make`

### 3. Using [rosmon](http://wiki.ros.org/rosmon)

Rosmon is a replacement for roslaunch tool.

Note: Rosmon requires roscore and will not launch roscore automatically like roslaunch

For this project, use
```
mon launch system_health_monitor system_health_monitor.launch --name=rosmon --disable-diagnostics
```

### 4. Subscribed Topics

`rosmon/state (rosmon_msgs/State)`

State of all nodes running via rosmon

### 5. Published Topics

`system_health (system_health_monitor/SystemNodeHealth)`

State of all nodes for GUI to subscribe to

### 6. Services

`~rosmon/start_stop (rosmon_msgs/StartStop)`

Start of stop a controlled node
