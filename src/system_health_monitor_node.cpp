#include <ros/ros.h>
#include <system_health_monitor/SystemHealthMonitor.hpp>
#include <ros/spinner.h>

int main(int argc, char** argv) {

    ros::init(argc, argv, "system_health_monitor_node", true);
    
    ros::NodeHandle nodeHandle("~");

    system_health_monitor::SystemHealthMonitor systemHealthMonitor(nodeHandle);

    ros::spin();

    return 0;
}