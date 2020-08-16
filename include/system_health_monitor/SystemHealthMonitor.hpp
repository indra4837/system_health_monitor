#pragma once

#include <ros/ros.h>
#include "system_health_monitor/SystemNodeHealth.h"
#include "rosmon_msgs/StartStop.h"
#include "rosmon_msgs/State.h"
#include <diagnostic_msgs/DiagnosticArray.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include <vector>
#include <string>

namespace system_health_monitor {

class SystemHealthMonitor {
public:
    SystemHealthMonitor(ros::NodeHandle& nodeHandle);

private:
    ros::NodeHandle nodeHandle_;
    ros::Subscriber node_state_sub, pub_rate_sub; 
    ros::Publisher system_health_pub;   // for GUI
    ros::ServiceClient client;          // respawns node if needed

    std::vector<std::string> nodeList;
    std::vector<std::string> normal_pubRate;
    std::string publication_rate, node_name;

    double warning_percentage;

    void nodeStateCallback(const rosmon_msgs::State::ConstPtr& nodeState);
    void pubRateCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& pubRate);
    
    int checkPublishRate();

    std::vector<std::string> retrieveParameters(std::string paramRetrieval, ros::NodeHandle& nodeHandle);
};

} // namespace system_health_monitor