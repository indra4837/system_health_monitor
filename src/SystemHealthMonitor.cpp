#include "system_health_monitor/SystemHealthMonitor.hpp"

namespace system_health_monitor {

SystemHealthMonitor::SystemHealthMonitor(ros::NodeHandle& nodeHandle) :
    nodeHandle_(nodeHandle)
{   

    if (!nodeHandle.getParam("warning_percentage", warning_percentage)) {
        ROS_ERROR("Could not find warning_percentage parameter!");
    }

    // parse yaml file and insert into list
    nodeList = retrieveParameters("nodes", nodeHandle);                     
    normal_pubRate = retrieveParameters("normal_pub_rate", nodeHandle);  

    node_state_sub = nodeHandle.subscribe("/rosmon/state", 100, &SystemHealthMonitor::nodeStateCallback, this);
    system_health_pub = nodeHandle.advertise<system_health_monitor::SystemNodeHealth>("/system_health", 100);
    pub_rate_sub = nodeHandle.subscribe("/diagnostics", 100, &SystemHealthMonitor::pubRateCallback, this);
    client = nodeHandle.serviceClient<rosmon_msgs::StartStop>("/rosmon_msgs/start_stop");
}

void SystemHealthMonitor::pubRateCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& pubRate) {
    
    for (int i=0; i<pubRate->status.size(); i++) {
        for (int j=0; j<nodeList.size(); j++) {
            if (pubRate->status[i].name.find(nodeList[j]) != std::string::npos) {
                for (int k=0; k<pubRate->status[i].values.size(); k++) {
                    if (pubRate->status[i].values[k].key.find("publication rate") != std::string::npos) {
                        publication_rate = pubRate->status[i].values[k].value;
                        node_name = nodeList[j];
                    }
                }
            }
        }
    }
}

std::vector<std::string> SystemHealthMonitor::retrieveParameters(std::string paramRetrieval, ros::NodeHandle& nodeHandle) {
    
    XmlRpc::XmlRpcValue paramList;

    std::vector<std::string> params;

    nodeHandle.getParam(paramRetrieval, paramList);

    for (int j=0; j<paramList.size(); j++) {
        std::string value = static_cast<std::string>(paramList[j]);
        params.push_back(value);
    }

    return params;
}

int SystemHealthMonitor::checkPublishRate() {
    
    double normal_rate, pub_rate;
    
    pub_rate = stod(publication_rate);

    for (int i=0; i<nodeList.size(); i++) {
        normal_rate =  stod(normal_pubRate[i]);
        if (node_name == nodeList[i] && pub_rate<warning_percentage*normal_rate) {
            return 1; // warning
        }    
    }

    return 0;
}

void SystemHealthMonitor::nodeStateCallback(const rosmon_msgs::State::ConstPtr& state) {
    
    system_health_monitor::SystemNodeHealth systemState;
    system_health_monitor::NodeHealth nodeState;
    rosmon_msgs::StartStop srv;

    for (int i=0;i<state->nodes.size();i++) {
    
        nodeState.name = state->nodes[i].name;

        if (state->nodes[i].state != 1) {
            // TODO: get service call to work
            ROS_INFO_STREAM(nodeState.name << " NOT running\n");
            srv.request.node = nodeState.name;
            srv.request.ns = state->nodes[i].ns;
            srv.request.action = 1; // start node    

            if (client.call(srv)) {
                ROS_INFO_STREAM("successfully called client");
                if (state->nodes[i].state != 1) {
                    ROS_INFO_STREAM("Failed to restart " << nodeState.name <<"\n");
                    nodeState.state = 2;
                } else {
                    ROS_INFO_STREAM(nodeState.name << " restarted and running\n");
                    nodeState.state = 0;
                }
            } else {
                ROS_ERROR("Failed to call service and restart node\n");
                nodeState.state = 2;
            }
            
        } else {
            nodeState.state = 0;
        }

        if (nodeState.name == node_name) {
            nodeState.state = checkPublishRate();
            nodeState.publication_rate = publication_rate;
        } else {
            nodeState.publication_rate = "None";
        }
        systemState.nodes.push_back(nodeState);
    }

    systemState.header.stamp = ros::Time::now();
    system_health_pub.publish(systemState);
}

} // namespace system_health_monitor