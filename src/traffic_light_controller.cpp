#include <ros/ros.h>
#include <traffic_light_detector/TrafficLightStatus.h>

// Callback function to handle traffic light status messages
void trafficLightStatusCallback(const traffic_light_detector::TrafficLightStatus::ConstPtr& msg)
{
    if (msg->status == "RED")
    {
        ROS_INFO("Traffic light is RED. Stopping the vehicle.");
        // Add logic to stop the vehicle
    }
    else if (msg->status == "GREEN")
    {
        ROS_INFO("Traffic light is GREEN. Continuing to drive.");
        // Add logic to drive the vehicle
    }
    else if (msg->status == "YELLOW")
    {
        ROS_INFO("Traffic light is YELLOW. Preparing to stop.");
        // Add logic to prepare the vehicle to stop
    }
    else
    {
        ROS_INFO("Traffic light status is UNKNOWN.");
        // Add logic for unknown status
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traffic_light_controller_node");
    ros::NodeHandle nh;

    ros::Subscriber traffic_light_sub = nh.subscribe("/traffic_light_status", 10, trafficLightStatusCallback);

    ros::spin();

    return 0;
}
