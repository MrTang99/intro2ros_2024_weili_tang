#include <ros/ros.h>
#include <semantic_camera/TrafficLightStatus.h>

// Callback function for traffic light status
void trafficLightCallback(const semantic_camera::TrafficLightStatus::ConstPtr& msg)
{
    if (msg->status == "RED")
    {
        ROS_INFO("Traffic light is RED. Stopping the vehicle.");
        // Add logic to stop the vehicle
    }
    else if (msg->status == "GREEN")
    {
        ROS_INFO("Traffic light is GREEN. Continuing to drive.");
        // Add logic to continue driving
    }
    else if (msg->status == "YELLOW")
    {
        ROS_INFO("Traffic light is YELLOW. Prepare to stop.");
        // Add logic to prepare to stop
    }
    ROS_INFO("Distance to traffic light: %.2f meters", msg->distance);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traffic_light_controller");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/traffic_light_status", 10, trafficLightCallback);

    ros::spin();

    return 0;
}
