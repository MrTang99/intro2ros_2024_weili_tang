#include <ros/ros.h>
#include <semantic_camera/TrafficLightStatus.h>

// Callback function for traffic light status
void trafficLightCallback(const semantic_camera::TrafficLightStatus::ConstPtr& msg)
{
    if (msg->status == "RED")
    {
        ROS_INFO("Traffic light is RED. Stopping the vehicle.");
        // Add logic to stop the vehicle
        // Example: publish stop command to the vehicle control topic
        // vehicle_control_pub.publish(stop_command);
    }
    else if (msg->status == "GREEN")
    {
        ROS_INFO("Traffic light is GREEN. Continuing to drive.");
        // Add logic to continue driving
        // Example: publish drive command to the vehicle control topic
        // vehicle_control_pub.publish(drive_command);
    }
    else if (msg->status == "YELLOW")
    {
        ROS_INFO("Traffic light is YELLOW. Prepare to stop.");
        // Add logic to prepare to stop
        // Example: publish slow down command to the vehicle control topic
        // vehicle_control_pub.publish(slow_down_command);
    }
    ROS_INFO("Distance to traffic light: %.2f meters", msg->distance);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "traffic_light_controller");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/traffic_light_status", 10, trafficLightCallback);

    // If you have a vehicle control publisher, initialize it here
    // ros::Publisher vehicle_control_pub = nh.advertise<control_msgs::Command>("vehicle_control_topic", 10);

    ros::spin();

    return 0;
}

