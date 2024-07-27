#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <semantic_camera/TrafficLightStatus.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// Detect traffic light status from semantic image
std::string detectTrafficLight(const cv::Mat& semantic_image)
{
    // Assume traffic light position is fixed
    cv::Rect traffic_light_area(270, 190, 100, 100);
    cv::Mat traffic_light_roi = semantic_image(traffic_light_area);

    // Detect traffic light status
    if (cv::countNonZero(traffic_light_roi == 4) > 500) // Red light
    {
        return "RED";
    }
    else if (cv::countNonZero(traffic_light_roi == 5) > 500) // Green light
    {
        return "GREEN";
    }
    else if (cv::countNonZero(traffic_light_roi == 6) > 500) // Yellow light
    {
        return "YELLOW";
    }
    return "UNKNOWN";
}

// Callback function for processing the semantic image and depth image
void imageCallback(const sensor_msgs::ImageConstPtr& semantic_msg, const sensor_msgs::ImageConstPtr& depth_msg)
{
    try
    {
        cv::Mat semantic_image = cv_bridge::toCvCopy(semantic_msg, "mono8")->image;
        cv::Mat depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;

        // Detect traffic light status
        semantic_camera::TrafficLightStatus traffic_light_msg;
        traffic_light_msg.status = detectTrafficLight(semantic_image);

        // Calculate distance to the traffic light
        if (traffic_light_msg.status == "UNKNOWN") {
            traffic_light_msg.distance = -1.0; // Indicate that no traffic light was detected
        } else {
            cv::Rect traffic_light_area(270, 190, 100, 100);
            cv::Mat depth_roi = depth_image(traffic_light_area);
            traffic_light_msg.distance = cv::mean(depth_roi)[0];
        }

        static ros::NodeHandle nh;
        static ros::Publisher traffic_light_pub = nh.advertise<semantic_camera::TrafficLightStatus>("/traffic_light_status", 10);
        traffic_light_pub.publish(traffic_light_msg);

        ROS_INFO("Traffic light status: %s, Distance: %.2f", traffic_light_msg.status.c_str(), traffic_light_msg.distance);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_camera_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> semantic_sub(nh, "/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/unity_ros/OurCar/Sensors/DepthCamera/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), semantic_sub, depth_sub);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    ros::spin();

    return 0;
}
