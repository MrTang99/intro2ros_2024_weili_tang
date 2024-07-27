#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <semantic_camera/TrafficLightStatus.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>

// Function to check the labels in the semantic image
void checkSemanticImageLabels(const cv::Mat& semantic_image)
{
    std::map<int, int> label_counts;

    for (int i = 0; i < semantic_image.rows; ++i)
    {
        for (int j = 0; j < semantic_image.cols; ++j)
        {
            int label = semantic_image.at<uchar>(i, j);
            if (label_counts.find(label) == label_counts.end())
            {
                label_counts[label] = 1;
            }
            else
            {
                label_counts[label]++;
            }
        }
    }

    for (const auto& pair : label_counts)
    {
        ROS_INFO("Label %d: %d pixels", pair.first, pair.second);
    }
}

// Detect traffic light areas from semantic image and select the one closest to the image center
cv::Rect detectTrafficLightArea(const cv::Mat& semantic_image)
{
    std::vector<std::vector<cv::Point>> contours;
    cv::Mat mask = (semantic_image == 215); // Assuming 215 is the label for traffic lights
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return cv::Rect();
    }

    cv::Point image_center(semantic_image.cols / 2, semantic_image.rows / 2);
    double min_distance = std::numeric_limits<double>::max();
    cv::Rect selected_area;

    for (const auto& contour : contours)
    {
        cv::Rect area = cv::boundingRect(contour);
        cv::Point area_center(area.x + area.width / 2, area.y + area.height / 2);
        double distance = cv::norm(area_center - image_center);

        if (distance < min_distance)
        {
            min_distance = distance;
            selected_area = area;
        }
    }

    // Expand the traffic light area vertically to better capture the light colors
    int expand_height = selected_area.height * 2;
    selected_area.y = std::max(0, selected_area.y - expand_height / 2);
    selected_area.height = std::min(semantic_image.rows - selected_area.y, selected_area.height + expand_height);

    ROS_INFO("Traffic light area detected: x=%d, y=%d, width=%d, height=%d", selected_area.x, selected_area.y, selected_area.width, selected_area.height);

    return selected_area;
}

// Identify the color of the traffic light using the RGB image
std::string identifyTrafficLightColor(const cv::Mat& rgb_image, const cv::Rect& traffic_light_area)
{
    if (traffic_light_area.area() == 0)
    {
        return "UNKNOWN";
    }

    // Ensure ROI is within the image bounds
    cv::Rect bounded_area = traffic_light_area & cv::Rect(0, 0, rgb_image.cols, rgb_image.rows);
    cv::Mat rgb_roi = rgb_image(bounded_area);

    // Convert ROI to HSV color space
    cv::Mat hsv_roi;
    cv::cvtColor(rgb_roi, hsv_roi, cv::COLOR_BGR2HSV);

    // Threshold the HSV image to get only red, green and yellow colors
    cv::Mat red_mask1, red_mask2, green_mask, yellow_mask;
    cv::inRange(hsv_roi, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), red_mask1);
    cv::inRange(hsv_roi, cv::Scalar(160, 100, 100), cv::Scalar(180, 255, 255), red_mask2);
    cv::inRange(hsv_roi, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), green_mask);
    cv::inRange(hsv_roi, cv::Scalar(20, 150, 150), cv::Scalar(30, 255, 255), yellow_mask);

    cv::Mat red_mask = red_mask1 | red_mask2;

    if (cv::countNonZero(red_mask) > 2)
    {
        return "RED";
    }
    else if (cv::countNonZero(green_mask) > 2)
    {
        return "GREEN";
    }
    else if (cv::countNonZero(yellow_mask) > 2)
    {
        return "YELLOW";
    }

    return "UNKNOWN";
}

// Callback function for processing the semantic image, depth image, and RGB image
void imageCallback(const sensor_msgs::ImageConstPtr& semantic_msg, const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg)
{
    try
    {
        cv::Mat semantic_image = cv_bridge::toCvCopy(semantic_msg, "mono8")->image;
        cv::Mat depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        cv::Mat rgb_image = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;

        // Check semantic image labels
        checkSemanticImageLabels(semantic_image);

        // Detect traffic light area in semantic image
        cv::Rect traffic_light_area = detectTrafficLightArea(semantic_image);

        // Ensure the traffic light area is within the depth image bounds
        if (traffic_light_area.x < 0 || traffic_light_area.y < 0 ||
            traffic_light_area.x + traffic_light_area.width > depth_image.cols ||
            traffic_light_area.y + traffic_light_area.height > depth_image.rows) {
            ROS_WARN("Traffic light area is out of depth image bounds");
            return;
        }

        // Identify traffic light color in RGB image
        std::string traffic_light_color = identifyTrafficLightColor(rgb_image, traffic_light_area);

        // Create and populate TrafficLightStatus message
        semantic_camera::TrafficLightStatus traffic_light_msg;
        traffic_light_msg.status = traffic_light_color;

        // Calculate distance to the traffic light
        cv::Mat depth_roi = depth_image(traffic_light_area);
        if (depth_roi.empty()) {
            ROS_WARN("Depth ROI is empty!");
        } else {
            ROS_INFO("Depth ROI mean: %f", cv::mean(depth_roi)[0]);
        }

        if (traffic_light_color == "UNKNOWN")
        {
            traffic_light_msg.distance = -1.0; // Indicate that no traffic light was detected
        }
        else
        {
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
    catch (cv::Exception& e)
    {
        ROS_ERROR("OpenCV exception: %s", e.what());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semantic_camera_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::Image> semantic_sub(nh, "/unity_ros/OurCar/Sensors/SemanticCamera/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/unity_ros/OurCar/Sensors/DepthCamera/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/unity_ros/OurCar/Sensors/RGBCameraRight/image_raw", 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), semantic_sub, depth_sub, rgb_sub);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2, _3));

    ros::spin();

    return 0;
}
