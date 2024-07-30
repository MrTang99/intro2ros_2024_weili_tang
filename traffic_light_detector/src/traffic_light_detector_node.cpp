#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <traffic_light_detector/TrafficLightStatus.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/package.h>
#include <limits>
#include <vector>

// Define the region of interest (ROI) for traffic light detection
cv::Rect defineROI(int img_width, int img_height) {
    int x = 100;
    int y = 14;
    int width = 100;
    int height = 400;

    // Ensure the ROI is within the image boundaries
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x + width > img_width) width = img_width - x;
    if (y + height > img_height) height = img_height - y;

    return cv::Rect(x, y, width, height);
}

// Function to detect traffic lights using color information
cv::Rect detectTrafficLightsUsingColor(const cv::Mat& rgb_image) {
    std::vector<cv::Rect> detected_areas;

    // Convert to HSV color space
    cv::Mat hsv_image;
    cv::cvtColor(rgb_image, hsv_image, cv::COLOR_BGR2HSV);

    // Define ROI
    cv::Rect roi_rect = defineROI(rgb_image.cols, rgb_image.rows);
    cv::Mat roi_image = hsv_image(roi_rect);

    // Detect yellow regions
    cv::Mat yellow_mask;
    cv::inRange(roi_image, cv::Scalar(15, 80, 80), cv::Scalar(35, 255, 255), yellow_mask);

    // Use morphological operations to reduce noise
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_OPEN, kernel);

    // Find contours of yellow regions
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(yellow_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    cv::Point roi_center(roi_rect.width / 2, roi_rect.height / 2);
    double min_distance = std::numeric_limits<double>::max();
    cv::Rect closest_area;

    for (const auto& contour : contours) {
        cv::Rect bounding_box = cv::boundingRect(contour);

        // Extract the region of interest (ROI)
        cv::Mat roi = roi_image(bounding_box);

        // Create color masks
        cv::Mat black_mask, red_mask1, red_mask2, green_mask, yellow_mask_in_roi;
        cv::inRange(roi, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 200), black_mask); // Relaxed range
        cv::inRange(roi, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), red_mask1);
        cv::inRange(roi, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), red_mask2);
        cv::inRange(roi, cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), green_mask);
        cv::inRange(roi, cv::Scalar(15, 80, 80), cv::Scalar(35, 255, 255), yellow_mask_in_roi);

        int black_count = cv::countNonZero(black_mask);
        int red_count = cv::countNonZero(red_mask1 | red_mask2);
        int green_count = cv::countNonZero(green_mask);
        int yellow_count = cv::countNonZero(yellow_mask_in_roi);

        // Filter out too small areas
        if (bounding_box.area() > 4 && black_count > 1 && (red_count > 1 || green_count > 1 || yellow_count > 1)) {
            detected_areas.push_back(bounding_box);

            // Calculate the distance to the center point
            cv::Point bounding_box_center(bounding_box.x + bounding_box.width / 2, bounding_box.y + bounding_box.height / 2);
            double distance = cv::norm(roi_center - bounding_box_center);

            // Update the area closest to the center point
            if (distance < min_distance) {
                min_distance = distance;
                closest_area = bounding_box;
            }
        }
    }

    // Convert the area within the ROI back to the coordinate system of the whole image
    if (closest_area.area() > 0) {
        closest_area.x += roi_rect.x;
        closest_area.y += roi_rect.y;
    }

    return closest_area;
}

// Function to identify the traffic light color
std::string identifyTrafficLightColor(const cv::Mat& rgb_image, const cv::Rect& traffic_light_area) {
    if (traffic_light_area.area() == 0) {
        return "UNKNOWN";
    }

    cv::Rect bounded_area = traffic_light_area & cv::Rect(0, 0, rgb_image.cols, rgb_image.rows);
    cv::Mat rgb_roi = rgb_image(bounded_area);

    cv::Mat hsv_roi;
    cv::cvtColor(rgb_roi, hsv_roi, cv::COLOR_BGR2HSV);

    // Strictly set color thresholds
    cv::Mat red_mask1, red_mask2, green_mask, yellow_mask;
    cv::inRange(hsv_roi, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), red_mask1);
    cv::inRange(hsv_roi, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), red_mask2);
    cv::inRange(hsv_roi, cv::Scalar(35, 50, 50), cv::Scalar(85, 255, 255), green_mask);
    cv::inRange(hsv_roi, cv::Scalar(20, 150, 150), cv::Scalar(30, 255, 255), yellow_mask);

    cv::Mat red_mask = red_mask1 | red_mask2;

    int red_count = cv::countNonZero(red_mask);
    int green_count = cv::countNonZero(green_mask);
    int yellow_count = cv::countNonZero(yellow_mask);

    ROS_INFO("Red pixels: %d, Green pixels: %d, Yellow pixels: %d", red_count, green_count, yellow_count);

    if (red_count > 2) {
        return "RED";
    } else if (green_count > 2) {
        return "GREEN";
    } else if (yellow_count > 2) {
        return "YELLOW";
    }

    return "UNKNOWN";
}

// Callback function for processing synchronized depth and RGB images
void imageCallback(const sensor_msgs::ImageConstPtr& depth_msg, const sensor_msgs::ImageConstPtr& rgb_msg) {
    try {
        // Convert ROS images to OpenCV images
        cv::Mat depth_image = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
        cv::Mat rgb_image = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8)->image;

        // Detect traffic light area
        cv::Rect traffic_light_area = detectTrafficLightsUsingColor(rgb_image);
        std::string traffic_light_color = identifyTrafficLightColor(rgb_image, traffic_light_area);

        // Create and publish the TrafficLightStatus message
        traffic_light_detector::TrafficLightStatus traffic_light_msg;
        traffic_light_msg.status = traffic_light_color;

        if (traffic_light_area.x < 0 || traffic_light_area.y < 0 ||
            traffic_light_area.x + traffic_light_area.width > depth_image.cols ||
            traffic_light_area.y + traffic_light_area.height > depth_image.rows) {
            ROS_WARN("Traffic light area is out of depth image bounds");
            traffic_light_msg.distance = -1.0;
        } else {
            cv::Mat depth_roi = depth_image(traffic_light_area);
            if (depth_roi.empty()) {
                ROS_WARN("Depth ROI is empty!");
                traffic_light_msg.distance = -1.0;
            } else {
                traffic_light_msg.distance = cv::mean(depth_roi)[0];
            }
        }

        static ros::NodeHandle nh;
        static ros::Publisher traffic_light_pub = nh.advertise<traffic_light_detector::TrafficLightStatus>("/traffic_light_status", 10);

        traffic_light_pub.publish(traffic_light_msg);

        ROS_INFO("Traffic light status: %s, Distance: %.2f", traffic_light_msg.status.c_str(), traffic_light_msg.distance);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (cv::Exception& e) {
        ROS_ERROR("OpenCV exception: %s", e.what());
    }
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "traffic_light_detector_node"); 
    ros::NodeHandle nh;

    // Subscribe to depth and RGB images
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/unity_ros/OurCar/Sensors/DepthCamera/image_raw", 10);
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/unity_ros/OurCar/Sensors/RGBCameraRight/image_raw", 10);
    
    // Synchronize the depth and RGB image messages
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), depth_sub, rgb_sub);
    sync.registerCallback(boost::bind(&imageCallback, _1, _2));

    // Spin to process callbacks
    ros::spin();

    return 0;
}
