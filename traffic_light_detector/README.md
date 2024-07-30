# Traffic Light Detector

## Overview

The `traffic_light_detector` package is designed for autonomous driving applications. It consists of two main nodes: `traffic_light_detector_node` and `traffic_light_controller_node`. The primary functionality of this package is to detect traffic lights, determine their colors, and control the vehicle based on the detected traffic light status. The detection algorithm prioritizes the traffic light closest to the camera's center to ensure accurate and relevant traffic light detection.

## Nodes

### traffic_light_detector_node

This node processes images from a depth camera and an RGB camera to detect traffic lights and identify their colors. It also calculates the distance to the detected traffic light and publishes this information.

#### Subscribed Topics

- `/unity_ros/OurCar/Sensors/DepthCamera/image_raw` (`sensor_msgs/Image`): Depth camera image.
- `/unity_ros/OurCar/Sensors/RGBCameraRight/image_raw` (`sensor_msgs/Image`): RGB camera image.

#### Published Topics

- `/traffic_light_status` (`traffic_light_detector/TrafficLightStatus`): Traffic light status and distance.

#### Parameters
Thresholds for red pixels, green pixels, yellow pixels and black pixels

Size of the area of the candidate region

Size of the Region of Interest

#### Functionality

The `traffic_light_detector_node` is responsible for detecting traffic lights and determining their status (Red, Green, Yellow) along with their distance from the vehicle. This node processes images from an RGB camera and a depth camera to identify the presence and color of traffic lights. The detection process involves image processing, ROI definition, color detection, contour detection, selection of the closest traffic light, color identification, distance calculation, and publishing of the detected traffic light status and distance.

### traffic_light_controller_node

This node subscribes to the traffic light status and controls the vehicle based on the detected traffic light color.

#### Subscribed Topics

- `/traffic_light_status` (`traffic_light_detector/TrafficLightStatus`): Traffic light status and distance.

#### Functionality

The `traffic_light_controller_node` is responsible for controlling the vehicle based on the traffic light status. It subscribes to the `/traffic_light_status` topic and makes decisions to control the vehicle's movement based on the traffic light status (RED, GREEN, YELLOW, UNKNOWN). It implements logic to stop, continue, or prepare to stop the vehicle based on the detected traffic light color.

## Custom Message Type

A custom message type, `TrafficLightStatus.msg`, has been defined to provide detailed information about the detected traffic lights to other nodes, such as the traffic light controller. The message includes the status of the traffic light (Red, Green, Yellow, or Unknown) and the distance to the traffic light.

### Message Definition

```plaintext
string status
float32 distance
```

## External Code Usage

OpenCV: The image processing and clustering algorithms heavily relied on OpenCV functions for color space conversion, thresholding, and contour detection.

cv_bridge: This ROS package was used to convert between ROS image messages and OpenCV images.

## Challenges

Low Resolution of RGB Camera: The low resolution of the RGB camera can lead to inaccurate traffic light detection, especially at a distance. This results in blurred and pixelated images, making it difficult for the algorithm to accurately detect and classify traffic lights.

Environmental Light Interference: Variations in environmental lighting conditions, such as shadows, glare, and changes in brightness, can affect the accuracy of color-based detection. These interferences can cause the algorithm to misclassify or fail to detect traffic lights.

Cluster Algorithm Limitations: The clustering algorithm used to identify traffic light colors sometimes fails due to the above issues, leading to incorrect traffic light color identification. This can impact the decision-making process of the vehicle control node.

## Link to the test video
https://drive.google.com/file/d/1h6ljJUdehGCnt2qPIu5kn_l8KJ_z_ggH/view?usp=sharing