# semantic_camera

## Overview

The `semantic_camera` package is designed for autonomous driving applications. It includes a semantic camera node to detect traffic lights and a controller node to control the vehicle based on the traffic light status. The package ensures that the traffic light closest to the camera's center is prioritized for detection.

## Nodes

### semantic_camera_node

This node processes images from a semantic camera, depth camera, and RGB camera to detect traffic lights and their colors. It also calculates the distance to the traffic light.

#### Subscribed Topics

- `/unity_ros/OurCar/Sensors/SemanticCamera/image_raw` (`sensor_msgs/Image`): Semantic camera image.
- `/unity_ros/OurCar/Sensors/DepthCamera/image_raw` (`sensor_msgs/Image`): Depth camera image.
- `/unity_ros/OurCar/Sensors/RGBCameraRight/image_raw` (`sensor_msgs/Image`): RGB camera image.

#### Published Topics

- `/traffic_light_status` (`semantic_camera/TrafficLightStatus`): Traffic light status and distance.

#### Parameters

No parameters are currently defined for this node.

### traffic_light_controller

This node subscribes to the traffic light status and controls the vehicle based on the traffic light color.

#### Subscribed Topics

- `/traffic_light_status` (`semantic_camera/TrafficLightStatus`): Traffic light status and distance.

#### Parameters

No parameters are currently defined for this node.

## Messages

### TrafficLightStatus

This message contains the status and distance of the traffic light.

#### Fields

- `string status`: Traffic light status (RED, GREEN, YELLOW, UNKNOWN).
- `float32 distance`: Distance to the traffic light in meters.

## Launch Files

## Running the Nodes

roslaunch semantic_camera semantic_camera_with_controller.launch

### semantic_camera_with_controller.launch

This launch file starts both the `semantic_camera_node` and `traffic_light_controller` nodes.


```xml
<launch>
  <node name="semantic_camera_node" pkg="semantic_camera" type="semantic_camera_node" output="screen" />
  <node name="traffic_light_controller" pkg="semantic_camera" type="traffic_light_controller" output="screen" />
</launch>

