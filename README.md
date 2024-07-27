# Semantic Camera ROS Package

This package contains a semantic camera node that generates semantic images, detects traffic lights (including yellow lights), calculates the distance to the traffic light using depth camera data, and publishes the information as a custom message type. A traffic light controller node subscribes to this custom message type to adjust the vehicle's behavior based on the traffic light status and distance.

## Nodes

### `semantic_camera_node`

This node simulates a semantic camera that generates semantic images, detects traffic lights (red, green, yellow), and calculates the distance to the traffic light using depth camera data.

**Published Topics:**
- `/traffic_light_status` (`semantic_camera/TrafficLightStatus`): The detected traffic light status and distance.

**Subscribed Topics:**
- `/unity_ros/OurCar/Sensors/SemanticCamera/image_raw` (`sensor_msgs/Image`): The semantic camera image.
- `/unity_ros/OurCar/Sensors/DepthCamera/image_raw` (`sensor_msgs/Image`): The depth camera image.

**Custom Message:**
- `TrafficLightStatus.msg`
  ```plaintext
  string status  # Traffic light status (RED, GREEN, YELLOW, UNKNOWN)
  float32 distance  # Distance to the traffic light in meters. If no traffic light is detected, distance is -1.0

