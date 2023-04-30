#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

/* The laserScanCallback function processes incoming laser scan data.
 The input parameter is a shared pointer to a constant LaserScan message.
 The LaserScan message type is part of the sensor_msgs package.

 LaserScan message structure:
 - Header header
 - float32 angle_min
 - float32 angle_max
 - float32 angle_increment
 - float32 time_increment
 - float32 scan_time
 - float32 range_min
 - float32 range_max
 - float32[] ranges
 - float32[] intensities

 Documentation: http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
*/
void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  // Print the entire laser scan ranges received
  ROS_INFO("Laser scan ranges received:");

  for (size_t i = 0; i < msg->ranges.size(); i++) {
    ROS_INFO("Range at index [%zu]: %f", i, msg->ranges[i]);
  }
}

int main(int argc, char** argv) {
  // Initialize the ROS node
  ros::init(argc, argv, "laser_scan_subscriber");

  // Create a NodeHandle to manage communication with the ROS system
  ros::NodeHandle nh;

  // Subscribe to the "/scan" topic with a queue size of 1000 and
  // provide a callback function to process the incoming messages
  ros::Subscriber sub = nh.subscribe("/scan", 1000, laserScanCallback);

  // Spin in a loop and call the laserScanCallback function when new messages arrive
  ros::spin();

  return 0;
}
