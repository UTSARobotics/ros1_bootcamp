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
  // The laserScanCallback function processes incoming laser scan data
  // provided in a shared pointer to a constant LaserScan message.
  //
  // The LaserScan message contains an array of range measurements (in meters)
  // that represent the distance to obstacles at different angles.
  // The array is indexed based on the angle, starting from angle_min and
  // increasing by angle_increment up to angle_max.

  // Initialize a variable to store the smallest distance measurement
  // Set its initial value to the maximum possible float value
  float min_range = std::numeric_limits<float>::max();

  // Iterate through the received laser scan ranges
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    // Check if the current range measurement is smaller than the
    // previously stored minimum range
    if (msg->ranges[i] < min_range) {
      // If so, update the minimum range with the current range measurement
      min_range = msg->ranges[i];
    }
  }

  // After iterating through all the range measurements, the min_range variable
  // contains the smallest distance measurement found.
  // Print the smallest distance measurement to the terminal
  ROS_INFO("Smallest distance measurement: %f", min_range);
}

int main(int argc, char** argv) 
{
  // Initialize the ROS node
  ros::init(argc, argv, "laser_scan_subscriber");

  // Create a NodeHandle to manage communication with the ROS system
  ros::NodeHandle nh;

  /*
	  // Subscribe to the "/scan" topic with a queue size of 1000 and
	  // provide a callback function to process the incoming messages
	  ros::Subscriber sub = nh.subscribe("/scan", 1000, laserScanCallback);	
   */

  // Read the topic name and queue size from the parameter server
  std::string scan_topic;
  int queue_size;
  nh.param<std::string>("laser_scan_subscriber/scan_topic", scan_topic, "/scan");
  nh.param<int>("laser_scan_subscriber/queue_size", queue_size, 1000);

  // Subscribe to the specified topic with the specified queue size and
  // provide a callback function to process the incoming messages
  ros::Subscriber sub = nh.subscribe(scan_topic, queue_size, laserScanCallback);
  // Spin in a loop and call the laserScanCallback function when new messages arrive
  ros::spin();

  return 0;
}
