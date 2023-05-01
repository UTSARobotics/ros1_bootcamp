#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// Global publisher for publishing twist commands to /cmd_vel topic
ros::Publisher cmd_vel_pub;

// P-controller gain
const double Kp = 1.0;
const double min_distance_threshold = 0.01; // Set the threshold as needed

void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
  // Find the index of the smallest distance measurement
  size_t min_range_idx = std::distance(
      msg->ranges.begin(),
      std::min_element(msg->ranges.begin(), msg->ranges.end()));

  // Calculate the angle corresponding to the minimum range index
  double angle_to_pillar = msg->angle_min + min_range_idx * msg->angle_increment;
  geometry_msgs::Twist twist;
  if (msg->ranges[min_range_idx] <= min_distance_threshold) 
  {
    // Stop the robot if it's close enough to the pillar
    twist.linear.x = 0;
    twist.angular.z = 0;
  } 
  
  else {
  // Calculate the linear and angular velocities using the P-controller
  twist.linear.x = msg->ranges[min_range_idx] * cos(angle_to_pillar) * Kp;
  twist.angular.z = angle_to_pillar * Kp;
  }
  
  // Publish the twist command to the /cmd_vel topic
  cmd_vel_pub.publish(twist);
}

int main(int argc, char** argv) 
{
  // Initialize the ROS node
  ros::init(argc, argv, "smb_highlevel_controller");

  // Create a NodeHandle to manage communication with the ROS system
  ros::NodeHandle nh;

  // Read the topic name and queue size from the parameter server
  std::string scan_topic;
  int queue_size;
  nh.param<std::string>("laser_scan_subscriber/scan_topic", scan_topic, "/scan");
  nh.param<int>("laser_scan_subscriber/queue_size", queue_size, 1000);

  // Subscribe to the specified topic with the specified queue size and
  // provide a callback function to process the incoming messages
  ros::Subscriber sub = nh.subscribe(scan_topic, queue_size, laserScanCallback);

  // Create a publisher to publish twist commands to the /cmd_vel topic
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Spin in a loop and call the laserScanCallback function when new messages arrive
  ros::spin();

  return 0;
}
