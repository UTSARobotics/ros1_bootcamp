// // Example from:  https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/
// 
// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// 
// 
// Adafruit_MPU6050 mpu;
// 
// void setup(void) {
// 	Serial.begin(115200);
// 
// 	// Try to initialize!
// 	if (!mpu.begin()) {
// 		Serial.println("Failed to find MPU6050 chip");
// 		while (1) {
// 		  delay(10);
// 		}
// 	}
// 	Serial.println("MPU6050 Found!");
// 
// 	// set accelerometer range to +-8G
// 	mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
// 
// 	// set gyro range to +- 500 deg/s
// 	mpu.setGyroRange(MPU6050_RANGE_500_DEG);
// 
// 	// set filter bandwidth to 21 Hz
// 	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
// 
// 	delay(100);
// }
// 
// void loop() {
// 	/* Get new sensor events with the readings */
// 	sensors_event_t a, g, temp;
// 	mpu.getEvent(&a, &g, &temp);
// 
// 	/* Print out the values */
// 	Serial.print("Acceleration X: ");
// 	Serial.print(a.acceleration.x);
// 	Serial.print(", Y: ");
// 	Serial.print(a.acceleration.y);
// 	Serial.print(", Z: ");
// 	Serial.print(a.acceleration.z);
// 	Serial.println(" m/s^2");
// 
// 	Serial.print("Rotation X: ");
// 	Serial.print(g.gyro.x);
// 	Serial.print(", Y: ");
// 	Serial.print(g.gyro.y);
// 	Serial.print(", Z: ");
// 	Serial.print(g.gyro.z);
// 	Serial.println(" rad/s");
// 
// 	Serial.print("Temperature: ");
// 	Serial.print(temp.temperature);
// 	Serial.println(" degC");
// 
// 	Serial.println("");
// 	delay(500);
// }


// #include <Adafruit_MPU6050.h>
// #include <Adafruit_Sensor.h>
// #include <Wire.h>
// #include <ros.h>
// #include <sensor_msgs/Imu.h>

// Adafruit_MPU6050 mpu;
// ros::NodeHandle nh;
// sensor_msgs::Imu imu_msg;
// ros::Publisher imu_pub("imu", &imu_msg);

// void setup() {
//   nh.getHardware()->setBaud(57600);
//   nh.initNode();
//   nh.advertise(imu_pub);

//   // Try to initialize the MPU6050
//   if (!mpu.begin()) {
//     nh.loginfo("Failed to find MPU6050 chip");
//     while (1) {
//       delay(10);
//     }
//   }

//   mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
//   mpu.setGyroRange(MPU6050_RANGE_500_DEG);
//   mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

//   // Set the IMU message frame_id
//   imu_msg.header.frame_id = "imu_link";
// }

// void loop() {
//   // Get new sensor events with the readings
//   sensors_event_t a, g, temp;

//   mpu.getEvent(&a, &g, &temp);

//   // Populate the IMU message
//   imu_msg.header.stamp = nh.now();
//   imu_msg.linear_acceleration.x = a.acceleration.x;
//   imu_msg.linear_acceleration.y = a.acceleration.y;
//   imu_msg.linear_acceleration.z = a.acceleration.z;
//   imu_msg.angular_velocity.x = g.gyro.x;
//   imu_msg.angular_velocity.y = g.gyro.y;
//   imu_msg.angular_velocity.z = g.gyro.z;

//   // Publish the IMU message
//   imu_pub.publish(&imu_msg);

//   // Process ROS communication
//   nh.spinOnce();

//   // Add a small delay
//   delay(10);
// }

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <MadgwickAHRS.h> // Include Madgwick filter library

Adafruit_MPU6050 mpu;
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);
Madgwick filter; // Create Madgwick filter object
const float sampleFreq = 100.0f; // sample frequency in Hz

void setup() {
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(imu_pub);

  // Try to initialize the MPU6050
  if (!mpu.begin()) {
    nh.loginfo("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Set the IMU message frame_id
  imu_msg.header.frame_id = "imu_link";

  filter.begin(sampleFreq); // Initialize Madgwick filter with sample frequency

}

void loop() {
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Populate the IMU message
  imu_msg.header.stamp = nh.now();
  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;
  imu_msg.angular_velocity.x = g.gyro.x;
  imu_msg.angular_velocity.y = g.gyro.y;
  imu_msg.angular_velocity.z = g.gyro.z;

  // Update Madgwick filter with accelerometer and gyroscope data
  filter.updateIMU(g.gyro.x, g.gyro.y, g.gyro.z, a.acceleration.x, a.acceleration.y, a.acceleration.z);

   float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw = filter.getYaw();
  // Convert roll, pitch, yaw to quaternion
  // https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
  float sinR = sin(roll * 0.5);
  float cosR = cos(roll * 0.5);
  float sinP = sin(pitch * 0.5);
  float cosP = cos(pitch * 0.5);
  float sinY = sin(yaw * 0.5);
  float cosY = cos(yaw * 0.5);

  imu_msg.orientation.w = cosR * cosP * cosY + sinR * sinP * sinY;
  imu_msg.orientation.x = sinR * cosP * cosY - cosR * sinP * sinY;
  imu_msg.orientation.y = cosR * sinP * cosY + sinR * cosP * sinY;
  imu_msg.orientation.z = cosR * cosP * sinY - sinR * sinP * cosY;

  // Publish the IMU message
  imu_pub.publish(&imu_msg);

  // Process ROS communication
  nh.spinOnce();

  // Add a small delay
  delay(10);
}

