/*
 * Automatic Addison
 * Date: May 20, 2021
 * ROS Version: ROS 1 - Melodic
 * Website: https://automaticaddison.com
 * Publishes odometry information for use with robot_pose_ekf package.
 *   This odometry information is based on wheel encoder tick counts.
 * Subscribe: ROS node that subscribes to the following topics:
 *  right_ticks : Tick counts from the right motor encoder (std_msgs/Int16)
 * 
 *  left_ticks : Tick counts from the left motor encoder  (std_msgs/Int16)
 * 
 *  initial_2d : The initial position and orientation of the robot.
 *               (geometry_msgs/PoseStamped)
 *
 * Publish: This node will publish to the following topics:
 *  odom_data_euler : Position and velocity estimate. The orientation.z 
 *                    variable is an Euler angle representing the yaw angle.
 *                    (nav_msgs/Odometry)
 *  odom_data_quat : Position and velocity estimate. The orientation is 
 *                   in quaternion format.
 *                   (nav_msgs/Odometry)
 * Modified from Practical Robotics in C++ book (ISBN-10 : 9389423465)
 *   by Lloyd Brombach
 */
 
// Include various libraries
#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

// Create odometry data publishers
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;
 
// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;
 
// Robot physical constants
const double TICKS_PER_REVOLUTION = 102; // For reference purposes.
const double WHEEL_RADIUS = 0.04; // Wheel radius in meters
const double WHEEL_BASE = 0.218; // Center of front axle to center of rear axle
const double TICKS_PER_METER = 406; // Original was 2800
 
// Distance all four wheels have traveled
double distanceLeft1 = 0;
double distanceRight1 = 0;
double distanceLeft2 = 0;
double distanceRight2 = 0;
 
// Flag to see if initial pose has been received
bool initialPoseRecieved = false;
 
using namespace std;
 
// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
 
  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
}

// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left1(const std_msgs::Int16& left1_wheel_tick_count) {
 
  static int lastCountL1 = 0;
  if(left1_wheel_tick_count.data != 0 && lastCountL1 != 0) {
         
    int left1Ticks = (left1_wheel_tick_count.data - lastCountL1);
 
    if (left1Ticks > 10000) {
      left1Ticks = 0 - (65535 - left1Ticks);
    }
    else if (left1Ticks < -10000) {
      left1Ticks = 65535-left1Ticks;
    }
    else{}
    distanceLeft1 = left1Ticks/TICKS_PER_METER;
  }
  lastCountL1 = left1_wheel_tick_count.data;
}

// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right1(const std_msgs::Int16& right1_wheel_tick_count) {
   
  static int lastCountR1 = 0;
  if(right1_wheel_tick_count.data && lastCountR1 != 0) {
 
    int right1Ticks = right1_wheel_tick_count.data - lastCountR1;
     
    if (right1Ticks > 10000) {
      distanceRight1= (0 - (65535 - distanceRight1))/TICKS_PER_METER;
    }
    else if (right1Ticks < -10000) {
      right1Ticks = 65535 - right1Ticks;
    }
    else{}
    distanceRight1 = right1Ticks/TICKS_PER_METER;
  }
  lastCountR1 = right1_wheel_tick_count.data;
}

// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left2(const std_msgs::Int16& left2_wheel_tick_count) {
 
  static int lastCountL2 = 0;
  if(left2_wheel_tick_count.data != 0 && lastCountL2 != 0) {
         
    int left2Ticks = (left2_wheel_tick_count.data - lastCountL2);
 
    if (left2Ticks > 10000) {
      left2Ticks = 0 - (65535 - left2Ticks);
    }
    else if (left2Ticks < -10000) {
      left2Ticks = 65535-left2Ticks;
    }
    else{}
    distanceLeft2 = left2Ticks/TICKS_PER_METER;
  }
  lastCountL2 = left2_wheel_tick_count.data;
}
 
// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right2(const std_msgs::Int16& right2_wheel_tick_count) {
   
  static int lastCountR2 = 0;
  if(right2_wheel_tick_count.data != 0 && lastCountR2 != 0) {
 
    int right2Ticks = right2_wheel_tick_count.data - lastCountR2;
     
    if (right2Ticks > 10000) {
      distanceRight2 = (0 - (65535 - distanceRight2))/TICKS_PER_METER;
    }
    else if (right2Ticks < -10000) {
      right2Ticks = 65535 - right2Ticks;
    }
    else{}
    distanceRight2 = right2Ticks/TICKS_PER_METER;
  }
  lastCountR2 = right2_wheel_tick_count.data;
}

// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat() {
 
  tf::Quaternion q;
         
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
 
  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
 
  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }
 
  odom_data_pub_quat.publish(quatOdom);
}
 
void update_odom() {
 
  // Calculate the average distance
  double cycleDistanceL = (distanceLeft1 + distanceLeft2) / 2;
  double cycleDistanceR = (distanceRight1 + distanceRight2) / 2;
  double cycleDistance = (cycleDistanceL + cycleDistanceR) / 2;
   
  // Calculate the number of radians the robot has turned since the last cycle
  double delta = (cycleDistanceR - cycleDistanceL);
  double cycleAngle = atan2(delta, WHEEL_BASE); 
  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;
     
  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}
 
  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;
 
  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }
 
  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
  else{}
 
  // Compute the velocity
  odomNew.header.stamp = ros::Time::now();
  odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec()); 

  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;
 
  // Publish the odometry message
  odom_data_pub.publish(odomNew);
}

int main(int argc, char **argv) {
   
  // Set the data fields of the odometry message
  odomNew.header.frame_id = "odom";
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.child_frame_id = "base_link";
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;
 
  // Launch ROS and create a node
  ros::init(argc, argv, "ekf_odom_pub");
  ros::NodeHandle node;
 
  // Subscribe to ROS topics
  ros::Subscriber subForright1_wheel_tick_count = node.subscribe("right1_ticks", 100, Calc_Right1, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForleft1_wheel_tick_count = node.subscribe("left1_ticks", 100, Calc_Left1, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForright2_wheel_tick_count = node.subscribe("right2_ticks", 100, Calc_Right2, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForleft2_wheel_tick_count = node.subscribe("left2_ticks", 100, Calc_Left2, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);
 
  // Publisher of simple odom message where orientation.z is an euler angle
  odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_data_euler", 100);
 
  // Publisher of full odom message where orientation is quaternion
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100);
 
  ros::Rate loop_rate(30); 
     
  while(ros::ok()) {
     
    if(initialPoseRecieved) {
      update_odom();
      publish_quat();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
 
  return 0;
}
