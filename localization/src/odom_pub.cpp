#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
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
// mobile robot´s basic values
const double TICKS_PER_REVOLUTION = 102;
const double WHEEL_RADIUS = 0.04;
const double WHEEL_BASE = 0.218;
const double TICKS_PER_METER = 406;
double distanceFrontLeft = 0;
double distanceFrontRight = 0;
double distanceRearLeft = 0;
double distanceRearRight = 0;
// To check if the initial position has been received
bool initialPoseRecieved = false;
using namespace std;
// Get initial_2d message from Rviz clicks
void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) {
  odomOld.pose.pose.position.x = rvizClick.pose.position.x;
  odomOld.pose.pose.position.y = rvizClick.pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
  initialPoseRecieved = true;
}
// calculate distance covered by all four wheels 
void Calc_FrontLeft(const std_msgs::Int16& FrontLeftCount) {
  static int lastCountFrontLeft = 0;
  if(FrontLeftCount.data != 0 && lastCountFrontLeft != 0) {     
    int frontleftTicks = (FrontLeftCount.data - lastCountFrontLeft);
    if (frontleftTicks > 10000) {
      frontleftTicks = 0 - (65535 - frontleftTicks);
    }
    else if (frontleftTicks < -10000) {
      frontleftTicks = 65535-frontleftTicks;
    }
    else{}
    distanceFrontLeft = frontleftTicks/TICKS_PER_METER;
  }
  lastCountFrontLeft = FrontLeftCount.data;
}
void Calc_FrontRight(const std_msgs::Int16& FrontRightCount) { 
  static int lastCountFrontRight = 0;
  if(FrontRightCount.data && lastCountFrontRight != 0) {
    int frontrightTicks = FrontRightCount.data - lastCountFrontRight; 
    if (frontrightTicks > 10000) {
      distanceFrontRight= (0 - (65535 - distanceFrontRight))/TICKS_PER_METER;
    }
    else if (frontrightTicks < -10000) {
      frontrightTicks = 65535 - frontrightTicks;
    }
    else{}
    distanceFrontRight = frontrightTicks/TICKS_PER_METER;
  }
  lastCountFrontRight = FrontRightCount.data;
}
void Calc_RearLeft(const std_msgs::Int16& RearLeftCount) {
  static int lastCountRearLeft = 0;
  if(RearLeftCount.data != 0 && lastCountRearLeft != 0) {
    int rearleftTicks = (RearLeftCount.data - lastCountRearLeft);
    if (rearleftTicks > 10000) {
      rearleftTicks = 0 - (65535 - rearleftTicks);
    }
    else if (rearleftTicks < -10000) {
      rearleftTicks = 65535-rearleftTicks;
    }
    else{}
    distanceRearLeft = rearleftTicks/TICKS_PER_METER;
  }
  lastCountRearLeft = RearLeftCount.data;
}
void Calc_RearRight(const std_msgs::Int16& RearRightCount) {
  static int lastCountRearRight = 0;
  if(RearRightCount.data != 0 && lastCountRearRight != 0) {
    int rearrightTicks = RearRightCount.data - lastCountRearRight;
    if (rearrightTicks > 10000) {
      distanceRearRight = (0 - (65535 - distanceRearRight))/TICKS_PER_METER;
    }
    else if (rearrightTicks < -10000) {
      rearrightTicks = 65535 - rearrightTicks;
    }
    else{}
    distanceRearRight = rearrightTicks/TICKS_PER_METER;
  }
  lastCountRearRight = RearRightCount.data;
}
// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat() {
  tf2::Quaternion q;
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
  double cycleDistanceLeft = (distanceFrontLeft + distanceRearLeft) / 2;
  double cycleDistanceRear = (distanceFrontRight + distanceRearRight) / 2;
  double cycleDistance = (cycleDistanceLeft + cycleDistanceRear) / 2;
  // Calculate the number of radians the robot has turned
  double delta = (cycleDistanceRear - cycleDistanceLeft);
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
  // theta stays in the correct range
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
  ros::Subscriber subForFrontRightCount = node.subscribe("frontright_ticks", 100, Calc_FrontRight, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForFrontLeftCount = node.subscribe("frontleft_ticks", 100, Calc_FrontLeft, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForRearRightCount = node.subscribe("rearright_ticks", 100, Calc_RearRight, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForRearLeftCount = node.subscribe("rearleft_ticks", 100, Calc_RearLeft, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);
  // Publisher of simple odom message where orientation.z is an euler angle
  odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_data_euler", 100);
  // Publisher of full odom message where orientation is quaternion
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100);
  ros::Rate loop_rate(10); 
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
