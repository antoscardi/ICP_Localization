#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "icp/eigen_kdtree.h"
#include "localizer2d.h"
#include "map.h"
#include "ros_bridge.h"

using namespace std; // aggiunto da me 

// Map callback definition
void callback_map(const nav_msgs::OccupancyGridConstPtr&);
// Initial pose callback definition
void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr&);
// Scan callback definition
void callback_scan(const sensor_msgs::LaserScanConstPtr&);

shared_ptr<Map> map_ptr = nullptr;  // modificato linea del prof, tolto std::
ros::Publisher pub_scan, pub_odom;
ros::Subscriber map_sub, init_pos_sub, base_scan_sub; // Aggiunte da me 

Localizer2D localizer;

int main(int argc, char** argv) {
  // TODO Initialize ROS system
  ros::init(argc, argv, "localizer_node");

  // TODO Create a NodeHandle to manage the node.
  // The namespace of the node is set to global
  ros::NodeHandle nh("/node_localizer");  //VEDI SE TOGLIERE IL NOME

  // TODO Create shared pointer for the Map object
  map_ptr = make_shared<Map>();

  // TODO Subscribe to the topics and assign the correct callbacks:
  // /map [nav_msgs::OccupancyGrid]
  map_sub = nh.subscribe("/map", 10, callback_map);
  // /initialpose [geometry_msgs::PoseWithCovarianceStamped]
  init_pos_sub = nh.subscribe("/initialpose", 10, callback_initialpose);
  // /base_scan [sensor_msgs::LaserScan]
  base_scan_sub = nh.subscribe("/base_scan", 10, callback_scan);
  // Advertise the following topic: /odom_out [nav_msgs::Odometry]
  pub_odom = nh.advertise<nav_msgs::Odometry>("/odom_out", 10);

  // Scan advertiser for visualization purposes
  pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_out", 10);

  ROS_INFO("Node started. Waiting for input data");

  // Spin the node
  ros::spin();

  return 0;
}

void callback_map(const nav_msgs::OccupancyGridConstPtr& msg_) {
  /** TODO
   * If the internal map is not initialized, load the incoming occupancyGrid and
   * set the localizer map accordingly
   * Remember to load the map only once during the execution of the map.
  */
  if(!(map_ptr->initialized())){
    ROS_INFO("Initialized?: %s", map_ptr->initialized() ? "true" : "false");
    // load the incoming occupancyGrid
    map_ptr->loadOccupancyGrid(msg_);
    ROS_INFO("The map has been loaded correctly. Initialized? %s", map_ptr->initialized() ? "true" : "false");
    // set the localizer map accordingly
    localizer.setMap(map_ptr);
  }
}

void callback_initialpose(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg_) {
  /** TODO (all function)
   * Convert the PoseWithCovarianceStamped message to an Eigen Isometry and
   * inform the localizer.
   * You can check ros_bridge.h for helps :)
   */

  // Pose info is stored in the ptr as geometry_msgs::Pose.
  geometry_msgs::Pose pose_ = msg_->pose.pose;

  // Print pose information.
  ROS_INFO("Received InitialPose: Posit (x=%.2f, y=%.2f), Orient Quatern (w=%.2f, x=%.2f, y=%.2f, z=%.2f)",
         pose_.position.x, pose_.position.y,
         pose_.orientation.w, pose_.orientation.x, pose_.orientation.y, pose_.orientation.z);
  
  //Define an Isometry Transform class to store the converted message
  Eigen::Isometry2f iso_initial_pose;
  //Convert the PoseWithCovarianceStamped message to an Eigen Isometry
  pose2isometry( pose_, iso_initial_pose);

  // Print result of conversion
  ROS_INFO("Converted Initial Pose to Isometry:\n%41s   | %10s\n%36.2f %2.2f %10.2f\n%36.2f %2.2f %10.2f",
          "Rotation:", "Translation:",
          iso_initial_pose.linear()(0, 0), iso_initial_pose.linear()(0, 1),
          iso_initial_pose.translation()(0),
          iso_initial_pose.linear()(1, 0), iso_initial_pose.linear()(1, 1),
          iso_initial_pose.translation()(1));

  // Inform the localizer: store info in the localizer class.
  localizer.setInitialPose(iso_initial_pose);
}

void callback_scan(const sensor_msgs::LaserScanConstPtr& msg_) {
  // Print the laser scan to check 
  ROS_INFO("SCAN RECEIVED at time %.2f: min %.2f, max %.2f, angle increment %.2f, angle min %.2f , angle max %.2f", 
           msg_->header.stamp.toSec(), msg_->range_min, msg_->range_max, msg_->angle_increment, msg_->angle_min, msg_->angle_max);

  /** TODO
   * Convert the LaserScan message into a Localizer2D::ContainerType
   * [std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>]
   */
  // Initialize the converted message.
  Localizer2D::ContainerType new_scan;
  // Convert the LaserScan message into a Localizer2D::ContainerType
  scan2eigen(msg_, new_scan);
  

  /** TODO
   * Set the laser parameters and process the incoming scan through the
   * localizer
   */
  // Set the laser parameters in the localizer
  localizer.setLaserParams(msg_->range_min,msg_->range_max,msg_->angle_min,msg_->angle_max, msg_->angle_increment);
  // Process the incoming scan  
  localizer.process(new_scan);

  /**  TODO
   * Send a transform message between FRAME_WORLD and FRAME_LASER.
   * The transform should contain the pose of the laser with respect to the map
   * frame.
   * You can use the isometry2transformStamped function to convert the isometry
   * to the right message type.
   * Look at include/constants.h for frame names
   *
   * The timestamp of the message should be equal to the timestamp of the
   * received message (msg_->header.stamp)
   */
  // Get pose of the laser with respect to the map frame
  Eigen::Isometry2f current_estimate = localizer.X();
  ROS_INFO("Current estimate:\n%41s   | %10s\n%36.2f %2.2f %10.2f\n%36.2f %2.2f %10.2f",
          "Rotation:", "Translation:",
          current_estimate.linear()(0, 0), current_estimate.linear()(0, 1),
          current_estimate.translation()(0),
          current_estimate.linear()(1, 0), current_estimate.linear()(1, 1),
          current_estimate.translation()(1));

  // Initialized TransformStamped message
  geometry_msgs::TransformStamped transformStamped_msg;

  // Convert isometry to transformStamped type
  isometry2transformStamped(current_estimate, transformStamped_msg, FRAME_WORLD, FRAME_LASER, msg_->header.stamp);

  static tf2_ros::TransformBroadcaster br;

   // Broadcast the transform
  br.sendTransform(transformStamped_msg); 
  // Print information to the console
  ROS_INFO("Broadcasted transform from %s frame to %s frame at time %.2f",
          transformStamped_msg.header.frame_id.c_str(),
          transformStamped_msg.child_frame_id.c_str(),
          transformStamped_msg.header.stamp.toSec());
          
  /** TODO
   * Send a nav_msgs::Odometry message containing the current laser_in_world
   * transform.
   * You can use transformStamped2odometry to convert the previously computed
   * TransformStamped message to a nav_msgs::Odometry message.
   */
  //
  // Create odometry message
  nav_msgs::Odometry odometry_msg;
  // You can use transformStamped2odometry to convert the previously computed TransformStamped message to a nav_msgs::Odometry message.
  transformStamped2odometry(transformStamped_msg,odometry_msg);
  //Send a nav_msgs::Odometry message containing the current laser_in_world transform.
  pub_odom.publish(odometry_msg);

  // Sends a copy of msg_ with FRAME_LASER set as frame_id
  // Used to visualize the scan attached to the current laser estimate.
  sensor_msgs::LaserScan out_scan = *msg_;
  out_scan.header.frame_id = FRAME_LASER;
  pub_scan.publish(out_scan);
}