#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialArray.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"

#include "ros_fiducial_tracker/resetOrigin.h"

ros::Publisher marker_pub;
ros::Publisher odom_pub;

int robot_id  = 0;
int ground_id = 1;
bool use_ground_marker = false;
std::string odom_frame_id;

geometry_msgs::Transform roboTransform;
geometry_msgs::Transform roboTransformOrigo;
geometry_msgs::Transform odomTransform;

tf::Transform robot_TF;
tf::Transform robot_TF_origo;

bool originHasBeenSet = false;
bool resetRobotOrigin(ros_fiducial_tracker::resetOrigin::Request  &req,
                      ros_fiducial_tracker::resetOrigin::Response &res) {
  roboTransformOrigo = roboTransform;
  res.result = originHasBeenSet = req.resetOrigin;
  if (originHasBeenSet) ROS_INFO("Origin set.");
  else ROS_INFO("Origin dropped.");
  return true;
}

void update_messages() {
  if (originHasBeenSet) {
    tf::transformMsgToTF(roboTransform, robot_TF);
    tf::transformMsgToTF(roboTransformOrigo, robot_TF_origo);
    tf::Transform dislocation = robot_TF_origo.inverseTimes(robot_TF);
    tf::transformTFToMsg(dislocation, odomTransform);
    
    //double robotHeading = tf::getYaw(odomTransform.rotation);
    //tf::quaternionTFToMsg(tf::createQuaternionFromYaw(robotHeading),odomTransform.rotation);

    // Visualization marker
    visualization_msgs::Marker points;
    points.header.frame_id = "odom";
    points.header.stamp    = ros::Time::now();
    points.ns = "fiducial_tracker";
    points.action = visualization_msgs::Marker::ADD;
    points.id = robot_id;
    points.pose.orientation = odomTransform.rotation;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.1;
    points.scale.y = 0.1;
    points.color.g = 1.0f;
    points.color.a = 1.0f;
    geometry_msgs::Point p;
    p.x = odomTransform.translation.x;
    p.y = odomTransform.translation.y;
    p.z = odomTransform.translation.z;
    points.points.push_back(p);
    
    // Odometry
    nav_msgs::Odometry odom;
    odom.child_frame_id = odom_frame_id;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x  = odomTransform.translation.x;
    odom.pose.pose.position.y  = odomTransform.translation.y;
    odom.pose.pose.position.z  = odomTransform.translation.z;
    odom.pose.pose.orientation = odomTransform.rotation;
    // TODO: calculate velocities

    // Publish messages
    marker_pub.publish(points);
    odom_pub.publish(odom);
  }
}

void fiducialTransFormArrayUpdate(const fiducial_msgs::FiducialTransformArray &ftf_array){
  auto array_length = std::size(ftf_array.transforms);
  int ground_idx = -1;
  int robot_idx = -1;
  if (array_length > 0) {
    // Check for ground and robot markers
    for (uint8_t i = 0; i < array_length; i++) {  
      if (ftf_array.transforms[i].fiducial_id == ground_id) {
        ground_idx = i;
      } else if (ftf_array.transforms[i].fiducial_id == robot_id) {
        robot_idx = i;
      }
    }
    // If ground marker is present, update its transform
    if (ground_idx >= 0) {
      roboTransformOrigo = ftf_array.transforms[ground_idx].transform;
      if (!originHasBeenSet) {
        originHasBeenSet = true;
        ROS_INFO_STREAM("Origin set using ground marker (ID = " << ground_id << ")");
      }
    }
    // If robot marker is present, update its transform and publish updated messages
    if (robot_idx >= 0) {
      ROS_INFO("Robot marker is present");
      roboTransform = ftf_array.transforms[robot_idx].transform;
      update_messages();
    }
  }
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "fiducial_tracker");
    ros::NodeHandle n;
    ros::Subscriber ftf_sub    = n.subscribe("/fiducial_transforms", 100, fiducialTransFormArrayUpdate);
    ros::ServiceServer service = n.advertiseService("/fiducial_tracker/reset_origin", resetRobotOrigin);
    odom_pub = n.advertise<nav_msgs::Odometry>("fiducial_tracker/odom", 50);
    marker_pub = n.advertise<visualization_msgs::Marker>("fiducial_markers", 10);

    if (ros::param::get("~robot_id", robot_id)) {}
    else {
      ROS_WARN("Failed to get param 'robot_id' - default: 0");
      robot_id = 0;
    }
    
    if (ros::param::get("~ground_id", ground_id)) {}
    else {
      ROS_WARN("Failed to get param 'ground_id' - default: 1");
      ground_id = 1;
    }

    if (ros::param::get("~odom_frame_id", odom_frame_id)) {}
    else {
      ROS_WARN("Failed to get param 'odom_frame_id' - default: base_link");
      odom_frame_id = "base_link";
    }

    ROS_INFO("Waiting for robot origin or ground marker to appear...");
    ros::spin();
    return 0;
}
