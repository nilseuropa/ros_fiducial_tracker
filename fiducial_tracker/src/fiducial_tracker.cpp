#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialArray.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"

#include "ros_fiducial_tracker/resetOrigin.h"

ros::Publisher marker_pub;
ros::Publisher odom_pub;

std::string origin_frame_id, camera_frame_id, robot_frame_id;
std::string node_ns;

int origin_id = 0;
int robot_id  = 1;
bool use_ground_marker = false;
bool publish_visualisation_markers = false;
bool publish_fiducial_tf = true;

geometry_msgs::Transform roboTransform;
geometry_msgs::Transform roboTransformOrigo;
geometry_msgs::Transform odomTransform;
geometry_msgs::Transform robotDisplacementTransform;

tf::Transform robot_TF;
tf::Transform robot_TF_prev;
tf::Transform robot_TF_origo;

tf2_ros::TransformBroadcaster* broadcaster = NULL;

ros::Time twist_timestamp;

bool originHasBeenSet = false;
bool resetRobotOrigin(ros_fiducial_tracker::resetOrigin::Request  &req,
                      ros_fiducial_tracker::resetOrigin::Response &res) {
  roboTransformOrigo = roboTransform;
  res.result = originHasBeenSet = req.resetOrigin;
  if (originHasBeenSet) ROS_INFO("Origin set.");
  else ROS_INFO("Origin dropped.");
  return true;
}

void update_messages(ros::Time message_time)
{

  if (originHasBeenSet)
  {

    nav_msgs::Odometry odom;
    visualization_msgs::Marker points;

    tf::transformMsgToTF(roboTransform, robot_TF);
    tf::Transform robot_displacement = robot_TF.inverseTimes(robot_TF_prev);
    robot_TF_prev = robot_TF;
    tf::transformMsgToTF(roboTransformOrigo, robot_TF_origo);
    tf::Transform dislocation = robot_TF_origo.inverseTimes(robot_TF);
    tf::transformTFToMsg(dislocation, odomTransform);
    tf::transformTFToMsg(robot_displacement, robotDisplacementTransform);

    if (publish_visualisation_markers)
    {
      points.header.frame_id = origin_frame_id;
      points.header.stamp    = message_time;
      points.ns = node_ns;
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
      marker_pub.publish(points);
    }

    double twist_dt_sec = (message_time-twist_timestamp).toSec();
    twist_timestamp = message_time;

    odom.header.frame_id = origin_frame_id;
    odom.header.stamp = message_time;
    odom.child_frame_id = robot_frame_id; // !!!
    odom.pose.pose.position.x  = odomTransform.translation.x;
    odom.pose.pose.position.y  = odomTransform.translation.y;
    odom.pose.pose.position.z  = odomTransform.translation.z;
    odom.pose.pose.orientation = odomTransform.rotation;
    odom.twist.twist.linear.x  = robotDisplacementTransform.translation.x / twist_dt_sec;
    odom.twist.twist.linear.y  = robotDisplacementTransform.translation.y / twist_dt_sec;
    odom.twist.twist.linear.z  = robotDisplacementTransform.translation.z / twist_dt_sec;
    odom_pub.publish(odom);

    if (publish_fiducial_tf)
    {
      geometry_msgs::TransformStamped transformStamped;
    	transformStamped.header.stamp = message_time;
    	transformStamped.header.frame_id = camera_frame_id;
    	transformStamped.child_frame_id = robot_frame_id;
    	transformStamped.transform.translation.x = roboTransform.translation.x;
    	transformStamped.transform.translation.y = roboTransform.translation.y;
    	transformStamped.transform.translation.z = roboTransform.translation.z;
    	transformStamped.transform.rotation = roboTransform.rotation;
    	broadcaster->sendTransform(transformStamped);
    	transformStamped.child_frame_id = origin_frame_id;
    	transformStamped.transform.translation.x = roboTransformOrigo.translation.x;
    	transformStamped.transform.translation.y = roboTransformOrigo.translation.y;
    	transformStamped.transform.translation.z = roboTransformOrigo.translation.z;
    	transformStamped.transform.rotation = roboTransformOrigo.rotation;
    	broadcaster->sendTransform(transformStamped);
    }

  }
}

void fiducialTransFormArrayUpdate(const fiducial_msgs::FiducialTransformArray &ftf_array)
{
  auto array_length = std::size(ftf_array.transforms);
  int origin_idx = -1;
  int robot_idx = -1;
  if (array_length > 0) {
    // Check for ground and robot markers
    for (uint8_t i = 0; i < array_length; i++) {
      if (ftf_array.transforms[i].fiducial_id == origin_id) {
        origin_idx = i;
      } else if (ftf_array.transforms[i].fiducial_id == robot_id) {
        robot_idx = i;
      }
    }
    // If ground marker is present, update its transform
    bool update_required = false;
    if (origin_idx >= 0) {
      roboTransformOrigo = ftf_array.transforms[origin_idx].transform;
      update_required = true;
      if (!originHasBeenSet) {
        originHasBeenSet = true;
        ROS_INFO_STREAM("Origin set using ground marker (ID = " << origin_id << ")");
      }
    }
    // If robot marker is present, update its transform and publish updated messages
    if (robot_idx >= 0) {
      update_required = true;
      roboTransform = ftf_array.transforms[robot_idx].transform;
    }
    if (update_required) update_messages(ftf_array.header.stamp);
  }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "fiducial_tracker");
    node_ns = ros::this_node::getName();

    ros::NodeHandle n;
    ros::NodeHandle nhLocal("~");
    broadcaster = new(tf2_ros::TransformBroadcaster);

    nhLocal.param("origin_id", origin_id, 0);
    nhLocal.param("robot_id", robot_id, 1);
    nhLocal.param("odom_frame", origin_frame_id, std::string("origin"));
    nhLocal.param("camera_frame", camera_frame_id, std::string("camera"));
    nhLocal.param("robot_frame", robot_frame_id, std::string("robot"));
    nhLocal.param("publish_markers", publish_visualisation_markers, false);
    nhLocal.param("publish_fiducial_tf", publish_fiducial_tf, true);

    ros::Subscriber ftf_sub    = n.subscribe("/fiducial_transforms", 100, fiducialTransFormArrayUpdate);
    ros::ServiceServer service = n.advertiseService("/fiducial_tracker/reset_origin", resetRobotOrigin);

    if (publish_visualisation_markers) marker_pub = n.advertise<visualization_msgs::Marker>("/fiducial_markers", 10);
    odom_pub = n.advertise<nav_msgs::Odometry>("/fiducial_tracker/odom", 50);

    twist_timestamp = ros::Time::now();
    ROS_INFO("Waiting for robot origin or ground marker to appear...");
    ros::spin();
    return 0;
}
