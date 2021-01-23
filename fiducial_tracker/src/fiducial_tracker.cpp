#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include "fiducial_msgs/Fiducial.h"
#include "fiducial_msgs/FiducialArray.h"
#include "fiducial_msgs/FiducialTransform.h"
#include "fiducial_msgs/FiducialTransformArray.h"

#include "ros_fiducial_tracker/resetOrigin.h"

ros::Publisher marker_pub;
ros::Publisher odom_pub;

std::string origin_frame_id, camera_frame_id, marker_frame_id, footprint_frame_id;
std::string node_ns;

int origin_id = 0;
int robot_id  = 1;

bool publish_fiducial_tf = true;
bool publish_odom_tf = false;
bool publish_odometry = true;
bool marker_to_footprint_from_urdf = false;

geometry_msgs::Transform markerToFootprint;
geometry_msgs::Transform markerToCamera;
geometry_msgs::Transform markerToCameraPrev;
geometry_msgs::Transform transformOrigin;
geometry_msgs::Transform odomTransform;
geometry_msgs::Transform robotDisplacementTransform;

tf::Transform marker_to_cam;
tf::Transform marker_to_cam_prev;
tf::Transform transform_origin_tf;
tf::Transform marker_to_footprint_tf;

tf2_ros::TransformBroadcaster* tf_broadcaster = NULL;

ros::Time twist_timestamp;

bool trackingEnabled  = false;
bool originNeedsUpdate = false;

bool resetRobotOrigin(ros_fiducial_tracker::resetOrigin::Request  &req,
                      ros_fiducial_tracker::resetOrigin::Response &res)
{
  if (req.resetOrigin)
  {
    originNeedsUpdate = true;
    ROS_INFO("Origin reset requested.");
  }
  else {
    ROS_INFO("Origin dropped. Tracking disabled.");
    trackingEnabled = false;
  }
  return true;
}

geometry_msgs::Vector3 update_angular_velocity(geometry_msgs::Quaternion q, geometry_msgs::Quaternion q0, double dt)
{
  // Note: Maybe use tf::Quaternion and use built-in methods
  // Calculate q-dot (time derivative of quaternion)
  double w = q.w;
  double x = q.x;
  double y = q.y;
  double z = q.z;
  double dw = (w-q0.w)/dt;
  double dx = (x-q0.x)/dt;
  double dy = (y-q0.y)/dt;
  double dz = (z-q0.z)/dt;
  // Calculate omega (Qw) from q-dot and q
  double denom = ((dw*dw)*(w*w) + 2*dw*w*(dx*x + dy*y) + (dx*x + dy*y)*(dx*x + dy*y) + (dz*dz)*((w*w) + (x*x) + (y*y)));
  double Qwx = (-2*dz*(dw*dy*w - dx*dz*w + dx*dy*x + dw*dz*x + ((dy*dy) + (dz*dz))*y))/denom;
  double Qwy = (2*dz*(dw*dx*w + dy*dz*w + (dx*dx)*x + (dz*dz)*x + dx*dy*y - dw*dz*y))/denom;
  double Qwz = (2*dz*((dw*dw)*w + dz*(dz*w - dy*x + dx*y) + dw*(dx*x + dy*y)))/denom;
  // Rotate omega with q
  Qwx =   y*(Qwz*w + Qwy*x - Qwx*y)  - z*(Qwy*w - Qwz*x + Qwx*z) + w*(Qwx*w + Qwz*y - Qwy*z) - x*(-(Qwx*x) - Qwy*y - Qwz*z);
  Qwy = -(x*(Qwz*w + Qwy*x - Qwx*y)) + w*(Qwy*w - Qwz*x + Qwx*z) + z*(Qwx*w + Qwz*y - Qwy*z) - y*(-(Qwx*x) - Qwy*y - Qwz*z);
  Qwz =   w*(Qwz*w + Qwy*x - Qwx*y)  + x*(Qwy*w - Qwz*x + Qwx*z) - y*(Qwx*w + Qwz*y - Qwy*z) - z*(-(Qwx*x) - Qwy*y - Qwz*z);
  geometry_msgs::Vector3 omega;
  omega.x = Qwx;
  omega.y = Qwy;
  omega.z = Qwz;
  return omega;
}

void update_messages(ros::Time message_time)
{

  if (trackingEnabled)
  {

    tf::transformMsgToTF(markerToCamera, marker_to_cam);

    if (marker_to_footprint_from_urdf)
    {
      tf::transformMsgToTF(markerToFootprint, marker_to_footprint_tf);
      marker_to_cam = marker_to_cam.inverseTimes(marker_to_footprint_tf);
    }

    tf::Transform marker_displacement_tf = marker_to_cam.inverseTimes(marker_to_cam_prev); // between two frames for velocity report

    tf::transformMsgToTF(transformOrigin, transform_origin_tf);
    tf::Transform marker_to_origin_tf = transform_origin_tf.inverseTimes(marker_to_cam); // for position report

    tf::transformTFToMsg(marker_to_origin_tf, odomTransform);
    tf::transformTFToMsg(marker_displacement_tf, robotDisplacementTransform);

    if (publish_odometry)
    {
      nav_msgs::Odometry odom;
      double twist_dt_sec = (message_time-twist_timestamp).toSec();
      geometry_msgs::Vector3 omega = update_angular_velocity(markerToCamera.rotation, markerToCameraPrev.rotation, twist_dt_sec);

      // Update "previous values" with current
      twist_timestamp = message_time;
      marker_to_cam_prev = marker_to_cam;
      markerToCameraPrev = markerToCamera;

      odom.header.frame_id = origin_frame_id;
      odom.header.stamp = message_time;
      odom.child_frame_id = footprint_frame_id; // = marker_frame_id if no urdf avail.
      odom.pose.pose.position.x  = odomTransform.translation.x;
      odom.pose.pose.position.y  = odomTransform.translation.y;
      odom.pose.pose.position.z  = odomTransform.translation.z;
      odom.pose.pose.orientation = odomTransform.rotation;
      odom.twist.twist.linear.x  = robotDisplacementTransform.translation.x / twist_dt_sec;
      odom.twist.twist.linear.y  = robotDisplacementTransform.translation.y / twist_dt_sec;
      odom.twist.twist.linear.z  = robotDisplacementTransform.translation.z / twist_dt_sec;
      odom.twist.twist.angular.x = omega.x;
      odom.twist.twist.angular.y = omega.y;
      odom.twist.twist.angular.z = omega.z;
      odom_pub.publish(odom);
    }

    if (publish_odom_tf)
    {
      geometry_msgs::TransformStamped transformStamped;
      transformStamped.header.stamp = message_time;
      transformStamped.header.frame_id = origin_frame_id;
      transformStamped.child_frame_id = footprint_frame_id; // = marker_frame_id if no urdf avail.
      transformStamped.transform.translation.x = odomTransform.translation.x;
      transformStamped.transform.translation.y = odomTransform.translation.y;
      transformStamped.transform.translation.z = odomTransform.translation.z;
      transformStamped.transform.rotation = odomTransform.rotation;
      tf_broadcaster->sendTransform(transformStamped);
    }
    else if (publish_fiducial_tf)
    {
      geometry_msgs::TransformStamped transformStamped;

      transformStamped.header.stamp = message_time;

      transformStamped.header.frame_id = camera_frame_id;
      transformStamped.child_frame_id = marker_frame_id;
      transformStamped.transform.translation.x = markerToCamera.translation.x;
      transformStamped.transform.translation.y = markerToCamera.translation.y;
      transformStamped.transform.translation.z = markerToCamera.translation.z;
      transformStamped.transform.rotation = markerToCamera.rotation;
      tf_broadcaster->sendTransform(transformStamped);

      transformStamped.child_frame_id = origin_frame_id;
      transformStamped.transform.translation.x = transformOrigin.translation.x;
      transformStamped.transform.translation.y = transformOrigin.translation.y;
      transformStamped.transform.translation.z = transformOrigin.translation.z;
      transformStamped.transform.rotation = transformOrigin.rotation;
      tf_broadcaster->sendTransform(transformStamped);
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
    if (origin_idx >= 0 && originNeedsUpdate) {
      transformOrigin = ftf_array.transforms[origin_idx].transform;
      originNeedsUpdate = false;
      ROS_INFO_STREAM("Origin set using ground marker (ID = " << origin_id << ")");
      trackingEnabled = true;
    }
    // If robot marker is present, update its transform and publish updated messages
    if (robot_idx >= 0) {
      if (originNeedsUpdate) {
        originNeedsUpdate = false;
        transformOrigin = markerToCamera;
        ROS_INFO_STREAM("Origin set using robot marker (ID = " << robot_id << ")");
        trackingEnabled = true;
      }
      markerToCamera = ftf_array.transforms[robot_idx].transform;
      update_required = true;
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

    tf_broadcaster = new(tf2_ros::TransformBroadcaster);

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    nhLocal.param("origin_id", origin_id, 0);
    nhLocal.param("robot_id", robot_id, 1);
    nhLocal.param("odom_frame", origin_frame_id, std::string("odom"));
    nhLocal.param("camera_frame", camera_frame_id, std::string("camera"));
    nhLocal.param("marker_frame", marker_frame_id, std::string("marker"));
    nhLocal.param("footprint_frame", footprint_frame_id, std::string("base_footprint"));
    nhLocal.param("publish_fiducial_tf", publish_fiducial_tf, true);
    nhLocal.param("publish_odom_tf", publish_odom_tf, false);
    nhLocal.param("publish_odometry", publish_odometry, true);
    nhLocal.param("marker_to_footprint_from_urdf", marker_to_footprint_from_urdf, false);

    ros::Subscriber ftf_sub    = n.subscribe("/fiducial_transforms", 100, fiducialTransFormArrayUpdate);
    ros::ServiceServer service = n.advertiseService("/fiducial_tracker/reset_origin", resetRobotOrigin);

    if (publish_odometry) {
      odom_pub = n.advertise<nav_msgs::Odometry>("/fiducial_tracker/odom", 50);
    }

    if (marker_to_footprint_from_urdf) {
      uint16_t tf_give_up_counter = 10;
      while (n.ok()) {
         geometry_msgs::TransformStamped transformStamped;
         try {
           transformStamped = tf_buffer.lookupTransform(marker_frame_id, footprint_frame_id, ros::Time(0));
           markerToFootprint = transformStamped.transform;
           ROS_INFO_STREAM("Got " << marker_frame_id << " to " << footprint_frame_id
            << " [X: "  << markerToFootprint.translation.x
            << "] [Y: " << markerToFootprint.translation.y
            << "] [Z: " << markerToFootprint.translation.z << "]"
          );
           break;
         }
         catch (tf2::TransformException &ex) {
           tf_give_up_counter--;
           if (tf_give_up_counter == 0) {
             ROS_ERROR_STREAM("Waiting for " << marker_frame_id << " to " << footprint_frame_id << " has timed out.");
             footprint_frame_id = marker_frame_id;
             marker_to_footprint_from_urdf = false;
             break;
           }
           ROS_WARN_STREAM(ex.what() << " Retrires left: " << tf_give_up_counter);
           ros::Duration(1.0).sleep();
           continue;
         }
      }
    }
    else {
      footprint_frame_id = marker_frame_id;
    }

    twist_timestamp = ros::Time::now();
    ROS_INFO("Waiting for reset_origin service call...");
    ros::spin();
    return 0;
}
