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
std::string odom_frame_id;

geometry_msgs::Transform roboTransform;
geometry_msgs::Transform roboTransformOrigo;
geometry_msgs::Transform odomTransform;

tf::Transform robot_TF;
tf::Transform robot_TF_origo;

bool originHasBeenSet = false;
bool resetRobotOrigin(ros_fiducial_tracker::resetOrigin::Request  &req,
                      ros_fiducial_tracker::resetOrigin::Response &res){
  roboTransformOrigo = roboTransform;
  res.result = originHasBeenSet = req.resetOrigin;
  if (originHasBeenSet) ROS_INFO("Origin set.");
  else ROS_INFO("Origin dropped.");
  return true;
}

// TODO: marker TF
void fiducialTransFormArrayUpdate(const fiducial_msgs::FiducialTransformArray &ftf_array){
  auto array_length = std::size(ftf_array.transforms);
  if (array_length > 0)
  {
    for (uint8_t i = 0; i < array_length; i++)
    {
      if (ftf_array.transforms[i].fiducial_id == robot_id)
      {
        roboTransform = ftf_array.transforms[i].transform;

        if (originHasBeenSet)
        {
          tf::transformMsgToTF(roboTransform,robot_TF);
          tf::transformMsgToTF(roboTransformOrigo,robot_TF_origo);
          tf::Transform dislocation = robot_TF_origo.inverseTimes(robot_TF);
          tf::transformTFToMsg(dislocation, odomTransform);
          double robotHeading = tf::getYaw(odomTransform.rotation);
          tf::quaternionTFToMsg(tf::createQuaternionFromYaw(robotHeading),odomTransform.rotation);

          // VISUALIZATION MARKER
          visualization_msgs::Marker points;
          points.header.frame_id = "odom";
          points.header.stamp    = ros::Time::now();
          points.ns = "fiducial_tracker";
          points.action = visualization_msgs::Marker::ADD;
          points.id = robot_id;
          points.pose.orientation.w = 1;
          points.type = visualization_msgs::Marker::POINTS;
          points.scale.x = 0.1;
          points.scale.y = 0.1;
          points.color.g = 1.0f;
          points.color.a = 1.0;
          geometry_msgs::Point p;
          p.x = odomTransform.translation.x;
          p.y = odomTransform.translation.y;
          p.z = 0;
          points.points.push_back(p);
          marker_pub.publish(points);

          // ODOMETRY
          nav_msgs::Odometry odom;
          odom.child_frame_id = odom_frame_id;
          odom.header.stamp = ros::Time::now();
          odom.header.frame_id = "odom";
          odom.pose.pose.position.x  = odomTransform.translation.x;
          odom.pose.pose.position.y  = odomTransform.translation.y;
          odom.pose.pose.position.z  = 0;
          odom.pose.pose.orientation = odomTransform.rotation;
          // TODO: calculate velocities

          // publish the message
          odom_pub.publish(odom);
        }
      }
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

    if (ros::param::get("~odom_frame_id", odom_frame_id)) {}
    else {
      ROS_WARN("Failed to get param 'odom_frame_id' - default: base_link");
      odom_frame_id = "base_link";
    }

    ROS_INFO("Waiting for robot origin...");
    ros::spin();
    return 0;
}
