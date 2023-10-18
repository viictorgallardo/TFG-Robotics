#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include "ros/ros.h"


class NodeSync
{
 public:
  NodeSync()
  {
    scan_sub_.subscribe(nh_, "/scan", 1);
    pose_sub_.subscribe(nh_, "/pose", 1);
    sync_.reset(new Sync(MySyncPolicy(10), scan_sub_, pose_sub_));
    sync_->registerCallback(boost::bind(&NodeSync::callback, this, _1, _2));
  }

  void callback(const sensor_msgs::LaserScan::ConstPtr& laser_scan, const nav_msgs::Odometry::ConstPtr& estimate_pose)
  {
    ROS_INFO("Synchronization successful");
    
    // Get position info
    tf::Quaternion q(	estimate_pose->pose.pose.orientation.x, 
    			estimate_pose->pose.pose.orientation.y, 
    			estimate_pose->pose.pose.orientation.z, 
    			estimate_pose->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Robot Pose x:%f ; y:%f ; th:%f ;", estimate_pose->pose.pose.position.x,estimate_pose->pose.pose.position.y,yaw);
    
    
    // Print first and last value of the laser scan
    ROS_INFO("First range data scan: %f",laser_scan->ranges[0]);
    ROS_INFO("Last range data scan: %f",laser_scan->ranges[laser_scan->ranges.size()-1]);
    
    
    /* Example loop over laser scan data
    for(int i=0;i<(laser_scan->ranges.size());i++){
      if(!(laser_scan->ranges.empty()))
         ROS_INFO("data %f",laser_scan->ranges[i]);
      else{
         ROS_INFO("NO VALUE");
      }
    } */
      
  }

 private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
  message_filters::Subscriber<nav_msgs::Odometry> pose_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, nav_msgs::Odometry> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "read_odom_scan");

  NodeSync synchronizer;

  ros::spin();
}













