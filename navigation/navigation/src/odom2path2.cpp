#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>

nav_msgs::Path  path_gps;
ros::Publisher  pub_gps;


void callback(const nav_msgs::Odometry::ConstPtr& odom)
{
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = odom -> pose.pose.position.x;
    this_pose_stamped.pose.position.y = odom -> pose.pose.position.y;
    this_pose_stamped.pose.position.z = odom -> pose.pose.position.z;

    this_pose_stamped.pose.orientation = odom -> pose.pose.orientation;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "map";

    path_gps.poses.push_back(this_pose_stamped);

    path_gps.header.stamp = ros::Time::now();
    path_gps.header.frame_id = "map";
    pub_gps.publish(path_gps);
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "odom2path");
    ros::NodeHandle ph;

    pub_gps = ph.advertise<nav_msgs::Path>("/odom_path", 10, true);
    ros::Subscriber sub_gps = ph.subscribe<nav_msgs::Odometry>("/odom", 10, callback);  
    
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
