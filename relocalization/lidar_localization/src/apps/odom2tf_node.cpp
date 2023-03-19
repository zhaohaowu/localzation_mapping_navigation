#include <bits/stdc++.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Geometry>
ros::Publisher  pub;
nav_msgs::Odometry msg;
tf::StampedTransform odom_trans;
int tf_flag = 0;
using namespace std;
void callback1(const nav_msgs::Odometry::ConstPtr& msgIn)
{
    tf_flag = 1;
    msg.pose.pose = msgIn->pose.pose;
    msg.pose.pose.position.z = 0;
    Eigen::Quaternionf q = {msg.pose.pose.orientation.w,
                            msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z};//w,x,y,z
    // q.normalize();
    odom_trans.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    odom_trans.setOrigin(tf::Vector3(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z));
}
void callback2(const geometry_msgs::Twist::ConstPtr& msgIn)
{
    msg.twist.twist.angular = msgIn->angular;
    msg.twist.twist.angular.z = msg.twist.twist.angular.z / 180 * 3.14;
    msg.twist.twist.linear = msgIn->linear;
}

int main (int argc, char **argv)
{
    ros::init (argc, argv, "odom2tf_node");
    ros::NodeHandle n;
    pub = n.advertise<nav_msgs::Odometry>("/odom", 10, true);
    ros::Subscriber sub1 = n.subscribe<nav_msgs::Odometry>("/laser_localization", 10, callback1);
    ros::Subscriber sub2 = n.subscribe<geometry_msgs::Twist>("/wheel/data", 10, callback2);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        if(tf_flag == 1){
            msg.header.stamp = ros::Time::now();
            msg.header.frame_id = "odom";
            pub.publish(msg);

            odom_trans.stamp_ = ros::Time::now();
            odom_trans.frame_id_ = "odom";
            odom_trans.child_frame_id_ = "base_link";
            odom_broadcaster.sendTransform(odom_trans);
        }
    }
    return 0;
}
