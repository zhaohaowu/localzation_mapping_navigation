#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

using namespace std;

class Pub_odom{
private:
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Subscriber sub;
public:
    Pub_odom(){
        pub = n.advertise<nav_msgs::Odometry>("/initial_odom",10); //定义发送端
        sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",10, &Pub_odom::pub_callback,this);
    }
    void pub_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msgIn){
        
        nav_msgs::Odometry msgOut;
        msgOut.header.frame_id = "map";
        msgOut.child_frame_id = "base_link";
        msgOut.header.stamp = msgIn->header.stamp;
        msgOut.pose.pose.position.x = msgIn->pose.pose.position.x;
        msgOut.pose.pose.position.y = msgIn->pose.pose.position.y;
        msgOut.pose.pose.position.z = msgIn->pose.pose.position.z;
        msgOut.pose.pose.orientation.x = msgIn->pose.pose.orientation.x;
        msgOut.pose.pose.orientation.y = msgIn->pose.pose.orientation.y;
        msgOut.pose.pose.orientation.z = msgIn->pose.pose.orientation.z;
        msgOut.pose.pose.orientation.w = msgIn->pose.pose.orientation.w;
        pub.publish(msgOut);
    }
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_odom"); //初始化节点
    Pub_odom pub_odom;
    ros::spin();
    return 0;
}
