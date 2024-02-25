// Step 1: Cluster PointClouds
// Step 2: If any N points (default: 10) within the cluster E Restricted FoV ==> Result

#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
using namespace std;

ros::Publisher pub_vessel_pointcloud;

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
    
}

int main(int argc, char **argv){
    ros::init(argc, argv, "main");
    ros::NodeHandle n;

    ros::Subscriber sub_raw_pointcloud = n.subscribe("/ouster1/points", 1, cloud_cb);
    pub_vessel_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/vessel_target/points", 1);

    ros::spin();
    ros::waitForShutdown();

    return 0;
}