// Step 1: Cluster PointClouds
// Step 2: If any N points (default: 10) within the cluster E Restricted FoV ==> Result

#include "ros/ros.h"
#include <cmath>
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32MultiArray.h"

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace std;

// ROSPARAM
float point_by_point_z;
float point_by_point_r;
float point_by_point_tail_theta;
float point_by_point_tail_length;

float clustering_tolerance;
float clustering_size_min;
float clustering_size_max;
///////////

ros::Subscriber sub_raw_pointcloud;
ros::Subscriber sub_cam_fov;
ros::Publisher pub_vessel_pointcloud;

float arr_cam_fov[2] = { 0 };
ros::Time process_start;
ros::Time process_end;
bool cloud_cb_lock = true;

pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl(new pcl::PointCloud<pcl::PointXYZI>);
Eigen::MatrixXf raw_mat;

Eigen::MatrixXf pcl_to_eigen(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& pcl){
    Eigen::MatrixXf mat;

    int numPoints = pcl->points.size();
    mat.setZero(numPoints, 4);

    for(int i = 0; i < numPoints; i++) {
        mat(i, 0) = pcl->points[i].x;
        mat(i, 1) = pcl->points[i].y;
        mat(i, 2) = pcl->points[i].z;
        mat(i, 3) = pcl->points[i].intensity;
    }

    return mat;
}

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg){
    if (!cloud_cb_lock){
        process_start = ros::Time::now();

        // PointByPoint Filtering
        pcl::fromROSMsg (*msg, *raw_pcl);
        raw_mat = pcl_to_eigen(raw_pcl);
        pcl::PointCloud<pcl::PointXYZI> pbp_pcl;
        
        // int row_num = raw_mat.rows();
        // float min_intensity = std::numeric_limits<float>::infinity();
        // float max_intensity = -std::numeric_limits<float>::infinity();
        // for (int i=0; i<row_num; i++){
        //     if (raw_mat.row(i)(3) > max_intensity){
        //         max_intensity = raw_mat.row(i)(3);
        //     }

        //     if (raw_mat.row(i)(3) < min_intensity){
        //         min_intensity = raw_mat.row(i)(3);
        //     }
        // }
        // cout << "Min Intensity: " << min_intensity << endl;
        // cout << "Max Intensity: " << max_intensity << endl;

        int row_num = raw_mat.rows();
        for (int i=0; i<row_num; i++){

            float x = raw_mat.row(i)(0);
            float y = raw_mat.row(i)(1);
            float z = raw_mat.row(i)(2);
            float I = raw_mat.row(i)(3);

            // cout << "XYZI: " << x << ", " << y << ", " << z << ", " << I << endl;

            if (z < point_by_point_z){
                //
            }else if (std::sqrt(std::pow(x, 2) + std::pow(y, 2)) < point_by_point_r){
                //
            }else if (  (std::sqrt(std::pow(x, 2) + std::pow(y, 2)) < point_by_point_tail_length)\
                        && (atan2(y, x) > (- point_by_point_tail_theta*(M_PI/180.0) - M_PI/2))\
                        && (atan2(y, x) < (  point_by_point_tail_theta*(M_PI/180.0) - M_PI/2))){
                //
            }else{
                pcl::PointXYZI point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.intensity = I;
                pbp_pcl.points.push_back(point);
            }
        }

        // Voxel Filtering
        pcl::VoxelGrid<pcl::PointXYZI> vg;
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pcl (new pcl::PointCloud<pcl::PointXYZI>);
        vg.setInputCloud(pbp_pcl.makeShared());
        vg.setLeafSize (0.5, 0.5, 0.16);
        vg.filter(*filtered_pcl);

        // Clustering
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
        if (filtered_pcl->points.size () > 0) {
            pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
            tree->setInputCloud(filtered_pcl);
            ec.setClusterTolerance(clustering_tolerance);
            ec.setMinClusterSize(clustering_size_min);
            ec.setMaxClusterSize(clustering_size_max);
            ec.setSearchMethod(tree);
            ec.setInputCloud(filtered_pcl);
            ec.extract(cluster_indices);
        }
        tree.reset(new pcl::search::KdTree<pcl::PointXYZI>);

        // Scan Vessel in Camera FoV

        // Additional Heuristic to remove wall (Get closest cluster among valid)
        float closest_cluster_distance = std::numeric_limits<float>::infinity();
        std::vector<pcl::PointIndices>::const_iterator closest_cluster_it;
        ////////////////////////////////////////////////////////////////////////

        int test_vessel_counter = 0;
        pcl::PointCloud<pcl::PointXYZI> vessel_pcl;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
            if (it->indices.size() != 0){
                Eigen::MatrixXf cluster_mat(it->indices.size(), 3);
                cluster_mat.setZero();

                // Count N_P_FoV (number of points within the cam FoV)

                // Additional Heuristic to remove wall (Get closest cluster among valid)
                float current_cluster_distance = 0.0;
                std::vector<pcl::PointIndices>::const_iterator current_cluster_it = it;
                ////////////////////////////////////////////////////////////////////////

                int N_P_FoV = 30;
                int N_counter = 0;
                for (std::vector<int>::const_iterator cluster_it = it->indices.begin (); cluster_it != it->indices.end (); ++cluster_it){
                    pcl::PointXYZI pt = filtered_pcl->points[*cluster_it];
                    
                    float theta = atan2(pt.y, pt.x);
                    current_cluster_distance = std::sqrt(std::pow(pt.x, 2) + std::pow(pt.y, 2));

                    if ((theta < arr_cam_fov[0]) && (theta > arr_cam_fov[1])){
                        N_counter += 1;   
                    }
                }

                bool remain_cluster = false;
                if (N_counter >= N_P_FoV){
                    remain_cluster = true;
                }

                // Additional Heuristic to remove wall (Get closest cluster among valid)
                if (remain_cluster){
                    if (current_cluster_distance < closest_cluster_distance){
                        closest_cluster_distance = current_cluster_distance;
                        closest_cluster_it = current_cluster_it;
                    }
                    test_vessel_counter += 1;
                }
                ////////////////////////////////////////////////////////////////////////
            }
        }
        // cout << "TEST... How Many Clusters Survive?: " << test_vessel_counter << endl;

        // (Heuristic) Publish the closest cluster
        if (test_vessel_counter > 0){
            for (std::vector<int>::const_iterator cit = closest_cluster_it->indices.begin (); cit != closest_cluster_it->indices.end (); ++cit){
                vessel_pcl.points.push_back(filtered_pcl->points[*cit]);
            }
        }
        ////////////////////////////////////////////////////////////////////////

        sensor_msgs::PointCloud2 vessel_msg;
        pcl::toROSMsg(vessel_pcl, vessel_msg);
        vessel_msg.header.frame_id = "os_sensor";
        pub_vessel_pointcloud.publish(vessel_msg);

        process_end = ros::Time::now();

        ros::Duration time_diff = process_end - process_start;
        ros::param::set("node2_processtime", std::round(time_diff.toSec() * 100000.0) / 100.0);

        cloud_cb_lock = true;
    }
    // else cloud_cb does not run
}

void fov_cb(const std_msgs::Float32MultiArray::ConstPtr& msg){
    if (cloud_cb_lock){
        arr_cam_fov[0] = msg->data[0];
        arr_cam_fov[1] = msg->data[1];
        cloud_cb_lock = false;
    }
    // else fov_cb does not run
}

int main(int argc, char **argv){
    ros::init(argc, argv, "vessel_pointcloud_scanner_node");
    ros::NodeHandle n;

    n.getParam("point_by_point_z", point_by_point_z);
    n.getParam("point_by_point_r", point_by_point_r);
    n.getParam("point_by_point_tail_theta", point_by_point_tail_theta);
    n.getParam("point_by_point_tail_length", point_by_point_tail_length);

    n.getParam("clustering_tolerance", clustering_tolerance);
    n.getParam("clustering_size_min", clustering_size_min);
    n.getParam("clustering_size_max", clustering_size_max);


    sub_raw_pointcloud = n.subscribe("/ouster1/points", 1, cloud_cb);
    sub_cam_fov = n.subscribe("/vps_node/cam_fov", 1, fov_cb);

    pub_vessel_pointcloud = n.advertise<sensor_msgs::PointCloud2>("/vps_node/points", 1);

    ros::spin();
    ros::waitForShutdown();

    return 0;
}