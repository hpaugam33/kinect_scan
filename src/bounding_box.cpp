#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/common.h>


#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>



#include <pcl/features/moment_of_inertia_estimation.h>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/thread/thread.hpp>

using namespace pcl;

ros::Publisher pub;


void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

    // Container for original & filtered data
    PCLPointCloud2* cloud = new  PCLPointCloud2;
    PCLPointCloud2ConstPtr cloudPtr(cloud);


    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    PointCloud< PointXYZ> point_cloud;
    PointCloud< PointXYZ>::Ptr point_cloudPtr(new  PointCloud< PointXYZ>);
    fromPCLPointCloud2( *cloudPtr, point_cloud);
    copyPointCloud(point_cloud, *point_cloudPtr);

    //Output cloud
    PCLPointCloud2 cloud_output;
    PointCloud< PointXYZ>::Ptr cloud_result(new  PointCloud< PointXYZ>);



    MomentOfInertiaEstimation < PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (point_cloudPtr);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    PointXYZ min_point_AABB;
    PointXYZ max_point_AABB;
    PointXYZ min_point_OBB;
    PointXYZ max_point_OBB;
    PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);


/*

    PointXYZ min_point_ABB_xmin = min_point_AABB;
    PointXYZ min_point_ABB_zmin = min_point_AABB;
    PointXYZ min_point_ABB_diag = min_point_AABB;

    min_point_ABB_xmin.x = max_point_AABB.x ;
    min_point_ABB_zmin.z = max_point_AABB.z ;

    min_point_ABB_diag.x = max_point_AABB.x;
    min_point_ABB_diag.z = max_point_AABB.z;



    PointXYZ  max_point_ABB_xmin =  max_point_AABB;
    PointXYZ  max_point_ABB_zmin =  max_point_AABB;
    PointXYZ  max_point_ABB_diag =  max_point_AABB;


    max_point_ABB_xmin.x = min_point_AABB.x ;
    max_point_ABB_zmin.z = min_point_AABB.z ;

    max_point_ABB_diag.x = min_point_AABB.x;
    max_point_ABB_diag.z = min_point_AABB.z;




    cloud_result->push_back(min_point_AABB);
    cloud_result->push_back(max_point_AABB);

    cloud_result->push_back(min_point_ABB_xmin);
    cloud_result->push_back(min_point_ABB_zmin);
    cloud_result->push_back(min_point_ABB_diag);

    cloud_result->push_back(max_point_ABB_xmin);
    cloud_result->push_back(max_point_ABB_zmin);
    cloud_result->push_back(max_point_ABB_diag);*/



//using OBB

  /*  PointXYZ min_point_OBB_xmin = min_point_OBB;
    PointXYZ min_point_OBB_zmin = min_point_OBB;
    PointXYZ min_point_OBB_diag = min_point_OBB;

    min_point_OBB_xmin.x = max_point_OBB.x ;
    min_point_OBB_zmin.z = max_point_OBB.z ;

    min_point_OBB_diag.x = max_point_OBB.x;
    min_point_OBB_diag.z = max_point_OBB.z;*/



    PointXYZ  max_point_OBB_xmin =  max_point_OBB;
    PointXYZ  max_point_OBB_zmin =  max_point_OBB;
    PointXYZ  max_point_OBB_diag =  max_point_OBB;






   // cloud_result->push_back(min_point_OBB);
    cloud_result->push_back(max_point_OBB);

  /*  cloud_result->push_back(min_point_OBB_xmin);
    cloud_result->push_back(min_point_OBB_zmin);
    cloud_result->push_back(min_point_OBB_diag);*/

  /*  cloud_result->push_back(max_point_OBB_xmin);
    cloud_result->push_back(max_point_OBB_zmin);
    cloud_result->push_back(max_point_OBB_diag);*/


    cloud_result->push_back(position_OBB);





    cloud_result->header.frame_id = point_cloudPtr->header.frame_id;
    toPCLPointCloud2(*cloud_result, cloud_output);
    // toPCLPointCloud2(*point_cloudPtr, cloud_filtered);



    pub.publish(cloud_output);
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");
    ros::NodeHandle nh ("~");

    //Read the node parameters


    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/input", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    // pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output1", 1);

    // Spin
    ros::spin ();
}
