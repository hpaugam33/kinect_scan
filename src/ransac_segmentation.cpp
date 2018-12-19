#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>


#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>


#include <pcl/surface/mls.h>

#include <iostream>
#include <pcl/io/ply_io.h>


#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <kinect_scan/ransac_segmentationConfig.h>




using namespace std;
using namespace pcl;

ros::Publisher pub;
//ros::Subscriber sub;

int nodes, topics;

double  DistanceThreshold, SearchRadius, seuil;

bool apply_change, computeMLS, change_topic;

string output_cloud_file, input_cloud;

// Output has the PointNormal type in order to store the normals calculated by MLS
PointCloud< PointNormal> mls_points;
PointCloud< PointNormal>::Ptr mls_points_ptr(new  PointCloud< PointNormal>);




void param_callback(kinect_scan::ransac_segmentationConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %f %f %f ",
             config.DistanceThreshold_Ransac,
             config.SearchRadius,
             config.seuil);
             //config.apply_change?"True":"False");


    DistanceThreshold = config.DistanceThreshold_Ransac;
    SearchRadius = config.SearchRadius;
    seuil = config.seuil;
    //apply_change = config.apply_change;
}




void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{



    // Container for original & filtered data
    PCLPointCloud2* cloud = new  PCLPointCloud2;
    PCLPointCloud2ConstPtr cloudPtr(cloud);
    PCLPointCloud2 cloud_filtered;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);


    //Conversion in PointXYZ
    PointCloud< PointXYZ > point_cloud;
    PointCloud< PointXYZ >::Ptr point_cloudPtr(new  PointCloud< PointXYZ >);
    fromPCLPointCloud2( *cloudPtr, point_cloud);
    copyPointCloud(point_cloud, *point_cloudPtr);

    cout <<point_cloudPtr->size()<<endl;

    if (point_cloudPtr->size() > 0)
    {
        cout<<"begin : " <<point_cloudPtr->size()<<endl;


        PointCloud< PointXYZ>::Ptr point_cloud_filtered(new  PointCloud< PointXYZ>);


        //Filter if point normal too much similar to another point
        for(int j = 0 ; j< point_cloudPtr->size()-1 ; j++)
        {
            double deltax = abs(point_cloudPtr->points[j+1].x - point_cloudPtr->points[j].x);
            double deltay = abs(point_cloudPtr->points[j+1].y - point_cloudPtr->points[j].y);
            double deltaz = abs(point_cloudPtr->points[j+1].z - point_cloudPtr->points[j].z);

            if(deltax > seuil|| deltay > seuil || deltaz > seuil)
                point_cloud_filtered->push_back(point_cloudPtr->points[j]);

        }

        if(point_cloud_filtered->size() == 0)
        {
            cout <<"Seuil too high for this cloud.. Reducing by 0.01" <<endl;
            seuil -= 0.01 ;
            //config.seuil = seuil;

        }



        //NORMALIZE
        // Create a KD-Tree
        search::KdTree< PointXYZ >::Ptr tree (new  search::KdTree< PointXYZ >);


        // Init object (second point type is for the normals, even if unused)
        MovingLeastSquares< PointXYZ ,  PointNormal> mls;


        ros::Time tic = ros::Time::now();
        // cout<< "Computing MLS method... ";

        mls.setComputeNormals (true);

        // Set parameters
        mls.setInputCloud (point_cloud_filtered);
        mls.setPolynomialFit (true);
        mls.setSearchMethod (tree);
        mls.setSearchRadius (SearchRadius);

        // Reconstruct
        mls.process (mls_points);
        *mls_points_ptr = mls_points;

        computeMLS = false;

        ros::Time toc = ros::Time::now();
        //cout<< "Done in "<< toc - tic <<" sec "<<endl;

        cout<<"end :" <<mls_points_ptr->size()<<endl;


        //convert pcl::PointNormal to pcl::PointXYZ
        pcl::PointCloud<pcl::PointXYZ>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointXYZ>);


        if(mls_points_ptr->size() == 0)
        {
            mls_cloud = point_cloudPtr;
            //cout <<"here"<<endl;
        }

        else
        {

            for (size_t i = 0; i < mls_points_ptr->points.size(); ++i)
            {
                const pcl::PointNormal &mls_pt = mls_points_ptr->points[i];
                pcl::PointXYZ pt(mls_pt.x, mls_pt.y, mls_pt.z);
                mls_cloud->push_back(pt);
            }
        }

        cout << mls_cloud->size()<<endl;

        // Creating the KdTree object for the search method of the extraction
        search::KdTree< PointXYZ >::Ptr tree2(new  search::KdTree< PointXYZ >);
        tree2->setInputCloud(mls_cloud);


        ModelCoefficients::Ptr coefficients (new  ModelCoefficients);
        PointIndices::Ptr inliers (new  PointIndices);



        // RANSAC SEGMENTATION
        PointCloud< PointXYZ > *point_cloud_Ransac_segmented(new  PointCloud< PointXYZ >);
        PointCloud< PointXYZ >::Ptr point_cloud_Ransac_segmented_Ptr(point_cloud_Ransac_segmented);


        // Create the segmentation object
        SACSegmentation< PointXYZ > seg1;
        // Optional
        seg1.setOptimizeCoefficients (false);
        // Mandatory
        seg1.setModelType ( SACMODEL_PLANE);
        seg1.setMethodType ( SAC_RANSAC);
        seg1.setDistanceThreshold (DistanceThreshold);

        seg1.setInputCloud (mls_cloud);
        seg1.segment (* inliers, * coefficients);


        // Create the filtering object
        ExtractIndices< PointXYZ > extract;

        //extract.setInputCloud (xyzCloudPtrFiltered);
        extract.setInputCloud (mls_cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (* point_cloud_Ransac_segmented_Ptr);



        // Convert to ROS data type
        point_cloud_Ransac_segmented_Ptr->header.frame_id = point_cloudPtr->header.frame_id;
        if(point_cloud_Ransac_segmented_Ptr->size())  toPCLPointCloud2(*point_cloud_Ransac_segmented_Ptr, cloud_filtered);
        else  toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output;
        pcl_conversions::fromPCL(cloud_filtered, output);


        // Publish the data

        pub.publish(output);

        if (point_cloud_Ransac_segmented_Ptr->size() >0)
        {
            io::savePCDFileBinary (output_cloud_file, *point_cloud_Ransac_segmented_Ptr);
        }



    }
    else
    {
        cout << "Input Cloud is empty !" <<endl;

    }

}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "ransac segmentation");
    ros::NodeHandle nh ("~");

    //read the param in the launchfile
    //nh.getParam("saving", saving);
    nh.getParam("output_cloud_file", output_cloud_file);


    //Initialize the dynamic reconfigure of the parameters
    dynamic_reconfigure::Server<kinect_scan::ransac_segmentationConfig> server;
    dynamic_reconfigure::Server<kinect_scan::ransac_segmentationConfig>::CallbackType f;
    f = boost::bind(&param_callback,_1, _2);
    server.setCallback(param_callback);


    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/input", 1, cloud_cb);


    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);


    // Spin
    ros::spin ();
}

