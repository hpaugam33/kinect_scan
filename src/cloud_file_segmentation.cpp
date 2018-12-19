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
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>


#include <dynamic_reconfigure/server.h>
#include <dynamic_reconfigure/Reconfigure.h>

#include <kinect_scan/from_file_segmentationConfig.h>

#include <iostream>
#include <pcl/io/ply_io.h>



using namespace std;
using namespace pcl;



double ClusterTolerance, DistanceThreshold,  Leafsize, zMin, zMax;
int MinClusterSize;
bool ZPassThrough, saving, changed ;


string output_cloud_file;

string input_cloud_file;

ros::Publisher pub, pub1, pub2, pub3, pub4, pub5, pub6, pub7, pub8, pub9, pub10, pub11;




void callback(kinect_scan::from_file_segmentationConfig &config, uint32_t level)
{
    ROS_INFO("Reconfigure Request: %f %d %f %f %f %f %s ",
             config.ClusterTolerance,
             config.MinClusterSize,
             config.DistanceThreshold,
             config.Leafsize,
             config.zMin,
             config.zMax,
             config.ZPassThrough?"True":"False");

    ClusterTolerance = config.ClusterTolerance;
    MinClusterSize = config.MinClusterSize;
    DistanceThreshold = config.DistanceThreshold;
    zMin = config.zMin;
    zMax = config.zMax;
    Leafsize =  config.Leafsize;
    ZPassThrough = config.ZPassThrough;

    changed = true;

 cout<<"File Segmentation Computing... ";
}




int main (int argc, char** argv)
{

    // Initialize ROS
    ros::init (argc, argv, "kinect_scan");
    ros::NodeHandle nh ("~");

//Initialize the dynamic reconfigure of the parameters
    dynamic_reconfigure::Server<kinect_scan::from_file_segmentationConfig> server;
    dynamic_reconfigure::Server<kinect_scan::from_file_segmentationConfig>::CallbackType f;


    f = boost::bind(&callback,_1, _2);
    server.setCallback(callback);


    //Take the name of the cloud files
    nh.getParam("output_cloud_file", output_cloud_file);
    nh.getParam("input_cloud_file", input_cloud_file);


    //Read the cloudfile in input
    PointCloud< PointXYZRGB> cloud;
    io::loadPCDFile< PointXYZRGB> (input_cloud_file, cloud);



    PointCloud< PointXYZRGB>::Ptr cloudPtr(new  PointCloud< PointXYZRGB>);
    copyPointCloud(cloud, *cloudPtr);


    sensor_msgs::PointCloud2 output_;


    output_.header.frame_id = "/map";
    output_.header.stamp    = ros::Time::now ();


    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
    pub1 = nh.advertise<sensor_msgs::PointCloud2> ("output1", 1);
    pub2 = nh.advertise<sensor_msgs::PointCloud2> ("output2", 1);
    pub3 = nh.advertise<sensor_msgs::PointCloud2> ("output3", 1);
    pub4 = nh.advertise<sensor_msgs::PointCloud2> ("output4", 1);
    pub5 = nh.advertise<sensor_msgs::PointCloud2> ("output5", 1);
    pub6 = nh.advertise<sensor_msgs::PointCloud2> ("output6", 1);
    pub7 = nh.advertise<sensor_msgs::PointCloud2> ("output7", 1);
    pub8 = nh.advertise<sensor_msgs::PointCloud2> ("output8", 1);
    pub9 = nh.advertise<sensor_msgs::PointCloud2> ("output9", 1);
    pub10 = nh.advertise<sensor_msgs::PointCloud2> ("output10", 1);
    pub11 = nh.advertise<sensor_msgs::PointCloud2> ("output11", 1);


    PCLPointCloud2 cloud_filtered0;
    PCLPointCloud2 cloud_filtered1;
    PCLPointCloud2 cloud_filtered2;
    PCLPointCloud2 cloud_filtered3;
    PCLPointCloud2 cloud_filtered4;
    PCLPointCloud2 cloud_filtered5;
    PCLPointCloud2 cloud_filtered6;
    PCLPointCloud2 cloud_filtered7;
    PCLPointCloud2 cloud_filtered8;
    PCLPointCloud2 cloud_filtered9;
    PCLPointCloud2 cloud_filtered10;
    PCLPointCloud2 cloud_filtered11;

    while(ros::ok())
    {



        // Perform the downsampling filter
        VoxelGrid<PointXYZRGB> sor;
        sor.setInputCloud (cloudPtr);
        sor.setLeafSize (Leafsize, Leafsize, Leafsize);
        sor.filter (*cloudPtr);


        PointCloud<PointXYZRGB> cloud_filtered;
        PointCloud< PointXYZRGB>::Ptr cloud_filteredPtr(new  PointCloud< PointXYZRGB>);
        copyPointCloud(cloud_filtered, *cloud_filteredPtr);




        if(ZPassThrough)
        {
            //Perform a z passthrough to elimine the ground and celling
            PassThrough< PointXYZRGB> pass;
            pass.setInputCloud (cloudPtr);
            pass.setFilterFieldName ("z");
            pass.setFilterLimits (zMin, zMax);
            //pass.setFilterLimitsNegative (true);
            pass.filter (*cloud_filteredPtr);

        }
        else
        {
            cloud_filteredPtr = cloudPtr;
        }



        // Creating the KdTree object for the search method of the extraction
        search::KdTree< PointXYZRGB>::Ptr tree(new  search::KdTree< PointXYZRGB>);
        tree->setInputCloud(cloud_filteredPtr);



        ModelCoefficients::Ptr coefficients (new  ModelCoefficients);
        PointIndices::Ptr inliers (new  PointIndices);



        // RANSAC SEGMENTATION
        PointCloud< PointXYZRGB> *point_cloud_Ransac_segmented(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_Ransac_segmented_Ptr(point_cloud_Ransac_segmented);


        // Create the segmentation object
        SACSegmentation< PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType ( SACMODEL_PLANE);
        seg.setMethodType ( SAC_RANSAC);
        seg.setDistanceThreshold (DistanceThreshold);

        seg.setInputCloud (cloud_filteredPtr);
        seg.segment (* inliers, * coefficients);


        // Create the filtering object
        ExtractIndices< PointXYZRGB> extract;

        //extract.setInputCloud (xyzCloudPtrFiltered);
        extract.setInputCloud (cloud_filteredPtr);
        extract.setIndices (inliers);
        extract.setNegative (true);
        extract.filter (* point_cloud_Ransac_segmented_Ptr);





        //Define cluster param
        std::vector< PointIndices> cluster_indices;
        EuclideanClusterExtraction< PointXYZRGB> ec;
        ec.setClusterTolerance(ClusterTolerance);
        ec.setMinClusterSize(MinClusterSize);
        ec.setMaxClusterSize(99000000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(point_cloud_Ransac_segmented_Ptr);
        ec.extract(cluster_indices);






        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented_total(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented1(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented2(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented3(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented4(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented5(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented6(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented7(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented8(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented9(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented10(new  PointCloud< PointXYZRGB>);
        PointCloud< PointXYZRGB>::Ptr point_cloud_segmented11(new  PointCloud< PointXYZRGB>);


        //vector<  PointCloud< PointXYZRGB>::Ptr> cloud_segmented_list;

        // cloud_segmented_list->push_back(point_cloud_segmented1);

        int j= 0;


        for (std::vector< PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
        {
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            {

                PointXYZRGB point;
                point.x = point_cloud_Ransac_segmented_Ptr->points[*pit].x;
                point.y = point_cloud_Ransac_segmented_Ptr->points[*pit].y;
                point.z = point_cloud_Ransac_segmented_Ptr->points[*pit].z;

                if (j == 0) //Blue	#FF0000	(255,0,0)
                {
                    point.r = 0;
                    point.g = 0;
                    point.b = 255;
                    point_cloud_segmented1->push_back(point);

                }
                else if (j == 1) //Green	#00FF00	(0,255,0)
                {
                    point.r = 0;
                    point.g = 255;
                    point.b = 0;
                    point_cloud_segmented2->push_back(point);
                }
                else if (j == 2) // Red	#0000FF	(0,0,255)
                {
                    point.r = 255;
                    point.g = 0;
                    point.b = 0;
                    point_cloud_segmented3->push_back(point);
                }
                else if (j == 3) // Yellow	#FFFF00	(255,255,0)
                {
                    point.r = 255;
                    point.g = 255;
                    point.b = 0;
                    point_cloud_segmented4->push_back(point);
                }
                else if (j == 4) //Cyan	#00FFFF	(0,255,255)
                {
                    point.r = 0;
                    point.g = 255;
                    point.b = 255;
                    point_cloud_segmented5->push_back(point);
                }
                else if (j == 5) // Magenta	#FF00FF	(255,0,255)
                {
                    point.r = 255;
                    point.g = 0;
                    point.b = 255;
                    point_cloud_segmented6->push_back(point);
                }
                else if (j == 6) // Olive	#808000	(128,128,0)
                {
                    point.r = 128;
                    point.g = 128;
                    point.b = 0;
                    point_cloud_segmented7->push_back(point);
                }
                else if (j == 7) // Teal	#008080	(0,128,128)
                {
                    point.r = 0;
                    point.g = 128;
                    point.b = 128;
                    point_cloud_segmented8->push_back(point);
                }
                else if (j == 8) // Purple	#800080	(128,0,128)
                {
                    point.r = 128;
                    point.g = 0;
                    point.b = 128;
                    point_cloud_segmented9->push_back(point);
                }
                else
                {
                    if (j % 2 == 0)
                    {
                        point.r = 255 * j / (cluster_indices.size());
                        point.g = 128;
                        point.b = 50;
                        point_cloud_segmented10->push_back(point);
                    }
                    else
                    {
                        point.r = 0;
                        point.g = 255 * j / (cluster_indices.size());
                        point.b = 128;
                        point_cloud_segmented11->push_back(point);
                    }
                }
                point_cloud_segmented_total->push_back(point);


            }
            j++;
        }

        if( saving)
             io::savePLYFileBinary (output_cloud_file, *point_cloud_segmented_total);






        // Convert to ROS data type
        point_cloud_segmented_total->header.frame_id = output_.header.frame_id;

        if(point_cloud_segmented_total->size()) pcl::toPCLPointCloud2(*point_cloud_segmented_total, cloud_filtered0);
        else toPCLPointCloud2(*cloud_filteredPtr, cloud_filtered0);
        pcl_conversions::fromPCL(cloud_filtered0, output_);


        point_cloud_segmented1->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented1, cloud_filtered1);
        // toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output1;
        pcl_conversions::fromPCL(cloud_filtered1, output1);


        point_cloud_segmented2->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented2, cloud_filtered2);
        // toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output2;
        pcl_conversions::fromPCL(cloud_filtered2, output2);


        point_cloud_segmented3->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented3, cloud_filtered3);
        // toPCLPointCloud3(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output3;
        pcl_conversions::fromPCL(cloud_filtered3, output3);


        point_cloud_segmented4->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented4, cloud_filtered4);
        // toPCLPointCloud4(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output4;
        pcl_conversions::fromPCL(cloud_filtered4, output4);


        point_cloud_segmented5->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented5, cloud_filtered5);
        // toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output5;
        pcl_conversions::fromPCL(cloud_filtered5, output5);

        point_cloud_segmented6->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented6, cloud_filtered6);
        // toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output6;
        pcl_conversions::fromPCL(cloud_filtered6, output6);

        point_cloud_segmented7->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented7, cloud_filtered7);
        // toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output7;
        pcl_conversions::fromPCL(cloud_filtered7, output7);

        point_cloud_segmented8->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented8, cloud_filtered8);
        // toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output8;
        pcl_conversions::fromPCL(cloud_filtered8, output8);

        point_cloud_segmented9->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented9, cloud_filtered9);
        // toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output9;
        pcl_conversions::fromPCL(cloud_filtered9, output9);

        point_cloud_segmented10->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented10, cloud_filtered10);
        // toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output10;
        pcl_conversions::fromPCL(cloud_filtered10, output10);

        point_cloud_segmented11->header.frame_id = output_.header.frame_id;
        toPCLPointCloud2(*point_cloud_segmented11, cloud_filtered11);
        // toPCLPointCloud2(*point_cloudPtr, cloud_filtered);
        sensor_msgs::PointCloud2 output11;
        pcl_conversions::fromPCL(cloud_filtered11, output11);



        // Publish the data
        pub.publish (output_);
        pub1.publish (output1);
        pub2.publish (output2);
        pub3.publish (output3);
        pub4.publish (output4);
        pub5.publish (output5);
        pub6.publish (output6);
        pub7.publish (output7);
        pub8.publish (output8);
        pub9.publish (output9);
        pub10.publish (output10);
        pub11.publish (output11);




if(changed)
{
    cout<<"Done"<<endl;
    changed =false;
}

        ros::spinOnce();

    }

    return 0;
}





