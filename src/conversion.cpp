#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>

#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/pcd_io.h>


#include <pcl/surface/mls.h>


#include <fstream>


using namespace std;
using namespace pcl;


string file_input, file_output;



int main(int argc, char**argv)
{

    // Initialize ROS
    ros::init (argc, argv, "kinect_scan");
    ros::NodeHandle nh ("~");

    //Read the node parameters
    nh.getParam("file_output", file_output);
    nh.getParam("file_input", file_input);

    //Define point cloud
    PointCloud< PointXYZ> cloud;

    //Extract cloud point from file
    io::loadPCDFile< PointXYZ> (file_input, cloud);

    //Open the output file
    ofstream file(file_output.c_str(), ios::in);

    if(file)
    {
        cout <<file_output <<" file opening !"<<endl;
        cout<< "Writing in file...";
        for (int i =0 ; i< cloud.size(); i++)
        {
            file << cloud.points[i].x << " " <<cloud.points[i].y << " "<< cloud.points[i].z << endl;

        }

        file.close();
        cout<<"Done"<<endl;
    }
    else
        cout << "Unable to open the " << file_output.c_str() <<" file !" << endl;



    // Spin
    ros::spinOnce();
}


