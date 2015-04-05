/*************************************************************************
	> File Name: pcl_segmentation.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: Sun 21 Dec 2014 10:37:39 AM CST
 ************************************************************************/
#include <iostream>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/impl/angles.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>

using namespace std ;
using namespace cv ;
#define threshod_diff 20  //set the threshod 

int main (int argc, char** argv){
    // Read in the cloud data
    pcl::PCDReader reader ;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>) ;
    //reader.read ("currentCloud.pcd", *cloud) ;
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << endl ; //*
    
    //process the images
    /*
    Mat src1, src2, gray1, gray2, dst1, dst2, dst3, dst ;
    src1 = imread( "../images/previousFrameImage.jpg" );
    src2 = imread( "../images/nextFrameImage.jpg" );
    if( !src1.data ){ cout << "image1 hasn't data!" << endl ; }
    if( !src2.data ){ cout << "image2 hasn't data!" << endl ; }
    cvtColor( src1, gray1, CV_BGR2GRAY );
    cvtColor( src2, gray2, CV_BGR2GRAY );
    GaussianBlur( gray1, dst1, Size(11,11), 0, 0 ) ;
    GaussianBlur( gray2, dst2, Size(11,11), 0, 0 ) ;
    absdiff( dst1, dst2, dst3 ) ;
    threshold(dst3,dst,90,255,cv::THRESH_BINARY); 
    */


    /*for (size_t i = 0; i < cloud->points.size (); ++i){
        if( cloud->points[i].z < 4.0 ){
            cout << " " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << endl;
        }
    }*/
    //Create the filtering object
    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pre_filter (new pcl::PointCloud<pcl::PointXYZRGB>) ;
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 4.0);
    //pass.setFilterFieldName("y");
    //pass.setFilterLimits()
    pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_pre_filter);
    
    //pcl::io::savePCDFileASCII("../images/pre_filtered.pcd", *cloud_pre_filter) ;
    cout << "PointCloud after filtering has: " << cloud_pre_filter->points.size () << " data points." << endl ; /
    */
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<pcl::PointXYZRGB> vg ;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>) ;
    vg.setInputCloud (cloud) ;
    vg.setLeafSize (0.01f, 0.01f, 0.01f) ;
    vg.filter (*cloud_filtered) ;
    cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << endl ; 
    //pcl::io::savePCDFileASCII ("../images/cloud_filtered_pcd.pcd", *cloud_filtered) ;

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZRGB> seg ; // Create the segmentation object.
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices) ;
    // Object for storing the plane model coefficients.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients) ;
    //
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ()) ;
    pcl::PCDWriter writer ;
    /*an optimization is performed over the estimated coefficients to minimize or reduce
     * the mean-squared-error with respect the all of the 3D points.If you know you have outliers
     * an no noise,an optimization needn't set to save computational time .*/
    seg.setOptimizeCoefficients (true) ; //
    //seg.setModelType (pcl::SACMODEL_PLANE) ;
    //seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f(0.0,1.0,0.0));
    seg.setEpsAngle (pcl::deg2rad (10.0));
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01); //1cm
    /*
    seg.setModelType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle (0);
    seg.setMethodType (pcl::SAC_RANSAC) ;
    seg.setMaxIterations (100) ;
    seg.setDistanceThreshold (0.01) ;
    */
     pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer"); 
    int i=0, nr_points = (int) cloud_filtered->points.size ();
    //remove the planar
    pcl::PCDWriter writer1 ;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane (new pcl::PointCloud<pcl::PointXYZRGB> ()) ;
    pcl::PointCloud<pcl::PointXYZRGB> plane  ;
    int n = 0 ;
    while (cloud_filtered->points.size () > 0.5 * nr_points){
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered->makeShared());
        seg.segment (*inliers, *coefficients);
        cout << "inliers->indices.size: " <<inliers->indices.size() << endl ;
        if (inliers->indices.size () < 1000){
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }
        //PAINT SURFACE 
        /*
        for (unsigned int i = 0; i < inliers->indices.size(); i++) 
        { 
            int idx = inliers->indices[i]; 
                cloud_filtered->points[idx].r = 255; 
                cloud_filtered->points[idx].g = 0; 
                cloud_filtered->points[idx].b = 0; 
        } */

        viewer.showCloud(cloud_filtered); 
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::stringstream ss;
        ss << "plane" << n << ".pcd";
        writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_plane, false);

        plane += *cloud_plane ;
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
        n++ ;
    }
    writer1.write<pcl::PointXYZRGB>( "Plane.pcd", plane, false );

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices ;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec ;
    ec.setClusterTolerance (0.02) ; // 2cm
    ec.setMinClusterSize (1000) ;
    ec.setMaxClusterSize (25000) ;
    ec.setSearchMethod (tree) ;
    ec.setInputCloud (cloud_filtered) ;
    ec.extract (cluster_indices) ;
    pcl::PointCloud<pcl::PointXYZRGB> result  ;
    result += plane ;
    pcl::PCDWriter writer2 ;
    int j = 0 ;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]); 
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        result += *cloud_cluster ;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
        j++;
    }
    writer2.write<pcl::PointXYZRGB>("result.pcd", result, false) ;
    //namedWindow("binary image",0) ;
    //imshow("binary image", dst) ;
    //waitKey(0) ;
    return (0);
}
