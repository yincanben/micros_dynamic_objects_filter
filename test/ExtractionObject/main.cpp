/*************************************************************************
	> File Name: main.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: Tue 07 Apr 2015 10:37:55 AM CST
 ************************************************************************/

#include<iostream>
//#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>
//#include <cv_bridge/cv_bridge.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>

//#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>

//#include <iostream>
//#include <stdio.h>
//#include <vector>
//#include <string>

//#include <opencv2/opencv.hpp>
//#include <opencv2/core/core.hpp>
//#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/calib3d/calib3d.hpp>
//#include <opencv2/nonfree/nonfree.hpp>
//#include <opencv2/nonfree/features2d.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/time.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/angles.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/io/pcd_io.h>
#include <pcl/pcl_base.h>



#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


typedef pcl::PointXYZRGB point_type ;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_type ;
using namespace std ;

int main(int argc, char**argv){
    cloud_type::Ptr clusterCloud (new cloud_type)  ;
    cloud_type::Ptr originalCloud(new cloud_type) ;
    if( pcl::io::loadPCDFile<point_type>(argv[1], *clusterCloud)==-1 || pcl::io::loadPCDFile<point_type>(argv[2], *originalCloud)==-1 ){
        cout << "Couldn't load PCD file!" << endl ;
        return -1 ;
    }

    double average_z = 0.0 ;
    for(int i = 0; i < clusterCloud->size(); i++){
        average_z += clusterCloud->points[i].z ;
    }
    average_z = average_z / clusterCloud->size() ;
    cloud_type::Ptr cluster (new cloud_type)  ;
    //pcl::visualization::CloudViewer viewCluster("cluster");
    cluster->points.resize(clusterCloud->size());

    for(int i = 0; i < clusterCloud->size();i++ ){
        cluster->points[i].x = clusterCloud->points[i].x ;
        cluster->points[i].y = clusterCloud->points[i].y ;
        cluster->points[i].z = average_z ;
        cluster->points[i].b = clusterCloud->points[i].b ;
        cluster->points[i].g = clusterCloud->points[i].g ;
        cluster->points[i].r = clusterCloud->points[i].r ;
    }
    cluster->width = cluster->points.size ();
    cluster->height = 1;
    cluster->is_dense = true;
    //viewCluster.showCloud(cluster) ;
    //while(! viewCluster.wasStopped()){
    //
    //}
    int j=0 ;
    pcl::PCDWriter writer ;
    std::stringstream ss;
    ss << "PlaneCloud_" << j << ".pcd";
    writer.write<pcl::PointXYZRGB> (ss.str (), *cluster, false);
    j++;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hull_points (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PointIndices::Ptr cloud_indices (new pcl::PointIndices);

    //    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    //
    //    // hull.setDimension (2); // not necessarily needed, but we need to check the dimensionality of the output
    //    hull.setInputCloud (cluster);
    pcl::ConcaveHull<pcl::PointXYZRGB> hull ;
    hull.setInputCloud (cluster);
    hull.setAlpha(0.1);
    hull.reconstruct (*hull_points);
    std::stringstream ss1;
    ss1 << "ConcaveHullCloud_" << j << ".pcd";
    writer.write<pcl::PointXYZRGB> (ss1.str (), *hull_points, false);
    j++ ;

    //    pcl::visualization::CloudViewer viewConvex("Convex hull") ;
    //    viewConvex.showCloud(hull_points);
    //    while(!viewConvex.wasStopped()){

//    }
    double z_min = -0.2, z_max = 0.2; // we want the points above the plane, no farther than 5 cm from the surface
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (hull.getDimension () == 2){
        pcl::ExtractPolygonalPrismData<pcl::PointXYZRGB> prism;
        prism.setInputCloud (originalCloud);
        prism.setInputPlanarHull (hull_points);
        prism.setHeightLimits (z_min, z_max);
        prism.segment (*cloud_indices);
        if(cloud_indices->indices.size() == 0){
            cout << "Cloud not find objects" << endl ;
            return 0;
        }else{
            cout << "indices.size() = " << cloud_indices->indices.size() << endl ;
        }
        extract.setInputCloud(originalCloud);
        extract.setIndices(cloud_indices);
        extract.filter(*objects);
        pcl::visualization::CloudViewer viewerObjects("Objects");
        viewerObjects.showCloud(objects);
        while (!viewerObjects.wasStopped())
        {
            // Do nothing but wait.
        }

    }else
        PCL_ERROR ("The input cloud does not represent a planar surface.\n");

    return 0 ;
}

