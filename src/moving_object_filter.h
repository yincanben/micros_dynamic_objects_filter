/*************************************************************************
	> File Name: moving_object_filter.h
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: 2015年01月09日 星期五 08时05分32秒
 ************************************************************************/

#ifndef _MOVING_OBJECT_FILTER_H
#define _MOVING_OBJECT_FILTER_H

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <iostream>
#include <stdio.h>
#include <vector>
#include <string>
#include <ctime>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigen>

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

#include <pcl/pcl_base.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>


typedef pcl::PointXYZRGB point_type ;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_type ;

#define threshod_binary 30
class MovingObjectFilter{
    public:
        MovingObjectFilter( int argc, char**argv ) ;

        void processData( const cv::Mat & image, const cv::Mat &depth,
                          float fx,float fy,
                          float cx,float cy );

    private:
        ros::Publisher rgbPub_ ;
        ros::Publisher depthPub_ ;
        ros::Publisher cloudPub_ ;

        //cv::Ptr<cv::FeatureDetector> featureDetector_ ;
        //cv::Ptr<cv::DescriptorExtractor> descriptorExtractor_;
        //cv::Ptr<cv::DescriptorMatcher> descriptorMatcher_ ;

        cv::Mat lastImage ;
        cv::Mat lastDepth ;
        cv::Mat lastBlurImage ;
        cv::Mat lastFrame ;
        cv::Mat currentFrame ;
        cv::Mat Homography ; //homography
        //cv::Mat restImage ;

        //static int num = 0  ;

        //save the PointCloud which is calculated
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr restCloud ;
        //save the last PointCloud
        cloud_type lastCloud ;

        //pcl::visualization::CloudViewer cloud_viewer ;
        pcl::visualization::CloudViewer result_viewer ;
        //pcl::visualization::CloudViewer viewer ;

        double threshod ;

        //match between two image and calculate the homography
        void computeHomography(cv::Mat &grayImage) ;

        //calculate the difference between
        void image_diff(const cv::Mat &image, cloud_type::ConstPtr cloud);

        //std::vector<cv::DMatch> match_and_filter(const cv::Mat& descriptors);
        //void image_diff(const cv::Mat &image, const cv::Mat &depth) ;
        //int decimation = 1 ;
        //cv::Rect MovingObjectFilter::computeRoi(const cv::Mat & image, const std::vector<float> & roiRatios);
        //std::vector<cv::KeyPoint> MovingObjectFilter::generateKeypoints(const cv::Mat & image, int maxKeypoints, const cv::Rect & roi) const;

        pcl::PointXYZ projectDepthTo3D(
                const cv::Mat & depthImage,
                float x, float y,
                float cx, float cy,
                float fx, float fy,
                bool smoothing,
                float maxZError = 0.02f );

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudFromDepthRGB(
                        const cv::Mat & imageRgb,
                        const cv::Mat & imageDepth,
                        float cx, float cy,
                        float fx, float fy,
                        int decimation);


        cv::Mat pcl_segmentation( cloud_type::ConstPtr cloud , const cv::Mat &image , const cv::Mat &depthImage, float cx, float cy, float fx, float fy ) ;
        bool image_extract_cluster( cloud_type::ConstPtr cluster_cloud, cloud_type::ConstPtr cloud, const cv::Mat &image , float cx, float cy, float fx, float fy , int num , int j ) ;
        pcl::PointCloud<pcl::PointXYZRGB> objectFromOriginalCloud(cloud_type::ConstPtr clusterCloud, cloud_type::ConstPtr cloud);
        cv::Mat getDepth(cloud_type::ConstPtr cloud , const cv::Mat &depthImage, float cx, float cy, float fx, float fy) ;
        void getImage(cloud_type::ConstPtr cloud , const cv::Mat &image, float cx, float cy, float fx, float fy) ;
        cv::Mat bgrFromCloud(const pcl::PointCloud<pcl::PointXYZRGB> & cloud, bool bgrOrder);
        cv::Mat depthFromCloud(
                const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
                float & fx,
                float & fy,
                bool depth16U);


        //std::vector<Eigen::Vector3f> previous_coordinate ;
        //std::vector<Eigen::Vector3f> current_coordinate ;
        //int count ;

    
};
#endif
