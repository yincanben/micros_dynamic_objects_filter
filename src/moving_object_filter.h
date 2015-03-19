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

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core> 

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
#include <pcl/surface/concave_hull.h>

typedef pcl::PointXYZRGB point_type ;
//typedef pcl::PointCloud<point_type> cloud_type ;
typedef pcl::PointCloud<pcl::PointXYZRGB> cloud_type ;

#define threshod_binary 30
class MovingObjectFilter{
    public:
        MovingObjectFilter( int argc, char**argv ) ;

        void processData( const cv::Mat & image, const cv::Mat &depth,
                          float fx,float fy,
                          float cx,float cy );

        //void ExtractObject( cloud_type::ConstPtr cloud, cv::Mat& gray_img ) ;
        //cloud_type::ConstPtr get_filter_cloud();
        //void clear_object_cloud(  );

    private:
        ros::Publisher rgbPub_ ;
        ros::Publisher depthPub_ ;
        ros::Publisher cloudPub_ ;
        cv::Ptr<cv::FeatureDetector> featureDetector_ ;
        cv::Ptr<cv::DescriptorExtractor> descriptorExtractor_;
        cv::Ptr<cv::DescriptorMatcher> descriptorMatcher_ ;
        std::vector<cv::KeyPoint> last_keypoints_ ;
        cv::Mat lastImage ;
        cv::Mat lastDepth ;
        cv::Mat lastFrame ;
        cv::Mat currentFrame ;

        double threshod ;


        void computeHomography(cv::Mat &grayImage) ;
        std::vector<cv::DMatch> match_and_filter(const cv::Mat& descriptors);
        void image_diff(const cv::Mat &image, const cv::Mat &depth) ;

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

        void image_separate( cloud_type::ConstPtr cloud ) ;
        void pcl_segmentation( cloud_type::ConstPtr cloud ) ;
        bool image_extract_cluster( cloud_type::ConstPtr cloud ) ;
        void transform_coordinate(cloud_type::ConstPtr cloud) ;
        cv::Mat last_image;
        cv::Mat binary_image ;
        cv::Mat previous_frame ;
        cv::Mat current_frame ;
        cv::Mat Homography ; //homography

        //pcl::PointCloud<pcl::PointXYZRGB>::Ptr last_cloud ;
        cloud_type last_cloud ;
        cloud_type object_cloud ; //save the moving object
        cloud_type filter_cloud ; //save the rest cloud(i.e don't include moving object)

        int frame_count  ;//set the interval of frame_count
        std::vector<Eigen::Vector3f> previous_coordinate ;
        std::vector<Eigen::Vector3f> current_coordinate ;
        double previous_z , current_z ;
        double deta_x , deta_y ;
        int count ;

    
};
#endif
