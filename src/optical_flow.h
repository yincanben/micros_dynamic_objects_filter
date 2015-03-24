/*************************************************************************
	> File Name: optical_flow.h
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: Tue 24 Mar 2015 04:23:18 PM CST
 ************************************************************************/

#ifndef _OPTICAL_FLOW_H
#define _OPTICAL_FLOW_H

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <boost/timer.hpp>

class OpticalFlow{
    public:
        OpticalFlow();
        ~OpticalFlow() ;
        void process (cv::Mat &image);

    private:

        void drawFeatures( cv::Mat & image, std::vector<cv::Point2f> const & old_corners, std::vector<cv::Point2f> const & new_corners) ;
        void filterPoints( std::vector<cv::Point2f> & p1, std::vector<cv::Point2f> & p2, std::vector<unsigned char> const & status) ;
        void trackFeatures(cv::Mat key_image, cv::Mat curr_image, std::vector<cv::Point2f> & corners, std::vector<cv::Point2f> & new_corners) ;
        cv::Mat key_image_;
        std::vector<cv::Point2f> key_corners_;

        int num_keypoints_param_;
        double matchscore_thresh_param_;


};

#endif
