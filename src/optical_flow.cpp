/*************************************************************************
	> File Name: optical_flow.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: Tue 24 Mar 2015 04:22:42 PM CST
 ************************************************************************/

//#include<iostream>
//using namespace std;
#include "optical_flow.h"
OpticalFlow::OpticalFlow(){
    num_keypoints_param_= 50 ;
    matchscore_thresh_param_ = 10e8 ;

}
OpticalFlow::~OpticalFlow(){

}

void OpticalFlow::process(cv::Mat &input_image){
    cv::pyrDown(input_image, input_image) ;

    // Grab a new keyframe whenever we have lost more than half of our tracks
    if(key_corners_.size() < size_t(num_keypoints_param_ / 2)){
        cv::goodFeaturesToTrack(input_image, key_corners_, num_keypoints_param_, 0.01, 30);
        key_image_ = input_image.clone();
    }
    // Track the features from the keyframe to the current frame
    std::vector<cv::Point2f> new_corners;
    trackFeatures(key_image_, input_image, key_corners_, new_corners);

    cv::Mat warped_key_image ;
    if(new_corners.size() < size_t(num_keypoints_param_/2)){
        // If we didn't track enough points, then kill the keyframe
        key_corners_.clear();
    }else{
        // Find the homography between the last frame and the current frame
        cv::Mat homography = cv::findHomography(key_corners_, new_corners, CV_RANSAC);

        // Warp the keyframe image to the new image, and find the squared difference
        cv::warpPerspective(key_image_, warped_key_image, homography, key_image_.size());
        cv::Mat matchScore;
        cv::matchTemplate(input_image, warped_key_image, matchScore, CV_TM_SQDIFF);
        //ROS_INFO("Match Score: %f", matchScore.at<float>(0,0));

             // If the difference between the warped template and the new frame is too large,
             // then kill the keyframe
        if(matchScore.at<float>(0,0) > matchscore_thresh_param_)
            key_corners_.clear();
    }
    // Draw the warped keyframe
    if(key_corners_.size())
        cv::imshow("homography", warped_key_image);

    // Draw the features on the input image
    if(key_corners_.size())
        drawFeatures(input_image, key_corners_, new_corners);
    cv::imshow("tracker (press key for keyframe)", input_image);

    if(cv::waitKey(2) != -1)
        key_corners_.clear();

}

// ######################################################################
//! Draw the features onto the image, and draw lines between matches
void OpticalFlow::drawFeatures( cv::Mat & image, std::vector<cv::Point2f> const & old_corners, std::vector<cv::Point2f> const & new_corners ){
    assert(old_corners.size() == new_corners.size());
    for(size_t i=0; i<new_corners.size(); ++i){
        cv::rectangle(image, old_corners[i]-cv::Point2f(3,3), old_corners[i]+cv::Point2f(2,2), 255, 5);
        cv::rectangle(image, old_corners[i]-cv::Point2f(3,3), old_corners[i]+cv::Point2f(3,3), 0,   1);

        cv::rectangle(image, new_corners[i]-cv::Point2f(3,3), new_corners[i]+cv::Point2f(2,2), 0,   5);
        cv::rectangle(image, new_corners[i]-cv::Point2f(3,3), new_corners[i]+cv::Point2f(3,3), 255, 1);

        cv::line(image, old_corners[i], new_corners[i], 0,   5);
        cv::line(image, old_corners[i], new_corners[i], 255, 1);
    }
}

// ######################################################################
//! Filter out all points from p1 and p2 who's corresponding status byte == 0
void OpticalFlow::filterPoints( std::vector<cv::Point2f> & p1, std::vector<cv::Point2f> & p2, std::vector<unsigned char> const & status){
    std::vector<cv::Point2f> p1_filt;     p1_filt.reserve(p1.size());
    std::vector<cv::Point2f> p2_filt; p2_filt.reserve(p1.size());

    std::vector<cv::Point2f>::iterator p1_it = p1.begin();
    std::vector<cv::Point2f>::iterator p2_it = p2.begin();
    std::vector<unsigned char>::const_iterator status_it = status.begin();

    while(status_it != status.end()){
        if(*status_it > 0){
            p1_filt.push_back(*p1_it);
            p2_filt.push_back(*p2_it);
        }
        ++p1_it;
        ++p2_it;
        ++status_it;
    }
    p1 = p1_filt;
    p2 = p2_filt;
}

// ######################################################################
//! Perform forward/backward LK tracking.
/*! @param key_image The old keyframe image
    @param curr_image The new incoming image
    @param corners The old corners, found (e.g. by cv::goodFeaturesToTrack) in the key_image
    @param new_corners A vector which will be filled with the locations of the corners in curr_image

    \note This method tries very hard to filter out bad tracks by performing both a forwards, and a backwards LK tracking step.
          All untrackable points will be removed from the corners vector, so that corners and new_corners will have the same size
          after trackFeatures() is finished. */
void OpticalFlow::trackFeatures(cv::Mat key_image, cv::Mat curr_image, std::vector<cv::Point2f> & corners, std::vector<cv::Point2f> & new_corners){
    cv::Size const searchWindow(15, 15);

    new_corners.resize(corners.size());

    if(corners.size() == 0) return;

    // Perform the forward LK step
    std::vector<unsigned char> status(corners.size());
    std::vector<float> error(corners.size());
    calcOpticalFlowPyrLK(key_image, curr_image, corners, new_corners, status, error, searchWindow, 5);

    // Filter out any untrackable points
    filterPoints(corners, new_corners, status);

    if(corners.size() == 0) return;

    // Perform the backwards LK step
    std::vector<cv::Point2f> back_corners;
    calcOpticalFlowPyrLK(curr_image, key_image, new_corners, back_corners, status, error, searchWindow, 5);

    // Filter out points that either:
    // a) Are untrackable by LK (their status byte == 0)
    // or
    // b) Were tracked by the backward LK step to a position that was too far away from the original point. This indicates
    //    that even though LK though it had a good track, the point path was ambiguous.
    std::vector<cv::Point2f> filt_corners;
    std::vector<cv::Point2f> filt_old_corners;
    std::vector<cv::Point2f>::iterator corners_it = corners.begin();
    std::vector<cv::Point2f>::iterator new_corners_it = new_corners.begin();
    std::vector<cv::Point2f>::iterator back_corners_it = back_corners.begin();
    std::vector<unsigned char>::iterator status_it = status.begin();
    while(status_it != status.end()){
        float const dist = sqrt(pow(new_corners_it->y - back_corners_it->y, 2) + pow(new_corners_it->y - back_corners_it->y, 2));
        if(*status_it && dist < 10){
            filt_old_corners.push_back(*corners_it);
            filt_corners.push_back(*new_corners_it);
        }
        ++corners_it;
        ++new_corners_it;
        ++back_corners_it;
        ++status_it;
    }
    new_corners = filt_corners;
    corners     = filt_old_corners;
}
