/*************************************************************************
	> File Name: wapImage.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: Tue 24 Mar 2015 08:34:01 AM CST
 ************************************************************************/

#include<iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
int main(int argc, char**argv){
    cv::Mat leftImage = cv::imread( argv[1], CV_LOAD_IMAGE_COLOR );
    cv::Mat rightImage = cv::imread( argv[2], CV_LOAD_IMAGE_COLOR );
    cv::Mat leftGray , rightGray ;
	if(!leftImage.data){
	    cout << "Cloud not read the image!" << endl ;
	 	return -1 ;
	}
    cv::cvtColor(leftImage, leftGray, CV_BGR2GRAY) ;
    cv::cvtColor(rightImage, rightGray, CV_BGR2GRAY) ;
	cv::ORB orb ;
    std::vector<cv::KeyPoint> leftKeypoints ;
    std::vector<cv::KeyPoint> rightKeypoints ;
     
    //featureDetector_ ->detect( grayImage, keypoints ) ;
    //ROS_INFO( "The size of keypoints: %d", keypoints.size() ) ;
    //if( keypoints.size() == 0 )
    //    return ;
    //Step 2: Calculate descriptors (features vectors)
    cv::Mat leftDescriptors ;
    cv::Mat rightDescriptors ;

     
    orb( leftGray, cv::Mat(), leftKeypoints, leftDescriptors) ;
    orb( rightGray, cv::Mat(), rightKeypoints, rightDescriptors );
    

    cv::BFMatcher matcher(cv::NORM_L2) ;
    //cv::BruteForceMatcher<cv::HammingLUT> matcher ;
    std::vector<cv::DMatch> matches ;
    //matcher.match( lastDescriptors, descriptors, matches );
    std::vector<cv::DMatch> goodMatches ;
    double minDist = 1000.0 , maxDist = 0.0 ;
    cv::Mat img_matches ;
    std::vector<cv::Point2f> leftPoint ;
    std::vector<cv::Point2f> rightPoint ;
    cv::Mat Homography ;
    cv::Mat shft ;


    if(!leftDescriptors.empty()){
        //cout << "************" << endl ;
        matcher.match( leftDescriptors, rightDescriptors, matches );
        for(int i=0; i<leftDescriptors.rows; i++){
            double dist = matches[i].distance ;
            if(dist < minDist)
                minDist = dist ;
             if(dist > maxDist)
                maxDist = dist ;    
        }

        for(int i=0; i<leftDescriptors.rows; i++){
            if( matches[i].distance < 0.6*maxDist ){
                goodMatches.push_back(matches[i]);
            }
        }

        //draw matches
         
        cv::namedWindow("matches") ;
        cv::drawMatches( leftGray, leftKeypoints, rightGray, rightKeypoints, goodMatches,img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        imshow("matches", img_matches) ;
        cv::imwrite( "matches.jpg", img_matches ) ;
        if(cv::waitKey(1) > 0){
            exit(0);
        }

        if(goodMatches.size() > 4){
            for( int i=0; i<goodMatches.size();i++ ){
                leftPoint.push_back( leftKeypoints[goodMatches[i].queryIdx].pt );
                rightPoint.push_back( rightKeypoints[goodMatches[i].trainIdx].pt );
            }
            Homography = cv::findHomography( leftPoint, rightPoint, CV_RANSAC ) ;
            shft = ( cv::Mat_<double>(3,3)<< 1.0, 0, leftImage.cols, 0, 1.0, 0, 0, 0, 1.0) ;
        }
        cv::Mat dst ;
        cv::warpPerspective( leftImage, dst, shft*Homography, cv::Size(leftImage.cols + rightImage.cols, leftImage.rows));
        rightImage.copyTo(cv::Mat( dst, cv::Rect( leftImage.cols, 0, rightImage.cols, rightImage.rows  ) )) ;
        cv::imwrite("warpImage.jpg", dst);
        cv::namedWindow("warpImage") ;
        cv::imshow("warpImage", dst) ;
        cv::waitKey(0) ;
     }
}

