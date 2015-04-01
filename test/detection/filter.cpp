/*************************************************************************
	> File Name: filter.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: Wed 01 Apr 2015 10:26:00 AM CST
 ************************************************************************/

#include<iostream>
#include "filter.h"
using namespace std;

void image_diff(const cv::Mat&lastImage, const cv::Mat &currentImage, cloud_type::ConstPtr lastCloud, cloud_type::ConstPtr currentCloud);
void computeHomography(cv::Mat &lastImage, cv::Mat &currentImage, cv::Mat Homography);
Eigen::Vector3f v1 ;
Eigen::Vector3f v2 ;
cv::Mat Homography ;
std::vector<Eigen::Vector3f> previous_coordinate ;
std::vector<Eigen::Vector3f> current_coordinate ;


int main(int argc, char**argv){
    //read image
    cv::Mat lastImage = cv::imread( argv[1], CV_LOAD_IMAGE_COLOR );
    cv::Mat currentImage = cv::imread( argv[2], CV_LOAD_IMAGE_COLOR );
    cv::Mat lastGray , currentGray ;
	if(!lastImage.data || !currentImage.data){
	    cout << "Could not read the image!" << endl ;
	 	return -1 ;
	}

    cv::cvtColor(lastImage, lastGray, CV_BGR2GRAY) ;
    cv::cvtColor(currentImage, currentGray, CV_BGR2GRAY) ;

    //read PointCloud
    /*
    cloud_type::ptr lastCloud (new cloud_type)  ;
    cloud_type::ptr currentCloud(new cloud_type) ;
    if( pcl::io::loadPCDFile<point_type>(argv[3], *lastCloud)==-1 || pcl::io::loadPCDFile<point_type>(argv[4], *currentCloud)==-1 ){
        cout << "Couldn't load PCD file!" << endl ;
        return -1 ;
    }*/
    
    //calculate Homography
    computeHomography(lastGray, currentGray, Homography);


	return 0;
}
void  computeHomography(cv::Mat &lastImage, cv::Mat &currentImage, cv::Mat Homography){

     //Step 1: Detect the keypoints using ORB Detector
     cv::ORB orb ;
     std::vector<cv::KeyPoint> lastKeypoints ;
     std::vector<cv::KeyPoint> currentKeypoints ;
     
     //Step 2: Calculate descriptors (features vectors)
     cv::Mat lastDescriptors ;
     cv::Mat currentDescriptors ;

     if(!lastImage.empty()){
         orb( lastImage, cv::Mat(), lastKeypoints, lastDescriptors) ;
         orb( currentImage, cv::Mat(), currentKeypoints, currentDescriptors );
     }

     cv::BFMatcher matcher(cv::NORM_L2) ;
     //cv::BruteForceMatcher<cv::HammingLUT> matcher ;
     std::vector<cv::DMatch> matches ;
     //matcher.match( lastDescriptors, descriptors, matches );
     std::vector<cv::DMatch> goodMatches ;
     double minDist = 1000.0 , maxDist = 0.0 ;
     cv::Mat img_matches ;
     std::vector<cv::Point2f> lastPoint ;
     std::vector<cv::Point2f> currentPoint ;
     cv::Mat shft ;
     //cv::Mat  Homography ;

     if(!lastDescriptors.empty()){
         //cout << "************" << endl ;
         matcher.match( lastDescriptors, currentDescriptors, matches );
         for(int i=0; i<lastDescriptors.rows; i++){
             double dist = matches[i].distance ;
             if(dist < minDist)
                 minDist = dist ;
             if(dist > maxDist)
                 maxDist = dist ;    
         }

         for(int i=0; i<lastDescriptors.rows; i++){
             if( matches[i].distance < 0.6*maxDist ){
                 goodMatches.push_back(matches[i]);
             }
         }

         //draw matches

         
         cv::namedWindow("matches") ;
         cv::drawMatches( lastImage, lastKeypoints, currentImage, currentKeypoints, goodMatches,img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
         imshow("matches", img_matches) ;
         cv::imwrite( "matches.jpg", img_matches ) ;
         

         if(cv::waitKey(1) > 0){
             exit(0);
         }
        
         if(goodMatches.size() > 4){
             for( int i=0; i<goodMatches.size();i++ ){
                 lastPoint.push_back( lastKeypoints[goodMatches[i].queryIdx].pt );
                 currentPoint.push_back(currentKeypoints[goodMatches[i].trainIdx].pt );
             }
             Homography = cv::findHomography( lastPoint, currentPoint, CV_RANSAC ) ;
             shft = ( cv::Mat_<double>(3,3)<< 1.0, 0, lastImage.cols, 0, 1.0, 0, 0, 0, 1.0) ;
         }

         //warp the image
         
         cv::Mat dst ;
         //cv::warpPerspective( lastImage, dst, Homography, cv::Size(lastImage.cols + grayImage.cols + lastImage.cols, lastImage.rows));
         cv::warpPerspective( lastImage, dst, shft*Homography, cv::Size(lastImage.cols + currentImage.cols + lastImage.cols, lastImage.rows));

         cv::Mat rightImage = currentImage ;
         rightImage.copyTo(cv::Mat( dst, cv::Rect( lastImage.cols, 0, currentImage.cols, currentImage.rows  ) )) ;
         cv::namedWindow("warpImage") ;
         cv::imshow("warpImage", dst) ;
         imwrite("warpImage.jpg", dst);
         cv::waitKey(1) ;
         

     }
     //return Homography ;

 }
void image_diff(const cv::Mat&lastImage, const cv::Mat &currentImage, cloud_type::ConstPtr lastCloud, cloud_type::ConstPtr currentCloud){
    cv::Mat lastBlurImage, currentBlurImage ;
    cv::GaussianBlur( lastImage, lastBlurImage, cv::Size( 11, 11 ), 0, 0 );
    cv::GaussianBlur( currentImage, currentBlurImage, cv::Size( 11, 11 ), 0, 0 );

    //Calculate the difference of image
    cv::Mat lastFrame( 480,640, CV_8UC1, cv::Scalar(0) );
    cv::Mat currentFrame( 480,640, CV_8UC1, cv::Scalar(0) );
    double threshod = 40 ;
    cv::namedWindow("lastFrame") ;
    cv::namedWindow("currentFrame") ;
    for( int rows=0; rows < lastImage.rows; rows++ ){
        for(int cols=0; cols< lastImage.cols; cols++){
            cv::Mat srcMat = cv::Mat::zeros(3,1,CV_64FC1);
            srcMat.at<double>(0,0) = cols ;
            srcMat.at<double>(1,0) = rows;
            srcMat.at<double>(2,0) = 1.0 ;
            cv::Mat warpMat = Homography * srcMat ;
            cv::Point warpPt ;
            warpPt.x = cvRound( warpMat.at<double>(0,0) / warpMat.at<double>(2,0) ) ;
            warpPt.y = cvRound( warpMat.at<double>(1,0) / warpMat.at<double>(2,0) ) ;

            if( warpPt.x>0 && warpPt.x<640  &&  warpPt.y>0 && warpPt.y< 480){

                double imageDiff = abs( lastBlurImage.at<unsigned char>(rows, cols) - currentBlurImage.at<unsigned char>(warpPt.y ,warpPt.x));

                double lastDepthValue = 0.0 ;
                double depthValue = 0.0 ;
                Eigen::Vector3f v1 ;
                Eigen::Vector3f v2 ;

                if( imageDiff > threshod ){
                        //currentFrame.at<unsigned char>(warpPt.y ,warpPt.x) = 255 ;

                    lastDepthValue = isnan( lastCloud->at( cols,rows).z) ? 20 : lastCloud->at(cols,rows).z ;
                    depthValue = isnan( currentCloud->at(warpPt.x, warpPt.y).z) ? 20 : currentCloud->at(warpPt.x, warpPt.y).z ;
                    if( lastDepthValue - depthValue > 0.2 ){
                        lastFrame.at<unsigned char>(rows, cols) = 255 ;
                        v1 << lastCloud->at(cols,rows).x , lastCloud->at(cols,rows).y , lastCloud->at(cols,rows).z ;
                        previous_coordinate.push_back(v1) ;
                    }else if( depthValue -lastDepthValue > 0.2 ){

                        currentFrame.at<unsigned char>(warpPt.y ,warpPt.x) = 255 ;
                        v2 << currentCloud->at(warpPt.x,warpPt.y).x , currentCloud->at(warpPt.x,warpPt.y).y , currentCloud->at(warpPt.x,warpPt.y).z ;
                            //v2 << cloud->at(cols,rows).x , cloud->at(cols,rows).y , cloud->at(cols,rows).z ;
                        current_coordinate.push_back(v2) ;

                    }else{
                        continue ;
                    }

                }else{
                    continue ;
                }
            }else{
                continue ;
            }
        }
    }

    cv::imshow("lastFrame",lastFrame);
    cv::imshow("currentFrame",currentFrame) ;
    
    if(cv::waitKey(1) > 0){
        exit(0);
    }


}
