/*************************************************************************
	> File Name: canny.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: Fri 27 Mar 2015 08:53:16 PM CST
 ************************************************************************/

#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
using namespace std;
#define PI 3.1415926

int main( int argc, char**argv ){
    cv::Mat src, gray;
    src = cv::imread( argv[1], CV_LOAD_IMAGE_COLOR ) ;
    if( !src.data ){
        cout << "Cloud not read the image!" << endl ;
        return -1 ;
    }
    cv::cvtColor( src, gray, CV_BGR2GRAY );
    //cv::cvtColor( src2, gray2, CV_BGR2GRAY );
    cv::Mat contours ;
    cv::Canny( gray, contours, 125, 350 );
    cv::threshold( contours, contours, 128, 255, cv::THRESH_BINARY );

    cv::Mat dst(contours.rows, contours.cols, CV_8UC1, cv::Scalar(0)) ;
    for( int rows = 1; rows<contours.rows; rows++){
        for( int cols = 1; cols<contours.cols; cols++){
            if( contours.at< unsigned char >(rows,cols) == 255 ){
                //dst.at<unsigned char>(rows,cols) = 255 ;

                dst.at<unsigned char>(rows-1, cols-1) = 255 ;
                dst.at<unsigned char>(rows-1, cols) = 255 ;
                dst.at<unsigned char>(rows-1, cols+1) = 255 ;
                dst.at<unsigned char>(rows, cols-1) = 255 ;
                dst.at<unsigned char>(rows, cols) = 255 ;
                dst.at<unsigned char>(rows, cols+1) = 255 ;
                dst.at<unsigned char>(rows+1, cols-1) = 255 ;
                dst.at<unsigned char>(rows+1, cols) = 255 ;
                dst.at<unsigned char>(rows+1, cols+1) = 255 ;

            }
        }
    }
    /*
    for( int rows = 1; rows<contours.rows; rows++){
        for( int cols = 1; cols<contours.cols; cols++){
            if( contours.at< unsigned char >(rows,cols) == 255 ){
                //cout << "The value is 255" << endl ;
            }
        }
    }*/
    /*
    vector<cv::Vec4i> lines;
    cv::HoughLinesP(contours,lines,1,CV_PI/180,80,50,10);
    drawDetectLines(src,lines,cv::Scalar(0,255,0));
    cv::namedWindow("Lines");
    cv::imshow("Lines",src);
    */
    
    //cout << "rows=" << contours.rows << " ;cols = " << contours.cols << endl ;
    std::vector<cv::Vec2f> lines;
    cv::HoughLines (contours,lines,1,PI/180,80);  
    std::vector<cv::Vec2f>::const_iterator it = lines.begin ();  
    std::cout<<lines.size ()<<std::endl;  
    while(it != lines.end()){  
        float rho = (*it)[0];  
        float theta = (*it)[1];  
        if(theta<PI/4.||theta>3.*PI/4){  
                //画交点在上下两边的直线   
            cv::Point pt1(rho/cos(theta),0);
            cv::Point pt2((rho-gray.rows*sin(theta))/cos(theta),gray.rows);
            cv::line(src,pt1,pt2,cv::Scalar(255),1);
        }else {
            //画交点在左右两边的直线
            cv::Point pt1(0,rho/sin(theta));
            cv::Point pt2(gray.cols,(rho-gray.cols*cos(theta)/sin(theta)));
            cv::line(src,pt1,pt2,cv::Scalar(255),1);
        }
        ++it;
    }
    cv::namedWindow ("hough");
    cv::imshow("hough",src);
    cv::namedWindow( "Canny" );
    cv::imshow( "Canny", contours );
    cv::waitKey() ;
    return 0;
}


