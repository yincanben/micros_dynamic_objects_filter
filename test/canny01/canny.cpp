/*************************************************************************
	> File Name: canny.cpp
	> Author: 
	> Mail: 
	> Created Time: Mon 06 Apr 2015 06:19:26 PM CST
 ************************************************************************/

#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
using namespace std;
using namespace cv;
int main(int argc, char** argv){
    Mat src = imread(argv[1], CV_LOAD_IMAGE_COLOR ) ;
    if(!src.data){
        cout << "Could not read image." << endl ;
        return -1;
    }
    cvtColor(src,src,CV_BGR2GRAY);
    vector<vector<Point> > contours ;
    findContours(src, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE) ;
    Mat result(src.size(),CV_8UC1,Scalar(0)) ;
    drawContours(result,contours,-1, Scalar(255),2) ;
    namedWindow("contours") ;
    imshow("contours", result) ;
    imwrite("result.jpg", result) ;
    //WaitKey(1) ;
    return 0;
}
