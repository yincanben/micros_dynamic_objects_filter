/*************************************************************************
	> File Name: image_proc.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: Fri 19 Dec 2014 09:28:24 AM CST
 ************************************************************************/

#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

int threshold_value = 0;
int threshold_type = 3;;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

char* window_name = "Threshold Demo";

char* trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
char* trackbar_value = "Value";

/// 自定义函数声明
Mat dst3, dst ;
Mat src1, src2, gray1, gray2, dst1, dst2, previousFrame, nextFrame ;
void Threshold_Demo( int, void* );

/**
 * @主函数
 */

int main(int argc , char** argv){

    double threshold_diff = 30 ;
    //double beta , input ;
    src1 = imread("../images/image1.jpg") ;
    src2 = imread("../images/image4.jpg") ;
    //previousFrame.create( src1.rows, src1.cols, CV_8UC1, Scalar::all(0)) ;
    //nextFrame.create( src1.rows, src1.cols, CV_8UC1, Scalar::all(0)) ;
    //dst.create( src1.rows, src1.cols, CV_8UC1, Scalar::all(0)) ;
    if(!src1.data)(cout << "image1 hasn't data.") ;
    if(!src2.data)(cout << "image1 hasn't data.") ;
    cvtColor( src1, gray1, CV_BGR2GRAY ) ;
    cvtColor( src2, gray2, CV_BGR2GRAY ) ;
    imshow("gray1", gray1) ;
    imshow("gray2", gray2) ;
    GaussianBlur( gray1, dst1, Size( 11, 11 ), 0, 0 );
    GaussianBlur( gray2, dst2, Size( 11, 11 ), 0, 0 );
    //imshow("GuassiBlur1",dst1) ;
    //imshow("GussiBlur",dst2) ;
    namedWindow("Diffence image", 1);
    //calculate the diffence of gray image and get the abs
    absdiff( dst1, dst2, dst3 ) ;
    /*
    for(int i=0; i<src1.rows; i++ ){
        for(int j=0; j<src1.cols; j++){
            if( (dst1.at<unsigned char>(i,j) - dst2.at<unsigned char>(i,j) ) >= threshold_diff ){
                dst.at<unsigned char>(i,j) = 255 ;
                previousFrame.at<unsigned char>(i,j) = 255 ;
            }else if( ( dst1.at<unsigned char>(i,j) - dst2.at<unsigned char>(i,j) ) <= -threshold_diff ){
                dst.at<unsigned char>(i,j) = 255 ;
                nextFrame.at<unsigned char>(i,j) = 255 ;
            }else{
                dst.at<unsigned char>(i,j) = 0 ;
                previousFrame.at<unsigned char>(i,j) = 0 ;
                nextFrame.at<unsigned char>(i,j) = 0 ;
            }
        }
    }*/
    //imshow ("previousFrame", previousFrame) ;
    //imshow ("nextFrame", nextFrame ) ; 
    imshow("Diffence image", dst3 );
    //waitKey(0);
    
    /// 创建一个窗口显示图片
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );


    /// 创建滑动条来控制阈值
    createTrackbar( trackbar_type, window_name, &threshold_type, max_type, Threshold_Demo );

    createTrackbar( trackbar_value, window_name, &threshold_value, max_value, Threshold_Demo );

    /// 初始化自定义的阈值函数
    Threshold_Demo( 0, 0 );

    /// 等待用户按键。如果是ESC健则退出等待过程。
    while(true){
        int c;
        c = waitKey( 20 );
        if( (char)c == 27 ){
            break; 
        }
    }
    return 0 ;
}
/**
 * @自定义的阈值函数
 */
 
void Threshold_Demo( int, void* )
{
    /* 0: 二进制阈值
     1: 反二进制阈值
     2: 截断阈值
     3: 0阈值
     4: 反0阈值
    */

    threshold( dst3, dst, threshold_value, max_BINARY_value,threshold_type );
    imshow( window_name, dst );
}
