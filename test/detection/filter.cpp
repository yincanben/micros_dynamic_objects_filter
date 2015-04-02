/*************************************************************************
	> File Name: filter.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: Wed 01 Apr 2015 10:26:00 AM CST
 ************************************************************************/

#include<iostream>
#include "filter.h"
using namespace std;

void image_diff(const cv::Mat &lastImage, const cv::Mat &currentImage, cloud_type::ConstPtr lastCloud, cloud_type::ConstPtr currentCloud, cv::Mat &Homography);

cv::Mat computeHomography(cv::Mat &lastImage, cv::Mat &currentImage );
bool image_extract_cluster( cloud_type::ConstPtr cloud ) ;
void pcl_segmentation( cloud_type::ConstPtr cloud ) ;
Eigen::Vector3f v1 ;
Eigen::Vector3f v2 ;

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
    cv::Mat Homography ;
    cv::cvtColor(lastImage, lastGray, CV_BGR2GRAY) ;
    cv::cvtColor(currentImage, currentGray, CV_BGR2GRAY) ;

    //read PointCloud
    
    cloud_type::Ptr lastCloud (new cloud_type)  ;
    cloud_type::Ptr currentCloud(new cloud_type) ;
    if( pcl::io::loadPCDFile<point_type>(argv[3], *lastCloud)==-1 || pcl::io::loadPCDFile<point_type>(argv[4], *currentCloud)==-1 ){
        cout << "Couldn't load PCD file!" << endl ;
        return -1 ;
    }
    
    //calculate Homography
    Homography = computeHomography(lastGray, currentGray );
    image_diff( lastGray, currentGray, lastCloud, currentCloud, Homography) ;
    pcl_segmentation(currentCloud) ;


	return 0;
}
cv::Mat  computeHomography(cv::Mat &lastImage, cv::Mat &currentImage ){

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
     cv::Mat  Homography ;

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
     return Homography ;

 }
void image_diff(const cv::Mat &lastImage, const cv::Mat &currentImage, cloud_type::ConstPtr lastCloud, cloud_type::ConstPtr currentCloud, cv::Mat &Homography){
    cv::Mat lastBlurImage, currentBlurImage ;
    cv::GaussianBlur( lastImage, lastBlurImage, cv::Size( 11, 11 ), 0, 0 );
    cv::GaussianBlur( currentImage, currentBlurImage, cv::Size( 11, 11 ), 0, 0 );

    //Calculate the difference of image
    cv::Mat lastFrame( 480,640, CV_8UC1, cv::Scalar(0) );
    cv::Mat currentFrame( 480,640, CV_8UC1, cv::Scalar(0) );
    double threshod = 40 ;
    cv::Mat  frame( 480,640, CV_8UC1, cv::Scalar(0) );
    //cv::namedWindow("lastFrame") ;
    //cv::namedWindow("currentFrame") ;

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
                    frame.at<unsigned char>(warpPt.y ,warpPt.x) = 255 ;
                    lastDepthValue = isnan( lastCloud->at( cols,rows).z) ? 20 : lastCloud->at(cols,rows).z ;
                    depthValue = isnan( currentCloud->at(warpPt.x, warpPt.y).z) ? 20 : currentCloud->at(warpPt.x, warpPt.y).z ;


                    if( lastDepthValue - depthValue > 0.2 ){
                        //if(!isnan( lastCloud->at( cols,rows).z)){
                            currentFrame.at<unsigned char>(warpPt.y ,warpPt.x) = 255 ;
                            //lastFrame.at<unsigned char>(rows, cols) = 255 ;
                            v1 << currentCloud->at(warpPt.x, warpPt.y).x , currentCloud->at(warpPt.x, warpPt.y).y , currentCloud->at(warpPt.x, warpPt.y).z ;
                            current_coordinate.push_back(v1) ;
                        //}
                    }else if( depthValue -lastDepthValue > 0.2 ){
                        //if(!isnan( currentCloud->at(warpPt.x, warpPt.y).z)){
                            lastFrame.at<unsigned char>(rows, cols) = 255 ;
                            //currentFrame.at<unsigned char>(warpPt.y ,warpPt.x) = 255 ;
                            v2 << lastCloud->at( cols,rows).x , lastCloud->at( cols,rows).y , lastCloud->at( cols,rows).z ;
                            previous_coordinate.push_back(v2) ;
                        //}
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

    //cv::imshow("lastFrame",lastFrame);
    imwrite( "lastFrame.jpg", lastFrame ) ;
    //cv::imshow("currentFrame",currentFrame) ;
    imwrite( "currentFrame.jpg", currentFrame );
    imwrite("diffImage.jpg", frame);
    
    if(cv::waitKey(1) > 0){
        exit(0);
    }


}
/*
void pcl_segmentation( cloud_type::ConstPtr cloud ){
    
    cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << endl ; //*
    pcl::VoxelGrid<pcl::PointXYZ> vg ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>) ;
    vg.setInputCloud (cloud) ;
    vg.setLeafSize (0.01f, 0.01f, 0.01f) ;
    vg.filter (*cloud_filtered) ;
    cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << endl ; 
    //pcl::io::savePCDFileASCII ("../images/cloud_filtered_pcd.pcd", *cloud_filtered) ;
    
    //Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg ; // Create the segmentation object.
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices) ;
    // Object for storing the plane model coefficients.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients) ;
    //
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ()) ;
    pcl::PCDWriter writer ;
    //*an optimization is performed over the estimated coefficients to minimize or reduce
    // * the mean-squared-error with respect the all of the 3D points.If you know you have outliers
    //* an no noise,an optimization needn't set to save computational time .
    seg.setOptimizeCoefficients (true) ; //
    //seg.setModelType (pcl::SACMODEL_PLANE) ;
    //seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setModelType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setEpsAngle (0);
    //seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    //seg.setAxis(Eigen::Vector3f(0,1,0));
    //seg.setEpsAngle (0);
    seg.setMethodType (pcl::SAC_RANSAC) ;
    seg.setMaxIterations (100) ;
    seg.setDistanceThreshold (0.01) ;
    pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer"); 
    int i=0, nr_points = (int) cloud_filtered->points.size ();
    //remove the planar 
    while (cloud_filtered->points.size () > 0.5 * nr_points){
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered->makeShared());
        seg.segment (*inliers, *coefficients);
        cout << "inliers->indices.size: " <<inliers->indices.size() << endl ;
        if (inliers->indices.size () < 1000){
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
    }
    //PAINT SURFACE

    
    viewer.showCloud(cloud_filtered);
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    
    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;
    
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }
    
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);
    
    std::vector<pcl::PointIndices> cluster_indices ;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec ;
    ec.setClusterTolerance (0.02) ; // 2cm
    ec.setMinClusterSize (1000) ;
    ec.setMaxClusterSize (25000) ;
    ec.setSearchMethod (tree) ;
    ec.setInputCloud (cloud_filtered) ;
    ec.extract (cluster_indices) ;
    
    int j = 0 ;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
    
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        std::stringstream ss;
        ss << "cloud_cluster_" << j << ".pcd";
        writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false);
        j++;
    }
    
}*/

void pcl_segmentation( cloud_type::ConstPtr cloud ){

    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Step 1: Filter out NaNs from data
    //ros::Time last = ros::Time::now() ;
    /*
    pcl::PassThrough<pcl::PointXYZRGB> pass ;
    pass.setInputCloud (cloud); 
    pass.setFilterFieldName ("z"); 
    pass.setFilterLimits (0.5, 6.0); 
    pass.filter (*cloud_filtered);
    */

    //
    //pass.setInputCloud (cloud_filtered); 
    //pass.setFilterFieldName ("x"); 
    //pass.setFilterLimits (-4.0, 4.0); 
    //pass.setFilterLimitsNegative (true); 
    //pass.filter (*cloud_filtered); 

    //pass.setInputCloud (cloud_filtered); 
    //pass.setFilterFieldName ("y"); 
    //pass.setFilterLimits (-4.0, 4.0); 
    //pass.filter (*cloud_filtered);
    //

    // Step 2: Filter out statistical outliers*****************influence the speed
    
    //pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor ;
    //sor.setInputCloud(cloud_filtered) ;
    //sor.setMeanK(50) ;
    //sor.setStddevMulThresh(1.0) ;
    //sor.filter(*cloud_filtered) ;

    // Step 3: Downsample the point cloud (to save time in the next step)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::VoxelGrid<pcl::PointXYZRGB> downSampler;
    downSampler.setInputCloud (cloud);
    downSampler.setLeafSize (0.01f, 0.01f, 0.01f); 
    downSampler.filter (*cloud_filtered);
    cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << endl ;
    
        
    // Step 4: Remove the ground plane using RANSAC 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); 
    pcl::SACSegmentation<pcl::PointXYZRGB> seg ;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices) ;
    seg.setOptimizeCoefficients (true); // Optional 

    seg.setMethodType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE) ;
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setEpsAngle (0);
    seg.setDistanceThreshold (0.01); //1cm
    //seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    //seg.setAxis(Eigen::Vector3f(0.0,1.0,0.0));
    //seg.setEpsAngle (pcl::deg2rad (10.0));


    //seg.setInputCloud (cloud_filtered2->makeShared());
    //seg.segment (*inliers, *coefficients);
    pcl::PCDWriter writer ;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ()) ;
    pcl::visualization::CloudViewer viewer("PCL OpenNI Viewer");
    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.5 * nr_points){
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud_filtered->makeShared());
        seg.segment (*inliers, *coefficients);
        cout << "inliers->indices.size: " <<inliers->indices.size() << endl ;
        if (inliers->indices.size () < 1000){
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        viewer.showCloud(cloud_filtered);
        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud_filtered);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud_filtered = *cloud_f;
    }

    /*
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud_filtered2);
    // Step 4.1: Extract the points that lie in the ground plane
    extract.setIndices (inliers);
    if( inliers->indices.size() > 4000 ){
        cout << "indices size =  " << inliers->indices.size() << endl ;
        extract.setNegative (false);
        extract.filter (*cloud_plane);
    }
    // Step 4.2: Extract the points that are objects(i.e. are not in the ground plane)
    extract.setNegative (true);
    extract.filter ( *cloud_filtered2 ) ;
        
    if (inliers->indices.size () == 0) { 
        PCL_ERROR ("Could not estimate a planar model for the given dataset."); 
        //exit(0); 
    } 
    */

        
    // Step 5: EuclideanCluster Extract the moving objects
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices ;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec ;
    ec.setClusterTolerance (0.02) ; // 2cm
    ec.setMinClusterSize (1000) ;
    ec.setMaxClusterSize (25000) ;
    ec.setSearchMethod (tree) ;
    ec.setInputCloud (cloud_filtered) ;
    ec.extract (cluster_indices) ;

    cout << "The size of cluster_indices : " << cluster_indices.size() << endl;
    int j = 0 ;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (cloud_filtered->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
        if(image_extract_cluster(cloud_cluster)) {
            std::stringstream ss;
            ss << "ResultCloud_" << j << ".pcd";
            writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false);
            j++;
        }

    }
    /*
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    double minX(0.0), minY(0.0), minZ(0.0), maxX(0.0), maxY(0.0), maxZ(0.0) ;
    point_type cluster_point ;
    int num = 0 ;


    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++){
            cluster_point = cloud_filtered2->points[*pit] ;
            cloud_cluster->points.push_back (cluster_point); 
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        //cloud_cluster->width = 640 ;
        //cloud_cluster->height= 480 ;
        //cloud_cluster->resize(cloud_cluster->width * cloud_cluster->height) ;
        cloud_cluster->is_dense = true;
        unsigned int filesSaved = 0 ;
        if(image_extract_cluster(cloud_cluster)) {
            stringstream stream;
            stream << "ResultCloud" << filesSaved << ".pcd";
            string filename = stream.str();
            if (pcl::io::savePCDFile(filename, *cloud_cluster, true) == 0){
                filesSaved++;
                cout << "Saved " << filename << "." << endl;
            }
            num++ ;
            //object_cloud += *cloud_cluster ;
            
            //if(!cloud_viewer.wasStopped()){
            //    cloud_viewer.showCloud(cloud_cluster);
            //}
            //cloud_viewer.showCloud( object_cloud );
            //current_coordinate.clear();
            //std::cout << "true" << std::endl ;
            //break ;
        }else{
            //cloud_cluster->clear() ;
            continue ;
        }
            
        //hull.setInputCloud(cloud_cluster) ;
        //hull.setAlpha(0.1);
        //hull.setDimension( 3 );
        //hull.reconstruct( *concaveHull, polygons );
        //for(int i = 0; i < polygons.size(); i++)
        //std::cout << polygons[i] << std::endl;
            
        std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    }
        
    cout << "num = " << num << endl ;
    */
    //ros::Duration next = ros::Time::now() - now ;
    //cout << "next = " << next.nsec << endl ;
    //viewer.showCloud(cloud_cluster); 
    //cloud_viewer->showCloud(cloud_plane) ;

}


bool image_extract_cluster( cloud_type::ConstPtr cloud ){
    double minX(100.0), minY(100.0), minZ(100.0), maxX(0.0), maxY(0.0), maxZ(0.0), averageZ(0.0) ;
    int count = 0 ;
    for(int i = 0; i<cloud->points.size(); i++){
        // for min  
        if ( cloud->points[i].x < minX )
            minX = cloud->points[i].x ;
        if ( cloud->points[i].y < minY )
            minY = cloud->points[i].y ;
        if (cloud->points[i].z < minZ)
            minZ = cloud->points[i].z ;
        //for max
        if (cloud->points[i].x > maxX)
            maxX = cloud->points[i].x ;
        if (cloud->points[i].y > maxY)
            maxY = cloud->points[i].y ;
        if (cloud->points[i].z > maxZ)
            maxZ = cloud->points[i].z ;

        averageZ += cloud->points[i].z ;
    }
    averageZ = averageZ / cloud->points.size() ;
    for (std::vector<Eigen::Vector3f>::const_iterator it = current_coordinate.begin(); it != current_coordinate.end(); ++it){
        //Eigen::Vector3f v = *it ;
        if( it->x()>minX && it->x()<maxX  &&  it->y()>minY && it->y()<maxY  &&  abs((it->z()) - averageZ )<0.2){
            if(it->z()<minZ || it->z()>maxZ)
                continue ;

            count ++ ;
        }
    }

    double area = ( maxX- minX ) * (maxY - minY) ;
    //cout << "area= " << area << endl ;
    double density = area / (double) count ;
    cout << "density: " << density << endl ;
    cout << "count: " << count << endl ;
    //cout << "The size of PointCloud: " << cloud->points.size() << endl ;
    //std::cout << "size = "  << current_coordinate.size << std::endl ;
    /*
    if(count > 1500)
        std::cout << "count = " << count << std::endl ;
    */
    if(density > 0 && density < 100 && count >2000 ){
        cout << "density: " << density << endl ;
        cout << "The size of PointCloud: " << cloud->points.size() << endl ;

        return true ;
    }else{
        return false ;
    }
}
