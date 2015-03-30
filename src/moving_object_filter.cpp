/*************************************************************************
	> File Name: moving_object_filter.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: 2015年01月09日 星期五 08时05分54秒
 ************************************************************************/
 #include "moving_object_filter.h"
 #include "umath.h"
 #include "optical_flow.h"
 
 MovingObjectFilter::MovingObjectFilter( int argc, char**argv ):cloud_viewer("Moving Object Viewer")
 {
    /*
    ROS_INFO("constructor");
    featureDetector_ = cv::FeatureDetector::create("ORB") ;//
    descriptorExtractor_ = cv::DescriptorExtractor::create("ORB") ;//
    descriptorMatcher_ = cv::DescriptorMatcher::create("BruteForce") ;
    if(featureDetector_.empty() || descriptorExtractor_.empty() || descriptorMatcher_.empty()){
        ROS_ERROR("Error creating detector, extractor or matcher.\n");
    }*/
    ros::NodeHandle nh ;
    rgbPub_ = nh.advertise<sensor_msgs::Image>( "rgb" , 10 ) ;
    depthPub_ = nh.advertise<sensor_msgs::Image>( "depth", 10 ) ;
    cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("pointcloud2",10) ;

    //std::cout << "constructor" << std::endl ;
    //previous_frame.create(640,480,)
}

 void MovingObjectFilter::processData( const cv::Mat & image, const cv::Mat &depth ,
                                       float cx,float cy,
                                       float fx,float fy){

     //ROS_INFO("*******");
     cv::Mat imageMono ;
     //convert to grayscale

     if(image.channels() > 1){
        cv::cvtColor(image, imageMono, cv::COLOR_BGR2GRAY) ;
     }else{
        imageMono = image ;
     }
     this->computeHomography(imageMono);

     //OpticalFlow of ;

     //of.process( imageMono );

     //cv::imshow("current View", imageMono) ;


     //ROS_INFO( "Process data" ) ;

     //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>) ;


     cloud = this->cloudFromDepthRGB( image, depth, cx, cy, fx, fy, 1.0 ) ;
     this->image_diff( imageMono, cloud ) ;


     this->pcl_segmentation(cloud) ;
     previous_coordinate.clear() ;
     current_coordinate.clear() ;
     /*
     sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2) ;
     pcl::toROSMsg(*cloud, *cloudMsg) ;
     cloudMsg->header.stamp = ros::Time::now() ;
     cloudMsg->header.frame_id = "Pointcloud" ;
     cloudPub_.publish(cloudMsg) ;*/


     /*
     //cv::namedWindow( "Display window", CV_WINDOW_AUTOSIZE );// Create a window for display.
     //cv::imshow( "Display window", image );                   // Show our image inside it.
     //cv::waitKey(0);
     //rgbPub_.publish(binary_image) ;
     */
     //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>) ;
     //cloud = this->cloudFromDepthRGB( image, depth, cx, cy, fx, fy, 1 ) ;
     //this->image_separate(cloud);


}

 void MovingObjectFilter::computeHomography(cv::Mat &grayImage){

     //Step 1: Detect the keypoints using ORB Detector
     cv::ORB orb ;
     std::vector<cv::KeyPoint> lastKeypoints ;
     std::vector<cv::KeyPoint> keypoints ;
     
     //featureDetector_ ->detect( grayImage, keypoints ) ;
     //ROS_INFO( "The size of keypoints: %d", keypoints.size() ) ;
     //if( keypoints.size() == 0 )
     //    return ;
     //Step 2: Calculate descriptors (features vectors)
     cv::Mat lastDescriptors ;
     cv::Mat descriptors ;

     if(!lastImage.empty()){
         orb( lastImage, cv::Mat(), lastKeypoints, lastDescriptors) ;
         orb( grayImage, cv::Mat(), keypoints, descriptors );
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

     if(!lastDescriptors.empty()){
         //cout << "************" << endl ;
         matcher.match( lastDescriptors, descriptors, matches );
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
         cv::drawMatches( lastImage, lastKeypoints, grayImage, keypoints, goodMatches,img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
         imshow("matches", img_matches) ;
         cv::imwrite( "matches.jpg", img_matches ) ;
         if(cv::waitKey(1) > 0){
             exit(0);
         }

         if(goodMatches.size() > 4){
             for( int i=0; i<goodMatches.size();i++ ){
                 lastPoint.push_back( lastKeypoints[goodMatches[i].queryIdx].pt );
                 currentPoint.push_back( keypoints[goodMatches[i].trainIdx].pt );
             }
             Homography = cv::findHomography( lastPoint, currentPoint, CV_RANSAC ) ;
             //shft = ( cv::Mat_<double>(3,3)<< 1.0, 0, lastImage.cols, 0, 1.0, 0, 0, 0, 1.0) ;
         }

         //warp the image
         /*
         cv::Mat dst ;
         cv::warpPerspective( grayImage, dst, shft*Homography, cv::Size(3*grayImage.rows, 2*grayImage.cols));
         cv::Mat rightImage = grayImage ;
         rightImage.copyTo(cv::Mat( dst, cv::Rect( lastImage.cols, 0, grayImage.cols, grayImage.rows  ) )) ;
         cv::namedWindow("warpImage") ;
         cv::imshow("warpImage", dst) ;
         */
     }

     lastImage = grayImage ;

 }





/*
 void MovingObjectFilter::ExtractObject( cloud_type::ConstPtr cloud, cv::Mat &gray_image ){
    //std::cout << "ExtractObject" << std::endl ;
    image_diff(gray_image) ;
    //transform_coordinate(cloud) ;
    image_separate( cloud ) ;
    double last1 = pcl::getTime() ;
    pcl_segmentation( cloud ) ;
    double now1 = pcl::getTime() ;
     std::cout << "Time = " << now1-last1 << std::endl ;
    
 }
 */

void MovingObjectFilter::image_diff(const cv::Mat &currentImage, cloud_type::ConstPtr cloud){
    cv::Mat BlurImage1, BlurImage2 ;
    if(lastBlurImage.empty()){
        cv::GaussianBlur( currentImage, BlurImage1, cv::Size( 11, 11 ), 0, 0 );
        lastBlurImage = BlurImage1 ;
        //lastDepth = depth ;
        lastCloud = *cloud ;
    }else{
        cv::GaussianBlur( currentImage, BlurImage2, cv::Size( 11, 11 ), 0, 0 );

        //cv::absdiff( BlurImage2, lastImage, diff_image ) ;

        //Calculate the difference of image
        cv::Mat last_frame( 480,640, CV_8UC1, cv::Scalar(0) );
        cv::Mat current_frame( 480,640, CV_8UC1, cv::Scalar(0) );
        lastFrame = last_frame ;
        currentFrame = current_frame ;
        threshod = 40 ;
        cv::namedWindow("lastFrame") ;
        cv::namedWindow("currentFrame") ;
        //cv::namedWindow("warpImage") ;


        for( int rows=0; rows < BlurImage2.rows; rows++ ){
            for(int cols=0; cols< BlurImage2.cols; cols++){
                cv::Mat srcMat = cv::Mat::zeros(3,1,CV_64FC1);
                srcMat.at<double>(0,0) = rows ;
                srcMat.at<double>(1,0) = cols ;
                srcMat.at<double>(2,0) = 1.0 ;
                cv::Mat warpMat = Homography * srcMat ;
                cv::Point warpPt ;
                warpPt.x = cvRound( warpMat.at<double>(0,0) / warpMat.at<double>(2,0) ) ;
                warpPt.y = cvRound( warpMat.at<double>(1,0) / warpMat.at<double>(2,0) ) ;
                //cout << "warpPt.x= " << warpPt.x <<" ; warpPt.y= " << warpPt.y << endl ;
                //float lastDepthValue = (float)lastDepth.at<uint16_t>( rows, cols )*0.001f ;
                //cout << "The rows of depth:" << rows << " ,The cols of depth: " << cols << endl ;

                if( warpPt.x>0 && warpPt.x<480  &&  warpPt.y>0 && warpPt.y<640 ){

                    double imageDiff = abs( lastBlurImage.at<unsigned char>(warpPt.x ,warpPt.y) - BlurImage2.at<unsigned char>(rows, cols));

                    double lastDepthValue = 0.0 ;
                    double depthValue = 0.0 ;
                    Eigen::Vector3f v1 ;
                    Eigen::Vector3f v2 ;

                    //cout << "After  abs" << endl;
                    //ROS_INFO("depth rows:%d ; cols:%d", depth.rows, depth.cols

                    if( imageDiff > threshod ){
                        lastDepthValue = isnan( lastCloud.at( cols,rows).z) ? 20 : lastCloud.at(cols,rows).z ;
                        depthValue = isnan( cloud->at(warpPt.y, warpPt.x).z) ? 20 : cloud->at(warpPt.y, warpPt.x).z ;
                        if( lastDepthValue - depthValue > 0.2 ){
                            lastFrame.at<unsigned char>(warpPt.x ,warpPt.y) = 255 ;
                            v1 << lastCloud.at(cols,rows).x , lastCloud.at(cols,rows).y , lastCloud.at(cols,rows).z ;
                            previous_coordinate.push_back(v1) ;

                        }else if( depthValue -lastDepthValue > 0.2 ){

                            currentFrame.at<unsigned char>(rows, cols) = 255 ;
                            v2 << cloud->at(cols,rows).x , cloud->at(cols,rows).y , cloud->at(cols,rows).z ;
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
        //cv::imshow("warpImage", dst) ;
        if(cv::waitKey(1) > 0){
            exit(0);
        }
        lastBlurImage = BlurImage2 ;
        lastCloud = *cloud ;
        //lastDepth = depth ;
        //cv::threshold( diff_image, binary_image, threshod_binary, 255, cv::THRESH_BINARY );
    }
}

pcl::PointXYZ MovingObjectFilter::projectDepthTo3D(
        const cv::Mat & depthImage,
        float x, float y,
        float cx, float cy,
        float fx, float fy,
        bool smoothing,
        float maxZError)
{
    //UASSERT(depthImage.type() == CV_16UC1 || depthImage.type() == CV_32FC1);

    pcl::PointXYZ pt;
    float bad_point = std::numeric_limits<float>::quiet_NaN ();

    int u = int(x+0.5f);
    int v = int(y+0.5f);

    if(!(u >=0 && u<depthImage.cols && v >=0 && v<depthImage.rows))
    {
        //UERROR("!(x >=0 && x<depthImage.cols && y >=0 && y<depthImage.rows) cond failed! returning bad point. (x=%f (u=%d), y=%f (v=%d), cols=%d, rows=%d)",
        //		x,u,y,v,depthImage.cols, depthImage.rows);
        pt.x = pt.y = pt.z = bad_point;
        return pt;
    }

    bool isInMM = depthImage.type() == CV_16UC1; // is in mm?


    // Inspired from RGBDFrame::getGaussianMixtureDistribution() method from
    // https://github.com/ccny-ros-pkg/rgbdtools/blob/master/src/rgbd_frame.cpp
    // Window weights:
    //  | 1 | 2 | 1 |
    //  | 2 | 4 | 2 |
    //  | 1 | 2 | 1 |
    int u_start = std::max(u-1, 0);
    int v_start = std::max(v-1, 0);
    int u_end = std::min(u+1, depthImage.cols-1);
    int v_end = std::min(v+1, depthImage.rows-1);

    float depth = isInMM?(float)depthImage.at<uint16_t>(v,u)*0.001f:depthImage.at<float>(v,u);
    if(depth!=0.0f && uIsFinite(depth))
    {
        if(smoothing)
        {
            float sumWeights = 0.0f;
            float sumDepths = 0.0f;
            for(int uu = u_start; uu <= u_end; ++uu)
            {
                for(int vv = v_start; vv <= v_end; ++vv)
                {
                    if(!(uu == u && vv == v))
                    {
                        float d = isInMM?(float)depthImage.at<uint16_t>(vv,uu)*0.001f:depthImage.at<float>(vv,uu);
                        // ignore if not valid or depth difference is too high
                        if(d != 0.0f && uIsFinite(d) && fabs(d - depth) < maxZError)
                        {
                            if(uu == u || vv == v)
                            {
                                sumWeights+=2.0f;
                                d*=2.0f;
                            }
                            else
                            {
                                sumWeights+=1.0f;
                            }
                            sumDepths += d;
                        }
                    }
                }
            }
            // set window weight to center point
            depth *= 4.0f;
            sumWeights += 4.0f;

            // mean
            depth = (depth+sumDepths)/sumWeights;
        }

        // Use correct principal point from calibration
        cx = cx > 0.0f ? cx : float(depthImage.cols/2) - 0.5f; //cameraInfo.K.at(2)
        cy = cy > 0.0f ? cy : float(depthImage.rows/2) - 0.5f; //cameraInfo.K.at(5)

        // Fill in XYZ
        pt.x = (x - cx) * depth / fx;
        pt.y = (y - cy) * depth / fy;
        pt.z = depth;
    }
    else
    {
        pt.x = pt.y = pt.z = bad_point;
    }
    return pt;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr MovingObjectFilter::cloudFromDepthRGB(
                const cv::Mat & imageRgb,
                const cv::Mat & imageDepth,
                float cx, float cy,
                float fx, float fy,
                int decimation)
{
        //UASSERT(imageRgb.rows == imageDepth.rows && imageRgb.cols == imageDepth.cols);
        //UASSERT(!imageDepth.empty() && (imageDepth.type() == CV_16UC1 || imageDepth.type() == CV_32FC1));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        if(decimation < 1)
        {
                return cloud;
        }

        bool mono;
        if(imageRgb.channels() == 3) // BGR
        {
                mono = false;
        }
        else if(imageRgb.channels() == 1) // Mono
        {
                mono = true;
        }
        else
        {
                return cloud;
        }

        //cloud.header = cameraInfo.header;
        cloud->height = imageDepth.rows/decimation;
        cloud->width  = imageDepth.cols/decimation;
        cloud->is_dense = false;
        cloud->resize(cloud->height * cloud->width);

        for(int h = 0; h < imageDepth.rows && h/decimation < (int)cloud->height; h+=decimation)
        {
                for(int w = 0; w < imageDepth.cols && w/decimation < (int)cloud->width; w+=decimation)
                {
                        pcl::PointXYZRGB & pt = cloud->at((h/decimation)*cloud->width + (w/decimation));
                        if(!mono)
                        {
                                pt.b = imageRgb.at<cv::Vec3b>(h,w)[0];
                                pt.g = imageRgb.at<cv::Vec3b>(h,w)[1];
                                pt.r = imageRgb.at<cv::Vec3b>(h,w)[2];
                        }
                        else
                        {
                                unsigned char v = imageRgb.at<unsigned char>(h,w);
                                pt.b = v;
                                pt.g = v;
                                pt.r = v;
                        }

                        pcl::PointXYZ ptXYZ = this->projectDepthTo3D(imageDepth, w, h, cx, cy, fx, fy, false);
                        pt.x = ptXYZ.x;
                        pt.y = ptXYZ.y;
                        pt.z = ptXYZ.z;
                }
        }
        return cloud;
}
/*
void MovingObjectFilter::image_separate( cloud_type::ConstPtr cloud ){

    //ROS_INFO("height=%f,width=%f",cloud->height,cloud->width);

    previous_frame.create( cloud->height, cloud->width, CV_8UC1 ) ;
    current_frame.create( cloud->height, cloud->width, CV_8UC1 ) ;
    double previous_z = 0.0 ;
    double current_z = 0.0 ;
    Eigen::Vector3f v1 ;

    if(last_cloud.size() == 0){ //for first frame
        last_cloud = *cloud ;
    }else{       
        for( int row = 0; row < cloud->height; row++ ){
            for(int col = 0; col < cloud->width; col++ ){
                previous_z = isnan( last_cloud.at(col,row).z) ? 20 : last_cloud.at(col,row).z ;
                current_z = isnan( cloud->at(col,row).z) ? 20 : cloud->at(col,row).z ;

                if(binary_image.at<unsigned char>(row,col) == 255){
                    if( previous_z < current_z ){               
                        previous_frame.at<unsigned char>(row,col) = 255 ;
                        current_frame.at<unsigned char>(row,col) = 0 ;
                        v1 << last_cloud.at(col,row).x , last_cloud.at(col,row).y , last_cloud.at(col,row).z ;
                        previous_coordinate.push_back(v1) ;

                    }else if( previous_z > current_z ){
                        previous_frame.at<unsigned char>(row,col) = 0 ;
                        current_frame.at<unsigned char>(row,col) = 0 ;
                        current_frame.at<unsigned char>(row,col) = 255 ;
                        v1 << cloud->at(col,row).x , cloud->at(col,row).y , cloud->at(col,row).z ;
                        current_coordinate.push_back(v1) ;


                    }else{
                        previous_frame.at<unsigned char>( row, col ) = 0 ;
                        current_frame.at<unsigned char>(row,col) = 0 ;
                    }

                }else{
                    previous_frame.at<unsigned char>(row,col) = 0 ;
                    current_frame.at<unsigned char>(row,col) = 0 ;
                }
            }//for col
        }//for row
        last_cloud = *cloud ;

        //cv::imshow("previous_frame",previous_frame) ;
        //cv::imshow("current_frame",current_frame) ;
        //std::cout << "other cloud" << std::endl ;
    }

}*/

void MovingObjectFilter::pcl_segmentation( cloud_type::ConstPtr cloud ){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Step 1: Filter out NaNs from data
    pcl::PassThrough<pcl::PointXYZRGB> pass ;
    pass.setInputCloud (cloud); 
    pass.setFilterFieldName ("z"); 
    pass.setFilterLimits (0.5, 6.0); 
    pass.filter (*cloud_filtered);

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
    
    pcl::VoxelGrid<pcl::PointXYZRGB> downSampler;
    downSampler.setInputCloud (cloud_filtered); 
    downSampler.setLeafSize (0.01f, 0.01f, 0.01f); 
    downSampler.filter (*cloud_filtered2);
    
        
    // Step 4: Remove the ground plane using RANSAC 
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients); 
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices); 
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true); // Optional 
    //seg.setMaxIteractions(100) ; //Optional,maybe can be lower
    // Mandatory 
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    //seg.setMethodType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE) ;
    seg.setAxis(Eigen::Vector3f(0.0,1.0,0.0));
    seg.setEpsAngle (pcl::deg2rad (10.0));
    //seg.setNormalDistanceWeight (0.1);
    seg.setMethodType (pcl::SAC_RANSAC); 
    seg.setDistanceThreshold (0.01); //1cm
    seg.setInputCloud (cloud_filtered2->makeShared()); 
    seg.segment (*inliers, *coefficients);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ()) ;
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
    // PAINT SURFACE
    //
    //for (unsigned int i = 0; i < inliers->indices.size(); i++){ 
    //    int idx = inliers->indices[i]; 
    //    cloud_filtered2->points[idx].r = 255; 
    //    cloud_filtered2->points[idx].b = 0; 
    //}
        
    // Step 5: EuclideanCluster Extract the moving objects
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud_filtered2);

    std::vector<pcl::PointIndices> cluster_indices ;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec ;
    ec.setClusterTolerance (0.02) ; // 2cm
    ec.setMinClusterSize (1000) ;
    ec.setMaxClusterSize (25000) ;
    ec.setSearchMethod (tree) ;
    ec.setInputCloud (cloud_filtered2) ;
    ec.extract (cluster_indices) ;



    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    float minX(0.0), minY(0.0), minZ(0.0), maxX(0.0), maxY(0.0), maxZ(0.0) ;
    point_type cluster_point ;


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
        
        if(image_extract_cluster(cloud_cluster)) {
            //object_cloud += *cloud_cluster ;
            if(!cloud_viewer.wasStopped()){
                cloud_viewer.showCloud(cloud_cluster);
            }
            //cloud_viewer.showCloud( object_cloud );
            current_coordinate.clear();
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
        


    //viewer.showCloud(cloud_cluster); 
    //cloud_viewer->showCloud(cloud_plane) ;

}

bool MovingObjectFilter::image_extract_cluster( cloud_type::ConstPtr cloud ){
    double minX(10.0), minY(10.0), minZ(10.0), maxX(0.0), maxY(0.0), maxZ(0.0), averageZ(0.0) ;
    count = 0 ;
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
            maxX = cloud->points[i].x;
        if (cloud->points[i].y > maxY)
            maxY = cloud->points[i].y;
        if (cloud->points[i].z > maxZ)
            maxZ = cloud->points[i].z;

        averageZ += cloud->points[i].z ;
    }
    averageZ = averageZ / cloud->points.size() ;
    for (std::vector<Eigen::Vector3f>::const_iterator it = current_coordinate.begin(); it != current_coordinate.end(); ++it){
        //Eigen::Vector3f v = *it ;
        if( it->x()>minX && it->x()<maxX && it->y()>minY && it->y()<maxY && abs((it->z()) - averageZ )<0.5){
            if(it->z()<minZ || it->z()>maxZ)    continue ;
            count ++ ;
        }
    }

    double area = ( maxX- minX ) * (maxY - minY) ;
    double density = area / (double) count ;
    cout << density << endl ;
    //std::cout << "size = "  << current_coordinate.size << std::endl ;
    /*
    if(count > 1500)
        std::cout << "count = " << count << std::endl ;
    */
    if(density > 0.1){
        //count = 0 ;
        return true ;
    }else{
        return false ;
    }
}
/*
cloud_type::ConstPtr MovingObjectFilter::get_filter_cloud(){
    return object_cloud.makeShared() ;
}

void MovingObjectFilter::clear_object_cloud(){
    object_cloud.clear();
}
*/

void MovingObjectFilter::transform_coordinate(cloud_type::ConstPtr cloud){
    double min_x = 10.0, min_y = 10.0 , min_z = 10.0, max_x = 0.0, max_y = 0.0 , max_z = 0.0 ;
    deta_x = 0.0 ;
    deta_y = 0.0 ;
    for( int row=0 ; row<cloud->height; row++ ){
        for(int col=0; col<cloud->width; col++){
            const point_type& pt = cloud->at(col, row) ;
            if(pt.x < min_x)    min_x = pt.x ;
            if(pt.y < min_y)    min_y = pt.y ;
            if(pt.z < min_z)    min_z = pt.z ;
            if(pt.x > max_x)    max_x = pt.x ;
            if(pt.y > max_y)    max_y = pt.y ;
            if(pt.z > max_z)    max_z = pt.z ;
        }
    }
    deta_x = max_x - min_x ;
    deta_y = max_y - min_y ;
    std::cout << "deta_x/640 = " << deta_x/640 << ";deta_y/480 = " << deta_y/480 << std::endl ;
}

