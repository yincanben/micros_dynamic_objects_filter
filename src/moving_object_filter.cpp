/*************************************************************************
	> File Name: moving_object_filter.cpp
	> Author: yincanben
	> Mail: yincanben@163.com
	> Created Time: 2015年01月09日 星期五 08时05分54秒
 ************************************************************************/
 #include "moving_object_filter.h"
 #include "umath.h"
 #include "optical_flow.h"
 
 MovingObjectFilter::MovingObjectFilter( int argc, char**argv ):result_viewer("Result")
 {//cloud_viewer("Moving Object Viewer"),

    ros::NodeHandle nh ;
    rgbPub_ = nh.advertise<sensor_msgs::Image>( "rgb" , 10 ) ;
    depthPub_ = nh.advertise<sensor_msgs::Image>( "depth", 10 ) ;
    cloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("pointcloud2",10) ;

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
    
     this->pcl_segmentation(cloud, imageMono, cx, cy, fx, fy) ;
     previous_coordinate.clear() ;
     current_coordinate.clear() ;
     
     sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2) ;
     pcl::toROSMsg(*cloud, *cloudMsg) ;
     cloudMsg->header.stamp = ros::Time::now() ;
     cloudMsg->header.frame_id = "camera_link" ;
     cloudPub_.publish(cloudMsg) ;


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

         /*
         cv::namedWindow("matches") ;
         cv::drawMatches( lastImage, lastKeypoints, grayImage, keypoints, goodMatches,img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
         imshow("matches", img_matches) ;
         //cv::imwrite( "matches.jpg", img_matches ) ;
         */

         if(cv::waitKey(1) > 0){
             exit(0);
         }

         if(goodMatches.size() > 4){
             for( int i=0; i<goodMatches.size();i++ ){
                 lastPoint.push_back( lastKeypoints[goodMatches[i].queryIdx].pt );
                 currentPoint.push_back( keypoints[goodMatches[i].trainIdx].pt );
             }
             Homography = cv::findHomography( lastPoint, currentPoint, CV_RANSAC ) ;
             shft = ( cv::Mat_<double>(3,3)<< 1.0, 0, lastImage.cols, 0, 1.0, 0, 0, 0, 1.0) ;
         }

         //warp the image
         /*
         cv::Mat dst ;
         //cv::warpPerspective( lastImage, dst, Homography, cv::Size(lastImage.cols + grayImage.cols + lastImage.cols, lastImage.rows));
         cv::warpPerspective( lastImage, dst, shft*Homography, cv::Size(lastImage.cols + grayImage.cols + lastImage.cols, lastImage.rows));

         cv::Mat rightImage = grayImage ;
         rightImage.copyTo(cv::Mat( dst, cv::Rect( lastImage.cols, 0, grayImage.cols, grayImage.rows  ) )) ;
         cv::namedWindow("warpImage") ;
         cv::imshow("warpImage", dst) ;
         */
     }

     lastImage = grayImage ;

 }



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
        cv::Mat diffFrame( 480,640, CV_8UC1, cv::Scalar(0) );
        cv::Mat last = lastBlurImage ;
        cv::Mat current = BlurImage2 ;
        lastFrame = last_frame ;
        currentFrame = current_frame ;
        threshod = 40 ;
        //cv::namedWindow("lastFrame") ;
        cv::namedWindow("currentFrame") ;
        //cv::namedWindow("diffFrame") ;




        for( int rows=0; rows < lastImage.rows; rows++ ){
            for(int cols=0; cols< lastImage.cols; cols++){
                cv::Mat srcMat = cv::Mat::zeros(3,1,CV_64FC1);
                //srcMat.at<double>(0,0) = rows ;
                //srcMat.at<double>(1,0) = cols ;
                srcMat.at<double>(0,0) = cols ;
                srcMat.at<double>(1,0) = rows;
                srcMat.at<double>(2,0) = 1.0 ;
                cv::Mat warpMat = Homography * srcMat ;
                cv::Point warpPt ;
                warpPt.x = cvRound( warpMat.at<double>(0,0) / warpMat.at<double>(2,0) ) ;
                warpPt.y = cvRound( warpMat.at<double>(1,0) / warpMat.at<double>(2,0) ) ;

                //cout << "warpPt.x= " << warpPt.x <<" ; warpPt.y= " << warpPt.y << endl ;
                //float lastDepthValue = (float)lastDepth.at<uint16_t>( rows, cols )*0.001f ;
                //cout << "The rows of depth:" << rows << " ,The cols of depth: " << cols << endl ;

                if( warpPt.x>0 && warpPt.x<640  &&  warpPt.y>0 && warpPt.y< 480){

                    double imageDiff = abs( lastBlurImage.at<unsigned char>(rows, cols) - BlurImage2.at<unsigned char>(warpPt.y ,warpPt.x));

                    double lastDepthValue = 0.0 ;
                    double depthValue = 0.0 ;
                    Eigen::Vector3f v1 ;
                    Eigen::Vector3f v2 ;

                    //cout << "After  abs" << endl;
                    //ROS_INFO("depth rows:%d ; cols:%d", depth.rows, depth.cols

                    if( imageDiff > threshod ){
                        diffFrame.at<unsigned char>(warpPt.y ,warpPt.x) = 255 ;

                        lastDepthValue = isnan( lastCloud.at( cols,rows).z) ? 20 : lastCloud.at(cols,rows).z ;
                        depthValue = isnan( cloud->at(warpPt.x, warpPt.y).z) ? 20 : cloud->at(warpPt.x, warpPt.y).z ;

                        if( lastDepthValue - depthValue > 0.2 && lastDepthValue<20 ){
                            currentFrame.at<unsigned char>(warpPt.y ,warpPt.x) = 255 ;
                            current.at<unsigned char>(warpPt.y ,warpPt.x) = 255 ;
                            v1 << cloud->at(warpPt.x, warpPt.y).x , cloud->at(warpPt.x, warpPt.y).y ,cloud->at(warpPt.x, warpPt.y).z ;
                            current_coordinate.push_back(v1) ;

                        }else if( depthValue -lastDepthValue > 0.2 && depthValue <20 ){

                            lastFrame.at<unsigned char>(rows, cols) = 255 ;
                            last.at<unsigned char>(rows, cols) = 255 ;
                            //v2 << lastCloud.at( cols,rows).x , lastCloud.at( cols,rows).y , lastCloud.at( cols,rows).z ;
                            //previous_coordinate.push_back(v2) ;

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

        //cv::imshow("lastFrame",last);
        //cv::waitKey(5);
        cv::imshow("currentFrame",current) ;
        cv::waitKey(5);
        //cv::imshow("diffFrame", diffFrame) ;
        //cv::waitKey(5);

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
void MovingObjectFilter::pcl_segmentation( cloud_type::ConstPtr cloud , const cv::Mat &image , float cx, float cy, float fx, float fy ){

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

    // Step 1: Filter out NaNs from data
    ros::Time last = ros::Time::now() ;
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

    pcl::VoxelGrid<pcl::PointXYZRGB> downSampler;
    downSampler.setInputCloud (cloud);
    downSampler.setLeafSize (0.01f, 0.01f, 0.01f);
    downSampler.filter (*cloud_filtered);


    // Step 4: Remove the ground plane using RANSAC
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true); // Optional

    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setAxis(Eigen::Vector3f(0.0,1.0,0.0));
    seg.setEpsAngle (pcl::deg2rad (10.0));
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01); //1cm
    /*
    seg.setMethodType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE) ;
    seg.setAxis(Eigen::Vector3f(0,0,1));
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setEpsAngle (0);
    seg.setDistanceThreshold (0.01); //1cm
    */
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ()) ;
    pcl::PointCloud<pcl::PointXYZRGB> plane  ;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    int  nr_points = (int) cloud_filtered->points.size ();
    while(cloud_filtered->points.size () > 0.5 * nr_points){
        seg.setInputCloud (cloud_filtered->makeShared());
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () < 1000){
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
        }

        extract.setInputCloud (cloud_filtered);
        // Step 4.1: Extract the points that lie in the ground plane
        extract.setIndices (inliers);
        extract.setNegative (false);
        extract.filter (*cloud_plane);
        plane += *cloud_plane ;

        // Step 4.2: Extract the points that are objects(i.e. are not in the ground plane)
        extract.setNegative (true);
        extract.filter ( *cloud_f ) ;
        *cloud_filtered = *cloud_f;
    }


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
    ros::Time now = ros::Time::now() ;
    ros::Duration time = now - last ;
    //cout << "Time : " << time.nsec << endl;

    cout << "The size of cluster_indices : " << cluster_indices.size() << endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr dynamic_object (new pcl::PointCloud<pcl::PointXYZRGB>);
    //double minX(0.0), minY(0.0), minZ(0.0), maxX(0.0), maxY(0.0), maxZ(0.0) ;
    //point_type cluster_point ;
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    int num = 0 ;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_object(new pcl::PointCloud<pcl::PointXYZRGB>)  ;
    *static_object = plane ;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->push_back( cloud_filtered->points[*pit]) ;

        cloud_cluster->width = cloud_cluster->points.size () ;
        cloud_cluster->height = 1 ;
        cloud_cluster->is_dense = true ;
        if(image_extract_cluster(cloud_cluster,image, cx, cy, fx, fy)) {
            *dynamic_object += *cloud_cluster ;
            /*
            if(!cloud_viewer.wasStopped()){
                cloud_viewer.showCloud(cloud_cluster);
                num++ ;
                cout << "num = "  << num << endl;
            }*/
        }else{
            *static_object += *cloud_cluster ;
            //continue ;
        }
        cloud_cluster->clear();
        //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    }
    if(!result_viewer.wasStopped()){
        result_viewer.showCloud(static_object);
    }


    ros::Duration next = ros::Time::now() - now ;
    //cout << "next = " << next.nsec << endl ;
    //viewer.showCloud(cloud_cluster);
    //cloud_viewer->showCloud(cloud_plane) ;
}
bool MovingObjectFilter::image_extract_cluster( cloud_type::ConstPtr cloud, const cv::Mat &image ,  float cx, float cy, float fx, float fy){
    //pcl::PointCloud<pcl::PointXYZ>  pt ;
    double minX(1000.0), minY(1000.0), minZ(1000.0), maxX(0.0), maxY(0.0), maxZ(0.0), averageZ(0.0) ;
    cv::Mat cluster( 480,640, CV_8UC1, cv::Scalar(0) );
    float x0 = 0.0 , y0 = 0.0 , z = 0.0 ;
    int x = 0 , y = 0 ;
    int count = 0 ;
    cv::Mat result = image ;
    for( int i = 0 ; i < cloud->points.size(); i++){
        z = cloud->points[i].z ;
        x0 = (cloud->points[i].x * fx)/z + cx ;
        y0 = (cloud->points[i].y * fy)/z + cy ;
        x = cvRound(x0) ;
        y = cvRound(y0) ;
        cluster.at<unsigned char>(y, x) = 255 ;
        //result.at<unsigned char>(y,x) = 255;

        if( x < minX )  minX = x ;
        if( y < minY )  minY = y ;
        //if( z < minZ )  minZ = z ;

        if( x > maxX )  maxX = x ;
        if( y > maxY )  maxY = y ;
        //if( z > maxZ )  maxZ = z ;

        //averageZ += z ;
    }
    //averageZ = averageZ / cloud->points.size()
    for(int row = 0 ; row < currentFrame.rows; row++){
        for(int col= 0; col < currentFrame.cols; col++){
            if( currentFrame.at<unsigned char>(row,col) == 255){
                if(row > minY && row<maxY && col>minX && col<maxX){
                    result.at<unsigned char>(row,col) = 255;
                    count ++ ;
                }
            }
        }
    }
    cout << "count= " << count << endl ;
    cv::namedWindow("cluster") ;
    cv::imshow("cluster", result) ;
    cv::waitKey(1) ;
    if(count > 3000){
        return true ;
    }else{
        return false ;
    }

    /*
    count ++ ;
    cout << "count= " << count << endl ;
    std::stringstream ss;
    ss << "Resultimage" << count << ".jpg";
    cv::namedWindow("cluster") ;
    cv::imshow("cluster", cluster) ;
    imwrite( ss.str(),cluster );
    cv::Mat new_cluster( 480,640, CV_8UC1, cv::Scalar(0) );
    cluster = new_cluster ;
    cv::waitKey(1) ;
    */

}
cv::Mat MovingObjectFilter::bgrFromCloud(const pcl::PointCloud<pcl::PointXYZRGBA> & cloud, bool bgrOrder)
{
    cv::Mat frameBGR = cv::Mat(cloud.height,cloud.width,CV_8UC3);

    for(unsigned int h = 0; h < cloud.height; h++)
    {
        for(unsigned int w = 0; w < cloud.width; w++)
        {
            if(bgrOrder)
            {
                frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).b;
                frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
                frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).r;
            }
            else
            {
                frameBGR.at<cv::Vec3b>(h,w)[0] = cloud.at(h*cloud.width + w).r;
                frameBGR.at<cv::Vec3b>(h,w)[1] = cloud.at(h*cloud.width + w).g;
                frameBGR.at<cv::Vec3b>(h,w)[2] = cloud.at(h*cloud.width + w).b;
            }
        }
    }
    return frameBGR;
}

// return float image in meter
cv::Mat MovingObjectFilter::depthFromCloud(
        const pcl::PointCloud<pcl::PointXYZRGBA> & cloud,
        float & fx,
        float & fy,
        bool depth16U)
{
    cv::Mat frameDepth = cv::Mat(cloud.height,cloud.width,depth16U?CV_16UC1:CV_32FC1);
    fx = 0.0f; // needed to reconstruct the cloud
    fy = 0.0f; // needed to reconstruct the cloud
    for(unsigned int h = 0; h < cloud.height; h++)
    {
        for(unsigned int w = 0; w < cloud.width; w++)
        {
            float depth = cloud.at(h*cloud.width + w).z;
            if(depth16U)
            {
                depth *= 1000.0f;
                unsigned short depthMM = 0;
                if(depth <= (float)USHRT_MAX)
                {
                    depthMM = (unsigned short)depth;
                }
                frameDepth.at<unsigned short>(h,w) = depthMM;
            }
            else
            {
                frameDepth.at<float>(h,w) = depth;
            }

            // update constants
            if(fx == 0.0f &&
               uIsFinite(cloud.at(h*cloud.width + w).x) &&
               uIsFinite(depth) &&
               w != cloud.width/2 &&
               depth > 0)
            {
                fx = cloud.at(h*cloud.width + w).x / ((float(w) - float(cloud.width)/2.0f) * depth);
                if(depth16U)
                {
                    fx*=1000.0f;
                }
            }
            if(fy == 0.0f &&
               uIsFinite(cloud.at(h*cloud.width + w).y) &&
               uIsFinite(depth) &&
               h != cloud.height/2 &&
               depth > 0)
            {
                fy = cloud.at(h*cloud.width + w).y / ((float(h) - float(cloud.height)/2.0f) * depth);
                if(depth16U)
                {
                    fy*=1000.0f;
                }
            }
        }
    }
    return frameDepth;
}

/*

bool MovingObjectFilter::image_extract_cluster( cloud_type::ConstPtr cloud ){
    double minX(100.0), minY(100.0), minZ(100.0), maxX(0.0), maxY(0.0), maxZ(0.0), averageZ(0.0) ;
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
            maxX = cloud->points[i].x ;
        if (cloud->points[i].y > maxY)
            maxY = cloud->points[i].y ;
        if (cloud->points[i].z > maxZ)
            maxZ = cloud->points[i].z ;

        averageZ += cloud->points[i].z ;
    }
    averageZ = averageZ / cloud->points.size() ;//TODO
    //cout << "before add,count = " << count << endl ;
    for (std::vector<Eigen::Vector3f>::const_iterator it = current_coordinate.begin(); it != current_coordinate.end(); ++it){
        //Eigen::Vector3f v = *it ;
        if( it->x()>minX && it->x()<maxX  &&  it->y()>minY && it->y()<maxY  &&  abs((it->z()) - averageZ )<0.2){
            if(it->z()<minZ || it->z()>maxZ){
                continue ;
            }else{
                count ++ ;
            }
        }
    }

    double area = ( maxX- minX ) * (maxY - minY) ;
    //cout << "area= " << area << endl ;
    double density = (double) count / area ;
    cout << "density: " << density << endl ;
    cout << "count:" << count << endl ;
    //cout << "The size of PointCloud: " << cloud->points.size() << endl ;
    //std::cout << "size = "  << current_coordinate.size << std::endl ;

    if(count > 1500)
        std::cout << "count = " << count << std::endl ;

    if(density > 3000 && count>3000 ){///TODO
        cout << "density = " << density << endl ;
        cout << "count = " << count << endl ;
        return true ;
    }else{
        return false ;
    }
}
*/

