/**
* This file is part of rgbdtam.
*
* Copyright (C) 2015 Alejo Concha Belenguer <alejocb at unizar dot es> (University of Zaragoza)
* and Javier Civera Sancho   <jcivera at unizar dot es> (University of Zaragoza)
*
* rgbdtam is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* rgbdtam is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with rgbdtam. If not, see <http://www.gnu.org/licenses/>.
*/

#include "rgbdtam/vo_system.h"
#include <fstream>
#include <iomanip>    // Needed for stream modifiers fixed and set precision

#include <ros/package.h>


vo_system::vo_system(){


    ///vo_system launch the three threads, tracking, semidense mapping and dense mapping (3D superpixels)

    cv::FileStorage  fs2( (ros::package::getPath("rgbdtam")+"/src/data.yml").c_str(), cv::FileStorage::READ);

    std::string camera_path;
    fs2["camera_path"] >> camera_path;
    fs2["use_ros"] >> use_ros;
    fs2["cameraMatrix"] >> cameraMatrix;
    fs2["distCoeffs"] >> distCoeffs;

    cont_frames = 0;
    counter_depth_images = 0;

    int calculate_superpixels = (int)fs2["calculate_superpixels"];

    image_transport::ImageTransport it(nh);

   // sub1_right = it.subscribe(camera_path_right,1, & vo_system::imgcb_right,this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("odom1", 50);
    /// advertising 3D map and camera poses in rviz
    pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("rgbdtam/map", 1);
    pub_poses = nh.advertise<sensor_msgs::PointCloud2> ("points_poses", 1);
    vis_pub = nh.advertise<visualization_msgs::Marker>( "rgbdtam/visualization_marker", 0 );
    /// advertising 3D map and camera poses in rviz
    /// pubishing current frame and the reprojection of the 3D map
    pub_image = it.advertise("rgbdtam/camera/image",1);
    /// pubishing current frame and the reprojection of the 3D map
    semidense_tracker.cont_frames = &cont_frames;
    semidense_tracker.frame_struct = &frame_struct;

    #pragma omp parallel num_threads(3)
    {
       switch(omp_get_thread_num())
       {
           case 0:
           {
               ///Launch semidense tracker thread
               boost::thread thread_semidense_tracker(&ThreadSemiDenseTracker,&images,&semidense_mapper,&semidense_tracker,&dense_mapper,&Map,&vis_pub,&pub_image);
           };break;
           case 1:
           {
               ///Launch semidense mapper thread
               boost::thread thread_semidense_mapper(&ThreadSemiDenseMapper,&images,&images_previous_keyframe,&semidense_mapper,&semidense_tracker,&dense_mapper,&Map,&pub_cloud);
           };break;
           case 2:
           {
               ///Launch viewer updater.
               boost::thread thread_viewer_updater(&ThreadViewerUpdater, &semidense_tracker,&semidense_mapper,&dense_mapper);
           }
        }
    }




    /*if (calculate_superpixels > 0.5)
    {
         ///launch dense mapper thread
         boost::thread thread_dense_mapper(&ThreadDenseMapper,&dense_mapper,&pub_cloud);
    }*/

    cout << "***    RGBDTAM is working     *** " <<  endl << endl;
    cout << "***    Launch the example sequences or use your own sequence / live camera and update the file 'data.yml' with the corresponding camera_path and calibration parameters    ***"  << endl;

    if(use_ros == 1)
    {
      sub1 = it.subscribe(camera_path,1, & vo_system::imgcb,this);
      if(semidense_tracker.use_kinect)
      sub2 = it.subscribe("/camera/depth/image",1, & vo_system::depthcb,this);
    }
}




void vo_system::imgcb(const sensor_msgs::Image::ConstPtr& msg)
{
    ///read images
    try
    {
        boost::mutex::scoped_lock lock(semidense_tracker.loopcloser_obj.guard);


        cv_bridge::CvImageConstPtr cv_ptr;
        cv_bridge::toCvShare(msg);
        cv_ptr = cv_bridge::toCvShare(msg);


        cv::Mat image =  cv_ptr->image.clone();


        /// add gaussian blur
        cv::Mat image_frame_aux;
        cv::GaussianBlur(image,image_frame_aux,cv::Size(0,0),3);
        cv::addWeighted(image,1.5,image_frame_aux,-0.5,0,image_frame_aux);
        image = image_frame_aux.clone();
        if (semidense_tracker.bgr2rgb > 0.5){cv::cvtColor(image,image,CV_RGB2BGR);}
        /// add gaussian blur

        frame_struct.image_frame =image.clone();
        frame_struct.stamps = cv_ptr->header.stamp.toSec();

        semidense_tracker.frame_struct_vector.push_back(frame_struct);


        cont_frames++;
     }
    catch (const cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}



void vo_system::depthcb(const sensor_msgs::Image::ConstPtr& msg)
{
    ///read images
    try {
        cv_bridge::CvImageConstPtr cv_ptr;

        cv_bridge::toCvShare(msg);
        cv_ptr = cv_bridge::toCvShare(msg);

        stamps_depth_ros =  cv_ptr->header.stamp;
        image_depth =  cv_ptr->image.clone();


        image_depth.convertTo(image_depth,CV_32FC1);

        for(int i =  0;i < image_depth.rows; i++){
            for(int j = 0; j< image_depth.cols; j++){
                 if( isnan( image_depth.at<float>(i,j)))
                  image_depth.at<float>(i,j) = 0;
            }
        }
        //UNDISTORT DEPTH MAP
        cv::Size ksize;
        ksize.width = image_depth.cols;
        ksize.height = image_depth.rows;
        cv::Mat undistorted_depth_map;
        if(semidense_tracker.mapX.rows>0)
        {cv::remap(image_depth,undistorted_depth_map,semidense_tracker.mapX,semidense_tracker.mapY,
                  CV_INTER_NN,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
        image_depth = undistorted_depth_map;}
        //UNDISTORT DEPTH MAP

        semidense_mapper.image_depth_keyframes[counter_depth_images%SIZE_DEPTH_VECTOR] = image_depth;
        semidense_mapper.stamps_depth_ros[counter_depth_images%SIZE_DEPTH_VECTOR] = stamps_depth_ros.toSec();
        counter_depth_images++;
    }
        catch (const cv_bridge::Exception& e)
        {
           ROS_ERROR("cv_bridge exception: %s", e.what());
        }
}
