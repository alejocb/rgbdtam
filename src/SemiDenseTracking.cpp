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
* GNU General Public License for more details.erp
*
* You should have received a copy of the GNU General Public License
* along with rgbdtam. If not, see <http://www.gnu.org/licenses/>.
*/

#include "rgbdtam/SemiDenseTracking.h"
#include "rgbdtam/vo_system.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <ros/ros.h>
#include <ros/package.h>

#include <image_transport/image_transport.h>

#define U_SEGSt(a)\
         gettimeofday(&tvt,0);\
         a = tvt.tv_sec + tvt.tv_usec/1000000.0
struct timeval tvt;
double t1t, t2t,t0t,t3t;
void tic_initt(){U_SEGSt(t0t);}
void toc_finalt(double &time){U_SEGSt(t3t); time =  (t3t- t0t)/1;}
void tict(){U_SEGSt(t1t);}
void toct(){U_SEGSt(t2t);
           cout << (t2t - t1t)/1 << endl;}

SemiDenseTracking::SemiDenseTracking()
{
   tic_initt();

   cv::FileStorage  fs2( (ros::package::getPath("rgbdtam")+
                          "/src/data.yml").c_str(), cv::FileStorage::READ);
   fs2["cameraMatrix"] >> cameraMatrix;
   fs2["distCoeffs"] >> distCoeffs;

   image_processing_semaphore = true;
   last_cont_frames = 0;
   frames_processed = 0 ;
   bgr2rgb = (int)fs2["bgr2rgb"];
   use_kinect =(int)fs2["use_kinect"];
   depth_rgb_offset =(float)fs2["depth_rgb_offset"];

   fs2["path_to_folder"] >> path_to_folder;

   init_frame =(int)fs2["init_frame"];
   use_ros =(int)fs2["use_ros"];
   use_depth_tracking =(int)fs2["use_depth_tracking"];


   loopcloser_obj.use_kinect = use_kinect;
   pyramid_levels = 4;


   local_maps_number = (int)fs2["local_maps_number"];


   if (use_ros == 0)
   {
     SemiDenseTracking::read_image_names(left_image_names,depth_image_names);
   }

   SemiDenseTracking::inititialize_vectors_pyramid_levels_size(pyramid_levels);
   SemiDenseTracking::init_local_maps(pyramid_levels*local_maps_number);
   SemiDenseTracking::init_poses_local_maps(local_maps_number);


   depth = 1;



   float distances_local_maps[local_maps_number];
   init_distances_local_maps(local_maps_number);

   for ( int jj =0;jj< local_maps_number;jj=jj+1)
   {
       distances_local_maps[jj] = INFINITY;
       set_distances_local_maps(distances_local_maps[jj],jj);
   }

   variance = (float)fs2["variance"];
   for (int j=0; j<pyramid_levels; j++)
   {variances[j] = variance;}


   int reduction_ = pow(2,pyramid_levels) / (2);
   for (int j=0; j<pyramid_levels ; j++)
   {
         reduction_pyramid[j] = reduction_;
         reduction_/=2;
   }




   image_n = 0;
   last_frame_tracked = image_n;
   processed_frames = 0;
   processed_frames_since_keyframe = 0;
   create_inv_depth_discretization = 0;


   boost::filesystem::remove_all((ros::package::getPath("rgbdtam")+"/src/map_and_poses").c_str());
   boost::filesystem::create_directory((ros::package::getPath("rgbdtam")+"/src/map_and_poses").c_str());
   boost::filesystem::remove_all((ros::package::getPath("rgbdtam")+"/src/evaluation").c_str());
   boost::filesystem::create_directory((ros::package::getPath("rgbdtam")+"/src/evaluation").c_str());
   boost::filesystem::remove_all((ros::package::getPath("rgbdtam")+"/src/results_depth_maps").c_str());
   boost::filesystem::create_directory((ros::package::getPath("rgbdtam")+"/src/results_depth_maps").c_str());

   R = (cv::Mat_<float>(3, 3) <<  1,0,0,0,1,0,0,0,1);
   t = (cv::Mat_<float>(3, 1) << 0,0,0);
   cv::Mat R_kf;
   cv::Mat t_kf;


   R_kf = R.clone();
   t_kf = t.clone();
   R_prev = R.clone();
   R_post = R.clone();
   t_prev = t.clone();
   t_post = t.clone();

   tracking_th = (float)fs2["tracking_th"];
   discretization = 100;

   local_maps_close_number = (int)fs2["local_maps_close_number"];
   local_maps_number = (int)fs2["local_maps_number"];
   SemiDenseTracking::init_points_last_keyframes(pyramid_levels);

   points_projected_in_image = 0;
   iter_th = (int)fs2["iter_th"];
   init_poses();

   frame_struct = new FrameStruct;
   cont_frames = new int;
   stamps_ros = new ros::Time;

   fs2.release();
}


void SemiDenseTracking::init_local_maps(int local_maps_number)
{
        local_maps.resize(local_maps_number);
}

void SemiDenseTracking::set_local_maps (cv::Mat local_maps_aux,int pos_map)
{
        local_maps[pos_map] = local_maps_aux.clone();
}

void SemiDenseTracking::init_poses () {
    cv::Mat poses_aux(0,3,CV_32FC1);
    poses = poses_aux.clone();
}

void SemiDenseTracking::set_poses (cv::Mat pose_aux) {
    poses.push_back(pose_aux);
}

void SemiDenseTracking::init_poses_local_maps(int poses_local_maps_number)
{
    poses_local_maps.resize(poses_local_maps_number);
}
void SemiDenseTracking::set_poses_local_maps (cv::Mat poses_local_maps_aux,int pos_map) {
    poses_local_maps[pos_map] = poses_local_maps_aux.clone();
}

void SemiDenseTracking::init_points_last_keyframes(int points_last_keyframes_number)
{
    points_last_keyframes.resize(points_last_keyframes_number);
}

void SemiDenseTracking::set_points_last_keyframes(vector<cv::Mat> points_last_keyframes_aux) {
    for (int i = 0; i < points_last_keyframes_aux.size();i++)
    {
        points_last_keyframes[i] =points_last_keyframes_aux[i].clone();
    }
}

void SemiDenseTracking::init_distances_local_maps(int distances_local_maps_number)
{
    distances_local_maps.resize(distances_local_maps_number);
}

void SemiDenseTracking::set_distances_local_maps(float distances_local_maps_aux,int pos_map) {

    distances_local_maps[pos_map] =  distances_local_maps_aux;
}

void SemiDenseTracking::inititialize_vectors_pyramid_levels_size(int pyramid_levels)
{
    jacobian.resize(pyramid_levels);
    jacobian_photo_no_weight.resize(pyramid_levels);
    jacobian_geo_no_weight.resize(pyramid_levels);
    hessian_vision_geo.resize(pyramid_levels);
    pixels_input.resize(pyramid_levels);
    variances.resize(pyramid_levels);
    reduction_pyramid.resize(pyramid_levels);
    gradient_by_epipolar.resize(pyramid_levels);
    potential_keyframe.resize(pyramid_levels);
    focalx.resize(pyramid_levels);
    focaly.resize(pyramid_levels);
    centerx.resize(pyramid_levels);
    centery.resize(pyramid_levels);
    color.resize(pyramid_levels);
    image_keyframe_pyramid.resize(pyramid_levels);
    points_map.resize(pyramid_levels);
    points3D.resize(pyramid_levels);
    points3D_geo.resize(pyramid_levels);
    weight_geo.resize(pyramid_levels);
    points_map_inImage.resize(pyramid_levels);
    image_reduced.resize(pyramid_levels);
    image_reduced_aux.resize(pyramid_levels);
    frame_depth_aux.resize(pyramid_levels);
    keyframe_depth.resize(pyramid_levels);
    frame_depth.resize(pyramid_levels);
    error_vector.resize(pyramid_levels);
    weight.resize(pyramid_levels);
    GX_ref.resize(pyramid_levels);
    GY_ref.resize(pyramid_levels);
}

void  SemiDenseTracking::read_image_names(vector<string> &left_image_names,vector<string> &depth_image_names)
{

    /// how to generalte files.txt -> file */* *>> ../files.txt


     string string_path_data_aux_left ;
     string_path_data_aux_left =  path_to_folder + "rgb/";


    string line;
    ifstream myfile (path_to_folder + "files_rgb.txt");

      if (myfile.is_open())
      {
        while ( getline (myfile,line)  )
        {
            string image_name;

            image_name = line.substr(0,21) ;

            string string_path_data =string_path_data_aux_left + image_name;
            left_image_names.push_back(string_path_data);
        }
      }



      string_path_data_aux_left = path_to_folder + "depth/";
      ifstream myfile_depth (path_to_folder + "files_depth.txt");


       if (myfile_depth.is_open())
       {
         while ( getline (myfile_depth,line)  )
         {
             string image_name;
             image_name = line.substr(0,21) ;
             string string_path_data =string_path_data_aux_left + image_name;
             depth_image_names.push_back(string_path_data);
         }
       }
 }

void undistort_image(cv::Mat &image_frame, cv::Mat &cameraMatrixAux,cv::Mat &distCoeffs,
                     cv::Mat &newCameraMatrix,cv::Mat &mapX,cv::Mat &mapY)
{
     if (mapX.rows==0)
     {
         if (image_frame.type()==CV_8UC1) {
                //input image is grayscale
                cv::cvtColor(image_frame, image_frame, CV_GRAY2RGB);
         }

         cv::Size ksize,ksize1;
         ksize.width = image_frame.cols;
         ksize.height = image_frame.rows;

         float alpha = 0;
         newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrixAux,distCoeffs,ksize,alpha,ksize1);


         cv::Mat image_frame_undistorted;
         cv::undistort(image_frame,image_frame_undistorted,cameraMatrixAux,distCoeffs,newCameraMatrix);


         cv::Mat R;
         cv::initUndistortRectifyMap(cameraMatrixAux,distCoeffs,R,newCameraMatrix,ksize,CV_16SC2,mapX,mapY);

         image_frame = image_frame_undistorted.clone();
    }
    else
    {
        if (image_frame.type()==CV_8UC1) {
               //input image is grayscale
               cv::cvtColor(image_frame, image_frame, CV_GRAY2RGB);
        }
        cv::Size ksize,ksize1;

        ksize.width = image_frame.cols;
        ksize.height = image_frame.rows;
        float alpha = 0;
        newCameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrixAux,distCoeffs,ksize,alpha,ksize1);

        cv::Mat undistorted_image_map;
        cv::remap(image_frame,undistorted_image_map,mapX,mapY,CV_INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
        image_frame = undistorted_image_map.clone();
    }
}

void prepare_image(SemiDenseTracking *semidense_tracker, cv::Mat &image_frame,
                                       cv::Mat &image_to_track, int &image_n,
                                       cv::Mat &image_gray, cv::Mat cameraMatrix, cv::Mat distCoeffs, \
                                       float &fx, float &fy, float &cx, float &cy)
{
     tic_initt();

     cv::Mat cameraMatrixAux = cameraMatrix.clone();
     cv::Mat newCameraMatrix;
     undistort_image(image_frame, cameraMatrixAux,distCoeffs,newCameraMatrix,semidense_tracker->mapX,semidense_tracker->mapY);

     fx = newCameraMatrix.at<float>(0,0);
     fy = newCameraMatrix.at<float>(1,1);
     cx = newCameraMatrix.at<float>(0,2);
     cy = newCameraMatrix.at<float>(1,2);

     image_n++;

     cv::cvtColor(image_frame,image_to_track,CV_RGB2GRAY);
     image_to_track.convertTo(image_to_track, CV_32FC1);
     image_gray = image_to_track.clone();
     image_to_track /= (255*1.0);
}


void ThreadViewerUpdater( SemiDenseTracking *semidense_tracker,SemiDenseMapping *semidense_mapper, DenseMapping *dense_mapper)
{
    while((ros::ok() && semidense_tracker->keepvisualizer))
    {
        {
          boost::mutex::scoped_lock lock( semidense_tracker->loopcloser_obj.guard);
          semidense_tracker->loopcloser_obj.viewer->spinOnce(1);
        }
          boost::this_thread::sleep(boost::posix_time::milliseconds(200));
    }
}

void ThreadImageProcessing( SemiDenseTracking *semidense_tracker,
                            SemiDenseMapping *semidense_mapper,
                            DenseMapping *dense_mapper)
{
    while(ros::ok() && dense_mapper->sequence_has_finished == false  || semidense_mapper->num_keyframes < 2)
    {
        if(!semidense_tracker->image_processing_semaphore)
        {
              if( *semidense_tracker->cont_frames > semidense_tracker->last_cont_frames )
              {

                  ///GET image undistortion
                  FrameStruct frame_struct_aux;
                  semidense_tracker->last_cont_frames = *semidense_tracker->cont_frames;

                  frame_struct_aux.image_frame  = (semidense_tracker->frame_struct->image_frame).clone();
                  frame_struct_aux.stamps = semidense_tracker->frame_struct->stamps;
                  /*if (semidense_tracker->bgr2rgb > 0.5){cv::cvtColor(frame_struct_aux.image_frame,
                                                                     frame_struct_aux.image_frame,CV_RGB2BGR);}*/

                  int image_n = 0;
                  prepare_image(semidense_tracker,frame_struct_aux.image_frame,frame_struct_aux.image_to_track,\
                        image_n,frame_struct_aux.image_gray,semidense_tracker->cameraMatrix,semidense_tracker->distCoeffs,\
                        frame_struct_aux.fx,frame_struct_aux.fy,frame_struct_aux.cx,frame_struct_aux.cy);
                  semidense_tracker->frame_struct_vector.push_back(frame_struct_aux);




                  /// GET image and depth pyramids
                  cv::Mat depth_frame;
                  if(semidense_tracker->use_kinect == 1)
                  {
                    get_depth_image( semidense_tracker,semidense_mapper,frame_struct_aux.stamps,depth_frame);
                  }

                  cv::Mat image_to_track;
                  frame_struct_aux.image_to_track.copyTo(image_to_track);

                  for (int j = semidense_tracker->pyramid_levels-1 ; j > -1; j--)
                  {
                      if ( (semidense_tracker->reduction_pyramid[j]) > 1)
                      {
                          cv::resize(image_to_track,semidense_tracker->image_reduced_aux[j],
                                     cv::Size(image_to_track.cols/(2),image_to_track.rows/(2)),
                                     0,0,cv::INTER_LINEAR);

                          if(semidense_tracker->use_kinect == 1)
                          {
                              semidense_tracker->frame_depth_aux[j] = reduce_depth_image(depth_frame,2);
                          }
                      }
                      else
                      {
                          semidense_tracker->image_reduced_aux[j] = image_to_track.clone();

                          if(semidense_tracker->use_kinect == 1)
                          {
                             semidense_tracker->frame_depth_aux[j] = depth_frame.clone();
                          }
                      }
                      image_to_track = semidense_tracker->image_reduced_aux[j].clone();
                      if(semidense_tracker->use_kinect == 1)
                      {
                          depth_frame =  semidense_tracker->frame_depth_aux[j].clone();
                      }
                  }


                  semidense_tracker->image_processing_semaphore = true;
              }else{
                  boost::this_thread::sleep(boost::posix_time::milliseconds(1));
              }
        }
    }
}

void ThreadSemiDenseTracker(Images_class *images,SemiDenseMapping *semidense_mapper,\
                            SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,\
                            MapShared *Map,ros::Publisher *vis_pub,image_transport::Publisher *pub_image)
{
    while(ros::ok() && dense_mapper->sequence_has_finished == false || semidense_mapper->num_keyframes < 2)
    {
        if (semidense_mapper->semaphore == false || (semidense_tracker->use_ros == 1))
        {
            //semidense_tracker->image_processing_semaphore = false;
            if(semidense_tracker->use_ros == 0 || semidense_tracker->image_processing_semaphore == true)
            {
                   semidense_tracking(images,semidense_mapper,semidense_tracker,dense_mapper,Map,vis_pub,pub_image);
                   //boost::this_thread::sleep(boost::posix_time::milliseconds(1));

                   /// check if the sequence has finished
                   double time_check_if_seq_is_finished = 0;
                   toc_finalt(time_check_if_seq_is_finished);

                   if (time_check_if_seq_is_finished > 8 && semidense_mapper->num_keyframes > 10)
                   {dense_mapper->sequence_has_finished = true;}
            }
        }
    }

    cout << "semidense tracker thread has finished " <<endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
    cout << "Printing keyframes after posegraph optimization" << endl;
    semidense_tracker->loopcloser_obj.print_keyframes_after_optimization();
    cout << "Total keyframes: " << semidense_tracker->loopcloser_obj.keyframes_vector.size() << endl;
    cout << "evaluating trayectory" << endl;


    /// WE keep the visualizer for a few seconds even though the sequence has already finished
    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
    semidense_tracker->keepvisualizer = false;
    /// WE keep the visualizer for a few seconds


   /* chdir("/home/alejo/catkin_ws/src/evaluate_error_monocular/evaluate_all");
    char buffer[200];

    sprintf(buffer,"cd");
    system(buffer);
    sprintf(buffer,"cd /home/alejo/catkin_ws/src/evaluate_error_monocular/evaluate_all");
    system(buffer);


    string str = "python evaluate_ate_scale.py --plot plot "  +
            semidense_tracker->path_to_folder.substr(33,semidense_tracker->path_to_folder.size()-1-33) +
            "-groundtruth.txt  /home/alejo/catkin_ws/src/rgbdtam/src/evaluation/trajectory_no_posegraph.txt";
    const char *cstr = str.c_str();
    system(cstr);





    sprintf(buffer,"cd");
    system(buffer);
    sprintf(buffer,"cd /home/alejo/catkin_ws/src/evaluate_error_monocular/evaluate_all");
    system(buffer);

    str = "python evaluate_ate_scale.py --plot plot "  +
            semidense_tracker->path_to_folder.substr(33,semidense_tracker->path_to_folder.size()-1-33) +
            "-groundtruth.txt  /home/alejo/catkin_ws/src/rgbdtam/src/evaluation/trajectory_posegraph.txt";
    const char *cstr2 = str.c_str();
    system(cstr2);*/


    cout << "thread tracking finished" << endl;
    ros::shutdown();
    return;
}

inline cv::Mat reduce_depth_image(cv::Mat &depth_image,int reduction)
{
     cv::Mat return_image;

     return_image = cv::Mat::zeros(depth_image.rows/reduction,
                                   depth_image.cols/reduction,CV_32FC1);

     cv::Mat image_blurred ;
     cv::medianBlur( depth_image, image_blurred, 3 );

     for(int i = 1; i < depth_image.rows/reduction-1; i++){
         for(int j = 1 ; j< depth_image.cols/reduction-1; j++){
             return_image.at<float>(i,j) = image_blurred.at<float>(2*i,2*j);
         }
     }
     return return_image;


    /* cv::resize(depth_image,depth_image,
                cv::Size(depth_image.cols/(reduction),depth_image.rows/(reduction)),
                0,0,cv::INTER_NEAREST);
     return depth_image;*/
}

inline void bilinear_interpolation(cv::Mat &image,float x_2,float y_2,float &value)
{
    int x_1 = static_cast<int>(x_2);
    int x_3 = x_1 +1;
    int y_1 = static_cast<int>(y_2);
    int y_3 = y_1 +1;

    float c1 = image.at<float>(y_1,x_1);
    float c2 = image.at<float>(y_3,x_1);

    float r1 = c1*(y_3-y_2) + c2*(y_2-y_1);

    c1 = image.at<float>(y_1,x_3);
    c2 = image.at<float>(y_3,x_3);

    float r2 = c1*(y_3-y_2) + c2*(y_2-y_1);
    value =  r1*(x_3-x_2) + r2*(x_2-x_1);
}

void correct_kf_after_posegraph_optimization(SemiDenseTracking *semidense_tracker, cv::Mat &pointsKF,int kf){

    if(semidense_tracker->loopcloser_obj.R_after_opt.size() > kf)
    {
           cv::Mat point_cloud_xyz = pointsKF.colRange(0,3);

           point_cloud_xyz = point_cloud_xyz.t();

           cv::Mat R_init,t_init, R_end,t_end,t_init_vector,t_end_vector;
           R_init = semidense_tracker->loopcloser_obj.keyframes_vector.at(kf).R.clone();
           t_init = semidense_tracker->loopcloser_obj.keyframes_vector.at(kf).t.clone();
           R_end = semidense_tracker->loopcloser_obj.R_after_opt.at(kf).clone();
           t_end = semidense_tracker->loopcloser_obj.t_after_opt.at(kf).clone();
           t_end = t_end.t();

           R_end.convertTo(R_end,CV_32FC1);
           t_end.convertTo(t_end,CV_32FC1);


           t_end_vector =   cv::repeat(t_end, 1,point_cloud_xyz.cols);
           t_init_vector =  cv::repeat(t_init,1,point_cloud_xyz.cols);


           point_cloud_xyz = R_init * point_cloud_xyz + t_init_vector;
           float scale = semidense_tracker->loopcloser_obj.s_after_opt.at(kf);

           point_cloud_xyz = scale *
                   R_end * point_cloud_xyz + t_end_vector;
           point_cloud_xyz = point_cloud_xyz.t();

           for (int j = 0 ; j < point_cloud_xyz.rows; j++)
           {
               pointsKF.at<float>(j,0) = point_cloud_xyz.at<float>(j,0);
               pointsKF.at<float>(j,1) = point_cloud_xyz.at<float>(j,1);
               pointsKF.at<float>(j,2) = point_cloud_xyz.at<float>(j,2);
           }
    }
}

void map_reuse(SemiDenseTracking *semidense_tracker,SemiDenseMapping *semidense_mapper,cv::Mat &image_frame_aux
               ,cv::Mat &R_kf, cv::Mat &t_kf){

    if(!semidense_tracker->SystemIsLost)
    {
          /* cv::Mat matchings_mapreuse = semidense_tracker->loopcloser_obj.keyframes_vector
                    [semidense_tracker->loopcloser_obj.keyframes_vector.size()-1].matchings_mapreuse.clone();*/

        // if you use this, you can remove current keyframe and comment get_score_loopclosure in loop closer.
           cv::Mat matchings_mapreuse ;
           int points_tracked = semidense_tracker->points_map[semidense_tracker->pyramid_levels-1].rows / 10;
           semidense_tracker->loopcloser_obj.get_potential_keyframes(image_frame_aux, matchings_mapreuse,points_tracked);

            int oldest_kf = 10000;
            if(matchings_mapreuse.rows > 0)
            {
                cv::Mat R_aux = semidense_tracker->R.clone();
                cv::Mat t_aux = semidense_tracker->t.clone();
                bool found_old_kf = false;

                for(int ii = matchings_mapreuse.rows-1;ii > -1  ; ii--)
                {
                    if( matchings_mapreuse.at<float>(ii,1) < oldest_kf)
                    {
                           cv::Mat  pointsKF = semidense_tracker->loopcloser_obj.keyframes_vector
                                  [matchings_mapreuse.at<float>(ii,1)].point_cloud_totrack[0].clone();


                           correct_kf_after_posegraph_optimization(semidense_tracker,pointsKF,matchings_mapreuse.at<float>(ii,1));

                           pointsKF = pointsKF.colRange(0,3);
                           pointsKF = pointsKF.t();
                           cv::Mat transformed_points_cam,coordinates_cam;

                           transform_points_return_3Dpoints (coordinates_cam, semidense_tracker->R,
                                                                semidense_tracker->t,semidense_tracker->focalx[0],
                                                                semidense_tracker->focaly[0],\
                                                                semidense_tracker->centerx[0],
                                                                semidense_tracker->centery[0],
                                                                pointsKF,transformed_points_cam);
                            float overlap=0;
                            coordinates_cam=coordinates_cam.t();

                            float min_x = 1000;float max_x = 0;
                            float min_y = 1000;float max_y = 0;
                            float min_x_t = 1000;float max_x_t = 0;
                            float min_y_t = 1000; float max_y_t = 0;

                            for(int jj = 0; jj < coordinates_cam.rows;jj++)
                            {
                                if(coordinates_cam.at<float>(jj,0) > 0  &&  coordinates_cam.at<float>(jj,1) > 0  &&
                                   coordinates_cam.at<float>(jj,0) < image_frame_aux.cols/8 &&  coordinates_cam.at<float>(jj,1) < image_frame_aux.rows/8 )
                                {
                                    overlap++;
                                    if(coordinates_cam.at<float>(jj,0)  > max_x)max_x = coordinates_cam.at<float>(jj,0) ;
                                    if(coordinates_cam.at<float>(jj,1)  > max_y)max_y = coordinates_cam.at<float>(jj,1) ;
                                    if(coordinates_cam.at<float>(jj,0)  < min_x)min_x = coordinates_cam.at<float>(jj,0) ;
                                    if(coordinates_cam.at<float>(jj,1)  < min_y)min_y = coordinates_cam.at<float>(jj,1) ;
                                }
                                if(coordinates_cam.at<float>(jj,0)  > max_x_t)max_x_t = coordinates_cam.at<float>(jj,0) ;
                                if(coordinates_cam.at<float>(jj,1)  > max_y_t)max_y_t = coordinates_cam.at<float>(jj,1) ;
                                if(coordinates_cam.at<float>(jj,0)  < min_x_t)min_x_t = coordinates_cam.at<float>(jj,0) ;
                                if(coordinates_cam.at<float>(jj,1)  < min_y_t)min_y_t = coordinates_cam.at<float>(jj,1) ;
                            }

                            if(overlap /  coordinates_cam.rows > 0.75  &&
                              (max_x - min_x) * (max_y-min_y) > image_frame_aux.cols/8*image_frame_aux.rows/8*0.25
                               //&& max_x_t < 90 && max_y_t < 70 && min_x_t > -10 && min_y_t > -10
                               )
                            {
                                cv::Mat color;
                                get_color(semidense_tracker->loopcloser_obj.keyframes_vector
                                          [matchings_mapreuse.at<float>(ii,1)].point_cloud_totrack[0],color);
                                float variance = 0.03; cv::Mat weight;
                                cv::Mat error_vector_sqrt = cv::Mat::zeros(pointsKF.cols,1,CV_32FC1);
                                cv::Mat error_vector = cv::Mat::zeros(pointsKF.cols,1,CV_32FC1);
                                cv::Mat error_check = cv::Mat::zeros(pointsKF.cols,1,CV_32FC1);
                                coordinates_cam = coordinates_cam.t();
                                compute_error( coordinates_cam,semidense_tracker->image_reduced[0],color, error_vector,
                                                variance,error_vector_sqrt,error_check,weight);

                                if( cv::mean(error_check)[0] < 0.035)
                                {
                                    oldest_kf = matchings_mapreuse.at<float>(ii,1);
                                    R_aux = semidense_tracker->R.clone();
                                    t_aux = semidense_tracker->t.clone();
                                    found_old_kf = true;
                                    semidense_mapper->reusing_map = true;
                                }
                            }
                    }
                }

                semidense_tracker->R = R_aux.clone();
                semidense_tracker->t = t_aux.clone();

                if(found_old_kf)
                {
                    cout << "REUSING previous Keyframe#:  "  << oldest_kf << "  Current Keyframes#:  " << semidense_tracker->loopcloser_obj.keyframes_vector.size()-1  << endl;

                    for(int j = 0 ; j < semidense_tracker->pyramid_levels; j++){
                        semidense_tracker->points_map[j] =  semidense_tracker->loopcloser_obj.keyframes_vector
                                [oldest_kf].point_cloud_totrack[j].clone();
                        correct_kf_after_posegraph_optimization(semidense_tracker,semidense_tracker->points_map[j],oldest_kf);
                    }

                    R_kf = semidense_tracker->loopcloser_obj.keyframes_vector
                            [oldest_kf].R.clone();
                    t_kf = semidense_tracker->loopcloser_obj.keyframes_vector
                            [oldest_kf].t.clone();

                    semidense_tracker->image_prev = semidense_tracker->loopcloser_obj.keyframes_vector
                            [oldest_kf].image.clone();

                    cv::Mat depth_frame;
                    if(semidense_tracker->use_kinect == 1)
                    {
                        get_depth_image( semidense_tracker, semidense_mapper,semidense_tracker->loopcloser_obj.keyframes_vector
                                [oldest_kf].stamps,depth_frame);

                        for (int j = semidense_tracker->pyramid_levels-1 ; j > -1; j--)
                        {
                            if ( (semidense_tracker->reduction_pyramid[j]) > 1)
                            {
                                semidense_tracker->keyframe_depth[j] = reduce_depth_image(depth_frame,2);
                            }
                            else
                            {
                                semidense_tracker->keyframe_depth[j] = depth_frame;
                            }
                            depth_frame =  semidense_tracker->keyframe_depth[j];
                        }
                    }
                }
            }
    } // !systemIsLOST
}

void relocalization(SemiDenseTracking *semidense_tracker,SemiDenseMapping *semidense_mapper,cv::Mat &image_frame_aux
                    ,cv::Mat &R_kf, cv::Mat &t_kf,image_transport::Publisher *pub_image){
    if(semidense_tracker->SystemIsLost)
    {
         int oldest_kf = -1;
         cv::Mat image_rgb =  image_frame_aux.clone();
         cv::Mat matchings(0,2,CV_32FC1);
         semidense_tracker->loopcloser_obj.relocalization(image_rgb,semidense_tracker->R,semidense_tracker->t, oldest_kf,matchings);
         float error_photo = 1;

         if( oldest_kf > -1){
             cv::Mat  pointsKF = semidense_tracker->loopcloser_obj.keyframes_vector
                    [oldest_kf].point_cloud_totrack[0].clone();
              pointsKF = pointsKF.colRange(0,3);
              pointsKF = pointsKF.t();
              cv::Mat transformed_points_cam,coordinates_cam;

              transform_points_return_3Dpoints (coordinates_cam, semidense_tracker->R,
                                                  semidense_tracker->t,semidense_tracker->focalx[0],
                                                  semidense_tracker->focaly[0],\
                                                  semidense_tracker->centerx[0],
                                                  semidense_tracker->centery[0],
                                                  pointsKF,transformed_points_cam);
              cv::Mat color;
              get_color(semidense_tracker->loopcloser_obj.keyframes_vector
                        [oldest_kf].point_cloud_totrack[0],color);
              float variance = 0.03; cv::Mat weight;
              cv::Mat error_vector_sqrt = cv::Mat::zeros(pointsKF.cols,1,CV_32FC1);
              cv::Mat error_vector = cv::Mat::zeros(pointsKF.cols,1,CV_32FC1);
              cv::Mat error_check = cv::Mat::zeros(pointsKF.cols,1,CV_32FC1);
              coordinates_cam = coordinates_cam.t();
              compute_error( coordinates_cam,semidense_tracker->image_reduced[0],color, error_vector,
                              variance,error_vector_sqrt,error_check,weight);
              error_photo = cv::mean(error_check)[0];
         }
         if(error_photo < 0.015)
         {
                 semidense_tracker->SystemIsLost = false;
                 semidense_mapper->reusing_map = true;


                 semidense_tracker->points_map = semidense_tracker->loopcloser_obj.keyframes_vector
                         [oldest_kf].point_cloud_totrack;


                 R_kf = semidense_tracker->loopcloser_obj.keyframes_vector
                         [oldest_kf].R.clone();
                 t_kf = semidense_tracker->loopcloser_obj.keyframes_vector
                         [oldest_kf].t.clone();

                 semidense_tracker->image_prev = semidense_tracker->loopcloser_obj.keyframes_vector
                         [oldest_kf].image.clone();


                semidense_tracker->image_reduced[semidense_tracker->pyramid_levels-1] = semidense_tracker->image_to_track.clone();
                 for (int j = semidense_tracker->pyramid_levels-2 ; j > -1; j--)
                 {
                     cv::resize(semidense_tracker->image_reduced[j+1],semidense_tracker->image_reduced[j],
                     cv::Size(semidense_tracker->image_reduced[j+1].cols/(2),semidense_tracker->image_reduced[j+1].rows/(2)),0,0,cv::INTER_LINEAR);
                 }


                 cv::Mat depth_frame;
                 if(semidense_tracker->use_kinect == 1)
                 {
                     get_depth_image( semidense_tracker, semidense_mapper,semidense_tracker->loopcloser_obj.keyframes_vector
                             [oldest_kf].stamps,depth_frame);


                     for (int j = semidense_tracker->pyramid_levels-1 ; j > -1; j--)
                     {
                         if ( semidense_tracker->reduction_pyramid[j] > 1)
                         {
                             semidense_tracker->keyframe_depth[j] = reduce_depth_image(depth_frame,2);
                         }
                         else
                         {
                             semidense_tracker->keyframe_depth[j] = depth_frame;
                         }
                         depth_frame =  semidense_tracker->keyframe_depth[j];
                     }
                 }
         }else{
             //SYSTEM IS LOST, print current frame;
             sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image_frame_aux).toImageMsg();
             pub_image->publish(msg);
         }
    }
}

void get_depth_image( SemiDenseTracking *semidense_tracker,
                      SemiDenseMapping *semidense_mapper,
                      double stamps_aux,cv::Mat &depth_frame)
{
    if(semidense_tracker->use_ros == 0)
    {
       double stamp_ref_image = stamps_aux;

       double stamp_error = 10000;
       int depth_index = 0;
       for (int i = 0; i< semidense_tracker->depth_image_names.size();i++)
       {
          string data = semidense_tracker->depth_image_names[i];
          data = data.substr((data.length()-21),data.length());
          std::string::size_type sz;     // alias of size_t
          double depth_stamp = std::stod (data,&sz);

          if (fabs(depth_stamp-stamp_ref_image + semidense_tracker->depth_rgb_offset) < stamp_error)
          {
            stamp_error = fabs(depth_stamp-stamp_ref_image);
            depth_index = i;
          }
      }
      depth_frame = cv::imread(semidense_tracker->depth_image_names[depth_index], CV_LOAD_IMAGE_UNCHANGED);
      depth_frame.convertTo(depth_frame,CV_32FC1);

      //UNDISTORT DEPTH MAP
      cv::Size ksize;
      ksize.width = depth_frame.cols;
      ksize.height = depth_frame.rows;
      cv::Mat undistorted_depth_map;
      cv::remap(depth_frame,undistorted_depth_map,semidense_tracker->mapX,semidense_tracker->mapY,
                CV_INTER_NN,cv::BORDER_CONSTANT,cv::Scalar(0,0,0));
      depth_frame = undistorted_depth_map.clone();
      //UNDISTORT DEPTH MAP

       depth_frame = (depth_frame / 5000.0);
    }
    else
    {
        double stamp_ref_image = stamps_aux;

        double stamp_error = 10000;
        int depth_index = 0;
        for (int i = 0; i< semidense_mapper->image_depth_keyframes.size();i++)
        {
            double depth_stamp = semidense_mapper->stamps_depth_ros[i];

            if (fabs(depth_stamp-stamp_ref_image-1*0.020) < stamp_error)
            {
                stamp_error = fabs(depth_stamp-stamp_ref_image);
                depth_index = i;
            }
        }
        depth_frame = semidense_mapper->image_depth_keyframes[depth_index];
    }
}

void semidense_tracking(Images_class *images,SemiDenseMapping *semidense_mapper,\
                        SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,\
                        MapShared *Map,ros::Publisher *vis_pub,image_transport::Publisher *pub_image)
{

    if (semidense_tracker->frames.size() <= 2 && semidense_tracker->use_ros  == 0)
    {
        if ( semidense_tracker->image_n < semidense_tracker->left_image_names.size() - semidense_tracker->init_frame)
        {
            cv::Mat image2read = cv::imread(semidense_tracker->left_image_names[semidense_tracker->image_n + semidense_tracker->init_frame]);

            /// add gaussian blur
            cv::Mat image_frame_aux;
            cv::GaussianBlur(image2read,image_frame_aux,cv::Size(0,0),3);
            cv::addWeighted(image2read,1.5,image_frame_aux,-0.5,0,image_frame_aux);
            image2read = image_frame_aux.clone();
            if (semidense_tracker->bgr2rgb > 0.5){cv::cvtColor(image2read,image2read,CV_RGB2BGR);}
            /// add gaussian blur


            semidense_tracker->frames.push_back(image2read);
            string data = semidense_tracker->left_image_names[semidense_tracker->image_n + semidense_tracker->init_frame];
            data = data.substr((data.length()-21),data.length());
            std::string::size_type sz;     // alias of size_t
            double earth = std::stod (data,&sz);
            semidense_tracker->frames_stamps.push_back(earth);
        }
    }

    // && *semidense_tracker->cont_frames > semidense_tracker->last_cont_frames
     if ((semidense_tracker->frames.size() > 2 && semidense_tracker->use_ros == 0) ||
             (semidense_tracker->use_ros == 1   &&
              semidense_tracker->frame_struct_vector.size() > 1))
     {
             cv::Mat image_frame_aux ;
             double stamps_aux;

             if(semidense_tracker->use_ros == 0)
             {
                  image_frame_aux  = semidense_tracker->frames[0].clone();
                  semidense_tracker->frames.erase (semidense_tracker->frames.begin(),semidense_tracker->frames.begin()+1);
                  stamps_aux =  semidense_tracker->frames_stamps[0];
                  semidense_tracker->frames_stamps.erase (semidense_tracker->frames_stamps.begin(),semidense_tracker->frames_stamps.begin()+1);

                  semidense_tracker->last_cont_frames = *semidense_tracker->cont_frames;

                  prepare_image(semidense_tracker,image_frame_aux,semidense_tracker->image_to_track,\
                                semidense_tracker->image_n,semidense_tracker->image_gray,semidense_tracker->cameraMatrix,semidense_tracker->distCoeffs,\
                                semidense_tracker->fx,semidense_tracker->fy,semidense_tracker->cx,semidense_tracker->cy);
              }else{

                 /* image_frame_aux  = (semidense_tracker->frame_struct->image_frame).clone();
                 stamps_aux = semidense_tracker->frame_struct->stamps;*/

                 image_frame_aux  = (semidense_tracker->frame_struct_vector[0].image_frame).clone();
                 stamps_aux = semidense_tracker->frame_struct_vector[0].stamps;

                   semidense_tracker->last_cont_frames = *semidense_tracker->cont_frames;

                   prepare_image(semidense_tracker,image_frame_aux,semidense_tracker->image_to_track,\
                                 semidense_tracker->image_n,semidense_tracker->image_gray,semidense_tracker->cameraMatrix,semidense_tracker->distCoeffs,\
                                 semidense_tracker->fx,semidense_tracker->fy,semidense_tracker->cx,semidense_tracker->cy);

                   semidense_tracker->frame_struct_vector.erase (semidense_tracker->frame_struct_vector.begin(),semidense_tracker->frame_struct_vector.begin()+1);
                   while(semidense_tracker->frame_struct_vector.size()>2){
                       semidense_tracker->frame_struct_vector.erase (semidense_tracker->frame_struct_vector.begin(),semidense_tracker->frame_struct_vector.begin()+1);
                   }
              }

            if (semidense_tracker->image_n > semidense_tracker->last_frame_tracked)
            {
            if (semidense_mapper->do_initialization_tracking > 0.5)
            {
                 cv::Mat R_kf, t_kf;
                 semidense_mapper->reusing_map = false;

                 for  ( int image_number = images->getNumberOfImages()-1; image_number > 0; image_number--)
                 {
                     semidense_tracker->image_prev = images->Im[image_number]->image.clone();
                     R_kf = images->Im[image_number]->R.clone();
                     t_kf = images->Im[image_number]->t.clone();

                     if (images->Im[image_number]->error < 0.5)
                     {

                         cv::Mat depth_frame;
                         if(semidense_tracker->use_kinect == 1)
                         {
                             get_depth_image( semidense_tracker,semidense_mapper,images->Im[image_number]->stamps,depth_frame);

                             for (int j = semidense_tracker->pyramid_levels-1 ; j > -1; j--)
                             {
                                 if ( (semidense_tracker->reduction_pyramid[j]) > 1)
                                 {
                                     semidense_tracker->keyframe_depth[j] = reduce_depth_image(depth_frame,2);
                                 }
                                 else
                                 {
                                     semidense_tracker->keyframe_depth[j] = depth_frame;
                                 }
                                 depth_frame =  semidense_tracker->keyframe_depth[j];
                             }
                         }
                         break;
                     }
                 }

                  /// RELEASE MEMORY
                 for (int i = 0;i<images->getNumberOfImages() ; i++)
                 {
                    delete images->Im[i];
                    images->Im[i] = NULL;
                 }
                 images->Im.clear();
                 /// RELEASE MEMORY



                for (int j=0; j<semidense_tracker->pyramid_levels ; j++)
                {
                     semidense_tracker->points_map[j] = semidense_mapper->get_points_new_map()[j].clone();
                }
                semidense_tracker->points_projected_in_image = 0;

                /// MAP REUSE
                map_reuse(semidense_tracker,semidense_mapper,image_frame_aux,R_kf, t_kf);
                /// MAP REUSE


                /// RELOCALIZATION IF NEEDED
                relocalization(semidense_tracker,semidense_mapper,image_frame_aux,R_kf, t_kf,pub_image);
                /// RELOCALIZATION IF NEEDED

               //// PREPARE SEMIDENSE
               if(!semidense_tracker->SystemIsLost)
               {

                    prepare_semidense(semidense_mapper,semidense_tracker->points_map,
                           semidense_tracker->R,semidense_tracker->t,\
                           semidense_tracker->get_points_last_keyframes(),\
                           semidense_tracker->pyramid_levels,semidense_tracker->focalx,semidense_tracker->focaly,\
                           semidense_tracker->centerx,semidense_tracker->centery,semidense_tracker->image_reduced,\
                           semidense_tracker->points_projected_in_image);

                    initialization_semidense(semidense_tracker,R_kf,t_kf,semidense_tracker->R_kf,\
                                       semidense_tracker->t_kf,semidense_tracker->image_prev,semidense_tracker->image_keyframe,\
                                       semidense_tracker->pyramid_levels, semidense_tracker->reduction_pyramid,\
                                       semidense_tracker->image_keyframe_pyramid);

                    for (int j = 0; j < semidense_tracker->pyramid_levels ; j++)
                    {
                       semidense_tracker->points_map_inImage[j] = semidense_tracker->points_map[j].clone();
                    }

                    // tracked points: some 3D points may be unaccurate
                    semidense_mapper->points_last_keyframe = \
                            semidense_tracker->points_map[semidense_tracker->pyramid_levels-1].clone();

                    // mapped points: accurate 3D points
                    if (semidense_mapper->local_map_points.rows > 100)
                    {semidense_mapper->points_last_keyframe = semidense_mapper->local_map_points.clone();}

                    semidense_tracker->processed_frames_since_keyframe = 0;
                    semidense_mapper->do_initialization_tracking = 0;
               } // !system is lost
               //// PREPARE SEMIDENSE

            } // do_initialization_tracking

            if (semidense_mapper->do_initialization > 0.5 )
            {
                cv::Mat depth_frame;


                depth_frame = cv::Mat::zeros(image_frame_aux.rows,image_frame_aux.cols,CV_32FC1);

                semidense_tracker->R = (cv::Mat_<float>(3, 3) <<  1,0,0,0,1,0,0,0,1);
                semidense_tracker->t = (cv::Mat_<float>(3, 1) << 0,0,0);
                semidense_tracker->R_kf =  semidense_tracker->R.clone();
                semidense_tracker->t_kf =  semidense_tracker->t.clone();

                if (semidense_mapper->kinect_initialization > 0.5)
                {
                     get_depth_image( semidense_tracker,semidense_mapper, stamps_aux,depth_frame);
                }
                if ( depth_frame.rows < 100)
                {
                    semidense_mapper->kinect_initialization  = 0;
                }


                initialization_semidense(semidense_tracker,semidense_tracker->R,semidense_tracker->t,semidense_tracker->R_kf,semidense_tracker->t_kf,image_frame_aux,semidense_tracker->image_keyframe,semidense_tracker->pyramid_levels,semidense_tracker->reduction_pyramid,semidense_tracker->focalx,semidense_tracker->focaly,\
                               semidense_tracker->image_keyframe_pyramid,semidense_tracker->points_map,semidense_tracker->color,semidense_tracker->points3D,semidense_tracker->jacobian,semidense_tracker->error_vector,semidense_tracker->weight,semidense_tracker->fx,semidense_tracker->fy, semidense_tracker->depth,\
                               depth_frame,semidense_mapper->kinect_initialization,semidense_tracker->cx,semidense_tracker->cy,semidense_tracker->centerx,semidense_tracker->centery,semidense_tracker->image_gray,semidense_mapper->limit_grad );

                semidense_tracker->processed_frames_since_keyframe = 0;
                semidense_mapper->points_last_keyframe = semidense_tracker->points_map[semidense_tracker->pyramid_levels-1].clone();
                semidense_mapper->do_initialization = 0;

                for (int j=0; j<semidense_tracker->pyramid_levels ; j++)
                {
                    semidense_tracker->points_map_inImage[j] = semidense_tracker->points_map[j].clone();
                }


                get_depth_image( semidense_tracker,semidense_mapper, stamps_aux,depth_frame);

                if(semidense_tracker->use_kinect == 1)
                {
                    cv::Mat depth_frame_aux;
                    get_depth_image( semidense_tracker,semidense_mapper, stamps_aux,depth_frame_aux);
                    for (int j = semidense_tracker->pyramid_levels-1 ; j > -1; j--)
                    {
                        if ( (semidense_tracker->reduction_pyramid[j]) > 1)
                        {
                           semidense_tracker->keyframe_depth[j] = reduce_depth_image(depth_frame_aux,2);
                        }
                        else
                        {
                            semidense_tracker->keyframe_depth[j] = depth_frame_aux;
                        }
                        depth_frame_aux =  semidense_tracker->keyframe_depth[j];
                    }
                }
            }


            if( !semidense_tracker->SystemIsLost)
            {
             cv::Mat  coordinates_cam_to_print;
             optimize_camera_pose(semidense_mapper->num_keyframes,semidense_tracker,semidense_mapper,*images,semidense_tracker->image_to_track,\
                            image_frame_aux,semidense_tracker->R,semidense_tracker->t,\
                            semidense_tracker->R_kf,semidense_tracker->t_kf,semidense_tracker->image_reduced,\
                            semidense_tracker->image_keyframe_pyramid,semidense_tracker->variance,\
                            semidense_tracker->reduction_pyramid,semidense_tracker->processed_frames_since_keyframe,\
                            semidense_tracker->jacobian,semidense_tracker->points_map_inImage,semidense_tracker->color,\
                            semidense_tracker->points3D,semidense_tracker->error_vector,semidense_tracker->weight,\
                            semidense_tracker->focalx,semidense_tracker->focaly,semidense_tracker->centerx, semidense_tracker->centery,\
                            semidense_tracker->pyramid_levels,semidense_tracker->overlap_tracking,\
                            semidense_tracker->tracking_th,semidense_tracker->iter_th,semidense_tracker->variances,\
                            semidense_tracker->image_gray, stamps_aux,semidense_mapper->mean_value,coordinates_cam_to_print);

             if(!semidense_tracker->SystemIsLost)
             {

                      semidense_tracker->frames_processed++;
                      if (images->getNumberOfImages() == 2 && semidense_mapper->num_keyframes % 10 == 0 && semidense_tracker->use_ros == 1)\
                      {
                          cout << "frames_processed -> " <<100.0 * semidense_tracker->frames_processed / *semidense_tracker->cont_frames << " %" << endl;
                      }


                     /// PRINT CAMERA IN PCL
                      {
                          boost::mutex::scoped_lock lock(semidense_tracker->loopcloser_obj.guard);
                          semidense_tracker->loopcloser_obj.addCameraPCL(semidense_tracker->R,semidense_tracker->t);
                      }
                     /// PRINT CAMERA IN PCL


                     Map->set_R(semidense_tracker->R);
                     Map->set_t(semidense_tracker->t);

                     semidense_tracker->R_post = semidense_tracker->R.clone();
                     semidense_tracker->t_post = semidense_tracker->t.clone();

                     if (semidense_tracker->points_projected_in_image < 5){semidense_tracker->points_projected_in_image = semidense_tracker->points_map[semidense_tracker->pyramid_levels-1].rows;}
                     semidense_mapper->overlap_tracking = 1.0*semidense_tracker->points_map_inImage[semidense_tracker->pyramid_levels-1].rows / semidense_tracker->points_map[semidense_tracker->pyramid_levels-1].rows;


                     if (semidense_tracker->image_n % 3 == 0)
                     {
                         int pyramid_level = semidense_tracker->pyramid_levels-1;

                         cv::Mat coordinates_cam_show;
                         cv::Mat points3D6_print = semidense_tracker->points_map[pyramid_level].colRange(0,3).t();

                         float focal_printx = semidense_tracker->focalx[pyramid_level];
                         float focal_printy = semidense_tracker->focaly[pyramid_level];
                         float center_printx = semidense_tracker->centerx[pyramid_level];
                         float center_printy = semidense_tracker->centery[pyramid_level];


                         coordinates_cam_show = coordinates_cam_to_print.clone();
                         int num_pixels_sd2project = coordinates_cam_show.cols;

                         if (dense_mapper->get_superpixels3Dtracked().rows > 0)
                         {
                             points3D6_print = dense_mapper->get_superpixels3Dtracked().clone();
                             points3D6_print=points3D6_print.t();

                             cv::Mat points3D_superpixels;

                             cv::Mat R_prueba = semidense_tracker->R.clone();
                             cv::Mat t_prueba =semidense_tracker->t.clone();
                             transform_points(points3D_superpixels,R_prueba ,t_prueba,focal_printx,focal_printy,center_printx,center_printy,points3D6_print);
                             points3D_superpixels = points3D_superpixels.t();
                             coordinates_cam_show = coordinates_cam_show.t();
                             coordinates_cam_show.push_back(points3D_superpixels);
                             coordinates_cam_show = coordinates_cam_show.t();
                         }


                         show_error_geo( semidense_tracker-> coordinates_depth_point_cloud,  image_frame_aux,
                                        semidense_tracker->weight_geo[semidense_tracker->pyramid_levels-1]);

                         show_error_photo( coordinates_cam_show,image_frame_aux,num_pixels_sd2project,
                                      pub_image,semidense_tracker->weight[semidense_tracker->pyramid_levels-1]);
                     }


                     if (semidense_tracker->create_inv_depth_discretization < 0.5)
                     {
                         float depth_step=0;
                         float mean_value;
                         cv::Mat depth_map,variance_points_tracked;

                         semidense_tracker->create_inv_depth_discretization=1;
                         get_inverse_depth(*images,semidense_mapper->points_last_keyframe,semidense_mapper->inv_depths,
                                           depth_step,0,semidense_tracker->discretization,mean_value,depth_map,1,variance_points_tracked);
                         semidense_tracker->mean_depth_value=mean_value;
                     }

                     semidense_tracker->last_frame_tracked = semidense_tracker->image_n;

                     semidense_tracker->processed_frames++;
                     semidense_tracker->processed_frames_since_keyframe++;

                     semidense_mapper->images_size = images->getNumberOfImages();


                     if (images->getNumberOfImages() > 1)
                     semidense_mapper->semaphore = true;
              } // if !SystemIsLost
            }// if !SystemIsLost
          } //  if (semidense_tracker->image_n > semidense_tracker->last_frame_tracked)
    } // if cont_frames >
} // semidense_mapping

void initialization_semidense(SemiDenseTracking *semidense_tracker,cv::Mat &R,cv::Mat &t,cv::Mat &R1,cv::Mat &t1,cv::Mat &image_rgb,cv::Mat &image_keyframe,\
                    int &pyramid_levels,vector<int> &reduction_pyramid, vector<cv::Mat> &image_keyframe_pyramid)
{
   image_rgb.copyTo(image_keyframe);
   cv::Mat image_p = image_rgb.clone();

   R.copyTo(R1);
   t.copyTo(t1);

   cv::cvtColor(image_p,image_p,CV_RGB2GRAY);
   image_p.convertTo(image_p, CV_32FC1);
   image_p /= (255*1.0);


   for (int j = pyramid_levels-1; j > -1 ; j--)
   {
          if((reduction_pyramid[j]) > 1.5)
         {
           cv::resize(image_p,image_keyframe_pyramid[j],cv::Size(image_p.cols/(2),image_p.rows/(2)),0,0,cv::INTER_LINEAR);
         }
         else
         {
           image_keyframe_pyramid[j] = image_p.clone();
         }
          image_p = image_keyframe_pyramid[j].clone();

         get_color(semidense_tracker->points_map[j],semidense_tracker->color[j]);
         semidense_tracker->points3D[j] = semidense_tracker->points_map[j].colRange(0,3).t();
         semidense_tracker->error_vector[j] = cv::Mat::zeros(semidense_tracker->points_map[j].rows,1,CV_32FC1);
         semidense_tracker->weight[j] = cv::Mat::zeros(semidense_tracker->points_map[j].rows,1,CV_32FC1) + 1;
         semidense_tracker->jacobian[j] = cv::Mat::zeros(semidense_tracker->points_map[j].rows,6,CV_32FC1);
   }
}

void initialization_semidense(SemiDenseTracking *semidense_tracker,cv::Mat &R,cv::Mat &t,cv::Mat &R1,cv::Mat &t1,cv::Mat &image_rgb,cv::Mat &image_keyframe,\
                    int &pyramid_levels,vector<int> &reduction_pyramid, vector<float> &focalx,vector<float> &focaly,vector<cv::Mat> &image_keyframe_pyramid,\
                    vector<cv::Mat> &points_map,vector<cv::Mat> &color,vector<cv::Mat> &points3D,vector<cv::Mat> &jacobian,vector<cv::Mat> &error_vector,vector<cv::Mat> &weight, float fx,float fy, float depth,\
                    cv::Mat depth_frame, int kinect_initialization,float cx,float cy,  vector<float> &centerx,vector<float> &centery,cv::Mat image_gray,float limit_grad  )
{

   image_rgb.copyTo(image_keyframe);

   cv::Mat image_p = image_rgb.clone();

   cv::Mat points_i(0,6, CV_32FC1);
   cv::Mat point_i(1,6, CV_32FC1);

   cv::Mat image_red = image_rgb.clone();


   float f_x = fx;
   float f_y = fy;
   float c_x = cx;
   float c_y = cy;


   cv::Mat points(0,6,CV_32FC1);
   float depth_aux = depth;


   for (int j = 0; j < image_red.rows-0; j = j+1)
   {
   for (int i = 0; i < image_red.cols-0; i = i+1)
   {
           if (kinect_initialization > 0.5)
           {
               if (fabs(depth_frame.at<float>(j,i))>0)
               {
                   depth = 1 / depth_frame.at<float>(j,i);
               }
               else
               {
                   depth = depth_aux/50;
               }
           }

           point_i.at<float>(0,0) =  -((i-c_x)/f_x)/(-depth);
           point_i.at<float>(0,1) =  -((c_y-j)/f_y)/(-depth);
           point_i.at<float>(0,2) =  1/(-depth);

           point_i.at<float>(0,3) =  image_gray.at<float>(j,i);
           point_i.at<float>(0,4) =  image_gray.at<float>(j,i);
           point_i.at<float>(0,5) =  image_gray.at<float>(j,i);
           points_i.push_back(point_i);
       }
   }
   points_i.copyTo(points);




   R.copyTo(R1);
   t.copyTo(t1);

   cv::cvtColor(image_p,image_p,CV_RGB2GRAY);
   image_p.convertTo(image_p, CV_32FC1);
   image_p /= (255*1.0);

   int reduction_ = pow(2,pyramid_levels) / (2);

   for (int j=0; j<pyramid_levels ; j++)
   {
         reduction_pyramid[j] = reduction_;
         reduction_/=2;
         cv::resize(image_p,image_keyframe_pyramid[j],cv::Size(image_p.cols/(reduction_pyramid[j]),
                                                               image_p.rows/(reduction_pyramid[j])),
                                                               0,0,cv::INTER_LINEAR);

         cv::Mat image_p1 = image_p.clone();
         resize_points(points_map[j],points,reduction_pyramid[j],image_p1,kinect_initialization,limit_grad);

         get_color(points_map[j],color[j]);
         points3D[j] = points_map[j].colRange(0,3).t();
         error_vector[j] = cv::Mat::zeros(points_map[j].rows,1,CV_32FC1);
         weight[j] = cv::Mat::zeros(points_map[j].rows,1,CV_32FC1) + 1;
         jacobian[j] = cv::Mat::zeros(points_map[j].rows,6,CV_32FC1);
         focalx[j] = fx/(reduction_pyramid[j]);
         focaly[j] = fy/(reduction_pyramid[j]);
         centerx[j] = cx/(reduction_pyramid[j]);
         centery[j] = cy/(reduction_pyramid[j]);
   }
}


void get_color (cv::Mat &points,cv::Mat &color)
{
    color = points.colRange(3,6);
    color.rowRange(0,color.rows).colRange(0,1) = 0.299*color.rowRange(0,color.rows).colRange(0,1) + 0.587*color.rowRange(0,color.rows).colRange(1,2) +0.114*color.rowRange(0,color.rows).colRange(2,3);
    color.rowRange(0,color.rows).colRange(0,1).copyTo(color);
    color /=(255*1.0);
}

void motion_model(vector<cv::Mat> &points_map,cv::Mat &R,cv::Mat &t,cv::Mat R_rel,cv::Mat t_rel,\
                vector<float> &focalx, vector<float> &focaly, vector<float> &centerx, vector<float> &centery,
                  vector<cv::Mat> &image_keyframe_pyramid,int pyramid_levels, bool &good_seed)
{
    cv::Mat pointsClouds3Dmap_cam;
    vector<cv::Mat> color(pyramid_levels);

    int i = 0;
    get_color(points_map[i],color[i]);
    cv::Mat error_vector(points_map[i].rows,1,CV_32FC1);
    cv::Mat error_vector_sqrt(points_map[i].rows,1,CV_32FC1);
    cv::Mat error_check(points_map[i].rows,1,CV_32FC1);
    cv::Mat weight(points_map[i].rows,1,CV_32FC1);

    float variance=0.003;


    cv::Mat pointsClouds3D =  points_map[i].t();
    pointsClouds3D = pointsClouds3D .rowRange(0,3);

    transform_points(pointsClouds3Dmap_cam, R,   t,   focalx[i],focaly[i],centerx[i],centery[i], pointsClouds3D   );
    compute_error( pointsClouds3Dmap_cam,image_keyframe_pyramid[i], color[i],error_vector,variance,error_vector_sqrt,error_check,weight);
    float error1 = mean(error_check)[0];
    cv::Mat t1 =  R*t_rel+t;
    cv::Mat R1 =  R*R_rel;

    transform_points(pointsClouds3Dmap_cam, R1,   t1,   focalx[i],focaly[i],centerx[i],centery[i], pointsClouds3D   );
    compute_error( pointsClouds3Dmap_cam,image_keyframe_pyramid[i], color[i],error_vector,variance,error_vector_sqrt,error_check,weight);
    float error2 = mean(error_check)[0];


    if (error2 < error1)
    {
        good_seed=true;
        R = R1.clone();
        t = t1.clone();
    }  
}

void optimize_camera_pose(int num_keyframes,SemiDenseTracking *semidense_tracker,
                    SemiDenseMapping *semidense_mapper,Images_class &images,cv::Mat &image_to_track,\
                    cv::Mat &image_rgb,cv::Mat &R,cv::Mat &t,cv::Mat &R1,cv::Mat &t1,\
                    vector<cv::Mat> &image_reduced, vector<cv::Mat> &image_keyframe_pyramid,
                    float &variance, vector<int> &reduction_pyramid,\
                    int &processed_frames, vector<cv::Mat> &jacobian, vector<cv::Mat> &points_map,\
                    vector<cv::Mat> &color,vector<cv::Mat> &points3D,vector<cv::Mat> &error_vector,
                    vector<cv::Mat> &weight,vector<float> &focalx,vector<float> &focaly,\
                    vector<float> &centerx,vector<float> &centery,int &pyramid_levels,
                    float &overlap_tracking,float &tracking_th, int iter_th,\
                    vector<float> &variances,cv::Mat &image_gray, double stamps,
                    float mean_depth_value,cv::Mat &coordinates_cam_to_print)
{
        /// REDUCING RGB AND DEPTH IMAGES
        #pragma omp parallel num_threads(2)
        {
           switch(omp_get_thread_num())
           {
                case 0:
                {
                        cv::Mat depth_frame;
                        if(semidense_tracker->use_kinect == 1)
                        {
                          get_depth_image( semidense_tracker,semidense_mapper,stamps,depth_frame);
                        }
                        for (int j = pyramid_levels-1 ; j > -1; j--)
                        {
                            if ( (reduction_pyramid[j]) > 1)
                            {
                                if(semidense_tracker->use_kinect == 1)
                                {
                                    semidense_tracker->frame_depth[j] = reduce_depth_image(depth_frame,2);
                                }
                            }
                            else
                            {
                                if(semidense_tracker->use_kinect == 1)
                                {
                                   semidense_tracker->frame_depth[j] = depth_frame;
                                }
                            }

                            if(semidense_tracker->use_kinect == 1)
                            {
                                depth_frame =  semidense_tracker->frame_depth[j];
                            }
                        }
               };break;
               case 1:{
                   for (int j = pyramid_levels-1 ; j > -1; j--)
                   {
                       if ( (reduction_pyramid[j]) > 1)
                       {
                            cv::resize(image_to_track,image_reduced[j],cv::Size(image_to_track.cols/(2), image_to_track.rows/(2)),
                                       0,0,cv::INTER_LINEAR);
                       }
                       else
                       {
                           image_reduced[j] = image_to_track;
                       }

                       image_to_track = image_reduced[j];
                   }
               }
            }
       }
       /// REDUCING RGB AND DEPTH IMAGES

    float error_p = 0;
    int iter = 0;


    cv::Mat R_p ;
    R1.copyTo(R_p);

    cv::Mat t_p;
    t1.copyTo(t_p);



    semidense_tracker->gain = 1;
    semidense_tracker->brightness = 0;

    int pyramid_levels_limit = pyramid_levels;

     /// CAMERA POSE ESTIMATION AND UPDATE OF ROBUST COST FUNCTION
     for (int j = 0 ; j <pyramid_levels_limit; j++)
     {
        //cv::Mat image_reduced_f = image_reduced[j].clone();
         //cv::Mat image_keyframe = image_keyframe_pyramid[j].clone();

         semidense_tracker->do_gain_brightness_estimation = false;
         if (j>0 && semidense_mapper->num_keyframes > 1)
         { semidense_tracker->do_gain_brightness_estimation = false;}


        gauss_estimation(semidense_tracker,coordinates_cam_to_print,R,t,R_p,t_p,\
                        focalx[j],focaly[j],centerx[j],centery[j],points_map[j],image_reduced[j],\
                        image_keyframe_pyramid[j],error_p,color[j],iter,variances[j],points3D[j],error_vector[j],weight[j],processed_frames,\
                        jacobian[j],overlap_tracking,tracking_th,iter_th,j,semidense_tracker->pixels_input[j],images.getNumberOfImages() == 0);

        cv::Mat sorted_error;
        cv::sort(cv::abs(error_vector[j]),sorted_error,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
        variances[j] = 3*(cv::mean(sorted_error)[0]);

        if(variances[j] == 0)
        {
            variances[j] =variance;
        }
        weight[j] = (1 + (error_vector[j]/(variances[j])));
        weight[j] = 1/weight[j];
        weight[j] = weight[j].mul(weight[j]);


        if(j==3){
            if ( semidense_tracker->PhotoError > 0   )
            {
                if( num_keyframes > 3 && ((sorted_error.at<float>(sorted_error.rows/2,0) / semidense_tracker->PhotoError > 7)||
                        isnan(sorted_error.at<float>(sorted_error.rows/2,0)))){
                  //semidense_tracker->SystemIsLost = true;
                  //cout << "SYSTEM IS LOST" << endl;
                }
            }
            semidense_tracker->PhotoError =  sorted_error.at<float>(sorted_error.rows/2,0)  ;
        }
    }
    /// CAMERA POSE ESTIMATION AND UPDATE OF ROBUST COST FUNCTION



    /// SEND CURRENT TRACKED MAP TO THE MAPPING THREAD AS INITIAL SEED
    if (images.getNumberOfImages() == 0  && !semidense_tracker->SystemIsLost )
    {
               //cout << "Points tracked: " << points_map[pyramid_levels-1].rows << endl;

                cv::Mat points3D_tracked = points_map[pyramid_levels-1].t();
                points3D_tracked = points3D_tracked.rowRange(0,3);

                cv::Mat color_points3D_tracked = color[pyramid_levels-1].clone();

                if(semidense_mapper->points3D_tracked.rows>0 && semidense_mapper->color_points3D_tracked.rows>0)
                {
                    points3D_tracked = semidense_mapper-> points3D_tracked.t();
                    points3D_tracked = points3D_tracked.rowRange(0,3);
                    color_points3D_tracked = semidense_mapper->color_points3D_tracked.clone();
                }
                else
                {
                    semidense_mapper->points3D_tracked  =  points_map[pyramid_levels-1].clone();
                }

                 cv::Mat error_vector_sqrt(points3D_tracked.cols,1,CV_32FC1);
                 cv::Mat weight_aux2( points3D_tracked.cols,1,CV_32FC1);
                 cv::Mat error_vector_aux( points3D_tracked.cols,1,CV_32FC1);
                 cv::Mat error_check(0,1,CV_32FC1);

                 cv::Mat coordinates_cam_p;
                 transform_points( coordinates_cam_p, R,t,focalx[pyramid_levels-1],focaly[pyramid_levels-1],
                         centerx[pyramid_levels-1],centery[pyramid_levels-1],points3D_tracked);

                 compute_error( coordinates_cam_p,image_reduced[pyramid_levels-1],
                         color_points3D_tracked,error_vector_aux,
                         variance,error_vector_sqrt,error_check,weight_aux2);
                 semidense_mapper->error_tracked_points =   error_vector_aux.clone();
    }


    float translational_ratio = 1;
    if (images.getNumberOfImages() > 0  && !semidense_tracker->SystemIsLost )
    {
        cv::Mat R2,t2,R1,t1;
        R2 =  images.Im[images.getNumberOfImages()-1]->R;
        t2 =  images.Im[images.getNumberOfImages()-1]->t;


        R1 =R.clone();
        t1 = t.clone();

        cv::Mat C1 = -R1.t()*t1;
        cv::Mat C2 = -R2.t()*t2;

        translational_ratio = (fabs(C1.at<float>(0,0) - C2.at<float>(0,0)) + fabs(C1.at<float>(1,0) - C2.at<float>(1,0)) + \
                              fabs(C1.at<float>(2,0) - C2.at<float>(2,0)) )  /  mean_depth_value;
        R2 =  images.Im[0]->R;
        t2 =  images.Im[0]->t;
        C2 = -R2.t()*t2;

        float  translational_ratio_wrt_kf = (fabs(C1.at<float>(0,0) - C2.at<float>(0,0)) + fabs(C1.at<float>(1,0) - C2.at<float>(1,0)) + \
                                      fabs(C1.at<float>(2,0) - C2.at<float>(2,0)) )  /  mean_depth_value;
        if (translational_ratio_wrt_kf < translational_ratio) { translational_ratio = translational_ratio_wrt_kf;}
    }


    if ((num_keyframes < 2 || images.getNumberOfImages()< 1.5 || translational_ratio >  0.002) &&   !semidense_tracker->SystemIsLost )
    {
        images.computeImage();
        int cont_images = images.getNumberOfImages()-1;

        images.Im[cont_images]->R = R.clone();
        images.Im[cont_images]->t = t.clone();
        images.Im[cont_images]->fx = focalx[pyramid_levels-1];
        images.Im[cont_images]->fy = focaly[pyramid_levels-1];
        images.Im[cont_images]->cx = centerx[pyramid_levels-1];
        images.Im[cont_images]->cy = centery[pyramid_levels-1];
        images.Im[cont_images]->num_keyframes = num_keyframes;

        images.Im[cont_images]->stamps = stamps;
        images.Im[cont_images]->is_used_for_mapping = 0;
        images.Im[cont_images]->image_gray = image_gray.clone();
        images.Im[cont_images]->image = image_rgb.clone();

        images.Im[cont_images]->error = 0;
        images.Im[cont_images]->image_number = semidense_tracker->image_n;
    }
    /// SEND CURRENT TRACKED MAP TO THE MAPPING THREAD AS INITIAL SEED
}

void show_error_photo( cv::Mat &coordinates_cam, cv::Mat &image_print,
                       int num_pixels_sd2project,image_transport::Publisher *pub_image,cv::Mat &weight)

{
    int imsize_x =image_print.cols-1;
    int imsize_y =image_print.rows-1;

    for (int k =0; k< coordinates_cam.cols;k++)
    {
       if (coordinates_cam.at<float>(1,k) > 1 && coordinates_cam.at<float>(1,k) < imsize_y-1 && coordinates_cam.at<float>(0,k) > 1
               && coordinates_cam.at<float>(0,k) < imsize_x-1)
         {
           //BILINEAR INTERPOLATION
           float x_2 = coordinates_cam.at<float>(0,k);
           float y_2 = coordinates_cam.at<float>(1,k);

           int xx = static_cast<int>(x_2);
           int yy = static_cast<int>(y_2);


           image_print.at<cv::Vec3b>(yy,xx)[2] = 0;
           image_print.at<cv::Vec3b>(yy,xx)[0] = 0;
           image_print.at<cv::Vec3b>(yy,xx)[1] = 0;
           image_print.at<cv::Vec3b>(yy+1,xx)[2] = 0;
           image_print.at<cv::Vec3b>(yy+1,xx)[0] = 0;
           image_print.at<cv::Vec3b>(yy+1,xx)[1] = 0;
           image_print.at<cv::Vec3b>(yy,xx+1)[2] = 0;
           image_print.at<cv::Vec3b>(yy,xx+1)[0] = 0;
           image_print.at<cv::Vec3b>(yy,xx+1)[1] = 0;
           image_print.at<cv::Vec3b>(yy-1,xx)[2] = 0;
           image_print.at<cv::Vec3b>(yy-1,xx)[0] = 0;
           image_print.at<cv::Vec3b>(yy-1,xx)[1] = 0;
           image_print.at<cv::Vec3b>(yy,xx-1)[2] = 0;
           image_print.at<cv::Vec3b>(yy,xx-1)[0] = 0;
           image_print.at<cv::Vec3b>(yy,xx-1)[1] = 0;

           if (k<num_pixels_sd2project )
           {
               image_print.at<cv::Vec3b>(yy,xx)[2] = 0;
               image_print.at<cv::Vec3b>(yy+1,xx)[2] = 0;
               image_print.at<cv::Vec3b>(yy,xx+1)[2] = 0;
               image_print.at<cv::Vec3b>(yy-1,xx)[2] = 0;
               image_print.at<cv::Vec3b>(yy,xx-1)[2] = 0;
               if (weight.at<float>(k,0) > 0.5)
               {
                   image_print.at<cv::Vec3b>(yy,xx)[2]=190;
                   image_print.at<cv::Vec3b>(yy+1,xx)[2]=190;
                   image_print.at<cv::Vec3b>(yy,xx+1)[2]=190;
                   image_print.at<cv::Vec3b>(yy-1,xx)[2]=190;
                   image_print.at<cv::Vec3b>(yy,xx-1)[2]=190;
               }
           }
           else
           {image_print.at<cv::Vec3b>(yy,xx)[0] = 255;
           image_print.at<cv::Vec3b>(yy+1,xx)[0] = 255;
           image_print.at<cv::Vec3b>(yy,xx+1)[0] = 255;
           image_print.at<cv::Vec3b>(yy-1,xx)[0] = 255;
           image_print.at<cv::Vec3b>(yy,xx-1)[0] = 255;}
        }
    }
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image_print).toImageMsg();
    pub_image->publish(msg);
}

void show_error_geo(cv::Mat &coordinates_cam,  cv::Mat &image_print, cv::Mat &weight_geo)
{
    int imsize_x =image_print.cols-1;
    int imsize_y =image_print.rows-1;

    for (int k =0; k< coordinates_cam.cols;k++)
    {
       if (coordinates_cam.at<float>(1,k) > 1 && coordinates_cam.at<float>(1,k) < imsize_y-1
               && coordinates_cam.at<float>(0,k) > 1 && coordinates_cam.at<float>(0,k) < imsize_x-1)
         {
           //BILINEAR INTERPOLATION
           float x_2 = coordinates_cam.at<float>(0,k);
           float y_2 = coordinates_cam.at<float>(1,k);

           int xx = static_cast<int>(x_2);
           int yy = static_cast<int>(y_2);

           image_print.at<cv::Vec3b>(yy,xx)[2] = 0;
           image_print.at<cv::Vec3b>(yy,xx)[0] = 0;
           image_print.at<cv::Vec3b>(yy,xx)[1] = 0;
           image_print.at<cv::Vec3b>(yy+1,xx)[2] = 0;
           image_print.at<cv::Vec3b>(yy+1,xx)[0] = 0;
           image_print.at<cv::Vec3b>(yy+1,xx)[1] = 0;
           image_print.at<cv::Vec3b>(yy,xx+1)[2] = 0;
           image_print.at<cv::Vec3b>(yy,xx+1)[0] = 0;
           image_print.at<cv::Vec3b>(yy,xx+1)[1] = 0;
           image_print.at<cv::Vec3b>(yy-1,xx)[2] = 0;
           image_print.at<cv::Vec3b>(yy-1,xx)[0] = 0;
           image_print.at<cv::Vec3b>(yy-1,xx)[1] = 0;
           image_print.at<cv::Vec3b>(yy,xx-1)[2] = 0;
           image_print.at<cv::Vec3b>(yy,xx-1)[0] = 0;
           image_print.at<cv::Vec3b>(yy,xx-1)[1] = 0;

           if(weight_geo.at<float>(k,0) > 0.5)
           {
               image_print.at<cv::Vec3b>(yy,xx)[0]=255;
               image_print.at<cv::Vec3b>(yy+1,xx)[0]=255;
               image_print.at<cv::Vec3b>(yy,xx+1)[0]=255;
               image_print.at<cv::Vec3b>(yy-1,xx)[0]=255;
               image_print.at<cv::Vec3b>(yy,xx-1)[0]=255;
           }
        }
    }
}

void resize_points(cv::Mat  &points2,cv::Mat  &points, float reduction,cv::Mat &image_p,
                   int kinect_initialization,float limit_grad)
{
    cv::Mat points_x,points_y,points_z,points_R,points_G,points_B;
    points.colRange(0,1).copyTo(points_x);
    points.colRange(1,2).copyTo(points_y);
    points.colRange(2,3).copyTo(points_z);
    points.colRange(3,4).copyTo(points_R);
    points.colRange(4,5).copyTo(points_G);
    points.colRange(5,6).copyTo(points_B);


    int rows = image_p.rows;
    points_x =  points_x.reshape(0,rows);
    points_y =  points_y.reshape(0,rows);
    points_z =  points_z.reshape(0,rows);
    points_R =  points_R.reshape(0,rows);
    points_G =  points_G.reshape(0,rows);
    points_B =  points_B.reshape(0,rows);

    int reduction_r = reduction;

    cv::resize(points_x,points_x,cv::Size(points_x.cols/reduction_r,points_x.rows/reduction_r),0,0,cv::INTER_LINEAR);
    cv::resize(points_y,points_y,cv::Size(points_y.cols/reduction_r,points_y.rows/reduction_r),0,0,cv::INTER_LINEAR);
    cv::resize(points_z,points_z,cv::Size(points_z.cols/reduction_r,points_z.rows/reduction_r),0,0,cv::INTER_LINEAR);
    cv::resize(points_R,points_R,cv::Size(points_R.cols/reduction_r,points_R.rows/reduction_r),0,0,cv::INTER_LINEAR);
    cv::resize(points_G,points_G,cv::Size(points_G.cols/reduction_r,points_G.rows/reduction_r),0,0,cv::INTER_LINEAR);
    cv::resize(points_B,points_B,cv::Size(points_B.cols/reduction_r,points_B.rows/reduction_r),0,0,cv::INTER_LINEAR);

    cv::Mat points_todos3 = points_R*0.07 + points_G *0.72 + points_B*0.21;
    points_todos3 /=255;

    cv::Mat GX;
    cv::Mat GY;
    GX=gradientX(points_todos3,1);
    GY=gradientY(points_todos3,1);

    cv::Mat G = cv::abs(GX)  + cv::abs(GY);

    float alpha = 10;
    cv::exp(-G*alpha,G);

    cv::Mat sorted_gradients ;
    G.reshape(0,G.rows*G.cols).copyTo(sorted_gradients);


    cv::sort(sorted_gradients,sorted_gradients,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);


    float limit ;
    limit = limit_grad;

     cv::Mat edges;
     cv::Mat gray_image_aux = points_todos3*255;
     gray_image_aux.convertTo(gray_image_aux,CV_8U);

     cv::Canny(gray_image_aux,edges,50,200,5,true);

     edges.convertTo(edges,CV_32FC1);

     cv::Mat G_expanded = G.clone()+1;
     for (int i = 5; i < G.rows-5; i++)
     {
         for (int j = 5; j < G.cols-5;j++)
         {
             if (G.at<float>(i,j) < limit && edges.at<float>(i,j) > 100)
             {
                G_expanded.at<float>(i,j) =   limit-0.1;
             }
         }
     }
     G = G_expanded.clone();

    cv::Mat sorted_depths, points_z2;
    points_z2 = points_z.clone();
    points_z2=points_z2.reshape(0, points_z.rows* points_z.cols);

    cv::sort(cv::abs(points_z2),sorted_depths,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);


    cv::Mat B;

    if (kinect_initialization > 0.5)
    { B = ((G < limit & cv::abs(points_z) < 40));}
    else
    { B = ((G < limit & cv::abs(points_z) < 40));}

    B = B.reshape(0,points_x.cols*points_x.rows);

    points_x =  points_x.reshape(0,points_x.cols*points_x.rows);
    points_y =  points_y.reshape(0,points_y.cols*points_y.rows);
    points_z =  points_z.reshape(0,points_z.cols*points_z.rows);
    points_R =  points_R.reshape(0,points_R.cols*points_R.rows);
    points_G =  points_G.reshape(0,points_G.cols*points_G.rows);
    points_B =  points_B.reshape(0,points_B.cols*points_B.rows);

    cv::Mat points_x1(1,0,CV_32FC1);
    cv::Mat points_y1(1,0,CV_32FC1);
    cv::Mat points_z1(1,0,CV_32FC1);
    cv::Mat points_r1(1,0,CV_32FC1);
    cv::Mat points_g1(1,0,CV_32FC1);
    cv::Mat points_b1(1,0,CV_32FC1);

    B.convertTo(B,CV_32FC1);
    for (int i = 0; i < points_x.rows; i++)
    {
        if (B.at<float>(i,0)>100)
        {
            points_x1.push_back(points_x.at<float>(i,0));
            points_y1.push_back(points_y.at<float>(i,0));
            points_z1.push_back(points_z.at<float>(i,0));
            points_r1.push_back(points_R.at<float>(i,0));
            points_g1.push_back(points_G.at<float>(i,0));
            points_b1.push_back(points_B.at<float>(i,0));
        }
    }

    points_x1.copyTo(points_x);
    points_y1.copyTo(points_y);
    points_z1.copyTo(points_z);
    points_r1.copyTo(points_R);
    points_g1.copyTo(points_G);
    points_b1.copyTo(points_B);

    cv::Mat points_aux(points_x.rows,6,CV_32FC1);
    points_x.copyTo(points_aux.colRange(0,1));
    points_y.copyTo(points_aux.colRange(1,2));
    points_z.copyTo(points_aux.colRange(2,3));
    points_R.copyTo(points_aux.colRange(3,4));
    points_G.copyTo(points_aux.colRange(4,5));
    points_B.copyTo(points_aux.colRange(5,6));

    points_aux.copyTo(points2);

    limit = 20000.0 / points2.rows;
    cv::Mat points2_aux(0, points2.cols ,CV_32FC1);
    for(int k = 0; k < points2.rows;k++)
    {
         if  (((rand() % 1000000 ) / 1000000.0) < limit)
         {
             points2_aux.push_back(points2.row(k));
         }
    }
    points2 = points2_aux.clone();
}


void compute_error( cv::Mat &coordinates_cam, cv::Mat image, cv::Mat &color, cv::Mat &error_vector,
                    float variance, cv::Mat &error_vector_sqrt,cv::Mat &error_check,cv::Mat &weight)
{
    int imsize_x =image.cols-1;
    int imsize_y =image.rows-1;

    float error_to_add = 0;
    int cont = 0;

    for (int k =0; k< coordinates_cam.cols;k++)
    {
       if (coordinates_cam.at<float>(1,k) >1&& coordinates_cam.at<float>(1,k) < imsize_y-1 &&
               coordinates_cam.at<float>(0,k) > 1&& coordinates_cam.at<float>(0,k) < imsize_x-1)
         {
           cont++;
            //BILINEAR INTERPOLATION
           float x_2 = coordinates_cam.at<float>(0,k);
           float x_1 = static_cast<int>(x_2);
           float x_3 = x_1 +1;
           float y_2 = coordinates_cam.at<float>(1,k);
           float y_1 = static_cast<int>(y_2);
           float y_3 = y_1 +1;

           float c1 = image.at<float>(y_1,x_1);
           float c2 = image.at<float>(y_3,x_1);

           float r1 = c1*(y_3-y_2) + c2*(y_2-y_1);

           c1 = image.at<float>(y_1,x_3);
           c2 = image.at<float>(y_3,x_3);

           float r2 = c1*(y_3-y_2) + c2*(y_2-y_1);

           float r = r1*(x_3-x_2) +  r2*(x_2-x_1);


           float error_to_add  =   (r -  color.at<float>(k,0));

           error_vector_sqrt.at<float>(k,0) = error_to_add;
           error_check.push_back(error_to_add);
        }
       else
       {
            error_to_add =  1;
            error_vector_sqrt.at<float>(k,0) = error_to_add;
       }
    }

    cv::pow(error_vector_sqrt, 2,error_vector);
    cv::pow(error_check, 2,error_check);

}

void compute_error_ic( cv::Mat &coordinates_cam,cv::Mat &coordinates_cam_p, cv::Mat &image,cv::Mat &image_p, \
                       cv::Mat &error_vector,float &variance, cv::Mat &error_vector_sqrt,float &error,\
                       cv::Mat &weight, cv::Mat &color_p, float &gain, float &brightness,float &overlap,\
                       cv::Mat &error_vector_check)
{
    int imsize_x =image.cols-1;
    int imsize_y =image.rows-1;

    overlap = 0;

    for (int k = 0; k < coordinates_cam.cols;k++)
    {

        if (coordinates_cam.at<float>(1,k) > 1 && coordinates_cam.at<float>(1,k) < imsize_y-1 && coordinates_cam.at<float>(0,k) > 1 && coordinates_cam.at<float>(0,k) < imsize_x-1
            && coordinates_cam_p.at<float>(1,k) >1 && coordinates_cam_p.at<float>(1,k) < imsize_y-1 && coordinates_cam_p.at<float>(0,k) > 1 && coordinates_cam_p.at<float>(0,k) < imsize_x-1)
        {
            //BILINEAR INTERPOLATION
            float x_2 = coordinates_cam.at<float>(0,k);
            float y_2 = coordinates_cam.at<float>(1,k);
            float r;
            bilinear_interpolation(image,  x_2,y_2,r);


            //BILINEAR INTERPOLATION
            x_2 = coordinates_cam_p.at<float>(0,k);
            y_2 = coordinates_cam_p.at<float>(1,k);
            float r_p;
            bilinear_interpolation(image_p,  x_2,y_2,r_p);


             color_p.at<float>(k,0) = r_p;
             error_vector_sqrt.at<float>(k,0) =(r*gain + brightness -  r_p);

             if ( fabs(error_vector_sqrt.at<float>(k,0) ) < 0.2)
             {
                overlap += 1.0/coordinates_cam.cols;
             }
       }
       else
       {
            if ( coordinates_cam_p.at<float>(1,k) > 1
                 && coordinates_cam_p.at<float>(1,k) < imsize_y-1
                 && coordinates_cam_p.at<float>(0,k) >1
                 && coordinates_cam_p.at<float>(0,k) < imsize_x-1)
            {
                //BILINEAR INTERPOLATION
                float x_2 = coordinates_cam_p.at<float>(0,k);
                float y_2 = coordinates_cam_p.at<float>(1,k);
                float r_p;
                bilinear_interpolation(image_p,  x_2,y_2,r_p);

                color_p.at<float>(k,0) = r_p;
            }
            else
            {
                color_p.at<float>(k,0) = sqrt(variance);
            }

            error_vector_sqrt.at<float>(k,0) = sqrt(variance);
       }
    }


    cv::pow(error_vector_sqrt, 2,error_vector);
    error_vector_check = error_vector.mul(weight);
    error = cv::sum(error_vector_check)[0];
}

void calculate_jacobian_geo(cv::Mat &frame_depth, cv::Mat &coordinates_cam,
                         cv::Mat &transformed_points_cam,float fx,float fy,
                         cv::Mat&jacobian_geo,cv::Mat &constant_error,
                            cv::Mat &max_error_vector,bool use_inv_depth)
{
    cv::Mat frame_depth_copy = frame_depth.clone();


    cv::Mat GX,GY;
    GX =(gradientX(frame_depth_copy,1));
    GY =(gradientY(frame_depth_copy,1));

    cv::Mat GX_id,GY_id;
    if(use_inv_depth)
    {

        for(int ii=0;ii<frame_depth_copy.rows;ii++ )
        {
            for(int jj=0;jj<frame_depth_copy.cols;jj++ )
            {
                if(fabs(frame_depth_copy.at<float>(ii,jj))>0)
                {
                    frame_depth_copy.at<float>(ii,jj) = 1/frame_depth_copy.at<float>(ii,jj);
                }
            }
        }


        GX_id =(gradientX(frame_depth_copy,1));
        GY_id =(gradientY(frame_depth_copy,1));

     }


    for (int ii = 0; ii < transformed_points_cam.cols; ii++)
    {
       if (constant_error.at<float>(ii,0) == 0 )
       {
               // BILINEAR
               float GX_value,GY_value;
               bilinear_interpolation(GX, coordinates_cam.at<float>(0,ii), coordinates_cam.at<float>(1,ii),GX_value);
               bilinear_interpolation(GY, coordinates_cam.at<float>(0,ii), coordinates_cam.at<float>(1,ii),GY_value);
               GX_value = -GX_value;
               GY_value = -GY_value;

               float GX_id_value,GY_id_value;
               if(use_inv_depth){
                   bilinear_interpolation(GX_id, coordinates_cam.at<float>(0,ii), coordinates_cam.at<float>(1,ii),GX_id_value);
                   bilinear_interpolation(GY_id, coordinates_cam.at<float>(0,ii), coordinates_cam.at<float>(1,ii),GY_id_value);
                   GX_id_value = -GX_id_value;
                   GY_id_value = -GY_id_value;
               }
               if(fabs(GX_value)  < 0.03  && fabs(GY_value) < 0.03)
               {
                   if(use_inv_depth)
                   {
                       float jac4 = GX_id_value*fx/transformed_points_cam.at<float>(2,ii);
                       float jac5 = -GY_id_value*fy/transformed_points_cam.at<float>(2,ii);
                       float jac6 = (GY_id_value*fy*transformed_points_cam.at<float>(1,ii)
                                      -GX_id_value*fx*transformed_points_cam.at<float>(0,ii))/
                                (transformed_points_cam.at<float>(2,ii)*transformed_points_cam.at<float>(2,ii));

                       jacobian_geo.at<float>(ii,3) =  jac4;
                       jacobian_geo.at<float>(ii,4) =  jac5;
                       jacobian_geo.at<float>(ii,5) =  jac6;
                       jacobian_geo.at<float>(ii,0) =  jac6*transformed_points_cam.at<float>(1,ii) + GY_id_value*fy ;
                       jacobian_geo.at<float>(ii,1) =  -jac6*transformed_points_cam.at<float>(0,ii) + GX_id_value*fx ;
                       jacobian_geo.at<float>(ii,2) =  -jac4*transformed_points_cam.at<float>(1,ii) +
                                jac5*transformed_points_cam.at<float>(0,ii);

                       jacobian_geo.at<float>(ii,0) +=  -1/(transformed_points_cam.at<float>(2,ii)*transformed_points_cam.at<float>(2,ii))*
                               transformed_points_cam.at<float>(1,ii);
                       jacobian_geo.at<float>(ii,1) +=  transformed_points_cam.at<float>(0,ii)*
                               1/(transformed_points_cam.at<float>(2,ii)*transformed_points_cam.at<float>(2,ii));
                       jacobian_geo.at<float>(ii,5) +=  -1/(transformed_points_cam.at<float>(2,ii)*transformed_points_cam.at<float>(2,ii));
                   }else{
                       float jac4 = GX_value*fx/transformed_points_cam.at<float>(2,ii);
                       float jac5 = -GY_value*fy/transformed_points_cam.at<float>(2,ii);
                       float jac6 = (GY_value*fy*transformed_points_cam.at<float>(1,ii)
                                      -GX_value*fx*transformed_points_cam.at<float>(0,ii))/
                                (transformed_points_cam.at<float>(2,ii)*transformed_points_cam.at<float>(2,ii));

                       jacobian_geo.at<float>(ii,3) =  jac4;
                       jacobian_geo.at<float>(ii,4) =  jac5;
                       jacobian_geo.at<float>(ii,5) =  jac6;
                       jacobian_geo.at<float>(ii,0) =  jac6*transformed_points_cam.at<float>(1,ii)  + GY_value*fy ;
                       jacobian_geo.at<float>(ii,1) =  -jac6*transformed_points_cam.at<float>(0,ii) + GX_value*fx ;
                       jacobian_geo.at<float>(ii,2) =  -jac4*transformed_points_cam.at<float>(1,ii) +
                                                        jac5*transformed_points_cam.at<float>(0,ii);

                       jacobian_geo.at<float>(ii,0) +=  transformed_points_cam.at<float>(1,ii);
                       jacobian_geo.at<float>(ii,1) +=  -transformed_points_cam.at<float>(0,ii);
                       jacobian_geo.at<float>(ii,5) +=  1;
                   }
               }
       }
    }
}

void calculate_jacobian_photo(cv::Mat &frame, cv::Mat &coordinates_cam_p2,
                         cv::Mat &transformed_points,float fx,float fy,
                         cv::Mat&jacobian_photo,int border_width, int imsize_y,int imsize_x)
{
    cv::Mat GX,GY;

    for(int i =0; i < 2; i ++){
        if (i==0){
           GX =(gradientX(frame,1));
        }else{
           GY =(gradientY(frame,1));
        }
    }

    for (int ii = 0; ii < transformed_points.cols; ii++)
    {
        if (coordinates_cam_p2.at<float>(1,ii) > border_width && coordinates_cam_p2.at<float>(1,ii) < imsize_y-border_width &&
                 coordinates_cam_p2.at<float>(0,ii) > border_width && coordinates_cam_p2.at<float>(0,ii) < imsize_x-border_width)
        {
            float GX_value,GY_value;
            bilinear_interpolation(GX, coordinates_cam_p2.at<float>(0,ii), coordinates_cam_p2.at<float>(1,ii),GX_value);
            bilinear_interpolation(GY, coordinates_cam_p2.at<float>(0,ii), coordinates_cam_p2.at<float>(1,ii),GY_value);

            float jac4 = GX_value*fx/transformed_points.at<float>(2,ii);
            float jac5 = -GY_value*fy/transformed_points.at<float>(2,ii);
            float jac6 = (GY_value*fy*transformed_points.at<float>(1,ii)
                          -GX_value*fx*transformed_points.at<float>(0,ii))/
                    (transformed_points.at<float>(2,ii)*transformed_points.at<float>(2,ii));

            jacobian_photo.at<float>(ii,3) =  jac4;
            jacobian_photo.at<float>(ii,4) =  jac5;
            jacobian_photo.at<float>(ii,5) =  jac6;
            jacobian_photo.at<float>(ii,0) =  jac6*transformed_points.at<float>(1,ii)  + GY_value*fy ;
            jacobian_photo.at<float>(ii,1) =  -jac6*transformed_points.at<float>(0,ii) + GX_value*fx ;
            jacobian_photo.at<float>(ii,2) =  -jac4*transformed_points.at<float>(1,ii)+ jac5*transformed_points.at<float>(0,ii);
        }
        else
        {
            for(int z=0;z<6;z++)
            jacobian_photo.at<float>(ii,z) = 0;
        }
    }
}

void gauss_newton_ic(SemiDenseTracking *semidense_tracker,cv::Mat &coordinates_cam_to_print,cv::Mat &R,cv::Mat &t,\
                     cv::Mat &R_p,cv::Mat &t_p,float fx,float fy,cv::Mat &points,cv::Mat &img,cv::Mat &img_p, \
                     float &error_opt,cv::Mat &color,float variance,cv::Mat &points3D,\
                     cv::Mat &error_vector_photo_opt, int initial_iteration,cv::Mat &jacobian, cv::Mat &init ,\
                     cv::Mat &weight,cv::Mat &coordinates_cam_p,cv::Mat &coordinates_cam,\
                     cv::Mat &error_vector_inicial, cv::Mat &error_vector_photo_sqrt, float &has_decreased,cv::Mat &color_p,\
                     int &processed_frames,cv::Mat &jacobian1k, float &overlap,float cx,float cy,int pyramid_level, bool first_frame)
{
    cv::Mat R_opt;
    cv::Mat t_opt;
    R.copyTo(R_opt);
    t.copyTo(t_opt);

    cv::Mat error_vector(points.rows,1,CV_32FC1);
    cv::Mat error_vector_sqrt(points.rows,1,CV_32FC1);

    int imsize_y =img.rows-1;
    int imsize_x =img.cols-1;
    int border_width = 2;
    //////////////////////////////////////// INVERSE5
    if (initial_iteration < 0.5)
    {
               cv::Mat transformed_points_cam,transformed_points_cam_p;
               transform_points_return_3Dpoints (coordinates_cam_p, R_p,t_p,fx,fy,cx,cy,points3D,transformed_points_cam_p);
               transform_points_return_3Dpoints (coordinates_cam  , R  ,t  ,fx,fy,cx,cy,points3D,transformed_points_cam  );

               if (semidense_tracker->do_gain_brightness_estimation)
               { img = (img-semidense_tracker->brightness)/semidense_tracker->gain;}

               cv::Mat error_vector_photo_weighted;
               compute_error_ic( coordinates_cam,coordinates_cam_p,img,img_p,error_vector_inicial,variance,error_vector_photo_sqrt,
                                 error_opt,weight,color_p,semidense_tracker->gain,semidense_tracker->brightness,
                                 overlap,error_vector_photo_weighted);
               error_vector_photo_opt = error_vector_photo_sqrt.mul(error_vector_photo_sqrt);



               coordinates_cam_to_print = coordinates_cam.clone();

               if (semidense_tracker->use_kinect == 1 && semidense_tracker->use_depth_tracking ==  1)
               {
                   bool only_depth = false;
                   bool use_inv_depth = true;


                   cv::Mat points3D_geo(0,3, CV_32FC1);
                   cv::Mat weight_geo;

                   cv::Mat transformed_points_keyframe = transformed_points_cam_p.clone();
                   cv::Mat coordinates_cam_keyframe = coordinates_cam_p.clone();


                   if (first_frame)
                   {
                        cv::Mat point_aux(1,3, CV_32FC1);

                       int step_size = 1;if (pyramid_level == 1) step_size = 4;
                       if (pyramid_level == 2) step_size = 4;
                       if (pyramid_level == 3) step_size = 4;
                       for (int i = 0; i < imsize_y; i = i+step_size)
                       {
                           for (int j = 0; j < imsize_x; j = j+step_size)
                           {
                               if (fabs(semidense_tracker->keyframe_depth[pyramid_level].at<float>(i,j)) > 0
                                       && fabs(semidense_tracker->keyframe_depth[pyramid_level].at<float>(i,j)) < 30.0  )
                               {
                                   point_aux.at<float>(0,0) = ((cx-j)/fx)*(-semidense_tracker->keyframe_depth[pyramid_level].at<float>(i,j));
                                   point_aux.at<float>(0,1) = ((i-cy)/fy)*(-semidense_tracker->keyframe_depth[pyramid_level].at<float>(i,j));
                                   point_aux.at<float>(0,2) = (-semidense_tracker->keyframe_depth[pyramid_level].at<float>(i,j));
                                   points3D_geo.push_back(point_aux);
                               }
                           }
                       }

                       cv::Mat t_p_repeat =  cv::repeat(t_p,1,points3D_geo.rows);
                       points3D_geo = R_p.t()*(points3D_geo.t()  - t_p_repeat);


                       semidense_tracker -> points3D_geo[pyramid_level] = points3D_geo.clone();
                       semidense_tracker -> weight_geo[pyramid_level] = cv::Mat::zeros(points3D_geo.cols,1,CV_32FC1)+1;
                       weight_geo = semidense_tracker -> weight_geo[pyramid_level].clone();
                    }else{
                       points3D_geo = semidense_tracker -> points3D_geo[pyramid_level].clone();
                       weight_geo = semidense_tracker -> weight_geo[pyramid_level].clone();
                    }

                    cv::Mat transformed_points_cam_geo,transformed_points_cam_geo_p,coordinates_cam_geo,coordinates_cam_geo_p;
                    transform_points_return_3Dpoints (coordinates_cam_geo_p, R_p,t_p,fx,fy,cx,cy,
                                                      points3D_geo,transformed_points_cam_geo_p);
                    transform_points_return_3Dpoints (coordinates_cam_geo  , R  ,t  ,fx,fy,cx,cy,
                                                      points3D_geo,transformed_points_cam_geo);

                    cv::Mat error_geo_vector = cv::Mat::zeros(points3D_geo.cols,1,CV_32FC1);
                    cv::Mat error_geo_vector_sqrt = cv::Mat::zeros(points3D_geo.cols,1,CV_32FC1);
                    cv::Mat constant_error   = cv::Mat::zeros(points3D_geo.cols,1,CV_32FC1) + 1;

                    cv::Mat weight_depth2camera = weight_geo.clone();
                    cv::Mat max_error_vector = weight_geo.clone();

                    for(int idx_weight = 0; idx_weight < weight_geo.rows; idx_weight++ )
                    {
                        if(use_inv_depth)
                        {
                           weight_depth2camera.at<float>(idx_weight,0) = 1;
                           max_error_vector.at<float>(idx_weight,0) = 0.05 * fabs(1/transformed_points_cam_geo_p.at<float>(2,idx_weight));
                        }else{
                            weight_depth2camera.at<float>(idx_weight,0) =  fabs(1/transformed_points_cam_geo_p.at<float>(2,idx_weight))*
                                    fabs(1/transformed_points_cam_geo_p.at<float>(2,idx_weight))*
                                   fabs(1/transformed_points_cam_geo_p.at<float>(2,idx_weight))*
                                                                    fabs(1/transformed_points_cam_geo_p.at<float>(2,idx_weight));
                            max_error_vector.at<float>(idx_weight,0) = 0.05 * fabs(transformed_points_cam_geo_p.at<float>(2,idx_weight));
                        }
                    }
                    weight_geo = weight_geo.mul(weight_depth2camera);

                    cv::Mat R_ = R.clone();
                    cv::Mat t_ = t.clone();


                    constant_error = cv::Mat::zeros(points3D_geo.cols,1,CV_32FC1) + 1;
                    compute_geo_error(coordinates_cam_geo,semidense_tracker->frame_depth[pyramid_level],
                                      transformed_points_cam_geo,error_geo_vector,error_geo_vector_sqrt,
                                      constant_error,max_error_vector,use_inv_depth);
                    semidense_tracker->coordinates_depth_point_cloud = coordinates_cam_geo.clone();


                    weight_geo = 0.3 * weight_geo * cv::sum(error_vector_photo_weighted)[0] / cv::sum(error_geo_vector.mul(weight_geo))[0];

                    cv::Mat error_geo_weighted = error_geo_vector.mul(weight_geo);


                    bool init_it = true;
                    int iterations = 0;



                    cv::Mat error_vector_total = error_vector_photo_weighted.clone();
                    error_vector_total.push_back(error_geo_weighted);

                    if(only_depth)
                    {error_vector_total = error_geo_weighted.clone();}

                    float error_total_min = cv::mean(error_vector_total)[0];
                    float error_total_curr = 0.999*error_total_min;

                    ///PHOTO and GEO ERROR AND JACOBIANS
                    cv::Mat lambda = cv::Mat::eye(6,6,CV_32FC1)*2000;

                    cv::Mat jacobian_total1 = cv::Mat::zeros(error_geo_vector.rows+error_vector_photo_weighted.rows,6,CV_32FC1);
                    cv::Mat jacobian_photo_weight = jacobian_total1(cv::Rect(0,0,6,error_vector_photo_weighted.rows));
                    cv::Mat jacobian_geo_weight = jacobian_total1(cv::Rect(0,error_vector_photo_weighted.rows,6,error_geo_vector.rows));

                    cv::Mat jacobian_total_no_weight1 = cv::Mat::zeros(error_geo_vector.rows+error_vector_photo_weighted.rows,6,CV_32FC1);
                    cv::Mat jacobian_photo = jacobian_total_no_weight1(cv::Rect(0,0,6,error_vector_photo_weighted.rows));
                    cv::Mat jacobian_geo = jacobian_total_no_weight1(cv::Rect(0,error_vector_photo_weighted.rows,6,error_geo_vector.rows));
                    ///PHOTO and GEO ERROR AND JACOBIANS


                    /*cv::Mat jacobian_geo_removed= cv::Mat::zeros(error_geo_vector.rows,7,CV_32FC1);
                    cv::Mat jacobian_photo_removed = cv::Mat::zeros(error_vector_photo.rows,7,CV_32FC1);
                    int number_geo_points_removed = 0;
                    int number_photo_points_removed = 0;*/


                    while (  (error_total_curr < error_total_min  &&
                           ((error_total_min-error_total_curr)/(error_total_min + 0.00001) > 0.001) || init_it ))
                    {
                        ++iterations;

                        if(error_total_curr < error_total_min)
                        error_total_min = error_total_curr;


                        cv::Mat w(3,1,CV_32FC1);
                        cv::Mat v(3,1,CV_32FC1);
                        cv::Mat S,U,V;



                        if (imsize_y >  60)
                        { lambda *= 0;}


                        if (init_it && processed_frames < 0.5)
                        {
                                #pragma omp parallel num_threads(2)
                                {
                                   switch(omp_get_thread_num())
                                   {
                                        case 0:
                                        {
                                                calculate_jacobian_geo(semidense_tracker->keyframe_depth[pyramid_level],coordinates_cam_geo_p,
                                                                       transformed_points_cam_geo_p,fx,fy, jacobian_geo,constant_error,max_error_vector,use_inv_depth);

                                                jacobian_geo = -jacobian_geo;
                                                semidense_tracker->jacobian_geo_no_weight[pyramid_level] = jacobian_geo;

                                                for (int ii = 0; ii < 6;ii++)
                                                {
                                                   jacobian_geo_weight.rowRange(0,jacobian_geo_weight.rows).colRange(ii,ii+1) =
                                                           weight_geo.mul(jacobian_geo.rowRange(0,jacobian_geo_weight.rows).colRange(ii,ii+1)) ;
                                                }
                                        };break;

                                        case 1:
                                        {
                                                calculate_jacobian_photo(img_p, coordinates_cam_keyframe,
                                                                         transformed_points_keyframe, fx, fy,
                                                                         jacobian_photo, border_width,  imsize_y, imsize_x);

                                                jacobian_photo = -jacobian_photo;

                                                semidense_tracker->jacobian_photo_no_weight[pyramid_level] = jacobian_photo;
                                                for (int ii = 0; ii < 6;ii++)
                                                {
                                                    jacobian_photo_weight.rowRange(0,jacobian_photo_weight.rows).colRange(ii,ii+1) =
                                                            weight.mul(jacobian_photo.rowRange(0,jacobian_photo.rows).colRange(ii,ii+1) );
                                                }
                                        };break;
                                   }
                                }
                        }else{
                            if(init_it)
                            {
                                    semidense_tracker->jacobian_geo_no_weight[pyramid_level].copyTo(jacobian_geo);
                                    //jacobian_geo  =  semidense_tracker->jacobian_geo_no_weight[pyramid_level];

                                    for(int ii = 0; ii < coordinates_cam_geo.cols; ii++)
                                    {
                                        if(constant_error.at<float>(ii,0) == 1)
                                        {
                                            for(int z = 0;z<6;z++)
                                            {
                                                //jacobian_geo_removed.at<float>(number_geo_points_removed,z) = jacobian_geo.at<float>(ii,z);
                                                jacobian_geo.at<float>(ii,z) = 0;
                                            }
                                            //jacobian_geo_removed.at<float>(number_geo_points_removed,6) = ii;
                                            //number_geo_points_removed++;
                                        }
                                    }


                                    for (int ii = 0; ii < 6;ii++)
                                    {
                                       jacobian_geo_weight.rowRange(0,jacobian_geo_weight.rows).colRange(ii,ii+1) =
                                               weight_geo.mul(jacobian_geo.rowRange(0,jacobian_geo_weight.rows).colRange(ii,ii+1)) ;
                                    }






                                    semidense_tracker->jacobian_photo_no_weight[pyramid_level].copyTo(jacobian_photo);
                                    for(int ii = 0; ii < coordinates_cam.cols; ii++)
                                    {
                                        if( coordinates_cam.at<float>(0,ii) < 1 || coordinates_cam.at<float>(1,ii) < 1 ||
                                                coordinates_cam.at<float>(0,ii)  > imsize_x-1 || coordinates_cam.at<float>(1,ii) > imsize_y-1 )
                                        {
                                            for(int z = 0;z<6;z++)
                                            {
                                              jacobian_photo.at<float>(ii,z) = 0;
                                            }
                                        }
                                    }

                                    for (int ii = 0; ii < 6;ii++)
                                    {
                                        jacobian_photo_weight.rowRange(0,jacobian_photo_weight.rows).colRange(ii,ii+1) =
                                                weight.mul(jacobian_photo.rowRange(0,jacobian_photo.rows).colRange(ii,ii+1) );
                                    }
                            }
                        }

                         ///PHOTO ERROR AND JACOBIAN
                         ///OPTIMIZATION

                         cv::Mat error_total_sqrt = -error_vector_photo_sqrt;
                         error_total_sqrt.push_back(error_geo_vector_sqrt);
                         cv::Mat weight_total = weight.clone();
                         weight_total.push_back(weight_geo);



                         if(only_depth)
                         {
                             jacobian_total1 = jacobian_geo_weight.clone();
                             jacobian_total_no_weight1 = jacobian_geo.clone();
                             error_total_sqrt = error_geo_vector.clone();
                             weight_total = weight_geo.clone();
                         }



                         cv::Mat tetha;

                         if ( (init_it && processed_frames < 0.5) || init_it)
                         {
                             semidense_tracker->hessian_vision_geo[pyramid_level] = -(jacobian_total_no_weight1.t()*jacobian_total1 +
                                                                                      lambda).inv(cv::DECOMP_SVD)*jacobian_total_no_weight1.t();
                            /* /// restore jacobians
                             for(int i = 0; i < number_geo_points_removed;i++){
                                 int ii = jacobian_geo_removed.at<float>(i,6);
                                 for(int z = 0;z<6;z++)
                                 {
                                     jacobian_geo.at<float>(ii,z)  = jacobian_geo_removed.at<float>(i,z);
                                 }
                             }
                             number_geo_points_removed = 0;
                             /// restore jacobians*/
                         }
                         tetha = semidense_tracker->hessian_vision_geo[pyramid_level] * error_total_sqrt.mul(weight_total);

                         init_it = false;

                         w.at<float>(0,0) = -tetha.at<float>(0,0);
                         w.at<float>(1,0) = -tetha.at<float>(1,0);
                         w.at<float>(2,0) = -tetha.at<float>(2,0);
                         v.at<float>(0,0) = -tetha.at<float>(3,0);
                         v.at<float>(1,0) = -tetha.at<float>(4,0);
                         v.at<float>(2,0) = -tetha.at<float>(5,0);

                         cv::Mat R_upd,t_upd;
                         exp_SE3 (R_upd,t_upd,w,v);
                         cv::SVD::compute(R_upd,S,U,V,cv::SVD::FULL_UV);
                         R_upd = U*V;

                         /// inverse geometric
                         R_ = R_upd*R;
                         t_ = R_upd*t+t_upd;


                        float gain_previous  = semidense_tracker->gain;
                        float brightness_previous  = semidense_tracker->brightness;



                        #pragma omp parallel num_threads(2)
                        {
                           switch(omp_get_thread_num())
                           {
                                case 0:
                                {
                                    transform_points_return_3Dpoints(coordinates_cam, R_,t_,fx,fy,cx,cy,points3D,transformed_points_cam  );
                                    if(!only_depth)
                                    {
                                       float error_photo_curr;
                                       compute_error_ic_ni( coordinates_cam,coordinates_cam_p,img,error_vector_inicial,variance,error_vector_photo_sqrt,
                                                            error_photo_curr,weight,color_p,overlap,semidense_tracker->gain,semidense_tracker->brightness,error_vector_photo_weighted);
                                    }
                                 };break;
                                 case 1:
                                 {
                                    transform_points_return_3Dpoints(coordinates_cam_geo,R_,t_,fx,fy,cx,cy,
                                                                      points3D_geo,transformed_points_cam_geo);
                                    constant_error = cv::Mat::zeros(points3D_geo.cols,1,CV_32FC1) + 1;

                                    compute_geo_error(coordinates_cam_geo,semidense_tracker->frame_depth[pyramid_level],
                                                     transformed_points_cam_geo,error_geo_vector,error_geo_vector_sqrt,
                                                     constant_error,max_error_vector,use_inv_depth);
                                  }
                           }
                        }



                        error_vector_total = error_vector_photo_weighted.clone();
                        cv::Mat error_geo_weighted = error_geo_vector.mul(weight_geo);
                        error_vector_total.push_back(error_geo_weighted);

                        if(only_depth)
                        error_vector_total = error_geo_weighted.clone();

                        error_total_curr = cv::mean(error_vector_total)[0];
                        ///OPTIMIZATION

                         if(error_total_curr < error_total_min  )
                         {
                             R = R_.clone();
                             t = t_.clone();

                             error_vector_photo_opt = error_vector_photo_sqrt.mul(error_vector_photo_sqrt);

                             semidense_tracker->coordinates_depth_point_cloud = coordinates_cam_geo.clone();
                             float count_close_points = weight_geo.rows;
                             count_close_points = 0;

                             /*if(pyramid_level > 1)
                             {
                                   float overlap_aux =  1 - cv::sum(constant_error)[0] / constant_error.rows;
                                   if (overlap_aux*1.00 < overlap) overlap = overlap_aux;
                             }*/
                         }
                         else
                         {
                             semidense_tracker->gain = gain_previous;
                             semidense_tracker->brightness = brightness_previous;
                         }
                     } // while
                     return;
            } //  if (semidense_tracker->use_kinect == 1)
           }
           else{
               if (semidense_tracker->use_kinect == 1 && semidense_tracker->use_depth_tracking == 1)return;
    }

    cv::Mat w(3,1,CV_32FC1);
    cv::Mat v(3,1,CV_32FC1);
    cv::Mat R1,t1,R2,t2;

    if (initial_iteration < 0.5)
    {
        if (processed_frames < 0.5)
        {
                cv::Mat jacobian_photo = cv::Mat::zeros(jacobian.rows,6,CV_32FC1);

                cv::Mat coordinates_cam_p2;
                cv::Mat transformed_points_p2;
                transform_points_return_3Dpoints(coordinates_cam_p2, R_p,t_p,fx,fy,cx,cy,points3D,transformed_points_p2);

                calculate_jacobian_photo(img_p, coordinates_cam_p2,
                                         transformed_points_p2, fx, fy,
                                         jacobian_photo, border_width,  imsize_y, imsize_x);


                semidense_tracker->jacobian_photo_no_weight[pyramid_level] = jacobian_photo.clone();
                for (int ii = 0; ii < 6;ii++)
                {
                    jacobian.rowRange(0,jacobian.rows).colRange(ii,ii+1) =
                            weight.mul(jacobian_photo.rowRange(0,jacobian_photo.rows).colRange(ii,ii+1) );
                }
                jacobian1k = jacobian_photo.clone();

                cv::Mat J_transpose_J = semidense_tracker->jacobian_photo_no_weight[pyramid_level].t()*jacobian;
                cv::Mat lambda = cv::Mat::eye(6,6,CV_32FC1)*20000;

                if (imsize_y >  60)
                { lambda *= 0;}
                init = (J_transpose_J + lambda).inv(cv::DECOMP_SVD)*jacobian.t();
        }
        else
        {       int imsize_x =img.cols-1;
                int imsize_y =img.rows-1;

                    for (int k = 0; k<coordinates_cam.cols; k++)
                    {
                        if (coordinates_cam.at<float>(1,k) > border_width && coordinates_cam.at<float>(1,k) < imsize_y-border_width &&
                                coordinates_cam.at<float>(0,k) > border_width && coordinates_cam.at<float>(0,k) < imsize_x-border_width)
                        {
                            for (int z = 0; z<6; z++)
                            {
                                jacobian.at<float>(k,z)=jacobian1k.at<float>(k,z);
                            }
                        }
                        else
                        {
                            for (int z = 0; z < 6; z++)
                            {
                                jacobian.at<float>(k,z) = 0;
                                weight.at<float>(k,0) = 0;
                            }
                        }
                    }

                    semidense_tracker->jacobian_photo_no_weight[pyramid_level] = jacobian.clone();
                    for (int z = 0;  z < 6; z++)
                    {
                       jacobian.rowRange(0,jacobian.rows).colRange(z,z+1) = weight.mul(jacobian.rowRange(0,jacobian.rows).colRange(z,z+1));
                    }


                    cv::Mat J_transpose_J = semidense_tracker->jacobian_photo_no_weight[pyramid_level].t()*jacobian;
                    //cv::Mat lambda = cv::Mat::eye(6,6,CV_32FC1)*cv::trace(J_transpose_J)[0]/6;
                    cv::Mat lambda = cv::Mat::eye(6,6,CV_32FC1)*20000;
                    //for (int ii =0;ii<6;ii++)
                    //{lambda.at<float>(ii,ii) = 5*J_transpose_J.at<float>(ii,ii) ;}

                    if (imsize_y >   60)
                    {
                        lambda *= 0;
                    }

                    init = (J_transpose_J + lambda).inv(cv::DECOMP_SVD)*jacobian.t();
        }
    }
    cv::Mat init2;
    init2 = init*(error_vector_photo_sqrt.mul(weight)) ;

    w.at<float>(0,0) = -init2.at<float>(0,0);
    w.at<float>(1,0) = -init2.at<float>(1,0);
    w.at<float>(2,0) = -init2.at<float>(2,0);
    v.at<float>(0,0) = -init2.at<float>(3,0);
    v.at<float>(1,0) = -init2.at<float>(4,0);
    v.at<float>(2,0) = -init2.at<float>(5,0);



    exp_SE3 (R1,t1,w,v);

    cv::Mat S,U,V;
    cv::SVD::compute(R1,S,U,V,cv::SVD::FULL_UV);
    R1 = U*V;


    t2 = R*R_p.t()*(R1.t()*(t_p-t1)-t_p)+t;
    R2 = R*R_p.t()*R1.t()*R_p;

   // R2 = R1*R;
   // t2 = R1*t+t1;

    float error_check;
    float gain_previous  = semidense_tracker->gain;
    float brightness_previous  = semidense_tracker->brightness;

    cv::Mat transformed_points,error_vector_photo;
    transform_points_return_3Dpoints(coordinates_cam, R2,t2,fx,fy,cx,cy,points3D,transformed_points);
    compute_error_ic_ni( coordinates_cam,coordinates_cam_p,img,error_vector,variance,error_vector_sqrt,
                         error_check,weight,color_p,overlap,semidense_tracker->gain,semidense_tracker->brightness,error_vector_photo);


    cv::SVD::compute(R2,S,U,V,cv::SVD::FULL_UV);
    R2 = U*V;

    if (error_check < error_opt)
    {
        error_opt = error_check;
        R2.copyTo(R_opt);
        t2.copyTo(t_opt);
        error_vector.copyTo(error_vector_photo_opt);
        error_vector.copyTo(error_vector_inicial);
        error_vector_sqrt.copyTo(error_vector_photo_sqrt);
        has_decreased = 1;
        coordinates_cam_to_print = coordinates_cam.clone();
    }
    else
    {
        semidense_tracker->gain = gain_previous;
        semidense_tracker->brightness = brightness_previous;
    }
    R_opt.copyTo(R);
    t_opt.copyTo(t);
}


void  gauss_estimation(SemiDenseTracking *semidense_tracker,cv::Mat &coordinates_cam_to_print,cv::Mat &R2,cv::Mat &t2,cv::Mat &R_p,
                       cv::Mat &t_p,float &fx,float &fy,float &cx,float &cy,cv::Mat &points, \
                       cv::Mat &img,cv::Mat &img_p,float &error_p,cv::Mat &color,int &iter, float variance,cv::Mat &points3D,\
                       cv::Mat &error_vector_photo_opt,cv::Mat &weight, int &processed_frames,cv::Mat &jacobian1k, float &overlap_tracking,
                       float &tracking_th, int iter_th,int pyramid_level,cv::Mat &coordinates_cam_p,bool first_frame)
{
    int initial_iteration = 0;
    cv::Mat jacobian(points.rows,6,CV_32FC1);

    cv::Mat init;

    cv::Mat coordinates_cam;
    cv::Mat error_vector_inicial(points.rows,1,CV_32FC1);
    cv::Mat error_vector_photo_sqrt(points.rows,1,CV_32FC1);
    cv::Mat color_p(points.rows,1,CV_32FC1);
    float has_decreased = 0;

    float overlap;
    gauss_newton_ic(semidense_tracker,coordinates_cam_to_print,R2,t2,R_p,t_p,fx,fy,points,\
                    img,img_p,error_p,color,variance,points3D,error_vector_photo_opt,\
                    initial_iteration,jacobian,init,weight,coordinates_cam_p,coordinates_cam,\
                    error_vector_inicial,error_vector_photo_sqrt,has_decreased,color_p,\
                    processed_frames,jacobian1k,overlap,cx,cy,pyramid_level,first_frame);
    iter ++;

    overlap_tracking = overlap;
    initial_iteration = 1;
    first_frame = false;

    float error_f = error_p;
    float error_f1 = 10;
    float error_f2 = 0;
    cv::Mat R=R2.clone();
    cv::Mat t=t2.clone();
    while (fabs((error_f1 - error_f2)/error_f1) > tracking_th  &&  has_decreased > 0.5 && error_f1 > error_f2 && iter < iter_th )
    {
            iter ++;
            error_f1 = error_f;
            has_decreased = 0;
            gauss_newton_ic(semidense_tracker,coordinates_cam_to_print,R2,t2,R_p,t_p,fx,fy,points,img,img_p,error_p,color,variance,\
                            points3D,error_vector_photo_opt,initial_iteration,jacobian,init,weight,coordinates_cam_p,coordinates_cam,\
                            error_vector_inicial,error_vector_photo_sqrt,has_decreased,color_p,processed_frames,jacobian1k,\
                            overlap,cx,cy,pyramid_level,first_frame);

            error_f2 = error_p;

            if (error_f1 > error_f2)
            {
                error_f =error_p;
                R=R2.clone();
                t=t2.clone();
                overlap_tracking = overlap;
            }
    }

    R2=R.clone();
    t2=t.clone();
    error_p = error_f;
}

void calculate_jac_geo_diff(SemiDenseTracking* semidense_tracker,cv::Mat& R_rel, cv::Mat& t_rel,cv::Mat& jac_geo_diff,
                            float fx,float fy, float cx, float cy,cv::Mat &error_init,cv::Mat &transformed_points_cam_p,
                            int pyramid_level,float max_error,cv::Mat &max_error_vector,bool use_inv_depth){
    cv::Mat error = error_init.clone();


    double epsilon = 0.000001;

    for(int i = 0; i < 6; i++)
    {

       cv::Mat w = cv::Mat::zeros(3,1,CV_32FC1);
       cv::Mat v = cv::Mat::zeros(3,1,CV_32FC1);


       if (i<3)
       {
           w.at<float>(i,0) = epsilon;
       }
       else{
           v.at<float>(i-3,0) = epsilon;
       }


       cv::Mat R_upd,t_upd;

       exp_SE3 (R_upd,t_upd,w,v);

       cv::Mat S,U,V;
       cv::SVD::compute(R_upd,S,U,V,cv::SVD::FULL_UV);
       R_upd = U*V;

       cv::Mat R_rel_ = R_upd*R_rel;
       cv::Mat t_rel_ = R_upd*t_rel+t_upd;


       cv::Mat coordinates_cam,transformed_points_cam;

       transform_points_return_3Dpoints(coordinates_cam,R_rel_,t_rel_,fx,fy,cx,cy,
                                        transformed_points_cam_p,transformed_points_cam);

       cv::Mat constant_error = cv::Mat::zeros(error.rows,1,CV_32FC1) + 1 ;
       cv::Mat error_sqrt = cv::Mat::zeros(error.rows,1,CV_32FC1) + 1 ;

       compute_geo_error(coordinates_cam,semidense_tracker->
                      frame_depth[pyramid_level],transformed_points_cam,
                         error,error_sqrt,constant_error,max_error_vector,use_inv_depth);

        cv::Mat jacobian_row = (error - error_init) / epsilon  ;

        for (int j = 0; j < jacobian_row.rows; j++)
        {
            jac_geo_diff.at<float>(j,i) = jacobian_row.at<float>(j,0);
        }
    }
}

void compute_geo_error(cv::Mat &coordinates_cam, cv::Mat &depth_map,cv::Mat &transformed_points,
                       cv::Mat &geo_errors,cv::Mat &geo_errors_sqrt,cv::Mat &constant_error,cv::Mat &max_error_vector,bool use_inv_depth)
{
      int imsize_x = depth_map.cols-1;
      int imsize_y = depth_map.rows-1;
      //#pragma omp parallel for num_threads(2)
      for (int k =0; k< coordinates_cam.cols;k++)
      {
          if (coordinates_cam.at<float>(1,k) > 1
                  && coordinates_cam.at<float>(1,k) < imsize_y-1
                  && coordinates_cam.at<float>(0,k) > 1
                  && coordinates_cam.at<float>(0,k) < imsize_x-1)
           {
             int x_2 = round(coordinates_cam.at<float>(0,k));
             int y_2 = round(coordinates_cam.at<float>(1,k));


             if( fabs(depth_map.at<float>((y_2),(x_2))) > 0 &&
                      fabs(depth_map.at<float>((y_2+1),(x_2))) > 0 &&
                      fabs(depth_map.at<float>((y_2),(x_2+1))) > 0 &&
                      fabs(depth_map.at<float>((y_2-1),(x_2))) > 0 &&
                      fabs(depth_map.at<float>((y_2),(x_2-1))) > 0)
             {
                 float r;
                 bilinear_interpolation(depth_map,  coordinates_cam.at<float>(0,k), coordinates_cam.at<float>(1,k),r);

                 if(use_inv_depth)
                 {
                     geo_errors_sqrt.at<float>(k,0)=( 1/r  + 1/transformed_points.at<float>(2,k));
                 }else{
                     geo_errors_sqrt.at<float>(k,0)=( r  + transformed_points.at<float>(2,k));
                 }
                 if(fabs(geo_errors_sqrt.at<float>(k,0)) > 2*max_error_vector.at<float>(k,0)){geo_errors_sqrt.at<float>(k,0) =  2*max_error_vector.at<float>(k,0);}
                 else{
                     constant_error.at<float>(k,0) = 0;
                 }
             }else{
                  geo_errors_sqrt.at<float>(k,0) =  max_error_vector.at<float>(k,0)*0.5;
             }
          }else{
               geo_errors_sqrt.at<float>(k,0) =  max_error_vector.at<float>(k,0)*0.5;
          }
      }
      geo_errors = geo_errors_sqrt.mul(geo_errors_sqrt);
}

void compute_error_ic_ni( cv::Mat &coordinates_cam,cv::Mat &coordinates_cam_p, cv::Mat &image, \
                       cv::Mat &error_vector,float &variance, cv::Mat &error_vector_sqrt,float &error, cv::Mat &weight,
                          cv::Mat &color_p, float &overlap,float &gain,float &brightness,cv::Mat &error_vector_check)
{
    int imsize_x =image.cols-1;
    int imsize_y =image.rows-1;

    float cont = 0;

    cv::Mat intensities_kf(0,1,CV_32FC1);
    cv::Mat intensities_frame(0,1,CV_32FC1);


    for (int k =0; k< coordinates_cam.cols;k++)
    {
        if (coordinates_cam.at<float>(1,k) > 1 && coordinates_cam.at<float>(1,k) < imsize_y-1 && coordinates_cam.at<float>(0,k) >1 && coordinates_cam.at<float>(0,k) < imsize_x-1
            && coordinates_cam_p.at<float>(1,k) > 1 && coordinates_cam_p.at<float>(1,k) < imsize_y-1 && coordinates_cam_p.at<float>(0,k) > 1 && coordinates_cam_p.at<float>(0,k) < imsize_x-1)
         {
            //BILINEAR INTERPOLATION
           float x_2 = coordinates_cam.at<float>(0,k);
           float y_2 = coordinates_cam.at<float>(1,k);

           float r;
           bilinear_interpolation(image,  x_2,y_2,r);



           float r_p = color_p.at<float>(k,0);

           if ( fabs(r -  r_p) < 0.2)
           {
               cont++;
               intensities_kf.push_back(r_p);
               intensities_frame.push_back(r);
           }

          error_vector_sqrt.at<float>(k,0) =(r*gain+brightness -  r_p);
        }
       else
       {
            error_vector_sqrt.at<float>(k,0) = sqrt(variance)*1;
       }
    }

    overlap = cont / coordinates_cam.cols;
    cv::pow(error_vector_sqrt, 2,error_vector);
    error_vector_check = error_vector.mul(weight);
    error = cv::sum(error_vector_check)[0];



    float  term1,term2;
    brightness = 0;
    cv::Mat term1_mat,term2_mat,bright_mat;
    term1 = 0;
    term2 = 0;
    term1_mat = intensities_frame.mul(intensities_kf);
    term2_mat = intensities_frame.mul(intensities_frame);
    term1 = cv::sum(term1_mat)[0];
    term2 = cv::sum(term2_mat)[0];
    gain = term1/term2;

    bright_mat  = intensities_kf - gain*intensities_frame;
    brightness = cv::sum(bright_mat)[0]/(1.0*bright_mat.rows);
}

void exp_SO3(cv::Mat &R, cv::Mat &w)
{
    float w1 = w.at<float>(0,0);
    float w2 = w.at<float>(1,0);
    float w3 = w.at<float>(2,0);

    float data[3][3] = { {0,  -w3,   w2},
                      {w3,   0,  -w1},
                     {-w2,   w1,  0}};
    cv::Mat wx = cv::Mat(3, 3, CV_32FC1, &data);

    float data1[3] = {   w1,w2,w3};

    float data2[3][3] = { {1,0,0},
                      {0,1,0},
                     {0,0,1}};

    cv::Mat eye = cv::Mat(3, 3, CV_32FC1, &data2);

   float  tetha = sqrt(w.at<float>(0,0)*w.at<float>(0,0) + w.at<float>(1,0)*w.at<float>(1,0) + w.at<float>(2,0)*w.at<float>(2,0));

    if (tetha < 0.00015)
        {R = eye + wx +0.5*(wx*wx);}
    else
        {R = eye+ sin(tetha)/tetha*wx + (1-cos(tetha))/(tetha*tetha)*(wx*wx);}
}

void  w2V(cv::Mat &V,float w1,float w2,float w3,cv::Mat &logR)
{
    float data1[3] = {   w1,w2,w3};
    cv::Mat w = cv::Mat(3, 1, CV_32FC1, &data1);

    float  theta = sqrt(w.at<float>(0,0)*w.at<float>(0,0) + w.at<float>(1,0)*w.at<float>(1,0) + w.at<float>(2,0)*w.at<float>(2,0));

    cv::Mat eye = (cv::Mat_<float>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if (theta < 0.000015)
    {
        V = eye + 0.5*logR + (1.0/6.0) * (logR * logR.t());
    }
    else
    {
        V = eye + ((1-cos(theta))/(theta*theta))*logR + (theta - sin(theta))/(theta*theta*theta) * (logR * logR.t());
    }
}


void log_SE3 (cv::Mat &R, cv::Mat &t, cv::Mat &w,cv::Mat &v)
{

    log_SO3(R,w);

    double w1 = w.at<float>(0,0);
    double w2 = w.at<float>(1,0);
    double w3 = w.at<float>(2,0);

    double d = 0.5*(cv::trace(R)[0]-1);
    cv::Mat logR;

    if ( (d > -1) &&  (d < 1))
        {logR = acos(d) * (R-R.t()) / (2*sqrt(1-d*d)) ;}
    else
        {logR = 0.5*(R-R.t()) ;}


    cv::Mat V;
    w2V(V,w1,w2,w3,logR);

    v=V.inv(cv::DECOMP_SVD)*t;
}


void exp_SE3 (cv::Mat &R,cv::Mat &t,cv::Mat &w,cv::Mat &v)
{
    float w1 = w.at<float>(0,0);
    float w2 = w.at<float>(1,0);
    float w3 = w.at<float>(2,0);

    exp_SO3( R,w);

    cv::Mat logR;
    float d;

    d = 0.5*(cv::trace(R)[0]-1);

    if (fabs(d) > 0.9999 &&  fabs(d)< 1.00001)
    {logR = 0.5*(R-R.t()) ;}
    else
    {logR = acos(d) * (R-R.t()) / (2*sqrt(1-d*d)) ;}

    cv::Mat V;
    w2V(V,w1,w2,w3,logR);

    t=V*v;
}


void log_SO3 (cv::Mat &R,cv::Mat &w)
{
    double d = 0.5*(cv::trace(R)[0]-1);
    cv::Mat logR;


    if ( (d > -1) &&  (d < 1))
        {logR = acos(d) * (R-R.t()) / (2*sqrt(1-d*d)) ;}
    else
        {logR = 0.5*(R-R.t()) ;}

    w.at<float>(0,0) = logR.at<float>(2,1);
    w.at<float>(1,0) = logR.at<float>(0,2);
    w.at<float>(2,0) = logR.at<float>(1,0);
}


void skew_SO3( cv::Mat &wx, cv::Mat &w)
{
    float w1 = w.at<float>(0,0);
    float w2 = w.at<float>(1,0);
    float w3 = w.at<float>(2,0);

    wx =  (cv::Mat_<float>(3,3) << 0, -w3, w2,  w3,0, -w1,-w2, w1,0);

}
void inv_skew_SO3( cv::Mat &wx, cv::Mat &w)
{
    w.at<float>(0,0) = wx.at<float>(2,1);
    w.at<float>(1,0) = wx.at<float>(0,2);
    w.at<float>(2,0) = wx.at<float>(1,0);
}

void J_r_SO3(cv::Mat &J_r,cv::Mat &w)
{
    cv::Mat eye = (cv::Mat_<float>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    cv::Mat wx;
    skew_SO3(wx, w);

    float tetha = cv::norm(w);

    if (tetha > 0.0005)
    {
        J_r = eye - (1-cos(tetha))/(tetha*tetha)*wx + (tetha-sin(tetha))/(tetha*tetha*tetha)*wx*wx;
    }
    else
    {
        J_r = eye.clone();
    }
}

cv::Mat MatToQuat(cv::Mat &Rot){
    cv::Mat  Quat(4,1,CV_32FC1);
    float tr = Rot.at<float>(0,0)+ Rot.at<float>(1,1)+ Rot.at<float>(2,2);
    int ii;
    ii=0;
    if (Rot.at<float>(1,1) > Rot.at<float>(0,0)) ii=1;
    if (Rot.at<float>(2,2) > Rot.at<float>(ii,ii)) ii=2;
    float s;
    if (tr >= 0){
        s = sqrt((tr + 1));
        Quat.at<float>(0,0) = s * 0.5;
        s = 0.5 / s;
        Quat.at<float>(1,0) = (Rot.at<float>(2,1) - Rot.at<float>(1,2)) * s;
        Quat.at<float>(2,0) = (Rot.at<float>(0,2) - Rot.at<float>(2,0)) * s;
        Quat.at<float>(3,0) = (Rot.at<float>(1,0) - Rot.at<float>(0,1)) * s;
    } else {
        switch(ii) {
         case 0:
                s = sqrt((Rot.at<float>(0,0)-Rot.at<float>(1,1)-Rot.at<float>(2,2)+1));
                Quat.at<float>(1,0) = s * 0.5;
                s = 0.5 / s;

                Quat.at<float>(2,0) = (Rot.at<float>(1,0) + Rot.at<float>(0,1)) * s;//Update pose estimation

                Quat.at<float>(3,0) = (Rot.at<float>(2,0) + Rot.at<float>(0,2)) * s;
                Quat.at<float>(0,0) = (Rot.at<float>(2,1) - Rot.at<float>(1,2)) * s;
                break;
         case 1:
                s = sqrt((Rot.at<float>(1,1)-Rot.at<float>(2,2)-Rot.at<float>(0,0)+1));
                Quat.at<float>(2,0) = s * 0.5;
                s = 0.5 / s;

                Quat.at<float>(3,0) = (Rot.at<float>(2,1) + Rot.at<float>(1,2)) * s;
                Quat.at<float>(1,0) = (Rot.at<float>(0,1) + Rot.at<float>(1,0)) * s;
                Quat.at<float>(0,0) = (Rot.at<float>(0,2) - Rot.at<float>(2,0)) * s;
                break;
         case 2:
                s = sqrt((Rot.at<float>(2,2)-Rot.at<float>(0,0)-Rot.at<float>(1,1)+1));
                Quat.at<float>(3,0) = s * 0.5;
                s = 0.5 / s;

                Quat.at<float>(1,0) = (Rot.at<float>(0,2) + Rot.at<float>(2,0)) * s;
                Quat.at<float>(2,0) = (Rot.at<float>(1,2) + Rot.at<float>(2,1)) * s;
                Quat.at<float>(0,0) = (Rot.at<float>(1,0) - Rot.at<float>(0,1)) * s;
                break;
         }
    }
    return Quat;
}

void decomposeR(cv::Mat &R, float &yaw, float &pitch, float &roll)
{
    roll   = atan2(R.at<float>(2,1),R.at<float>(2,2));
    pitch = atan2(-R.at<float>(2,0), sqrt(R.at<float>(2,1)*R.at<float>(2,1)+R.at<float>(2,2)*R.at<float>(2,2))  );
    yaw  = atan2(R.at<float>(1,0),R.at<float>(0,0));
}

void euler2quaternion(float &yaw, float &pitch, float &roll, float &x, float &y, float &z, float &w)
{
    w = cos(roll/2)*cos(pitch/2)*cos(yaw/2) +  sin(roll/2)*sin(pitch/2)*sin(yaw/2) ;
    x = sin(roll/2)*cos(pitch/2)*cos(yaw/2) -  cos(roll/2)*sin(pitch/2)*sin(yaw/2) ;
    y = cos(roll/2)*sin(pitch/2)*cos(yaw/2) +  sin(roll/2)*cos(pitch/2)*sin(yaw/2) ;
    z = cos(roll/2)*cos(pitch/2)*sin(yaw/2) -  sin(roll/2)*sin(pitch/2)*cos(yaw/2) ;
}


void prepare_semidense(SemiDenseMapping *semidense_mapper,
               vector<cv::Mat> &points_map,cv::Mat R,cv::Mat t,
               vector<cv::Mat> point_clouds,int pyramid_levels,\
               vector<float> &focalx, vector<float> &focaly,
               vector<float> &centerx, vector<float> &centery,   \
               vector<cv::Mat> &image_keyframe_pyramid,
               float  &points_projected_in_image)
{
   vector<cv::Mat> color(pyramid_levels);

    float  maximum_points_to_track = 10000 / 4 / 4 / 4 / 4;
    //float  maximum_points_to_track = 600;
    for (int i = 0; i < pyramid_levels;i++)
    {
        int imsize_y = image_keyframe_pyramid[i].rows;
        int imsize_x = image_keyframe_pyramid[i].cols;


        maximum_points_to_track *= 4;
        //maximum_points_to_track *= 3;

        cv::Mat edges;
        cv::Mat gray_image_aux = image_keyframe_pyramid[i]*255;
        gray_image_aux.convertTo(gray_image_aux,CV_8U);

        cv::Canny(gray_image_aux,edges,50,200,5,true);
        edges.convertTo(edges,CV_32FC1);
        if (i < pyramid_levels - 1) { edges = 255;}




        cv::Mat GX =gradientX(image_keyframe_pyramid[i],1);
        cv::Mat GY =gradientY(image_keyframe_pyramid[i],1);
        cv::Mat G = cv::abs(GX)  + cv::abs(GY);
        float alpha = 0.05;
        cv::exp(-G*alpha,G);
        cv::Mat G_reshaped = G.clone();
        G_reshaped = G_reshaped.reshape(0,G_reshaped.rows*G_reshaped.cols);
        cv::sort(G_reshaped,G_reshaped,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
        int number_of_pixels_limit =  0.40 * image_keyframe_pyramid[i].cols * image_keyframe_pyramid[i].rows;
        float limit_grad_aux = G_reshaped.at<float>(number_of_pixels_limit-100,0);

        cv::Mat points_joined(0, points_map[i].cols , CV_32FC1);
        cv::Mat points_joined_previous_map(0, points_map[i].cols , CV_32FC1);
        cv::Mat pointsClouds3Dmap_cam;

        cv::Mat points_map_aux = points_map[i].clone();
        point_clouds[i] = points_map_aux.clone();

        if (i==pyramid_levels-1)
        {
            semidense_mapper->points3D_tracked  =  points_map[pyramid_levels-1].clone();
            get_color(points_map[pyramid_levels-1],semidense_mapper->color_points3D_tracked);
        }




        cv::Mat pointsClouds3D =  point_clouds[i].t();
        pointsClouds3D = pointsClouds3D.rowRange(0,3);

        transform_points(pointsClouds3Dmap_cam, R,   t,   focalx[i],focaly[i],centerx[i],centery[i], pointsClouds3D   );
        get_color(point_clouds[i],color[i]);

        cv::Mat error_vector(point_clouds[i].rows,1,CV_32FC1);
        cv::Mat error_vector_sqrt(point_clouds[i].rows,1,CV_32FC1);
        cv::Mat error_check(0,1,CV_32FC1);
        cv::Mat weight(point_clouds[i].rows,1,CV_32FC1);

        float variance=0.03;

        compute_error( pointsClouds3Dmap_cam,image_keyframe_pyramid[i], color[i],error_vector,
                       variance,error_vector_sqrt,error_check,weight);


        int border_width = 4 * image_keyframe_pyramid[i].rows / image_keyframe_pyramid[pyramid_levels-1].rows ;

        cv::Mat check_repatead_points = cv::Mat::zeros(imsize_y,imsize_x,CV_32FC1);
        float cont2 = 0;

        check_repatead_points = cv::Mat::zeros(imsize_y,imsize_x,CV_32FC1);

        for(int k = 0; k < pointsClouds3Dmap_cam.cols;k++)
        {
            if (  fabs(error_vector.at<float>(k,0))< 0.10 && pointsClouds3Dmap_cam.at<float>(1,k) > border_width
                  && pointsClouds3Dmap_cam.at<float>(1,k) < imsize_y -border_width && \
                    pointsClouds3Dmap_cam.at<float>(0,k) > border_width &&pointsClouds3Dmap_cam.at<float>(0,k) < imsize_x-border_width)
            {
                bool edge_restriction = false;

                int n_x = pointsClouds3Dmap_cam.at<float>(0,k);
                int n_y = pointsClouds3Dmap_cam.at<float>(1,k);

                if (edges.at<float>(n_y,n_x) > 100) edge_restriction = true;
                if (edges.at<float>(n_y+1,n_x) > 100) edge_restriction = true;
                if (edges.at<float>(n_y,n_x+1) > 100) edge_restriction = true;
                if (edges.at<float>(n_y+1,n_x+1) > 100) edge_restriction = true;


                if(  edge_restriction == true &&
                     G.at<float>(round(pointsClouds3Dmap_cam.at<float>(1,k)),round(pointsClouds3Dmap_cam.at<float>(0,k))) < limit_grad_aux&&
                     check_repatead_points.at<float>(round(pointsClouds3Dmap_cam.at<float>(1,k)),round(pointsClouds3Dmap_cam.at<float>(0,k))) < 1)
                {
                        points_joined_previous_map.push_back(point_clouds[i].row(k));
                        check_repatead_points.at<float>(round(pointsClouds3Dmap_cam.at<float>(1,k)),round(pointsClouds3Dmap_cam.at<float>(0,k)))+=1;

                        cont2++;
                        if (pointsClouds3Dmap_cam.at<float>(1,k) > border_width && pointsClouds3Dmap_cam.at<float>(1,k) < imsize_y -border_width && \
                                pointsClouds3Dmap_cam.at<float>(0,k) > border_width &&pointsClouds3Dmap_cam.at<float>(0,k) < imsize_x-border_width \
                                && i==pyramid_levels-1)
                        {
                            points_projected_in_image++;
                        }
                }
            }
        }
        float limit = (maximum_points_to_track) / ( cont2);
        cv::Mat points_joined_previous_map_aux(0, points_map[i].cols ,CV_32FC1);
        for(int k = 0; k < points_joined_previous_map.rows;k++)
        {
             if  (((rand() % 1000000 ) / 1000000.0) < limit)
             {
                 points_joined_previous_map_aux.push_back(points_joined_previous_map.row(k));
             }
        }
        points_joined_previous_map = points_joined_previous_map_aux.clone();
        points_joined.push_back(points_joined_previous_map);
        points_map[i] = points_joined.clone();
    }
}
