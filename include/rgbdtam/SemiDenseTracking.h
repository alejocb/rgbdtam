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

#ifndef __SEMIDENSETRACKING_H
#define __SEMIDENSETRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stack>
#include <ctime>

//chadir
#include <unistd.h>
// reading a text file
#include <iostream>
#include <fstream>
#include <string>
//directorio
#include <dirent.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>


#include "rgbdtam/SemiDenseMapping.h"
using namespace std;

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include <tf/transform_broadcaster.h>



/// loopcloser
#include "rgbdtam/loopcloser.h"
/// loopcloser

struct FrameStruct{
    cv::Mat image_frame,image_rgb,image_gray,image_to_track;
    double stamps;
    float fx,fy,cx,cy;
};

class SemiDenseTracking  :public Images_class {
  public:
    SemiDenseTracking();

    FrameStruct* frame_struct;
    FrameStruct frame_struct_aux;
    vector<FrameStruct> frame_struct_vector;


    loopcloser loopcloser_obj;
    cv::Mat image_full_size;


    float mean_depth_value;

    void init_local_maps(int);
    void set_local_maps(cv::Mat,int);
    vector<cv::Mat> get_local_maps(){return local_maps;}

    vector<string> left_image_names;
    vector<string> depth_image_names;
    void read_image_names(vector<string> &left_image_names,vector<string> &depth_image_names);
    int use_kinect;


    void init_poses_local_maps(int);
    void set_poses_local_maps(cv::Mat,int);
    vector<cv::Mat> get_poses_local_maps(){return poses_local_maps;}


    cv::Mat image_keyframe_canny;
    int bgr2rgb;

    int tum_dataset;
    int processed_frames;
    int processed_frames_since_keyframe;

    int use_ros;
    vector<cv::Mat>  keyframe_depth, frame_depth, frame_depth_aux;

    void init_points_last_keyframes(int);
    void set_points_last_keyframes(vector<cv::Mat>);
    vector<cv::Mat> get_points_last_keyframes(){return points_last_keyframes;}

    void init_distances_local_maps(int);
    void set_distances_local_maps(float,int);
    vector<float> get_distances_local_maps(){return distances_local_maps;}

    vector<cv::Mat> points_map_inImage;

    int pyramid_levels;
    int local_maps_number, local_maps_close_number;


    void init_poses();
    void set_poses(cv::Mat);
    cv::Mat poses;

    cv::Mat coordinates_depth_point_cloud;

    int init_frame;

    vector<cv::Mat> jacobian,jacobian_photo_no_weight,  jacobian_geo_no_weight, hessian_vision_geo;
    void inititialize_vectors_pyramid_levels_size(int);


    vector<int> reduction_pyramid;
    vector<cv::Mat> potential_keyframe;
    vector<cv::Mat> gradient_by_epipolar;

    cv::Mat mapX,mapY;

    vector<cv::Mat>frames;
    vector<double>frames_stamps;
    vector<float> focalx,focaly,centerx,centery;

    vector<cv::Mat> image_keyframe_pyramid;
    vector<cv::Mat> points_map;


    int frames_processed;

    vector<cv::Mat> color;
    vector<cv::Mat> points3D,points3D_geo,weight_geo;

    vector<cv::Mat> pixels_input;
    vector<cv::Mat> image_reduced,image_reduced_aux;
    vector<cv::Mat> error_vector;
    vector<cv::Mat> weight;
    vector<cv::Mat> GX_ref,GY_ref;

    vector<float> variances;
    void init_variances(int);

    float variance;

    cv::Mat R,t,R_kf,t_kf,R_prev,R_post,t_prev,t_post;
    cv::Mat image_rgb, image_to_track,image_gray,image_prev,image_keyframe;
    int image_n;

    float gain;
    float brightness;
    bool do_gain_brightness_estimation;

    int use_depth_tracking;

    int *cont_frames;
    ros::Time *stamps_ros;

    int last_cont_frames;

    cv::Mat distCoeffs,cameraMatrix;

    float cx,cy,fx,fy;

    float points_projected_in_image;

    float overlap_tracking;

    int create_inv_depth_discretization,last_frame_tracked;
    float tracking_th;

    int iter_th;

    int discretization;

    float depth;


    bool SystemIsLost{false};
    bool image_processing_semaphore;
    float PhotoError{0.0};

private:
    vector<cv::Mat> local_maps;
    vector<cv::Mat> poses_local_maps;
    vector<cv::Mat> points_last_keyframes;
    vector<float> distances_local_maps;
};


class SemiDenseMapping;  // It is defined also here due to cross reference (#include) issues

/// undistort image
 void undistort_image(cv::Mat &image_frame, cv::Mat &cameraMatrixAux,cv::Mat &distCoeffs,
                            cv::Mat &newCameraMatrix,cv::Mat &mapX,cv::Mat &mapY);

///prepare the image for tracking
 void prepare_image(SemiDenseTracking *semidense_tracker, cv::Mat &image_frame,
                          cv::Mat &image_to_track, int &image_n,
                          cv::Mat &image_gray, cv::Mat cameraMatrix, cv::Mat distCoeffs, \
                          float &fx, float &fy, float &cx, float &cy);

 cv::Mat reduce_depth_image(cv::Mat &depth_image,int reduction);
/// Image processing thread
void ThreadImageProcessing(SemiDenseTracking *semidense_tracker, SemiDenseMapping *semidense_mapper,
                             DenseMapping *dense_mapper);
///Viewer updater thread
void ThreadViewerUpdater( SemiDenseTracking *semidense_tracker,SemiDenseMapping *semidense_mapper, DenseMapping *dense_mapper);

///Semidense tracker thread
void ThreadSemiDenseTracker(Images_class *images,SemiDenseMapping *semidense_mapper,\
                            SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,MapShared *Map,ros::Publisher *vis_pub,image_transport::Publisher *pub_image);
///semidense_tracking function
void  semidense_tracking(Images_class *images,SemiDenseMapping *semidense_mapper,\
                         SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,MapShared *Map, ros::Publisher *vis_pub,image_transport::Publisher *pub_image);
///initialize semidense map to be tracked
void initialization_semidense(SemiDenseTracking *semidense_tracker, cv::Mat &R, cv::Mat &t, cv::Mat &R1, cv::Mat &t1, cv::Mat &image_rgb, cv::Mat &image_keyframe, \
                    int &pyramid_levels, vector<int> &reduction_pyramid, vector<cv::Mat> &image_keyframe_pyramid);
///first semidense map initialization
void initialization_semidense(SemiDenseTracking *semidense_tracker,cv::Mat &R,cv::Mat &t,cv::Mat &R1,cv::Mat &t1,cv::Mat &image_rgb,cv::Mat &image_keyframe,\
                    int &pyramid_levels,vector<int> &reduction_pyramid, vector<float> &focalx,vector<float> &focaly,vector<cv::Mat> &image_keyframe_pyramid,\
                    vector<cv::Mat> &points_map,vector<cv::Mat> &color,vector<cv::Mat> &points3D,\
                    vector<cv::Mat> &jacobian,vector<cv::Mat> &error_vector,vector<cv::Mat> &weight, float fx,float fy, float depth,\
                    cv::Mat depth_frame, int kinect_initialization,\
                    float cx,float cy,  vector<float> &centerx,vector<float> &centery,cv::Mat image_gray,float limit_grad  );

/// Get color from point cloud.
void get_color (cv::Mat &points,cv::Mat &color);

/// constant velocity motion model
void motion_model(vector<cv::Mat> &points_map,cv::Mat &R,cv::Mat &t,cv::Mat R_rel,cv::Mat t_rel,\
                vector<float> &focalx, vector<float> &focaly, vector<float> &centerx, vector<float> &centery,
                  vector<cv::Mat> &image_keyframe_pyramid,int pyramid_levels, bool &good_seed);

/// function to estimate the pose of the current frame
void optimize_camera_pose(int num_keyframes, SemiDenseTracking *semidense_tracker, SemiDenseMapping *semidense_mapper, Images_class &images, cv::Mat &image_to_track, \
                    cv::Mat &image_rgb, cv::Mat &R, cv::Mat &t, cv::Mat &R1, cv::Mat &t1, \
                    vector<cv::Mat> &image_reduced, vector<cv::Mat> &image_keyframe_pyramid, float &variance, vector<int> &reduction_pyramid,
                    int &processed_frames, vector<cv::Mat> &jacobian, vector<cv::Mat> &points_map, \
                    vector<cv::Mat> &color, vector<cv::Mat> &points3D, vector<cv::Mat> &error_vector, vector<cv::Mat> &weight, vector<float> &focalx, vector<float> &focaly, \
                    vector<float> &centerx, vector<float> &centery, int &pyramid_levels, float &overlap_tracking, float &tracking_th, int iter_th, \
                    vector<float> &variances, cv::Mat &image_gray, double stamps,
                    float mean_depth_value, cv::Mat &points3D_cam_to_print);



/// show the photometric reprojection of the map into the current frame
void show_error_photo(cv::Mat &points3D_cam,  cv::Mat &image_print, int num_pixels_sd2project, image_transport::Publisher *pub_image, cv::Mat &weight);

/// Shoe the geometric reprojection of the map into the current frame
void show_error_geo(cv::Mat &coordinates_cam,  cv::Mat &image_print, cv::Mat& weight_geo);

void resize_points(cv::Mat  &points2,cv::Mat  &points, float reduction,cv::Mat &image_p,
                   int kinect_initialization,float limit_grad);

/// compute error of the reprojection of the map into the current frame
void compute_error( cv::Mat &points3D_cam, cv::Mat image, cv::Mat &color, cv::Mat &error_vector,float variance, cv::Mat &error_vector_sqrt,cv::Mat &error_check,cv::Mat &weight);

/// compute error of the reprojection of the map into the current frame using the inverse compositional approach
void compute_error_ic( cv::Mat &points3D_cam,cv::Mat &points3D_cam_p, cv::Mat &image,cv::Mat &image_p, \
                       cv::Mat &error_vector,float &variance, cv::Mat &error_vector_sqrt,float &error,
                       cv::Mat &weight, cv::Mat &color_p, float &gain, float &brightness,cv::Mat &error_vector_check);
/// gauss newton optimization using inverse compotional approach
void gauss_newton_ic(SemiDenseTracking *semidense_tracker, cv::Mat &points3D_cam_to_print, cv::Mat &R, cv::Mat &t, \
                     cv::Mat &R_p, cv::Mat &t_p, float fx, float fy, cv::Mat &points, cv::Mat &img, cv::Mat &img_p, \
                     float &error_opt, cv::Mat &color, float variance, cv::Mat &points3D,\
                     cv::Mat &error_vector_opt, int initial_iteration, cv::Mat &jacobian, cv::Mat &init , \
                     cv::Mat &weight, cv::Mat &points3D_cam_p, cv::Mat &points3D_cam, \
                     cv::Mat &error_vector_inicial, cv::Mat &error_vector_sqrt_inicial, float &has_decreased, cv::Mat &color_p, \
                     int &processed_frames, cv::Mat &jacobian1k, float &overlap, float cx, float cy, int pyramid_level, bool first_frame);
///estimation of the pose of the camera using gauss newton
void  gauss_estimation(SemiDenseTracking *semidense_tracker, cv::Mat &points3D_cam_to_print, cv::Mat &R2, cv::Mat &t2, cv::Mat &R_p, cv::Mat &t_p, float &fx, float &fy, float &cx, float &cy, cv::Mat & points, \
                       cv::Mat &img, cv::Mat &img_p, float &error_p, cv::Mat &color, int &iter, float variance, cv::Mat &points3D, \
                       cv::Mat &error_vector_opt, cv::Mat &weight, int &processed_frames, cv::Mat &jacobian1k, float &overlap_tracking,
                       float &tracking_th, int iter_th, int pyramid_level, cv::Mat &points3D_cam, bool first_frame);

/// Reuse previous keyframes instead of creating a new one.
void map_reuse(SemiDenseTracking *semidense_tracker, SemiDenseMapping *semidense_mapper, cv::Mat &image_frame_aux
               , cv::Mat &R_kf, cv::Mat &t_kf);

void compute_error_ic_ni(cv::Mat &points3D_cam, cv::Mat &points3D_cam_p, cv::Mat &image, \
                       cv::Mat &error_vector, float &variance, cv::Mat &error_vector_sqrt, float &error, cv::Mat &weight,
                         cv::Mat &color_p, float &overlap, float &gain, float &brightness, cv::Mat &error_vector_check);

void transform_points_return_3Dpoints(cv::Mat &points3D_cam, cv::Mat &R,cv::Mat &t,float fx,float fy,float cx,float cy,cv::Mat &img, cv::Mat &points3D, cv::Mat &transformed_points);

void resize_points(cv::Mat  &points2,cv::Mat  &points, float reduction,
                   cv::Mat &image_p, int kinect_initialization,float limit_grad);


void exp_SO3(cv::Mat &R, cv::Mat &w);

void  w2V(cv::Mat &V,float w1,float w2,float w3,cv::Mat &logR);

void exp_SE3 (cv::Mat &R,cv::Mat &t,cv::Mat &w,cv::Mat &v);

void log_SO3 (cv::Mat &R,cv::Mat &w);

void skew_SO3( cv::Mat &wx, cv::Mat &w);

void inv_skew_SO3( cv::Mat &wx, cv::Mat &w);

void J_r_SO3(cv::Mat &J_r,cv::Mat &w);


void euler2quaternion(float &yaw, float &pitch, float &roll, float &x, float &y, float &z, float &w);

void decomposeR(cv::Mat &R, float &yaw, float &pitch, float &roll);


cv::Mat MatToQuat(cv::Mat &Rot);

/// join several 3D maps to make the tracking of the camera more robust to scale drift
void prepare_semidense(SemiDenseMapping *semidense_mapper,vector<cv::Mat> &points_map,cv::Mat R,cv::Mat t,vector<cv::Mat> point_clouds,int pyramid_levels,\
                vector<float> &focalx, vector<float> &focaly, vector<float> &centerx, vector<float> &centery,   \
              vector<cv::Mat> &image_keyframe_pyramid, float  &points_projected_in_image);

/// Search the depth image closer stamps_aux.
void get_depth_image(SemiDenseTracking *semidense_tracker, SemiDenseMapping *semidense_mapper, double stamps_aux, cv::Mat &depth_frame);

inline void bilinear_interpolation(cv::Mat &image, float x_2, float y_2, float &value);

void compute_geo_error(cv::Mat &points3D_cam, cv::Mat &depth_map, cv::Mat &transformed_points,
                       cv::Mat &geo_errors, cv::Mat &geo_errors_sqrt, cv::Mat &constant_error, cv::Mat &max_error_vector, bool use_inv_depth);


void calculate_jac_geo_diff(SemiDenseTracking* semidense_tracker, cv::Mat& R_rel, cv::Mat& t_rel, cv::Mat& jac_geo_diff,
                            float fx, float fy, float cx, float cy, cv::Mat &error_init, cv::Mat &points3D_geo, int pyramid_level, float max_error, cv::Mat& max_error_vector, bool use_inv_depth);

void calculate_jacobian_geo(cv::Mat &frame_depth, cv::Mat &points3D_cam, cv::Mat &transformed_points_cam, float fx, float fy, cv::Mat&jacobian_geo,
                        cv::Mat &constant_error, cv::Mat &max_error_vector,bool use_inv_depht);
#endif
