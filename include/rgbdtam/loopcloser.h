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


#ifndef __LOOPCLOSER_H
#define __LOOPCLOSER_H

#include <iostream>
#include <vector>

#include "BowVector.h" // defines Surf64Vocabulary and Surf64Database

#include "DBoW2.h" // defines Surf64Vocabulary and Surf64Database

//#include "DUtils/DUtils.h"
//#include "DUtilsCV/DUtilsCV.h" // defines macros CVXX
//#include "DVision/DVision.h"

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#if CV24
#include <opencv2/nonfree/features2d.hpp>
#endif


///read images
///
/// #include<iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include<dirent.h>
#include<string.h>

#include "rgbdtam/keyframe.h"
#include "rgbdtam/posegraph_Optimizer.h"


using namespace std;
using namespace DBoW2;
//using namespace DUtils;


//PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>


class loopcloser
{
  public:
    loopcloser();

    // function borrowed from ORB_SLAM code
    std::vector <std::string> read_directory( const std::string& path );

    void changeStructure_orb(const cv::Mat &plain, vector<cv::Mat> &out,int L);
    void  calculate_orb_and_load_features( cv::Mat &image,vector<cv::Mat> &features_back,vector<cv::KeyPoint> &keypoints_orb,
    cv::Mat &descriptors_orb);

    void calculate_orb_features(  cv::Mat &image,vector<cv::Mat> &features_back,
                                                       vector<cv::KeyPoint> &keypoints_orb,cv::Mat &descriptors_orb);

    void feature_matching_and_edge_estimation(cv::Mat &matchings,cv::Mat &matchings_inliers,vector<cv::Mat>
                                              &R_vector_edges,vector<cv::Mat> &t_vector_edges,vector<float> &s_vector_edges);

    void relocalization(cv::Mat &image, cv::Mat &R, cv::Mat &t,int &oldest_kf, cv::Mat &matchings);


    void get_score_for_relocalization(vector<cv::Mat> feature,cv::Mat &matchings,
                                     int features_size,float keyframe_number, float threshold,
                                     float min_kf_diff,float max_kf_diff);

    void get_potential_keyframes(cv::Mat &image, cv::Mat &matchings);

    void get_scores_for_mapreuse(vector<cv::Mat> feature,cv::Mat &matchings,
                                     int features_size,float keyframe_number, float threshold,
                                     float min_kf_diff,float max_kf_diff);

    posegraphOptimizer posegraph_Optimizer_obj;

    void compute_keyframe(cv::Mat &R, cv::Mat &t, cv::Mat &image, int num_keyframe,
                                      cv::Mat &final_depth_map, cv::Mat &point_cloud_toprint, vector<cv::Mat> &point_cloud_totrack,
                                      float &fx, float &fy, float &cx, float &cy, double stamps, int size_first_level);
    void get_score_for_loopclosure(vector<cv::Mat> feature,cv::Mat &matchings,
                                     int features_size,float keyframe_number, float threshold,
                                     float min_kf_diff,float max_kf_diff);
    void backproject_matchedKpts_from_depth_map(cv::Mat &coordinates1, cv::Mat &final_depth_map1, cv::Mat &R1, cv::Mat &t1,
                                    cv::Mat &coordinates2, cv::Mat &final_depth_map2, cv::Mat &R2, cv::Mat &t2, \
                                    float &fx, float &fy, float &cx, float &cy, cv::Mat &points3D1, cv::Mat &points3D2);
    void print_poses(cv::Mat &points, char buffer[],int color,int points1rows);
    void horn_alignment(cv::Mat &model, cv::Mat &data, cv::Mat  &R_rel, cv::Mat  &t_rel, float &scale);
    void project_points_calculate_geometric_error(cv::Mat &points3D1, double fx,double fy,
                                                  double cx,double cy,cv::Mat &coordinates2,
                                                  cv::Mat &R2, cv::Mat &t2, cv::Mat &errors,float &error);

    void backproject_matchedKpts_from_depth_map(cv::Mat &coordinates1, cv::Mat &coordinates2, cv::Mat &final_depth_map1, cv::Mat &R1, cv::Mat &t1,
                                                float &fx, float &fy, float &cx, float &cy, cv::Mat &points3D1);

    void calculate_median_of_a_vector(cv::Mat &errors,float &error);
    void  print_point_cloud(cv::Mat &points, char buffer[]);

    void check_area_covered_by_features(float cx,float cy, cv::Mat &coordinates,float &area);

    cv::Mat matchings_inliers;
    vector<cv::Mat> R_vector_edges;
    vector<cv::Mat> t_vector_edges;
    vector<float> s_vector_edges;

    vector<cv::Mat> R_after_opt;
    vector<cv::Mat> t_after_opt;
    vector<float> s_after_opt;

    int init_keyframes;

    bool camera2PCLadded;

    void ransac_for_alignment(cv::Mat &model, cv::Mat &data, cv::Mat  &R_rel, cv::Mat  &t_rel,
                              float &scale, cv::Mat &matchings, cv::Mat &points3D1, cv::Mat &coordinates2,
                              cv::Mat &keypoint_Scale, float &error, int matchings_row, cv::Mat  &R_model,
                              cv::Mat  &t_model, cv::Mat  &R_data, cv::Mat  &t_data,
                              int &inliers_minimum);

    float pixel_error;
    int inliers_minimun;
    int number_loops_found;
    int initial_kf;
    int patch_size_for_depths;
    int use_kinect;
    bool viewer_mutex;

    void evaluation(cv::Mat poses,char buffer_evaluation[]);
    void print_keyframes_after_optimization();

    vector<keyframe> keyframes_vector;
    ORBVocabulary orb_voc;
    ORBDatabase orb_db;
    vector<vector<cv::Mat>> features;

    int depth_map_iterator;

    pcl::visualization::PCLVisualizer* viewer ;

    void populate_denseMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, cv::Mat pointcloud1);
    void addCameraPCL(cv::Mat &R, cv::Mat &t, char buffer[], bool blue);
    void updateCameraPCL(cv::Mat &R, cv::Mat &t, char buffer[], bool blue);

    private:
};

#endif
