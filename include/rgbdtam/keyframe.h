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


#ifndef __KEYFRAME_H
#define __KEYFRAME_H

// OpenCV
#include <opencv/cv.h>
#include <opencv/highgui.h>
#if CV24
#include <opencv2/nonfree/features2d.hpp>
#endif



#include <vector>
using namespace std;



class keyframe
{
public:
    cv::Mat R,t;
    cv::Mat image,matchings_mapreuse;
    vector<cv::Mat> features;
    float score_against_previousKf;
    int num_keyframe;
    cv::Mat final_depth_map;
    cv::Mat point_cloud_toprint;
    vector<cv::Mat> point_cloud_totrack;
    float fx,fy,cx,cy;
    double stamps;

    cv::Mat R_rel,t_rel;
    float scale_rel;
    float scale;
    vector<cv::KeyPoint> keypoints_orb;
    cv::Mat descriptors_orb;
};

#endif
