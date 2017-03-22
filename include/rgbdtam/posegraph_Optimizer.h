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


#ifndef __POSEGRAPH_OPTIMIZER_H
#define __POSEGRAPH_OPTIMIZER_H

#include <Eigen/StdVector>
#include <iostream>
#include <stdint.h>

#include <unordered_set>

#include "ThirdParty/g2o/g2o/core/sparse_optimizer.h"
#include "ThirdParty/g2o/g2o/core/block_solver.h"
#include "ThirdParty/g2o/g2o/core/solver.h"
#include "ThirdParty/g2o/g2o/core/robust_kernel_impl.h"
#include "ThirdParty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "ThirdParty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "ThirdParty/g2o/g2o/solvers/cholmod/linear_solver_cholmod.h"
#include "ThirdParty/g2o/g2o/solvers/dense/linear_solver_dense.h"
#include "ThirdParty/g2o/g2o/types/sba/types_six_dof_expmap.h"
//#include "ThirdParty/g2o/g2o/solvers/structure_only/structure_only_solver.h"

#include "ThirdParty/g2o/g2o/types/sim3/types_seven_dof_expmap.h"
#include <Eigen/StdVector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

using namespace Eigen;
using namespace std;

class posegraphOptimizer
{
public:
    void posegraphOptimization(vector<cv::Mat> &R_vector,vector<cv::Mat> &t_vector,
                               vector<cv::Mat> &R_vector_after_opt,vector<cv::Mat> &t_vector_after_opt, vector<float> &s_vector_after_opt,
                               vector<cv::Mat> &R_vector_edges,vector<cv::Mat> &t_vector_edges,
                               cv::Mat &edges_index,cv::Mat &poses1);
    void posegraphOptimization_Sim3(vector<cv::Mat> &R_vector,vector<cv::Mat> &t_vector,
                                    vector<cv::Mat> &R_vector_after_opt,vector<cv::Mat> &t_vector_after_opt, vector<float> &s_vector_after_opt,
                                    vector<cv::Mat> &R_vector_edges,vector<cv::Mat> &t_vector_edges,vector<float> &s_vector_edges,
                                    cv::Mat &edges_index,cv::Mat &poses1);
    g2o::SE3Quat convert_SE3_to_quat(cv::Mat &R_aux, cv::Mat &t_aux);
    Eigen::Matrix<double,3,3> convert_R_toEigen(cv::Mat &R_aux);
    Eigen::Matrix<double,3,1> convert_t_toEigen(cv::Mat &t_aux);
};
#endif
