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


#include "rgbdtam/posegraph_Optimizer.h"

#include <opencv2/core/eigen.hpp>

void print_poses_(cv::Mat &points, char buffer[])
{
      ofstream out(buffer);

     int num_points = points.rows;

     int val = points.rows-1;
     val = num_points;
     out << "ply" << endl;out << "format ascii 1.0" << endl;out << "element face 0" << endl;out << "property list uchar int vertex_indices" << endl;
     out << "element vertex ";out << val << endl;out << "property float x" << endl;out << "property float y" << endl;out << "property float z" << endl;
     out <<  "property uchar diffuse_red"<<endl;out << "property uchar diffuse_green" << endl;out << "property uchar diffuse_blue" << endl;out << "end_header" << endl;
     for (int i = 0; i<= points.rows-1;i++)
     {

        double val1 = points.at<double>(i,0);
        double val2 = points.at<double>(i,1);
        double val3 = points.at<double>(i,2);


        int  color1 =  255;
        int  color2 =  0;
        int  color3 =  0 ;

        {
            out << fixed << setprecision(5) << val1<< " " << fixed << setprecision(5) << val2 <<  " " << fixed << setprecision(5) << val3 \
            << " "<< color1 << " "<< color2 << " "<< color3 << endl;
        }
     }
     out.close();
 }

g2o::SE3Quat posegraphOptimizer::convert_SE3_to_quat(cv::Mat &R_aux, cv::Mat &t_aux)
{
       Eigen::Matrix<double,3,3> R_eigen;
       R_eigen << R_aux.at<float>(0,0), R_aux.at<float>(0,1), R_aux.at<float>(0,2),
            R_aux.at<float>(1,0), R_aux.at<float>(1,1), R_aux.at<float>(1,2),
            R_aux.at<float>(2,0), R_aux.at<float>(2,1), R_aux.at<float>(2,2);
       Eigen::Matrix<double,3,1> t_eigen(t_aux.at<float>(0,0), t_aux.at<float>(1,0), t_aux.at<float>(2,0));

       return g2o::SE3Quat(R_eigen,t_eigen);
}

Eigen::Matrix<double,3,3> posegraphOptimizer::convert_R_toEigen(cv::Mat &R_aux)
{
       Eigen::Matrix<double,3,3> R_eigen;
       R_eigen << R_aux.at<float>(0,0), R_aux.at<float>(0,1), R_aux.at<float>(0,2),
            R_aux.at<float>(1,0), R_aux.at<float>(1,1), R_aux.at<float>(1,2),
            R_aux.at<float>(2,0), R_aux.at<float>(2,1), R_aux.at<float>(2,2);

       return  R_eigen;
}


Eigen::Matrix<double,3,1> posegraphOptimizer::convert_t_toEigen(cv::Mat &t_aux)
{
       Eigen::Matrix<double,3,1> t_eigen(t_aux.at<float>(0,0), t_aux.at<float>(1,0), t_aux.at<float>(2,0));
       return  t_eigen;
}


void posegraphOptimizer::posegraphOptimization(vector<cv::Mat> &R_vector,vector<cv::Mat> &t_vector,
                                               vector<cv::Mat> &R_vector_after_opt,vector<cv::Mat> &t_vector_after_opt,
                                               vector<float> &s_vector_after_opt,
                                               vector<cv::Mat> &R_vector_edges,vector<cv::Mat> &t_vector_edges,
                                               cv::Mat &edges_index_aux,cv::Mat &poses1)
{
    cv::Mat edges_index = edges_index_aux.clone();
    edges_index.convertTo(edges_index,CV_16UC1);

    g2o::SparseOptimizer sparse_optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linear_solver = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * block_solver= new g2o::BlockSolver_6_3(linear_solver);
    g2o::OptimizationAlgorithmLevenberg* solver_levenberg = new g2o::OptimizationAlgorithmLevenberg(block_solver);
    //g2o::OptimizationAlgorithmGaussNewton* solver_gauss = new g2o::OptimizationAlgorithmGaussNewton(block_solver);
    solver_levenberg->setUserLambdaInit(1e-15);
    sparse_optimizer.setAlgorithm(solver_levenberg);

    int num_kfs = R_vector.size();

    /////  VERTICES
    for(int i=0; i<num_kfs;i++)
    {
        g2o::VertexSE3Expmap* Vertex_SE3 = new g2o::VertexSE3Expmap();

        if (i > num_kfs-2){Vertex_SE3->setFixed(true);}
        else{Vertex_SE3->setFixed(false);}

        cv::Mat R2graph = R_vector.at(i);
        cv::Mat t2graph = t_vector.at(i);
        g2o::SE3Quat SE3_2graph = convert_SE3_to_quat( R2graph,  t2graph);
        Vertex_SE3->setEstimate(SE3_2graph);

        Vertex_SE3->setId(i);
        sparse_optimizer.addVertex(Vertex_SE3);
    }

    Eigen::Matrix<double,6,6> information_matrix = Eigen::Matrix<double,6,6>::Identity();


    /// EDGES BETWEEN CONSECUTIVE KEYFRAMES
    for(int i=0; i<num_kfs-1;i++)
    {
            int j = i+1;

            g2o::SE3Quat T_world_i;
            g2o::SE3Quat T_world_j;


            cv::Mat R2graph = R_vector.at(i);
            cv::Mat t2graph = t_vector.at(i);
            T_world_i = convert_SE3_to_quat( R2graph,  t2graph);

            R2graph = R_vector.at(j);
            t2graph = t_vector.at(j);
            T_world_j = convert_SE3_to_quat( R2graph,  t2graph);

            g2o::SE3Quat T_j_i = T_world_j*T_world_i.inverse();

            g2o::EdgeSE3Expmap* edge = new g2o::EdgeSE3Expmap();
            edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(sparse_optimizer.vertex(i)));
            edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(sparse_optimizer.vertex(j)));
            edge->setMeasurement(T_j_i);

            edge->information() = information_matrix;
            sparse_optimizer.addEdge(edge);
    }


    /// LOOP CLOSURE EDGES
    for(int i=0; i < R_vector_edges.size(); i++)
    {
        for(int j=0; j <1; j++)
        {
            g2o::SE3Quat T_j_i;
            T_j_i = convert_SE3_to_quat( R_vector_edges.at(i),  t_vector_edges.at(i));
            T_j_i = T_j_i.inverse();

            g2o::EdgeSE3Expmap* edge_loop = new g2o::EdgeSE3Expmap();
            edge_loop->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(sparse_optimizer.vertex(edges_index.at<unsigned short>(i,0))));
            edge_loop->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(sparse_optimizer.vertex(edges_index.at<unsigned short>(i,1))));
            edge_loop->setMeasurement(T_j_i);

            edge_loop->information() = information_matrix;
            sparse_optimizer.addEdge(edge_loop);
        }
    }

    sparse_optimizer.initializeOptimization();
    sparse_optimizer.optimize(30);

    cv::Mat poses(0,3,CV_64FC1);

    for(int i=0; i<num_kfs;i++)
    {
        g2o::VertexSE3Expmap* VSE3 = static_cast<g2o::VertexSE3Expmap*>(sparse_optimizer.vertex(i));
        g2o::SE3Quat T =  VSE3->estimate();

        Eigen::Matrix3d R = T.rotation().toRotationMatrix();
        Eigen::Vector3d t = T.translation();

        cv::Mat t_mat,R_mat;
        cv::eigen2cv(t,t_mat);
        cv::eigen2cv(R,R_mat);
        t_mat.convertTo(t_mat,CV_64FC1);
        R_mat.convertTo(R_mat,CV_64FC1);
        t_mat = t_mat.t();

        float scale = 1;
        s_vector_after_opt.push_back(scale);
        R_vector_after_opt.push_back(R_mat);
        t_vector_after_opt.push_back(t_mat);

        poses.push_back(t_mat);
    }

    char buffer[150];
    sprintf(buffer,"src/rgbdtam/src/map_and_poses/tracking_posegraph.ply");
    print_poses_(poses,buffer);
    poses1 = poses.clone();
}



void posegraphOptimizer::posegraphOptimization_Sim3(vector<cv::Mat> &R_vector,vector<cv::Mat> &t_vector,
                                               vector<cv::Mat> &R_vector_after_opt,vector<cv::Mat> &t_vector_after_opt,vector<float> &s_vector_after_opt,
                                               vector<cv::Mat> &R_vector_edges,vector<cv::Mat> &t_vector_edges, vector<float> &s_vector_edges,
                                               cv::Mat &edges_index_aux,cv::Mat &poses1)
{
    cv::Mat edges_index = edges_index_aux.clone();
    edges_index.convertTo(edges_index,CV_16UC1);

    g2o::SparseOptimizer sparse_optimizer;
    g2o::BlockSolver_7_3::LinearSolverType * linear_solver = new g2o::LinearSolverCholmod<g2o::BlockSolver_7_3::PoseMatrixType>();
    g2o::BlockSolver_7_3 * block_solver= new g2o::BlockSolver_7_3(linear_solver);
    g2o::OptimizationAlgorithmLevenberg* solver_levenberg = new g2o::OptimizationAlgorithmLevenberg(block_solver);
    //g2o::OptimizationAlgorithmGaussNewton* solver_gauss = new g2o::OptimizationAlgorithmGaussNewton(block_solver);
    solver_levenberg->setUserLambdaInit(1e-15);
    sparse_optimizer.setAlgorithm(solver_levenberg);

    int num_kfs = R_vector.size();

    /////  VERTICES
    for(int i=0; i<num_kfs;i++)
    {
        g2o::VertexSim3Expmap* Vertex_Sim3 = new g2o::VertexSim3Expmap();


        if (i > num_kfs-2){Vertex_Sim3->setFixed(true);}
        else{Vertex_Sim3->setFixed(false);}


        cv::Mat R2graph = R_vector.at(i);
        cv::Mat t2graph = t_vector.at(i);


        g2o::Sim3 Sim3_2graph(convert_R_toEigen(R2graph),convert_t_toEigen(t2graph),1.0);
        Vertex_Sim3->setEstimate(Sim3_2graph);


        Vertex_Sim3->setId(i);
        sparse_optimizer.addVertex(Vertex_Sim3);
    }

    Eigen::Matrix<double,7,7> information_matrix = Eigen::Matrix<double,7,7>::Identity();


    /// EDGES BETWEEN CONSECUTIVE KEYFRAMES
    for(int i=0; i<num_kfs-1;i++)
    {

            int j = i+1;

            cv::Mat R2graph = R_vector.at(i);
            cv::Mat t2graph = t_vector.at(i);


            g2o::Sim3 Sim3_world_i(convert_R_toEigen(R2graph),convert_t_toEigen(t2graph),1.0);

            R2graph = R_vector.at(j);
            t2graph = t_vector.at(j);


            g2o::Sim3 Sim3_world_j(convert_R_toEigen(R2graph),convert_t_toEigen(t2graph),1.0);
            g2o::Sim3 Sim3_j_i = Sim3_world_j*Sim3_world_i.inverse();



            g2o::EdgeSim3* edge = new g2o::EdgeSim3();
            edge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(sparse_optimizer.vertex(i)));
            edge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(sparse_optimizer.vertex(j)));
            edge->setMeasurement(Sim3_j_i);

            edge->information() = information_matrix;
            sparse_optimizer.addEdge(edge);
    }

    /// LOOP CLOSURE EDGES
    for(int i=0; i < R_vector_edges.size(); i++)
    {
        for(int j=0; j < 1; j++)
        {
            g2o::Sim3 Sim3_j_i(convert_R_toEigen(R_vector_edges.at(i)),convert_t_toEigen( t_vector_edges.at(i)),s_vector_edges.at(i));
            Sim3_j_i = Sim3_j_i.inverse();

            g2o::EdgeSim3* edge_loop = new g2o::EdgeSim3();
            edge_loop->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(sparse_optimizer.vertex(edges_index.at<unsigned short>(i,0))));
            edge_loop->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(sparse_optimizer.vertex(edges_index.at<unsigned short>(i,1))));
            edge_loop->setMeasurement(Sim3_j_i);

            edge_loop->information() = information_matrix;
            sparse_optimizer.addEdge(edge_loop);
        }
    }

    sparse_optimizer.initializeOptimization();
    sparse_optimizer.optimize(30);

    cv::Mat poses(0,3,CV_64FC1);

    for(int i=0; i<num_kfs;i++)
    {

        g2o::VertexSim3Expmap* VertexSim3 = static_cast<g2o::VertexSim3Expmap*>(sparse_optimizer.vertex(i));
        g2o::Sim3 Sim3=  VertexSim3->estimate();

        Eigen::Matrix3d R = Sim3.rotation().toRotationMatrix();
        Eigen::Vector3d t = Sim3.translation();
        float scale = Sim3.scale();

        cv::Mat t_mat,R_mat;
        cv::eigen2cv(t,t_mat);
        cv::eigen2cv(R,R_mat);
        t_mat.convertTo(t_mat,CV_64FC1);
        R_mat.convertTo(R_mat,CV_64FC1);
        t_mat = t_mat.t();


        R_vector_after_opt.push_back(R_mat);
        t_vector_after_opt.push_back(t_mat);
        s_vector_after_opt.push_back(scale);

        poses.push_back(t_mat);
    }

    char buffer[150];
    sprintf(buffer,"src/rgbdtam/src/map_and_poses/tracking_posegraph.ply");
    print_poses_(poses,buffer);
    poses1 = poses.clone();
}
