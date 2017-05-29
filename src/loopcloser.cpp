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



#include "rgbdtam/loopcloser.h"
#include <ros/package.h>
#include <Eigen/Dense>

loopcloser::loopcloser()
{
    cv::FileStorage  fs2( (ros::package::getPath("rgbdtam")+"/src/data.yml").c_str(), cv::FileStorage::READ);

    use_kinect  = (int)fs2["use_kinect"];

    cout << "Reading the Vocabulary and creating a database..." << endl;

    char buffer[150];

    sprintf (buffer,(ros::package::getPath("rgbdtam") + "/ThirdParty/DBoW2/ORBvoc.txt").c_str());


    try{
        std::ifstream infile(buffer);
        if(!infile.good()) throw 100;
        // Reading vocabulary created by ORB-SLAM authors
        orb_voc.loadFromTextFile(buffer);
     }
      catch (int e){
        cout <<  "Vocabulary not found,  place it in ThirdParty/DBoW2/ORBvoc.txt (check README.md)" << endl ;
        exit(0);
     }



    ORBDatabase orb_db_aux(orb_voc, false, 0);

    orb_db = orb_db_aux;
    cout << orb_db << endl;
    cout << "end!!" << endl;

    pixel_error = 1.5;
    pixel_error = 2.5;
    inliers_minimun = 20;
    initial_kf = 0;
    patch_size_for_depths = 0;
    if(use_kinect == 0) patch_size_for_depths = 3;

    number_loops_found = 0;
    cv::Mat matchings_inliers_aux(0,2,CV_32FC1);
    matchings_inliers = matchings_inliers_aux.clone();

    camera2PCLadded = false;

    viewer = new pcl::visualization::PCLVisualizer("Dense Map and camera position");
    viewer->setBackgroundColor (0.75f,0.75f, 0.75f);
    viewer->initCameraParameters();
    viewer->setPosition(0,0);
    viewer->setSize(3*640,2*480);
    viewer->setCameraClipDistances(0.01,10.01);
    //viewer->setCameraFieldOfView(1.0);

    //TODO: the viewer will not work if this auxiliar viewer1 is not added.
    pcl::visualization::PCLVisualizer viewer1 ("aux viewer");
    viewer1.setSize(3,3);

    depth_map_iterator = 1;
    if(use_kinect == 1) depth_map_iterator = 3;
}

void print_evaluation_(cv::Mat points,  char buffer[])
{
    ofstream out(buffer);
    int num_points = points.rows;


    int val = points.rows-1;
    val = num_points;

    for (int i = 0; i<= points.rows-1;i++)
    {
        double val1 = points.at<double>(i,0);
        double val2 = points.at<double>(i,1);
        double val3 = points.at<double>(i,2);

        double val4 = points.at<double>(i,3);
        double val5 = points.at<double>(i,4);
        double val6 = points.at<double>(i,5);
        double val7 = points.at<double>(i,6);
        {
            out << fixed << setprecision(5) << val1<< " " << fixed << setprecision(5) << val2 <<
                   " " << fixed << setprecision(5) << val3 \
                << " "<< val4 << " "<< val5 << " "<< val6  << " "<< val7 << endl;
        }
    }
    out.close();
}

void loopcloser::populate_denseMap(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,cv::Mat pointcloud1)
{
    if(pointcloud1.rows > 0 && pointcloud1.cols > 3)
    {
        cv::Mat pointcloud = pointcloud1.clone();
        cv::Mat color = pointcloud.colRange(3,6).clone();
        color.convertTo(color,CV_16U);

        cloud->width = pointcloud.rows;
        cloud->height = 1;
        cloud->is_dense = false;
        pointcloud.convertTo(pointcloud,CV_32FC1);

        cloud->points.resize (cloud->width * cloud->height);
        for(int i = 0; i < cloud->points.size(); i=i+1)
        {
            cloud->points[i].x = pointcloud.at<float>(i,0) ;
            cloud->points[i].y = pointcloud.at<float>(i,1) ;
            cloud->points[i].z = pointcloud.at<float>(i,2) ;

            int r1 = pointcloud1.at<float>(i,3);
            int r2 = pointcloud1.at<float>(i,4);
            int r3 = pointcloud1.at<float>(i,5);

            cloud->points[i].r =  r1;
            cloud->points[i].g =  r2;
            cloud->points[i].b =  r3;
        }
    }
}

void loopcloser::calculate_median_of_a_vector(cv::Mat &errors,float &error)
{
    /// MEDIAN
    /*cv::Mat sorted_errors;
    cv::sort(errors,sorted_errors,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
    error = sorted_errors.at<float>(round(sorted_errors.rows/2),0);*/

    /// MEAN
    error = cv::mean(errors)[0];
}

void loopcloser::project_points_calculate_geometric_error(cv::Mat &points3D1, double fx,double fy,
                                                          double cx,double cy,cv::Mat &coordinates2,
                                                          cv::Mat &R2, cv::Mat &t2,cv::Mat &errors,float &error)
{
    cv::Mat t_r2 =  cv::repeat(t2,1,points3D1.cols);
    points3D1= R2*points3D1  + t_r2;

    points3D1.colRange(0,points3D1.cols).rowRange(0,1) = -points3D1.colRange(0,points3D1.cols).rowRange(0,1) * fx;
    points3D1.colRange(0,points3D1.cols).rowRange(1,2) =  points3D1.colRange(0,points3D1.cols).rowRange(1,2) * fy;


    cv::divide(points3D1.colRange(0,points3D1.cols).rowRange(0,1), points3D1.colRange(0,points3D1.cols).rowRange(2,3), points3D1.colRange(0,points3D1.cols).rowRange(0,1),1);
    cv::divide(points3D1.colRange(0,points3D1.cols).rowRange(1,2), points3D1.colRange(0,points3D1.cols).rowRange(2,3), points3D1.colRange(0,points3D1.cols).rowRange(1,2),1);
    cv::divide(points3D1.colRange(0,points3D1.cols).rowRange(2,3), points3D1.colRange(0,points3D1.cols).rowRange(2,3), points3D1.colRange(0,points3D1.cols).rowRange(2,3),1);

    points3D1.colRange(0,points3D1.cols).rowRange(0,1) =  cx + points3D1.colRange(0,points3D1.cols).rowRange(0,1) ;
    points3D1.colRange(0,points3D1.cols).rowRange(1,2) =  points3D1.colRange(0,points3D1.cols).rowRange(1,2) + cy;

    points3D1 = points3D1.t();
    for (int i = 0; i < points3D1.rows;i++)
    {
        errors.at<float>(i,0) = sqrt ((coordinates2.at<float>(i,0) - points3D1.at<float>(i,0))*(coordinates2.at<float>(i,0) - points3D1.at<float>(i,0)) + (coordinates2.at<float>(i,1) - points3D1.at<float>(i,1))*(coordinates2.at<float>(i,1) - points3D1.at<float>(i,1)));
    }
    calculate_median_of_a_vector(errors,error);
}

void loopcloser::print_poses(cv::Mat &points, char buffer[],int color,int points1rows)
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
        double val1 = points.at<float>(i,0);
        double val2 = points.at<float>(i,1);
        double val3 = points.at<float>(i,2);

        int  color1 =  255;
        int  color2 =  0;
        int  color3 =  0 ;
        if (i<points1rows)
        {
            color3 = 255;
        }else{
            color1 = 0;
        }
        {
            out << fixed  << val1<< " " << fixed  << val2 <<  " " << fixed << val3 \
                << " "<< color1 << " "<< color2 << " "<< color3 << endl;
        }
    }
    out.close();
}

void loopcloser::print_point_cloud(cv::Mat &points, char buffer[])
{
    points.convertTo(points,CV_32FC1);
    ofstream out(buffer);

    int num_points = points.rows;

    int val = points.rows-1;
    val = num_points;
    out << "ply" << endl;out << "format ascii 1.0" << endl;out << "element face 0" << endl;out << "property list uchar int vertex_indices" << endl;
    out << "element vertex ";out << val << endl;out << "property float x" << endl;out << "property float y" << endl;out << "property float z" << endl;
    out <<  "property uchar diffuse_red"<<endl;out << "property uchar diffuse_green" << endl;out << "property uchar diffuse_blue" << endl;out << "end_header" << endl;
    for (int i = 0; i<= points.rows-1;i++)
    {
        double val1 = points.at<float>(i,0);
        double val2 = points.at<float>(i,1);
        double val3 = points.at<float>(i,2);

        int  color1 =  points.at<float>(i,3);
        int  color2 =  points.at<float>(i,4);
        int  color3 =  points.at<float>(i,5);

        {
            out << fixed  << val1<< " " << fixed  << val2 <<  " " << fixed << val3 \
                << " "<< color1 << " "<< color2 << " "<< color3 << endl;
        }
    }
    out.close();
}

// function borrowed from ORB_SLAM code
std::vector <std::string> loopcloser::read_directory( const std::string& path )
{
    std::vector <std::string> result;
    dirent* de;
    DIR* dp;
    dp = opendir( path.empty() ? "." : path.c_str() );
    if (dp)
    {
        while (true)
        {
            de = readdir( dp );
            if (de == NULL) break;
            result.push_back( std::string( de->d_name ) );
        }
        closedir( dp );
        std::sort( result.begin(), result.end() );
    }
    return result;
}

void loopcloser::print_keyframes_after_optimization()
{
    cv::Mat point_cloud_keyframes_without_opt(0,6,CV_64FC1);
    cv::Mat point_cloud_keyframes_after_opt(0,6,CV_64FC1);

    cv::Mat point_cloud_keyframes_without_opt_total(0,6,CV_64FC1);
    cv::Mat point_cloud_keyframes_after_opt_total(0,6,CV_64FC1);

    int kf_size = 1;

    for (int i = 0 ; i < R_after_opt.size(); i = i+1)
    {
        if(keyframes_vector.at(i).num_keyframe%depth_map_iterator==0 && keyframes_vector.at(i).point_cloud_toprint.rows > 0)
        {
            cv::Mat point_cloud_keyframe = keyframes_vector.at(i).point_cloud_toprint.clone();
            point_cloud_keyframe.convertTo(point_cloud_keyframe,CV_64FC1);

            bool print_this_kf = false;
            for (int  ii = 0 ; ii < matchings_inliers.rows; ii = ii+1)
            {
                if (matchings_inliers.at<float>(ii,0) == i)
                {
                    print_this_kf = true;
                }
                if (matchings_inliers.at<float>(ii,1) == i)
                {
                    print_this_kf = true;
                }
            }

            point_cloud_keyframe = point_cloud_keyframe.colRange(0,6);

            if (point_cloud_keyframe.rows > 0  && point_cloud_keyframe.cols == 6)
            {
                cv::Mat point_cloud_xyz = point_cloud_keyframe.colRange(0,3);

                point_cloud_xyz = point_cloud_xyz.t();

                cv::Mat R_init,t_init, R_end,t_end,t_init_vector,t_end_vector;
                R_init = keyframes_vector.at(i).R.clone();
                t_init = keyframes_vector.at(i).t.clone();
                R_end = R_after_opt.at(i).clone();
                t_end = t_after_opt.at(i).clone();
                t_end = t_end.t();

                R_init.convertTo(R_init,CV_64FC1);
                t_init.convertTo(t_init,CV_64FC1);
                R_end.convertTo(R_end,CV_64FC1);
                t_end.convertTo(t_end,CV_64FC1);

                t_end_vector =   cv::repeat(t_end, 1,point_cloud_xyz.cols);
                t_init_vector =  cv::repeat(t_init,1,point_cloud_xyz.cols);

                point_cloud_xyz = R_init * point_cloud_xyz + t_init_vector;
                point_cloud_xyz =   s_after_opt.at(i) * R_end * point_cloud_xyz + t_end_vector;

                point_cloud_xyz = point_cloud_xyz.t();

                for (int j = 0 ; j < point_cloud_xyz.rows; j++)
                {
                    point_cloud_keyframe.at<double>(j,0) = point_cloud_xyz.at<double>(j,0);
                    point_cloud_keyframe.at<double>(j,1) = point_cloud_xyz.at<double>(j,1);
                    point_cloud_keyframe.at<double>(j,2) = point_cloud_xyz.at<double>(j,2);
                }

                if(print_this_kf  )
                    point_cloud_keyframes_after_opt.push_back(point_cloud_keyframe);

                point_cloud_keyframes_after_opt_total.push_back(point_cloud_keyframe);


                /// UPDATE POINTCLOUD IN THE VISUALIZER
                char buffer[150];
                point_cloud_keyframe.convertTo(point_cloud_keyframe,CV_32FC1);
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
                populate_denseMap(cloud, point_cloud_keyframe);
                sprintf(buffer,"denseMap%d", keyframes_vector.at(i).num_keyframe);
                viewer->updatePointCloud (cloud,buffer);
                /// UPDATE POINTCLOUD IN THE VISUALIZER
            }
        }
    }

    for (int i = 0 ; i < keyframes_vector.size(); i = i+kf_size)
    {
        if(keyframes_vector.at(i).num_keyframe % depth_map_iterator==0 && keyframes_vector.at(i).point_cloud_toprint.rows > 0)
        {

            bool print_this_kf = false;
            for (int ii = 0 ; ii < matchings_inliers.rows; ii = ii+1)
            {
                if (matchings_inliers.at<float>(ii,0)== i)
                {
                    print_this_kf = true;
                }
                if (matchings_inliers.at<float>(ii,1)== i)
                {
                    print_this_kf = true;
                }
            }

            cv::Mat point_cloud_keyframe = keyframes_vector.at(i).point_cloud_toprint.clone();
            point_cloud_keyframe = point_cloud_keyframe.colRange(0,6);
            point_cloud_keyframe.convertTo(point_cloud_keyframe,CV_64FC1);

            if (print_this_kf )
            {
                if (point_cloud_keyframe.rows > 0 && point_cloud_keyframe.cols == 6)
                    point_cloud_keyframes_without_opt.push_back(point_cloud_keyframe);
            }
            if (point_cloud_keyframe.rows > 0  && point_cloud_keyframe.cols == 6)
                point_cloud_keyframes_without_opt_total.push_back(point_cloud_keyframe);
        }

    }
    //if(use_kinect == 0)
    {
        if (point_cloud_keyframes_after_opt.rows == 0)
        {
            if (point_cloud_keyframes_without_opt.rows == 0)
            {
                point_cloud_keyframes_after_opt = point_cloud_keyframes_without_opt.clone();
                point_cloud_keyframes_after_opt_total = point_cloud_keyframes_without_opt_total.clone();
            }
        }
        char buffer[150];
        /*sprintf (buffer,"src/rgbdtam/src/results_depth_maps/reconstruction_after_optimization.ply");
        print_point_cloud(point_cloud_keyframes_after_opt,buffer);
        sprintf (buffer,"src/rgbdtam/src/results_depth_maps/reconstruction_without_optimization.ply");
        print_point_cloud(point_cloud_keyframes_without_opt,buffer);*/
        sprintf (buffer,"src/rgbdtam/src/results_depth_maps/reconstruction_after_optimization_total.ply");
        print_point_cloud(point_cloud_keyframes_after_opt_total,buffer);
        sprintf (buffer,"src/rgbdtam/src/results_depth_maps/reconstruction_without_optimization_total.ply");
        print_point_cloud(point_cloud_keyframes_without_opt_total,buffer);
    }

    return;
}



void loopcloser::changeStructure_orb(const cv::Mat &plain, vector<cv::Mat> &out,int L)
{
    for(int i = 0; i < plain.rows; ++i)
    {
        out.push_back(plain.row(i));
    }
}


void loopcloser::calculate_orb_and_load_features(  cv::Mat &image,vector<cv::Mat> &features_back,
                                                   vector<cv::KeyPoint> &keypoints_orb,cv::Mat &descriptors_orb)
{
    cv::Mat image_orb = image.clone();
    if (image_orb.rows >100 )
    {
        features.push_back(vector<cv::Mat>());
        int nFeatures = 1000;
        int sizePatch = 31;

        //cv::ORB orb(nFeatures,1.2,8,sizePatch,0,2,cv::ORB::HARRIS_SCORE,sizePatch);
        static cv::Ptr<cv::ORB> orb = cv::ORB::create(nFeatures,1.2,8,sizePatch,0,2,cv::ORB::HARRIS_SCORE,sizePatch);  // change for opencv 3
        cv::Mat mask_orb;
        
        // begin : added for opencv 3
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");
        
        orb->detectAndCompute( image_orb, cv::noArray(), keypoints_orb, descriptors_orb );
        /*  cv::ORB  inherited from cv::FeatureMatcher2D 
        
        virtual void 	detectAndCompute (
            InputArray image, 
            InputArray mask, 
            std::vector< KeyPoint > &keypoints, 
            OutputArray descriptors, 
            bool useProvidedKeypoints=false
            )
        
        detector->detectAndCompute(frame,   noArray(),   kp,   desc)
        
        */  // end added for opencv3
        
        //orb(image_orb, mask_orb, keypoints_orb, descriptors_orb);
        changeStructure_orb(descriptors_orb, features.back(), orb->descriptorSize());
        features_back = features.back();
    }
}


void loopcloser::calculate_orb_features(  cv::Mat &image,vector<cv::Mat> &features_back,
                                          vector<cv::KeyPoint> &keypoints_orb,cv::Mat &descriptors_orb)
{
    cv::Mat image_orb = image.clone();
    if (image_orb.rows >100 )
    {
        vector<vector<cv::Mat>> features_aux;

        features_aux.push_back(vector<cv::Mat>());
        int nFeatures = 1000;
        int sizePatch = 31;

        //cv::ORB orb(nFeatures,1.2,8,sizePatch,0,2,cv::ORB::HARRIS_SCORE,sizePatch);
        static cv::Ptr<cv::ORB> orb = cv::ORB::create(nFeatures,1.2,8,sizePatch,0,2,cv::ORB::HARRIS_SCORE,sizePatch);  // change for opencv 3
        cv::Mat mask_orb;
        //orb(image_orb, mask_orb, keypoints_orb, descriptors_orb);
        orb->detectAndCompute( image_orb, cv::noArray(), keypoints_orb, descriptors_orb );
        changeStructure_orb(descriptors_orb, features_aux.back(), orb->descriptorSize());
        features_back = features_aux.back();
    }
}



void loopcloser::backproject_matchedKpts_from_depth_map(cv::Mat &coordinates1,cv::Mat &final_depth_map1,cv::Mat &R1,cv::Mat &t1,
                                                        cv::Mat &coordinates2,cv::Mat &final_depth_map2,cv::Mat &R2,cv::Mat &t2,\
                                                        float &fx,float &fy,float &cx,float &cy,cv::Mat &points3D1,cv::Mat &points3D2)
{
    cv::Mat points1(0,3,CV_32FC1);
    cv::Mat points2(0,3,CV_32FC1);

    cv::Mat coordinates1_final(0,3,CV_32FC1);
    cv::Mat coordinates2_final(0,3,CV_32FC1);

    for (int i = 0 ; i < coordinates1.rows; i++)
    {
        int x1 = coordinates1.at<float>(i,0);
        int y1 = coordinates1.at<float>(i,1);
        int x2 = coordinates2.at<float>(i,0);
        int y2 = coordinates2.at<float>(i,1);

        float depth1 = 0;
        float num_depths1 = 0;
        float depth2 = 0;
        float num_depths2 = 0;

        for (int ii = x1-patch_size_for_depths; ii <= x1+patch_size_for_depths;ii++)
        {
            for (int jj = y1-patch_size_for_depths; jj <= y1+patch_size_for_depths;jj++)
            {
                if (ii > 0 && jj > 0 && jj < final_depth_map2.rows && ii < final_depth_map2.cols)
                {

                    if (fabs(final_depth_map1.at<float>(jj,ii)) > 0 )
                    {
                        num_depths1++;
                        depth1+= final_depth_map1.at<float>(jj,ii);
                    }
                }
            }
        }

        if (num_depths1 > 0)
        {depth1 /= num_depths1; }

        for (int ii = x2-patch_size_for_depths; ii <= x2+patch_size_for_depths;ii++)
        {
            for (int jj = y2-patch_size_for_depths; jj <= y2+patch_size_for_depths;jj++)
            {
                if (ii > 0 && jj > 0 && jj < final_depth_map2.rows && ii < final_depth_map2.cols)
                {
                    if (fabs(final_depth_map2.at<float>(jj,ii)) > 0 )
                    {
                        num_depths2++;
                        depth2+= final_depth_map2.at<float>(jj,ii);
                    }
                }
            }
        }
        if (num_depths2 > 0) depth2 /= num_depths2;

        if (fabs(depth1) > 0 && fabs(depth2) > 0)
        {

            cv::Mat point(3,1,CV_32FC1);
            point.at<float>(0,0) = (cx - x1)/(fx*depth1);
            point.at<float>(1,0) =  (y1 - cy)/(fy*depth1);
            point.at<float>(2,0) = 1/depth1;

            point = R1.t() * (point - t1);
            point = point.t();
            points1.push_back(point);
            point = point.t();


            point.at<float>(0,0) = (cx - x2)/(fx*depth2);
            point.at<float>(1,0) =  (y2 - cy)/(fy*depth2);
            point.at<float>(2,0) =  1/depth2;

            point = R2.t() * (point - t2);
            point = point.t();
            points2.push_back(point);

            coordinates2_final.push_back(coordinates2.row(i));
            coordinates1_final.push_back(coordinates1.row(i));

        }
    }

    points3D1 = points1.clone();
    points3D2 = points2.clone();
    coordinates1 = coordinates1_final.clone();
    coordinates2 = coordinates2_final.clone();
}




void loopcloser::backproject_matchedKpts_from_depth_map(cv::Mat &coordinates1,cv::Mat &coordinates2,cv::Mat &final_depth_map1,cv::Mat &R1,cv::Mat &t1,
                                                        float &fx,float &fy,float &cx,float &cy,cv::Mat &points3D1)
{
    cv::Mat points1(0,3,CV_32FC1);

    cv::Mat coordinates1_final(0,3,CV_32FC1);
    cv::Mat coordinates2_final(0,3,CV_32FC1);

    for (int i = 0 ; i < coordinates1.rows; i++)
    {
        int x1 = coordinates1.at<float>(i,0);
        int y1 = coordinates1.at<float>(i,1);


        float depth1 = 0;
        float num_depths1 = 0;

        for (int ii = x1-patch_size_for_depths; ii <= x1+patch_size_for_depths;ii++)
        {
            for (int jj = y1-patch_size_for_depths; jj <= y1+patch_size_for_depths;jj++)
            {
                if (ii > 0 && jj > 0 && jj < final_depth_map1.rows && ii < final_depth_map1.cols)
                {

                    if (fabs(final_depth_map1.at<float>(jj,ii)) > 0 )
                    {
                        num_depths1++;
                        depth1+= final_depth_map1.at<float>(jj,ii);
                    }
                }
            }
        }

        if (num_depths1 > 0)
        {depth1 /= num_depths1; }



        if (fabs(depth1) > 0)
        {

            cv::Mat point(3,1,CV_32FC1);
            point.at<float>(0,0) = (cx - x1)/(fx*depth1);
            point.at<float>(1,0) =  (y1 - cy)/(fy*depth1);
            point.at<float>(2,0) = 1/depth1;

            point = R1.t() * (point - t1);
            point = point.t();
            points1.push_back(point);

            coordinates1_final.push_back(coordinates1.row(i));
            coordinates2_final.push_back(coordinates2.row(i));

        }
    }

    points3D1 = points1.clone();
    coordinates1 = coordinates1_final.clone();
    coordinates2 = coordinates2_final.clone();
}



void loopcloser::check_area_covered_by_features(float cx,float cy, cv::Mat &coordinates,float &area){
    area = 1;
    float min_x = 1000;
    float min_y = 1000;
    float max_x = 0;
    float max_y = 0;
    for(int i = 0; i < coordinates.rows;i++)
    {
        if (coordinates.at<float>(i,0)>max_x)
        {
            max_x = coordinates.at<float>(i,0);
        }
        if (coordinates.at<float>(i,1)>max_y)
        {
            max_y = coordinates.at<float>(i,1);
        }
        if (coordinates.at<float>(i,0)<min_x)
        {
            min_x = coordinates.at<float>(i,0);
        }
        if (coordinates.at<float>(i,1)<min_y)
        {
            min_y = coordinates.at<float>(i,1);
        }
    }

    area = (max_x - min_x) *  (max_y - min_y) / (cx*cy);
}

void loopcloser::ransac_for_alignment(cv::Mat &model,cv::Mat &data,cv::Mat  &R_rel, cv::Mat  &t_rel,\
                                      float &scale,cv::Mat &matchings,cv::Mat &points3D1,cv::Mat &points3D2,
                                      cv::Mat &coordinates2,  cv::Mat &keypoint_scale,float &error, int matchings_row,\
                                      cv::Mat  &R_model, cv::Mat  &t_model,cv::Mat  &R_data,\
                                      cv::Mat  &t_data,int &inliers)
{
    float scale_final;
    cv::Mat R_model_aux = R_model.clone();
    cv::Mat t_model_aux = t_model.clone();
    cv::Mat R_data_aux = R_data.clone();
    cv::Mat t_data_aux = t_data.clone();

    int i = matchings_row;

    //cv::Mat points3D1_aux = points3D1.t
    cv::Mat model_aux = model.clone();
    cv::Mat data_aux = data.clone();
    cv::Mat coordinates2_aux = coordinates2.clone();
    cv::Mat errors2filtered;
    float error2filtered;

    cv::Mat R_rel_final, t_rel_final,points3D1_FINAL,points3D2_FINAL;



    float error_final = 10;

    float v = 0.7; // prob of observing an outlier
    float p = 0.99; // desired probability
    int n_iter = log(1 - p ) / log(1 - (1-v)*(1-v)*(1-v));

    float area_min = 0;



    for (int j = 0 ;  j < n_iter ; j++)
    {
        // SELECT 5 POINTS
        model = model_aux.clone();
        data =  data_aux.clone();
        coordinates2 = coordinates2_aux.clone();

        data  = data.t();
        model  = model.t();
        cv::Mat model_5points(0,3,CV_32FC1);
        cv::Mat data_5points(0,3,CV_32FC1);
        for (int i=0; i < 3 ; i++)
        {
            int row = round((model.rows-1)*((rand() % 1000000 ) / 1000000.0));
            model_5points.push_back(model.row(row));
            data_5points.push_back(data.row(row));

        }
        model_5points = model_5points.t();
        data_5points = data_5points.t();

        data  = data.t();
        model  = model.t();



        horn_alignment(model_5points,data_5points,R_rel,t_rel,scale);

        // SELECT 5 POINT
        // CALCULATE ERRORS

        cv::Mat t_rel_vector =  cv::repeat(t_rel,1,model.cols);


        model = scale*R_rel*model + t_rel_vector;
        points3D1 = model.t();

        cv::Mat errors(points3D1.rows,1,CV_32FC1);
        cv::Mat points3D1_aux = points3D1.t();

        project_points_calculate_geometric_error(points3D1_aux,
                                                 keyframes_vector[matchings.at<float>(i,0)].fx,
                keyframes_vector[matchings.at<float>(i,0)].fy,
                keyframes_vector[matchings.at<float>(i,0)].cx,
                keyframes_vector[matchings.at<float>(i,0)].cy,
                coordinates2,keyframes_vector[matchings.at<float>(i,1)].R,
                keyframes_vector[matchings.at<float>(i,1)].t,errors,error);

        errors2filtered = errors.clone();
        error2filtered = error;
        // CALCULATE ERRORS


        // REMOVE OUTLIERS
        model = model_aux.clone();
        data =  data_aux.clone();
        coordinates2 = coordinates2_aux.clone();


        model = model.t();
        data = data.t();

        cv::Mat model_filtered(0,3,CV_32FC1);
        cv::Mat keypoint_scales_filtered(0,1,CV_32FC1);
        cv::Mat coordinates2_filtered(0,2,CV_32FC1);
        cv::Mat data_filtered(0,3,CV_32FC1);
        cv::Mat error3D_filtered(0,1,CV_32FC1);

        for (int k = 0 ; k < errors2filtered.rows; k++)
        {
            if (errors2filtered.at<float>(k,0) < pixel_error * std::pow(1.2,keypoint_scale.at<float>(k,0)))
            {
                float error3Daux = fabs(model.at<float>(k,0)-data.at<float>(k,0)) +
                        fabs(model.at<float>(k,1)-data.at<float>(k,1))+fabs(model.at<float>(k,2)-data.at<float>(k,2));
                error3D_filtered.push_back(error3Daux);
            }
        }

        if (error3D_filtered.rows > inliers_minimun)
        {
            cv::Mat error3D  = cv::abs(data-model);
            error3D = error3D.colRange(0,1) + error3D.colRange(1,2) + error3D.colRange(2,3);

            cv::Mat sorted_3Derror;
            cv::sort(error3D_filtered,sorted_3Derror,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

            for (int k = 0 ; k < errors2filtered.rows; k++)
            {
                if (errors2filtered.at<float>(k,0) < pixel_error * std::pow(1.2,keypoint_scale.at<float>(k,0)) )
                {
                    model_filtered.push_back(model.row(k));
                    data_filtered.push_back(data.row(k));
                    coordinates2_filtered.push_back(coordinates2.row(k));
                    keypoint_scales_filtered.push_back(keypoint_scale.at<float>(k,0));
                }
            }
            // REMOVE OUTLIERS
        }


        // UPDATE BEST TRANSFORMATION
        if (model_filtered.rows > inliers_minimun  )
        {
            model = model_filtered.clone();
            data = data_filtered.clone();
            coordinates2 = coordinates2_filtered.clone();

            model = model.t();
            data = data.t();


            float area_covered = 0;
            check_area_covered_by_features(keyframes_vector[0].image.cols,keyframes_vector[0].image.rows,coordinates2,area_covered);

            if (model.cols > inliers  && area_covered > 0.15 && area_covered > area_min)
            {
                horn_alignment(model,data,R_rel_final,t_rel_final,scale);
                cv::Mat t_rel_vector =  cv::repeat(t_rel_final,1,model.cols);

                model = scale*R_rel_final*model + t_rel_vector;

                /// CALCULATE ERROR FINAL
                points3D1 = model.t();
                points3D1_FINAL = model.t();
                points3D2_FINAL = data.t();

                cv::Mat errors(points3D1.rows,1,CV_32FC1);
                cv::Mat points3D1_aux = points3D1.t();

                project_points_calculate_geometric_error(points3D1_aux,
                                                         keyframes_vector[matchings.at<float>(i,0)].fx,
                        keyframes_vector[matchings.at<float>(i,0)].fy,
                        keyframes_vector[matchings.at<float>(i,0)].cx,
                        keyframes_vector[matchings.at<float>(i,0)].cy,
                        coordinates2,keyframes_vector[matchings.at<float>(i,1)].R,
                        keyframes_vector[matchings.at<float>(i,1)].t,errors,error);

                int inliers_aux = 0;
                for (int k = 0 ; k < errors.rows; k++)
                {
                    if (errors.at<float>(k,0) < pixel_error * std::pow(1.2,keypoint_scales_filtered.at<float>(k,0)) )
                    {
                        inliers_aux++;
                    }
                }

                if(inliers_aux > inliers)
                {
                    inliers = model.cols;
                    inliers = inliers_aux;
                    errors2filtered = errors.clone();
                    error_final = error;
                    scale_final = scale;
                    area_min = area_covered;
                    keyframes_vector[matchings.at<float>(i,1)].scale = scale;
                }
                /// CALCULATE ERROR FINAL
            }
        }
        // UPDATE BEST TRANSFORMATION
    }


    if (inliers  > inliers_minimun)
    {
        t_model_aux = -R_model_aux.t()*t_model_aux;
        t_data_aux = -R_data_aux.t()*t_data_aux;
        R_model_aux = R_model_aux.t();
        R_data_aux = R_data_aux.t();

        cv::Mat t_rel_aux = t_model_aux - R_model_aux*R_data_aux.t()*t_data_aux;
        cv::Mat R_rel_aux = R_model_aux*R_data_aux.t();

        ///////////
        t_rel_aux = scale_final*R_rel_final*t_rel_aux + t_rel_final;
        R_rel_aux = R_rel_final*R_rel_aux;
        //////////

        R_relative[matchings.at<float>(i,0)]=(R_rel_final);
        t_relative[matchings.at<float>(i,0)]=(t_rel_final);
        s_relative[matchings.at<float>(i,0)]=(scale_final);

        R_rel = R_rel_aux.clone();
        t_rel = t_rel_aux.clone();

        scale = scale_final;
        error = error_final;

        points3D1 = points3D1_FINAL.clone();
        points3D2 = points3D2_FINAL.clone();
    }
    else
    {
        error = 10;
    }
}




void loopcloser::horn_alignment(cv::Mat &model,cv::Mat &data,cv::Mat  &R_rel, cv::Mat  &t_rel,
                                float &scale)
{

    //Align two trajectories using the method of Horn (closed-form).

    //Input:
    //model -- first trajectory (3xn)
    //data -- second trajectory (3xn)
    //Output:
    //R_rel -- rotation matrix (3x3)
    //t_rel -- translation vector (3x1)

    cv::Mat model_zerocentered =model.clone();
    cv::Mat data_zerocentered =data.clone();

    cv::Mat data_mean(3,1,CV_32FC1);
    cv::Mat model_mean(3,1,CV_32FC1);


    data_mean.at<float>(0,0) = cv::mean(data.rowRange(0,1))[0];
    data_mean.at<float>(1,0) = cv::mean(data.rowRange(1,2))[0];
    data_mean.at<float>(2,0) = cv::mean(data.rowRange(2,3))[0];

    model_mean.at<float>(0,0) = cv::mean(model.rowRange(0,1))[0];
    model_mean.at<float>(1,0) = cv::mean(model.rowRange(1,2))[0];
    model_mean.at<float>(2,0) = cv::mean(model.rowRange(2,3))[0];

    model_zerocentered.rowRange(0,1) = model_zerocentered.rowRange(0,1) - cv::mean(model.rowRange(0,1))[0];
    model_zerocentered.rowRange(1,2) = model_zerocentered.rowRange(1,2) - cv::mean(model.rowRange(1,2))[0];
    model_zerocentered.rowRange(2,3) = model_zerocentered.rowRange(2,3) - cv::mean(model.rowRange(2,3))[0];

    data_zerocentered.rowRange(0,1) = data.rowRange(0,1) - cv::mean(data.rowRange(0,1))[0];
    data_zerocentered.rowRange(1,2) = data.rowRange(1,2) - cv::mean(data.rowRange(1,2))[0];
    data_zerocentered.rowRange(2,3) = data.rowRange(2,3) - cv::mean(data.rowRange(2,3))[0];

    cv::Mat W = cv::Mat::zeros(3,3,CV_32FC1);

    for (int i = 0; i< model.cols;i++)
    {
        W += model_zerocentered.col(i)*data_zerocentered.col(i).t();
    }

    W = W.t();
    cv::Mat S,U,V;
    cv::SVD::compute(W,S,U,V,cv::SVD::FULL_UV);
    S = (cv::Mat_<float>(3,3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    if ((cv::determinant(U)*cv::determinant(V))<0)
    {
        S.at<float>(2,2)=-1;
    }

    R_rel = U*S*V;


    cv::Mat data_zerocentered_transformed = R_rel.t()*data_zerocentered;

    float nom = model_zerocentered.dot(data_zerocentered_transformed);
    cv::pow(data_zerocentered_transformed,2,data_zerocentered_transformed);
    float den = 0;

    for(int i=0; i<data_zerocentered_transformed.rows; i++)
    {
        for(int j=0; j<data_zerocentered_transformed.cols; j++)
        {
            den+=data_zerocentered_transformed.at<float>(i,j);
        }
    }

    scale =  1/(nom/den);

    if (use_kinect > 0.5)
    {
        scale = 1.0;
    }

    t_rel = model_mean -  (1/scale)*R_rel.t()*data_mean;
    t_rel = -scale*R_rel*t_rel;
}

void loopcloser::feature_matching_and_edge_estimation(cv::Mat &matchings,cv::Mat &matchings_inliers,
                                                      vector<cv::Mat> &R_vector_edges,
                                                      vector<cv::Mat> &t_vector_edges,
                                                      vector<float> &s_vector_edges)
{
    int img_cols = 0;
    int img_rows = 0;
    for (int i = 0 ; i< matchings.rows ; i++)
    {
        vector<cv::KeyPoint> keypoints_orb1;
        cv::Mat descriptors_orb1;
        //cv::Mat img1;

        vector<cv::KeyPoint> keypoints_orb2;
        cv::Mat descriptors_orb2;
        //cv::Mat img2;


        for (int j = 0 ; j < 2 ; j++)
        {

            img_cols = keyframes_vector[matchings.at<float>(i,j)].image.cols;
            img_rows =  keyframes_vector[matchings.at<float>(i,j)].image.rows;

            if (j == 0)
            {
                descriptors_orb1 = keyframes_vector[matchings.at<float>(i,j)].descriptors_orb.clone();
                keypoints_orb1   = keyframes_vector[matchings.at<float>(i,j)].keypoints_orb;
            }
            else
            {
                descriptors_orb2 = keyframes_vector[matchings.at<float>(i,j)].descriptors_orb.clone();
                keypoints_orb2   = keyframes_vector[matchings.at<float>(i,j)].keypoints_orb;
            }
        }


        cv::BFMatcher matcher( cv::NORM_HAMMING, true );
        std::vector< cv::DMatch > matches;
        matcher.match( descriptors_orb1, descriptors_orb2, matches );


        std::vector< cv::DMatch > matches_good;


        cv::Mat coordinates1(0,2,CV_32FC1);
        cv::Mat coordinates2(0,2,CV_32FC1);
        cv::Mat keypoint_scale(0,1,CV_32FC1);
        cv::Mat coordinates_aux(1,2,CV_32FC1);

        float minimun_distance = INFINITY;
        float median_distance = 0;
        float mean_distance = 0;

        cv::Mat distances_btw_mathces(matches.size(),1,CV_32FC1);
        for (int ii = 0;ii < matches.size();ii++)
        {
            mean_distance += matches[ii].distance /  matches.size();
            distances_btw_mathces.at<float>(ii,0) = matches[ii].distance;
            if (matches[ii].distance < minimun_distance)
            {
                minimun_distance = matches[ii].distance;

            }
        }
        calculate_median_of_a_vector(distances_btw_mathces,median_distance);

        for (int ii = 0;ii < matches.size();ii++)
        {
            if (matches[ii].distance < minimun_distance*3)
                //if (matches[ii].distance < mean_distance)
                //if (matches[ii].distance < median_distance)
            {
                matches_good.push_back(matches[ii]);

                if (keypoints_orb1[matches[ii].queryIdx ].pt.x > 0 && keypoints_orb1[matches[ii].queryIdx ].pt.x < img_cols
                        && keypoints_orb1[matches[ii].queryIdx ].pt.y > 0 && keypoints_orb1[matches[ii].queryIdx ].pt.y < img_rows
                        && keypoints_orb2[matches[ii].trainIdx ].pt.x > 0 && keypoints_orb2[matches[ii].trainIdx ].pt.x < img_cols
                        && keypoints_orb2[matches[ii].trainIdx ].pt.y > 0 && keypoints_orb2[matches[ii].trainIdx ].pt.y < img_rows
                        && keypoints_orb1[matches[ii].queryIdx ].octave   < 5
                        && keypoints_orb2[matches[ii].trainIdx ].octave < 5)
                {
                    coordinates_aux.at<float>(0,0) = keypoints_orb1[matches[ii].queryIdx ].pt.x;
                    coordinates_aux.at<float>(0,1) = keypoints_orb1[matches[ii].queryIdx ].pt.y;
                    coordinates1.push_back(coordinates_aux);
                    coordinates_aux.at<float>(0,0) = keypoints_orb2[matches[ii].trainIdx ].pt.x;
                    coordinates_aux.at<float>(0,1) = keypoints_orb2[matches[ii].trainIdx ].pt.y;
                    coordinates2.push_back(coordinates_aux);


                    if (keypoints_orb1[matches[ii].queryIdx ].octave <= keypoints_orb2[matches[ii].trainIdx ].octave )
                    {keypoint_scale.push_back(keypoints_orb1[matches[ii].queryIdx ].octave );}else{
                        keypoint_scale.push_back(keypoints_orb2[matches[ii].trainIdx ].octave );
                    }
                }
            }
        }



        cv::Mat points3D1,points3D2;
        backproject_matchedKpts_from_depth_map(coordinates1,keyframes_vector[matchings.at<float>(i,0)].final_depth_map,
                keyframes_vector[matchings.at<float>(i,0)].R, keyframes_vector[matchings.at<float>(i,0)].t,
                coordinates2,keyframes_vector[matchings.at<float>(i,1)].final_depth_map,
                keyframes_vector[matchings.at<float>(i,1)].R, keyframes_vector[matchings.at<float>(i,1)].t,
                keyframes_vector[matchings.at<float>(i,0)].fx, keyframes_vector[matchings.at<float>(i,0)].fy,
                keyframes_vector[matchings.at<float>(i,0)].cx, keyframes_vector[matchings.at<float>(i,0)].cy,\
                points3D1,points3D2);


        if (points3D1.rows > inliers_minimun)
        {

            cv::Mat R_rel,t_rel;
            cv::Mat model = points3D1.t();
            cv::Mat data = points3D2.t();
            float scale;
            float error = 0;


            int inliers = inliers_minimun;
            ransac_for_alignment(model,data,R_rel,t_rel,scale,matchings,points3D1,points3D2,coordinates2,keypoint_scale,error,i,
                                 keyframes_vector[matchings.at<float>(i,0)].R,keyframes_vector[matchings.at<float>(i,0)].t,
                                 keyframes_vector[matchings.at<float>(i,1)].R,keyframes_vector[matchings.at<float>(i,1)].t, inliers);

            if (inliers > inliers_minimun  )
            {
                cv::Mat U,V,S;
                cv::SVD::compute(R_rel,S,U,V,cv::SVD::FULL_UV);

                matchings_inliers.push_back(matchings.row(i));

                R_vector_edges.push_back(R_rel);
                t_vector_edges.push_back(t_rel);
                s_vector_edges.push_back(scale);

                /*if(points3D1.rows == points3D2.rows){
                int  kf2print = matchings.at<float>(i,0)*10+matchings.at<float>(i,1)*1;
                char buffer[150];

                points3D1.push_back(points3D2);
                sprintf (buffer,"src/rgbdtam/src/results_depth_maps/img%d_1_aligned.ply",kf2print);
                print_poses(points3D1,buffer,0,points3D2.rows);}*/
            }
        }
    }
}

void loopcloser::get_potential_keyframes(cv::Mat &image, cv::Mat &matchings, int points_tracked)
{
    vector<cv::KeyPoint> keypoints_orb;
    cv::Mat descriptors_orb;
    vector<cv::Mat> features_orb;
    calculate_orb_features( image, features_orb ,keypoints_orb,descriptors_orb);


    if(matchings.rows == 0 && points_tracked >  0.25*image.rows/8*image.cols/8)
        get_scores_for_mapreuse(features_orb,matchings,features_orb.size(),keyframes_vector.size(),0.30,5,1000);
}

void loopcloser::relocalization(cv::Mat &image,cv::Mat &R, cv::Mat &t, int &oldest_kf, cv::Mat &matchings)
{
    vector<cv::KeyPoint> keypoints_orb;
    cv::Mat descriptors_orb;
    vector<cv::Mat> features_orb;
    calculate_orb_features( image, features_orb ,keypoints_orb,descriptors_orb);



    if(matchings.rows == 0)
        get_score_for_relocalization(features_orb,matchings,features_orb.size(),1,0.30,INT64_MIN,INT64_MAX);



    int img_cols,img_rows;
    bool isRelocalized = false;

    for (int i = 0 ; i < matchings.rows ; i++)
    {
        if(!isRelocalized)
        {
            vector<cv::KeyPoint> keypoints_orb1;
            cv::Mat descriptors_orb1;

            vector<cv::KeyPoint> keypoints_orb2;
            cv::Mat descriptors_orb2;


            img_cols = keyframes_vector[matchings.at<float>(i,1)].image.cols;
            img_rows =  keyframes_vector[matchings.at<float>(i,1)].image.rows;


            descriptors_orb1 = keyframes_vector[matchings.at<float>(i,1)].descriptors_orb.clone();
            keypoints_orb1   = keyframes_vector[matchings.at<float>(i,1)].keypoints_orb;

            descriptors_orb2 = descriptors_orb.clone();
            keypoints_orb2   = keypoints_orb;




            cv::BFMatcher matcher( cv::NORM_HAMMING, true );
            std::vector< cv::DMatch > matches;
            matcher.match( descriptors_orb1, descriptors_orb2, matches );


            std::vector< cv::DMatch > matches_good;


            cv::Mat coordinates1(0,2,CV_32FC1);
            cv::Mat coordinates2(0,2,CV_32FC1);
            cv::Mat keypoint_scale(0,1,CV_32FC1);
            cv::Mat coordinates_aux(1,2,CV_32FC1);

            float minimun_distance = INFINITY;
            float median_distance = 0;
            float mean_distance = 0;

            cv::Mat distances_btw_mathces(matches.size(),1,CV_32FC1);
            for (int ii = 0;ii < matches.size();ii++)
            {
                mean_distance += matches[ii].distance /  matches.size();
                distances_btw_mathces.at<float>(ii,0) = matches[ii].distance;
                if (matches[ii].distance < minimun_distance)
                {
                    minimun_distance = matches[ii].distance;
                }
            }
            calculate_median_of_a_vector(distances_btw_mathces,median_distance);

            for (int ii = 0;ii < matches.size();ii++)
            {
                //if (matches[ii].distance < minimun_distance*3)
                {
                    matches_good.push_back(matches[ii]);
                    /*if (keypoints_orb1[matches[ii].queryIdx ].pt.x > 0 && keypoints_orb1[matches[ii].queryIdx ].pt.x < img_cols
                            && keypoints_orb1[matches[ii].queryIdx ].pt.y > 0 && keypoints_orb1[matches[ii].queryIdx ].pt.y < img_rows
                            && keypoints_orb2[matches[ii].trainIdx ].pt.x > 0 && keypoints_orb2[matches[ii].trainIdx ].pt.x < img_cols
                            && keypoints_orb2[matches[ii].trainIdx ].pt.y > 0 && keypoints_orb2[matches[ii].trainIdx ].pt.y < img_rows)*/
                    // && keypoints_orb1[matches[ii].queryIdx ].octave < 5
                    // && keypoints_orb2[matches[ii].trainIdx ].octave < 5)
                    {
                        coordinates_aux.at<float>(0,0) = keypoints_orb1[matches[ii].queryIdx ].pt.x;
                        coordinates_aux.at<float>(0,1) = keypoints_orb1[matches[ii].queryIdx ].pt.y;
                        coordinates1.push_back(coordinates_aux);
                        coordinates_aux.at<float>(0,0) = keypoints_orb2[matches[ii].trainIdx ].pt.x;
                        coordinates_aux.at<float>(0,1) = keypoints_orb2[matches[ii].trainIdx ].pt.y;
                        coordinates2.push_back(coordinates_aux);


                        if (keypoints_orb1[matches[ii].queryIdx ].octave <= keypoints_orb2[matches[ii].trainIdx ].octave )
                        {keypoint_scale.push_back(keypoints_orb1[matches[ii].queryIdx ].octave );}else{
                            keypoint_scale.push_back(keypoints_orb2[matches[ii].trainIdx ].octave );
                        }
                    }
                }
            }


            cv::Mat points3D1;
            backproject_matchedKpts_from_depth_map(coordinates1,coordinates2,keyframes_vector[matchings.at<float>(i,1)].final_depth_map,
                    keyframes_vector[matchings.at<float>(i,1)].R, keyframes_vector[matchings.at<float>(i,1)].t,
                    keyframes_vector[matchings.at<float>(i,1)].fx, keyframes_vector[matchings.at<float>(i,1)].fy,
                    keyframes_vector[matchings.at<float>(i,1)].cx, keyframes_vector[matchings.at<float>(i,1)].cy,\
                    points3D1);

            cv::Mat cameraMatrix = (cv::Mat_<float>(3,3) << keyframes_vector[matchings.at<float>(i,1)].fx, 0., keyframes_vector[matchings.at<float>(i,1)].cx,
                    0., keyframes_vector[matchings.at<float>(i,1)].fy , keyframes_vector[matchings.at<float>(i,1)].cy,
                    0., 0., 1.);
            cv::Mat distCoeffs = (cv::Mat_<float>(4,1) <<  0.0, 0.0, 0.0, 0.0);

            cv::Mat R_pnp,r_pnp,t_pnp;

            if(coordinates1.rows   > 100)
            {
                cv::Mat inliers;

                cv::solvePnPRansac(points3D1,coordinates2,cameraMatrix,distCoeffs,r_pnp,t_pnp,false,100,2.0,100,inliers,cv::SOLVEPNP_ITERATIVE);

                if(inliers.rows > 100 )
                {
                    cv::Mat R_rel = (cv::Mat_<double>(3,3) <<   1,   0,   0,
                                     0., -1,   0,
                                     0.,  0., -1.);
                    cv::Rodrigues(r_pnp,R_pnp);
                    R_pnp = R_rel * R_pnp ;
                    t_pnp =   R_rel * t_pnp;
                    t_pnp.convertTo(t_pnp,CV_32FC1);
                    R_pnp.convertTo(R_pnp,CV_32FC1);

                    // UPDATE KEYFRAME INFO AND current frame POSE
                    R = R_pnp.clone();
                    t = t_pnp.clone();
                    oldest_kf = matchings.at<float>(0,1);
                    isRelocalized = true;
                    // UPDATE KEYFRAME INFO AND current frame POSE
                }
            }
        } // if !isRelocalized
    } // for loop
}


void loopcloser::evaluation(cv::Mat poses,char buffer_evaluation[])
{
    cv::Mat evaluation_frame(1,7,CV_64FC1);
    cv::Mat evaluation_frame_total(0,7,CV_64FC1);
    for (int j = 0; j < keyframes_vector.size();j++)
    {
        evaluation_frame.at<double>(0,0) =  keyframes_vector.at(j).stamps;
        evaluation_frame.at<double>(0,1) = poses.at<double>(j,0);
        evaluation_frame.at<double>(0,2) = poses.at<double>(j,1);
        evaluation_frame.at<double>(0,3) = poses.at<double>(j,2);
        evaluation_frame.at<double>(0,4) = 0;
        evaluation_frame.at<double>(0,5) = 0;
        evaluation_frame.at<double>(0,6) = 0;
        evaluation_frame.at<double>(0,7) = 0;
        evaluation_frame_total.push_back(evaluation_frame);
    }

    print_evaluation_(evaluation_frame_total,buffer_evaluation);

}

void loopcloser::addCameraPCL(cv::Mat &R, cv::Mat &t){

    cv::Mat R_inv = R.t();
    cv::Mat t_inv = -R.t()*t;
    Eigen::Matrix4f  T;
    T(0,0) = R_inv.at<float>(0,0);     T(0,1) = R_inv.at<float>(0,1);     T(0,2) = R_inv.at<float>(0,2);  T(0,3) = t_inv.at<float>(0,0);
    T(1,0) = R_inv.at<float>(1,0);     T(1,1) = R_inv.at<float>(1,1);     T(1,2) = R_inv.at<float>(1,2);  T(1,3) = t_inv.at<float>(1,0);
    T(2,0) = R_inv.at<float>(2,0);     T(2,1) = R_inv.at<float>(2,1);     T(2,2) = R_inv.at<float>(2,2);  T(2,3) = t_inv.at<float>(2,0);
    T(3,0) = 0;                    T(3,1) = 0;                    T(3,2) = 0;                 T(3,3) = 1;
    Eigen::Affine3f t_affine(T);

    if(!camera2PCLadded)
    {
        viewer->addCoordinateSystem (0.10,  t_affine, "camera", 0);
        camera2PCLadded = true;
    }else{
        viewer->updateCoordinateSystemPose( "camera",  t_affine);
    }
}


void loopcloser::compute_keyframe(cv::Mat &R, cv::Mat &t, cv::Mat &image, int num_keyframe,cv::Mat &final_depth_map,
                                  cv::Mat &point_cloud_toprint,vector<cv::Mat> &point_cloud_totrack,
                                  float &fx, float &fy, float &cx, float &cy,
                                  double stamps,int size_first_level)
{
    num_keyframe = keyframes_vector.size();
    keyframe keyframe_aux;

    if(!viewer->wasStopped()){

        boost::mutex::scoped_lock lock(guard);

        char buffer[150];
        if(num_keyframe % depth_map_iterator == 0)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
            populate_denseMap(cloud, point_cloud_toprint);
            sprintf(buffer,"denseMap%d", num_keyframe);
            viewer->addPointCloud (cloud,buffer);
            keyframe_aux.point_cloud_toprint = point_cloud_toprint.clone();
        }
    }

    cv::Mat matchings(0,2,CV_32FC1);
    cv::Mat U,V,S;
    cv::SVD::compute(R,S,U,V,cv::SVD::FULL_UV);
    R = U*V;

    keyframe_aux.R = R.clone();
    keyframe_aux.t = t.clone();
    keyframe_aux.final_depth_map = final_depth_map;
    keyframe_aux.fx = fx;
    keyframe_aux.fy = fy;
    keyframe_aux.cx = cx;
    keyframe_aux.cy = cy;
    keyframe_aux.point_cloud_totrack = point_cloud_totrack;
    keyframe_aux.stamps = stamps;
    keyframe_aux.image = image;
    keyframe_aux.num_keyframe = num_keyframe;
    keyframe_aux.scale = 1;

    vector<cv::KeyPoint> keypoints_orb;
    cv::Mat descriptors_orb;
    calculate_orb_and_load_features( image,keyframe_aux.features /*vector<cv::Mat>*/,keypoints_orb,descriptors_orb);

    keyframe_aux.keypoints_orb = keypoints_orb;
    keyframe_aux.descriptors_orb = descriptors_orb.clone();

    orb_db.add(keyframe_aux.features);

    if(size_first_level > 0.25*image.rows/8*image.cols/8)
        get_score_for_loopclosure(keyframe_aux.features,matchings,features.size(),num_keyframe,0.30,50,1000);

    /*  cv::Mat matchings_mapreuse(0,2,CV_32FC1);
    if(size_first_level > 0.25*image.rows/8*image.cols/8)
    get_score_for_loopclosure(keyframe_aux.features,matchings_mapreuse,features.size(),num_keyframe,0.50,5,100);
    keyframe_aux.matchings_mapreuse = matchings_mapreuse.clone();*/


    keyframes_vector.push_back(keyframe_aux);


    cv::Mat poses_no_optimized(3,0,CV_64FC1);

    vector<cv::Mat> R_vector;
    vector<cv::Mat> t_vector;
    vector<cv::Mat> R_vector_after_opt;
    vector<cv::Mat> t_vector_after_opt;
    vector<float> s_vector_after_opt;

    for (int j = 0; j < keyframes_vector.size();j++)
    {
        cv::Mat R_aux,t_aux;

        t_aux = -keyframes_vector.at(j).R.t()*keyframes_vector.at(j).t;
        R_aux = keyframes_vector.at(j).R.t();

        R_vector.push_back(R_aux);
        t_vector.push_back(t_aux);

        t_aux = t_aux.t();
        t_aux.convertTo(t_aux,CV_64FC1);
        poses_no_optimized.push_back(t_aux);
    }

    char buffer_evaluation[150];
    sprintf(buffer_evaluation,"src/rgbdtam/src/evaluation/trajectory_no_posegraph.txt");
    evaluation(poses_no_optimized,buffer_evaluation);



    char buffer[150];
    poses_no_optimized.convertTo(poses_no_optimized,CV_32FC1);
    sprintf(buffer,"src/rgbdtam/src/results_depth_maps/tracking_NO_posegraph.ply");
    print_poses(poses_no_optimized,buffer,0,poses_no_optimized.rows);


    //if (matchings_inliers.rows > 0)
    {
        feature_matching_and_edge_estimation(matchings,matchings_inliers,R_vector_edges,t_vector_edges,s_vector_edges);

        if (R_vector.size() > 5)
        {
            if( matchings_inliers.rows > number_loops_found){
                cout << "Closing loop. Number of links: " << matchings_inliers.rows << endl;
            }
            number_loops_found = matchings_inliers.rows;

            cv::Mat poses;

            if (use_kinect > 0.5)
            {     posegraph_Optimizer_obj.posegraphOptimization(R_vector,t_vector,\
                                                                R_vector_after_opt,t_vector_after_opt,s_vector_after_opt,\
                                                                R_vector_edges,t_vector_edges,\
                                                                matchings_inliers,poses);}
            else
            {
                posegraph_Optimizer_obj.posegraphOptimization_Sim3(R_vector,t_vector,\
                                                                   R_vector_after_opt,t_vector_after_opt, s_vector_after_opt,\
                                                                   R_vector_edges,t_vector_edges, s_vector_edges,\
                                                                   matchings_inliers,poses);
            }

            R_after_opt = R_vector_after_opt;
            t_after_opt = t_vector_after_opt;
            s_after_opt = s_vector_after_opt;

            char buffer_evaluation[150];
            sprintf(buffer_evaluation,"src/rgbdtam/src/evaluation/trajectory_posegraph.txt");
            evaluation(poses,buffer_evaluation);
        }
    }
}

void loopcloser::get_score_for_loopclosure(vector<cv::Mat> feature,cv::Mat &matchings,
                                           int features_size,float keyframe_number, float threshold,
                                           float min_kf_diff,float max_kf_diff)
{
    QueryResults ret;

    orb_db.query(feature, ret, features_size );

    float previous_keyframe_score = 0.20;
    int counter = 0;
    cv::Mat match(1,2,CV_32FC1);
    for(QueryResults::iterator qit = ret.begin(); qit != ret.end(); ++qit)
    {
        counter ++;

        if (qit->Id  == features_size-2)
        {
            previous_keyframe_score = qit->Score;
        }
    }

    /*if (previous_keyframe_score < 0.040)
      {
         previous_keyframe_score = 0.040;
      }*/

    counter = 0;
    for(QueryResults::iterator qit = ret.begin(); qit != ret.end(); ++qit)
    {
        counter++;
        if (qit->Score / previous_keyframe_score > threshold && counter > 2 &&
                fabs(keyframe_number-qit->Id ) > min_kf_diff &&   fabs(keyframe_number-qit->Id ) < max_kf_diff )
        {
            match.at<float>(0,0) = keyframe_number;
            match.at<float>(0,1) = qit->Id;
            matchings.push_back(match);
        }
    }
}

void loopcloser::get_score_for_relocalization(vector<cv::Mat> feature,cv::Mat &matchings,
                                              int features_size,float keyframe_number, float threshold,
                                              float min_kf_diff,float max_kf_diff)
{
    QueryResults ret;

    orb_db.query(feature, ret, features_size );

    float previous_keyframe_score = 0.05;
    int counter = 0;
    cv::Mat match(1,2,CV_32FC1);



    counter = 0;
    for(QueryResults::iterator qit = ret.begin(); qit != ret.end(); ++qit)
    {
        counter++;
        if (qit->Score / previous_keyframe_score > threshold  &&
                fabs(keyframe_number-qit->Id ) > min_kf_diff &&   fabs(keyframe_number-qit->Id ) < max_kf_diff &&
                qit->Score / previous_keyframe_score  > 0.5)
        {
            match.at<float>(0,0) = keyframe_number;
            match.at<float>(0,1) = qit->Id;
            matchings.push_back(match);
        }
    }
}

void loopcloser::get_scores_for_mapreuse(vector<cv::Mat> feature,cv::Mat &matchings,
                                         int features_size,float keyframe_number, float threshold,
                                         float min_kf_diff,float max_kf_diff)
{
    QueryResults ret;

    orb_db.query(feature, ret, features_size );

    float previous_keyframe_score = 0.05;
    int counter = 0;
    cv::Mat match(1,2,CV_32FC1);

    counter = 0;
    for(QueryResults::iterator qit = ret.begin(); qit != ret.end(); ++qit)
    {
        counter++;
        if (qit->Score / previous_keyframe_score > threshold  &&
                fabs(keyframe_number-qit->Id ) > min_kf_diff &&   fabs(keyframe_number-qit->Id ) < max_kf_diff &&
                qit->Score / previous_keyframe_score  > 0.5)
        {
            match.at<float>(0,0) = keyframe_number;
            match.at<float>(0,1) = qit->Id;
            matchings.push_back(match);
        }
    }
}
