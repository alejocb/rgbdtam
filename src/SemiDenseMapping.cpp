
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

#include "rgbdtam/SemiDenseMapping.h"
#include "rgbdtam/vo_system.h"
#include "rgbdtam/loopcloser.h"
#include <ros/package.h>

#define U_SEGS(a)\
         gettimeofday(&tv,0);\
         a = tv.tv_sec + tv.tv_usec/1000000.0
double time_for_vector;
struct timeval tv;
double t1, t2,t0,t3;
void tic_init(){U_SEGS(t0);}
void toc_final(double &time){U_SEGS(t3); time =  (t3- t0)/1;}
void tic(){U_SEGS(t1);}
void toc(){U_SEGS(t2); time_for_vector = t2-t1;
           cout << (t2 - t1)/1 << endl;}

SemiDenseMapping::SemiDenseMapping():do_initialization(1),do_optimization(0), do_initialization_tracking(0), do_var_mapping(0),
    num_cameras_mapping(0), num_keyframes(0), do_init_semi(1), images_size(0), overlap_tracking(1),
    frames_previous_keyframe_processed(0),frames_previous_keyframe_used(0),convergence(1),convergence_total(0)
{
    cv::FileStorage  fs2( (ros::package::getPath("rgbdtam")+"/src/data.yml").c_str(), cv::FileStorage::READ);

    int pyramid_levels = 4;


    SemiDenseMapping::init_points_new_map(pyramid_levels);
    points3D_toprint.resize(5000);


    num_cameras_mapping_th = (int)fs2["num_cameras_mapping_th"];
    previous_images = num_cameras_mapping_th/2+1;
    percentage_converged_points = 0;
    percentage_reused_points = 0;

    semaphore = false;

    translational_ratio_th_min = (float)fs2["translational_ratio_th_min"];
    translational_ratio_th_min_aux = (float)fs2["translational_ratio_th_min"];
    num_cameras_mapping_th = (int)fs2["num_cameras_mapping_th"];
    num_cameras_mapping_th_aux = (int)fs2["num_cameras_mapping_th"];
    use_kinect = (int)fs2["use_kinect"];


    ///ROSKINECT
    image_depth_keyframes.resize(SIZE_DEPTH_VECTOR);
    stamps_depth_ros.resize(SIZE_DEPTH_VECTOR,0);
    associated_frames = 0;
    ///ROSKINECT

    limit_grad = (float)fs2["limit_grad"];
    kinect_initialization = (float)fs2["kinect_initialization"];
    overlap_tracking_th = (float)fs2["overlap_tracking_th"];

    max_pixel_color  = 255;
    minim_points_converged  = (int)fs2["minim_points_converged"];
    overlap_tracking = 1;
    num_potential_points = 0;

    int discretization = 100;
    for (int l=0; l<discretization; l++)
    {
        X.computeError();
        X_gx_ex.computeError();
        X_gy_ey.computeError();
    }

    last_frame_mapped  = 0;

    cv::Mat map_points_aux(0,6,CV_32FC1);
    cv::Mat local_map_points_aux(0,6,CV_32FC1);
    local_map_points = local_map_points_aux.clone();
    map_points = map_points_aux.clone();

    cv::Mat inv_depths_aux(discretization,1, CV_32FC1);
    inv_depths = inv_depths_aux.clone();
    translational_ratio = 0;

    init_keyframes = 12;


    if (kinect_initialization > 0.5)
    {
        init_keyframes = 3;
        if (use_kinect > 0.5 || kinect_initialization > 0.5) init_keyframes = -3;
    }
    fs2.release();
}



void SemiDenseMapping::init_points_new_map(int pyramid_levels)
{
    points_new_map.resize(pyramid_levels);
}


void SemiDenseMapping::set_points_new_map (vector<cv::Mat> points_new_map_aux)
{
    for (int i = 0; i < points_new_map.size();i++)
    {
        points_new_map[i] = points_new_map_aux[i].clone();
    }
}



void SemiDenseMapping::set_map_points_print(cv::Mat map_points_print_aux)
{
    map_points_print = map_points_print_aux.clone();
}
cv::Mat SemiDenseMapping::get_map_points_print()
{
    return map_points_print;
}

void copy_previous_kf_images(Images_class &images,Images_class *pimages_previous_keyframe,SemiDenseMapping *semidense_mapper,int &init_mapping,int &end_mapping,int &images_size, bool &optimize_previous_frame)
{
    if (  (semidense_mapper->frames_previous_keyframe_used < pimages_previous_keyframe->getNumberOfImages()-1)   )
    {
        optimize_previous_frame = true;
        for (int ii = 0;ii < 1;ii++)
        {
                images.computeImage();
                int current_images_size = images.getNumberOfImages()-1;
                images.Im[current_images_size]->image = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->image.clone();
                images.Im[current_images_size]->R = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->R.clone();
                images.Im[current_images_size]->image_gray=pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->image_gray.clone();
                images.Im[current_images_size]->t = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->t.clone();
                images.Im[current_images_size]->t_r = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->t_r.clone();
                images.Im[current_images_size]->fx = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->fx;
                images.Im[current_images_size]->fy = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->fy;
                images.Im[current_images_size]->cx = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->cx;
                images.Im[current_images_size]->cy = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->cy;
                images.Im[current_images_size]->error = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->error;
                images.Im[current_images_size]->image_number = pimages_previous_keyframe->Im[ pimages_previous_keyframe->getNumberOfImages()-1-semidense_mapper->frames_previous_keyframe_used]->image_number;
        }
        semidense_mapper->frames_previous_keyframe_used++;

        images_size = images.getNumberOfImages();
        init_mapping = images_size-1;
        end_mapping = init_mapping+1;
    }
}


void ThreadSemiDenseMapper(Images_class *images,Images_class *images_previous_keyframe,SemiDenseMapping *semidense_mapper,\
                           SemiDenseTracking *semidense_tracker,DenseMapping *dense_mapper,MapShared *Map, ros::Publisher *pub_cloud)
{
    /// loopcloser
    while(ros::ok() && dense_mapper->sequence_has_finished == false  || semidense_mapper->num_keyframes < 2)
    /// loopcloser
    {
                boost::this_thread::sleep(boost::posix_time::milliseconds(1));

                semidense_mapper->images_size = images->getNumberOfImages();
                bool insert_frame4mapping = false;

                if ( semidense_mapper->do_initialization_tracking < 0.5 &&  semidense_mapper->images_size  > 1 )
                {insert_frame4mapping = true;}

                if (semidense_mapper->num_cameras_mapping > semidense_mapper->num_cameras_mapping_th+1  && \
                        (semidense_mapper->translational_ratio > -0.03 || (semidense_mapper->num_keyframes < semidense_mapper->init_keyframes && semidense_mapper->translational_ratio > -0.030 )))
                {semidense_mapper->do_var_mapping = 1;}

                if (semidense_mapper->semaphore == true ||  semidense_tracker->use_ros == 1)
                {
                    if (insert_frame4mapping || (semidense_mapper->do_var_mapping == 1 && insert_frame4mapping) )
                    {
                        semidense_mapping(dense_mapper,semidense_mapper,semidense_tracker,Map,images,images_previous_keyframe,pub_cloud);
                    }
                    semidense_mapper->semaphore = false;

                }
                if(semidense_tracker->SystemIsLost)  semidense_mapper->do_initialization_tracking = 1;
    }

    cout << "thread mapping finished" << endl;
    return;
}

void calculate_min_translation(SemiDenseMapping *semidense_mapper, SemiDenseTracking *semidense_tracker,float &translational_ratio_th_min)
{
    //if(semidense_mapper->num_keyframes > semidense_mapper->init_keyframes -1)
    {
        translational_ratio_th_min = semidense_mapper-> translational_ratio_th_min_aux*(semidense_mapper->num_cameras_mapping+1);
        // translational_ratio_th_min = semidense_mapper-> translational_ratio_th_min_aux*pow(semidense_mapper->num_cameras_mapping+1,1.2);

        if (semidense_mapper->num_cameras_mapping > semidense_mapper->num_cameras_mapping_th_aux)
        {
            translational_ratio_th_min = semidense_mapper-> translational_ratio_th_min_aux*(semidense_mapper->num_cameras_mapping_th_aux+1) \
                    +2*semidense_mapper-> translational_ratio_th_min_aux * (semidense_mapper-> num_cameras_mapping-semidense_mapper-> num_cameras_mapping_th_aux);
         }
     }

    if(semidense_mapper -> use_kinect == 1)
    {
        if( semidense_mapper->num_cameras_mapping == 0){
            translational_ratio_th_min = 0;
        }
        if (semidense_tracker->overlap_tracking < 0.85
                && semidense_mapper->num_cameras_mapping > 0)
        {
            semidense_mapper->do_var_mapping = 1;
        }
    }
}

void semidense_mapping(DenseMapping *dense_mapper,SemiDenseMapping *semidense_mapper,SemiDenseTracking *semidense_tracker,\
                       MapShared  *Map,Images_class *pimages,Images_class  *pimages_previous_keyframe,ros::Publisher *pub_cloud)
{
                float translational_ratio_th_min = semidense_mapper-> translational_ratio_th_min;

                float spatio_temporal_th = 1.5;
                float inv_depth_disparity_print_th = spatio_temporal_th;


                float inv_depth_disparity_th = spatio_temporal_th;
                float spatial_threshold = spatio_temporal_th;


                if (semidense_mapper->num_keyframes <  semidense_mapper -> init_keyframes)
                {semidense_mapper->num_cameras_mapping_th= 8; if (semidense_mapper->num_keyframes < 2 && semidense_mapper->kinect_initialization == 0) semidense_mapper->num_cameras_mapping_th= 6;
                translational_ratio_th_min = 0.06;}
                else
                {
                     semidense_mapper->num_cameras_mapping_th=semidense_mapper->num_cameras_mapping_th_aux;

                     float count_points_converged = 0;
                     for (int i=0;i<semidense_mapper->points_convergence.rows;i++)
                     {
                        if (semidense_mapper->points_convergence.at<float>(i,0)>2)
                        {count_points_converged++;}
                     }

                     if( 100 * count_points_converged/ semidense_mapper->points_convergence.rows <   semidense_mapper -> minim_points_converged &&  semidense_mapper->num_keyframes > semidense_mapper->init_keyframes+3
                         && semidense_mapper-> num_cameras_mapping < semidense_mapper->num_cameras_mapping_th_aux*3)
                     {
                         semidense_mapper->num_cameras_mapping_th += 2;
                         semidense_mapper->do_var_mapping = 0;
                     }
                }

               calculate_min_translation(semidense_mapper,semidense_tracker,translational_ratio_th_min);
               Images_class images;
               copy_first_and_last_images(*pimages,images,semidense_mapper->mean_value,translational_ratio_th_min);



                cv::Mat points6;
                cv::Mat points6_aux ;
                if (semidense_mapper->points3D_toprint[semidense_mapper->num_keyframes].rows > 20)
                {points6_aux =  semidense_mapper -> points3D_toprint[semidense_mapper->num_keyframes].clone();}
                if (semidense_mapper->num_keyframes > 2 && semidense_mapper->points3D_toprint[semidense_mapper->num_keyframes-1].rows > 20)
                {points6_aux.push_back(semidense_mapper -> points3D_toprint[semidense_mapper->num_keyframes-1]);}
                if (semidense_mapper->num_keyframes > 3 &&  semidense_mapper->points3D_toprint[semidense_mapper->num_keyframes-2].rows > 20)
                {points6_aux.push_back(semidense_mapper -> points3D_toprint[semidense_mapper->num_keyframes-2]);}



               int using_close_points  = 1;
               if (points6_aux.rows > 1000){points6 = points6_aux.clone();}
               else {using_close_points = 0;points6 = semidense_mapper->points_last_keyframe.clone();}


                float limit_grad = semidense_mapper->limit_grad;
                int reference_image = 0;

                int num_keyframes = semidense_mapper->num_keyframes;

                int window_size = 6;

                int corner =  window_size * 2;
                int discretization = 100;


                cv::Mat R1,R2,C1,C2,t1,t2;

                int reference_camera = 0;
                if (images.getNumberOfImages() > 2 && semidense_mapper->num_keyframes > semidense_mapper->init_keyframes){reference_camera = 1;}

                R1 =  images.Im[reference_camera ]->R;
                R2 =  images.Im[images.getNumberOfImages()-1]->R;
                t1 =  images.Im[reference_camera ]->t;
                t2 =  images.Im[images.getNumberOfImages()-1]->t;



                C1 = -R1.t()*t1;
                C2 = -R2.t()*t2;

                semidense_mapper -> translational_ratio =(fabs(C1.at<float>(0,0) - C2.at<float>(0,0)) + fabs(C1.at<float>(1,0) - C2.at<float>(1,0)) +
                                       fabs(C1.at<float>(2,0) - C2.at<float>(2,0)) )  / semidense_mapper-> mean_value;

                semidense_mapper->previous_images = semidense_mapper->num_cameras_mapping_th/2+1;


                bool optimize_previous_frame  = false;
                int images_size,init_mapping,end_mapping;

                bool little_texture = false;
                /// if there is little texture, we have experimentally observed that keyframes must be inserted faster. Threfore
                /// if little_texture = true then we only use old frames for mapping and now wait for new frames with enough parallax
                /// to create a new keyframe.
                if(semidense_mapper-> num_potential_points <  0.25*images.Im[reference_image]->image_gray.rows/8*
                        images.Im[reference_image]->image_gray.cols/8){
                    little_texture = true;
                }

                if (semidense_mapper->num_keyframes > semidense_mapper->init_keyframes - 1)
                {
                    if (semidense_mapper->num_cameras_mapping % 2 == 0 || little_texture)
                       copy_previous_kf_images(images,pimages_previous_keyframe,semidense_mapper,init_mapping,end_mapping,images_size, optimize_previous_frame);

                     if (semidense_mapper->num_cameras_mapping  >  semidense_mapper->num_cameras_mapping_th_aux)
                     {
                        int init_mapping_aux = init_mapping;

                        for(int i = 0; i < 10; i ++)
                        {
                           copy_previous_kf_images(images,pimages_previous_keyframe,semidense_mapper,init_mapping,end_mapping,images_size, optimize_previous_frame);
                        }
                        init_mapping = init_mapping_aux;
                     }
                }
                else
                {
                    if (semidense_mapper->num_cameras_mapping % 2 == 0 || little_texture)
                    copy_previous_kf_images(images,pimages_previous_keyframe,semidense_mapper,init_mapping,end_mapping,images_size, optimize_previous_frame);
                }


                int  regularization_size = 1;
                ////////////////////////////GET PHOTOMETRIC TERM_SD term_sd ///////////////////////////////////////
                    if (semidense_mapper->do_init_semi > 0.5 )
                    {
                        cv::Mat points_ref_im_sd_init(0,3, CV_32FC1);
                        cv::Mat image_points_byFocal_sd_init(0,3, CV_32FC1);

                        semidense_mapper->points_ref_im_sd = points_ref_im_sd_init.clone();
                        semidense_mapper-> image_points_byFocal_sd = image_points_byFocal_sd_init.clone();

                        semidense_mapper->do_init_semi=0;
                        semidense_mapper->depth_map = semidense_mapper->depth_map*0;

                        cv::Mat inv_depths_aux = semidense_mapper-> inv_depths.clone();
                        cv::Mat depth_map_points_tracked = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);
                        cv::Mat variance_points_tracked = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);
                        cv::Mat points3D_tracked = semidense_mapper->points3D_tracked.clone();
                        cv::Mat points_aux(0,points3D_tracked.cols,CV_32FC1);


                        float points_without_enough_parallax  = 0;
                        for (int i = 0;i<points3D_tracked.rows;i++)
                        {
                            if ( semidense_mapper -> error_tracked_points.at<float>(i,0) < 0.10  && \
                                 (points3D_tracked.at<float>(i,7) == 1 || points_aux.rows < 30) )
                            {
                                points_aux.push_back(points3D_tracked.row(i));
                            }
                            else
                            {
                                if (points3D_tracked.at<float>(i,7) == 0)  points_without_enough_parallax++;
                            }
                        }


                        if (semidense_tracker->use_kinect == 1)
                        {
                            cv::Mat depth_frame;
                            get_depth_image( semidense_tracker,semidense_mapper, images.Im[0]->stamps,depth_frame);

                            depth_frame =  -depth_frame;
                            semidense_mapper->image_depth_keyframe = depth_frame.clone();

                            depth_map_points_tracked = depth_frame.clone();
                            variance_points_tracked = cv::abs(depth_frame);
                        }


                        get_inverse_depth(images,points_aux,inv_depths_aux,semidense_mapper-> depth_step,
                                          reference_image,discretization,semidense_mapper-> mean_value,depth_map_points_tracked,0,variance_points_tracked);

                        semidense_mapper -> depth_map_points_tracked = depth_map_points_tracked.clone();
                        semidense_mapper -> variance_points_tracked = variance_points_tracked.clone();

                        get_inverse_depth(images,points6,semidense_mapper-> inv_depths,semidense_mapper-> depth_step,reference_image,
                                         discretization,semidense_mapper-> mean_value,semidense_mapper->depth_map,using_close_points,variance_points_tracked);


                        cv::Mat gray_image = images.Im[reference_image]->image_gray.clone();

                       cv::Mat GX =gradientX(gray_image,1);
                       cv::Mat GY =gradientY(gray_image,1);


                       cv::Mat G = cv::abs(GX)  + cv::abs(GY);

                       float alpha = 0.05;
                       cv::exp(-G*alpha,G);


                       cv::Mat edges;
                       cv::Mat gray_image_aux = gray_image.clone();
                       gray_image_aux.convertTo(gray_image_aux,CV_8U);
                       cv::Canny(gray_image_aux,edges,50,200,5,true);


                       edges.convertTo(edges,CV_32FC1);


                       cv::Mat G_aux =cv::Mat::zeros(G.rows,G.cols,CV_32FC1) + 255;
                       cv::Mat  gradients(0,1,CV_32FC1);


                       vector<vector< cv::Mat>> gradients_matrix;
                       gradients_matrix.resize(8);
                       for(int i = 0; i<8;i++){
                          gradients_matrix[i].resize(8);
                       }
                       for(int i = 0; i<8;i++){
                             for(int j = 0; j<8;j++){
                                 gradients_matrix[i][j] = gradients.clone();
                             }
                        }


                       int size_y = G.rows / 8;
                       int size_x  = G.cols / 8;


                       float potential_points = 0;
                       float num_potential_points = 0;
                       for (int i=corner; i<G.rows-corner ; i++)
                       {
                           int idx_y = i/size_y;
                           for (int j=corner; j < G.cols-corner;j++)
                           {
                               int idx_x = j/size_x;
                               if (edges.at<float>(i,j) >  100 &  G.at<float>(i,j) < limit_grad)
                               {
                                     G_aux.at<float>(i,j) = 0;
                                     potential_points++;
                                     num_potential_points++;

                                     gradients.push_back( G.at<float>(i,j) );
                                     gradients_matrix[idx_y][idx_x].push_back(G.at<float>(i,j));
                               }
                           }
                       }
                       semidense_mapper->num_potential_points = num_potential_points / 10;


/*
                       for(int i = 0; i<8;i++){
                             for(int j = 0; j<8;j++){
                                 cv::sort( gradients_matrix[i][j], gradients_matrix[i][j],CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
                             }
                        }

                       cv::sort(gradients,gradients,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
                       int max_number = 10000;
                       if (max_number >  gradients.rows-1) max_number = gradients.rows-1;
                       float limit_grad = gradients.at<float>(max_number,0);
                       num_potential_points = 0; potential_points = 0;
                       for (int i=corner; i<G.rows-corner ; i++)
                       {
                           int idx_y = i/size_y;
                           for (int j=corner; j < G.cols-corner;j++)
                           {
                               int idx_x = j/size_x;

                               int max_number_aux = 155;
                               if(max_number_aux > gradients_matrix[idx_y][idx_x].rows-1) max_number_aux = gradients_matrix[idx_y][idx_x].rows-1;
                               if(max_number_aux > 10)
                               limit_grad = gradients_matrix[idx_y][idx_x].at<float>(max_number_aux,0);


                               if (edges.at<float>(i,j) >  100 &  G.at<float>(i,j) < limit_grad )
                               {
                                     G_aux.at<float>(i,j) = 0;
                                     potential_points++;
                                     num_potential_points++;
                                }
                           }
                       }*/
                       //cout << "POTENTIAL   " << num_potential_points << endl;

                        G = G_aux;
                        semidense_mapper->G_expanded = G_aux;
                        semidense_mapper->high_gradient_point = G_aux < limit_grad & gray_image < semidense_mapper -> max_pixel_color ;
                        semidense_mapper->point_was_previously_estimated = G_aux < limit_grad & gray_image < semidense_mapper -> max_pixel_color ;
                        if (semidense_mapper-> num_keyframes >  semidense_mapper -> init_keyframes)
                        {
                            semidense_mapper->point_was_previously_estimated= cv::abs(depth_map_points_tracked) > 0  ;
                        }
                        semidense_mapper->high_gradient_point.convertTo(semidense_mapper->high_gradient_point,CV_32FC1);
                        semidense_mapper->point_was_previously_estimated.convertTo(semidense_mapper->point_was_previously_estimated,CV_32FC1);


                        cv::Mat points_i_sd(0,3, CV_32FC1);
                        cv::Mat point_i_sd(1,3, CV_32FC1);
                        cv::Mat point_ref_im_sd(1,3, CV_32FC1);


                        for (int i = corner; i < images.Im[reference_image]->image.rows-corner; i++)
                        {
                            for (int j = corner; j < images.Im[reference_image]->image.cols-corner; j++)
                            {
                                if (semidense_mapper->high_gradient_point.at<float>(i,j) > 100 &&
                                   (semidense_mapper->point_was_previously_estimated.at<float>(i,j) < 100
                                  ||semidense_mapper->num_keyframes <  semidense_mapper -> init_keyframes+1 ))
                                {
                                    point_i_sd.at<float>(0,0) = (images.Im[reference_image]->cx-j)/images.Im[reference_image]->fx;
                                    point_i_sd.at<float>(0,1) = (i-images.Im[reference_image]->cy)/images.Im[reference_image]->fy;
                                    point_i_sd.at<float>(0,2) = 1;
                                    points_i_sd.push_back(point_i_sd);
                                    semidense_mapper-> image_points_byFocal_sd.push_back(point_i_sd);

                                    point_ref_im_sd.at<float>(0,0) = i;
                                    point_ref_im_sd.at<float>(0,1) = j;
                                    point_ref_im_sd.at<float>(0,2) = 1;
                                    semidense_mapper->points_ref_im_sd.push_back( point_ref_im_sd);
                                }
                            }
                        }

                        //cout << "high_gradient_points_to_map  -> " <<  semidense_mapper->points_ref_im_sd.rows<< endl;
                        if (semidense_mapper->num_keyframes > semidense_mapper->init_keyframes)
                        {
                            semidense_mapper->percentage_reused_points += 1.0*( num_potential_points - semidense_mapper->points_ref_im_sd.rows )/( points3D_tracked.rows - points_without_enough_parallax);
                           // cout << "percentage_reused_points ->  " <<  semidense_mapper->percentage_reused_points / (semidense_mapper -> num_keyframes -semidense_mapper->init_keyframes)<< endl;
                        }

                        vector<cv::Mat>  initial_inv_depth_inEveryCamera_aux(points_i_sd.rows);
                        semidense_mapper-> initial_inv_depth_inEveryCamera_uncertainty= initial_inv_depth_inEveryCamera_aux;
                        semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax = initial_inv_depth_inEveryCamera_aux;


                        points_i_sd = points_i_sd.t();
                        semidense_mapper-> initial_inv_depth_sd = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + semidense_mapper-> inv_depths.at<float>(0,0);
                        semidense_mapper-> initial_inv_depth_sd = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + semidense_mapper-> inv_depths.at<float>(0,0);
                        semidense_mapper-> initial_inv_depth_sd = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + semidense_mapper-> inv_depths.at<float>(0,0);
                        semidense_mapper-> max_inv_depth_initial_seed = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + semidense_mapper-> inv_depths.at<float>(0,0) - 1000;
                        semidense_mapper-> min_inv_depth_initial_seed = cv::Mat::zeros(points_i_sd.cols, 1, CV_32FC1) + semidense_mapper-> inv_depths.at<float>(0,0) + 1000;
                        semidense_mapper-> t_r_ref = cv::repeat(images.Im[reference_image]->t, 1, points_i_sd.cols).clone();
                        semidense_mapper->points_by_depth = points_i_sd.clone();
                        cv::Mat points_i2_sd;

                        for (unsigned int l = 0; l<discretization;l++)
                        {
                            semidense_mapper->point_limits_for_sd.computeError();
                            if (l==0 || l == discretization-1)
                            {
                                points_i2_sd = points_i_sd / semidense_mapper-> inv_depths.at<float>(l,0);
                                points_i2_sd = images.Im[reference_image]->R.t() * (points_i2_sd - semidense_mapper-> t_r_ref);
                                semidense_mapper->point_limits_for_sd.ph_error[l] = points_i2_sd.clone();
                            }
                        }
                    }
                    ////////////////////////////GET PHOTOMETRIC TERM_SD term_sd ///////////////////////////////////////

                int keyframe_obj;

                float camera_translation;
                cv::Mat camera1;

                if (optimize_previous_frame == 0)
                {
                    init_mapping = images.getNumberOfImages()-1;
                    end_mapping =  images.getNumberOfImages();
                }

                cv::Mat sorted_baselines;
                if(pimages_previous_keyframe->getNumberOfImages()>100)
                {
                        cv::Mat baselines(0,1,CV_32FC1);
                        for(int i = 0;i<pimages_previous_keyframe->getNumberOfImages();i++)
                        {
                            R1 = images.Im[reference_image]->R;
                            t1 = images.Im[reference_image]->t;
                            R2 = pimages_previous_keyframe->Im[i]->R;
                            t2 = pimages_previous_keyframe->Im[i]->t;
                            C1 = -R1.t()*t1;
                            C2 = -R2.t()*t2;

                            camera1 = C1.clone();

                            float translational_ratio = (fabs(C1.at<float>(0,0) - C2.at<float>(0,0)) +\
                                                         fabs(C1.at<float>(1,0) - C2.at<float>(1,0)) + \
                                                         fabs(C1.at<float>(2,0) - C2.at<float>(2,0)) )  / fabs(semidense_mapper-> mean_value);
                            baselines.push_back(translational_ratio);
                        }
                        cv::sort(baselines,sorted_baselines,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
                 }

                if (semidense_mapper->do_var_mapping < 0.5)
                {
                    for (int i=init_mapping; i< end_mapping ; i = i+1)
                    {
                                    keyframe_obj=i;
                                    if (images.Im[keyframe_obj]-> is_used_for_mapping == 0 && semidense_mapper->do_var_mapping == 0)
                                    {
                                            R1 = images.Im[reference_image]->R;
                                            t1 = images.Im[reference_image]->t;
                                            R2 = images.Im[keyframe_obj]->R;
                                            t2 = images.Im[keyframe_obj]->t;
                                            C1 = -R1.t()*t1;
                                            C2 = -R2.t()*t2;

                                            camera1 = C1.clone();


                                            float translational_ratio = (fabs(C1.at<float>(0,0) - C2.at<float>(0,0)) +\
                                                                         fabs(C1.at<float>(1,0) - C2.at<float>(1,0)) + \
                                                                  fabs(C1.at<float>(2,0) - C2.at<float>(2,0)) )  / fabs(semidense_mapper-> mean_value);
                                            float camera_motion  = (fabs(C1.at<float>(0,0) - C2.at<float>(0,0)) +\
                                                                    fabs(C1.at<float>(1,0) - C2.at<float>(1,0)) + \
                                                             fabs(C1.at<float>(2,0) - C2.at<float>(2,0)) ) ;

                                            camera_translation = (fabs(C1.at<float>(0,0) - C2.at<float>(0,0)) + fabs(C1.at<float>(1,0) - C2.at<float>(1,0)) + \
                                                                  fabs(C1.at<float>(2,0) - C2.at<float>(2,0)) );

                                            calculate_min_translation(semidense_mapper,semidense_tracker,translational_ratio_th_min);

                                            if (translational_ratio > translational_ratio_th_min && images.Im[keyframe_obj]->error > -1)
                                            {
                                                cv::Mat epipolar_gradients;

                                                semidense_mapper->using_prev_frame = optimize_previous_frame;

                                                get_photometric_errors_matrix_sd_exhaustive(semidense_mapper,images, camera_motion, semidense_mapper-> inv_depths, semidense_mapper->X,\
                                                                                  semidense_mapper->X_gx_ex,semidense_mapper->X_gy_ey,reference_image,
                                                                                  semidense_mapper-> initial_inv_depth_sd, keyframe_obj,  \
                                                                                  semidense_mapper->point_limits_for_sd,semidense_mapper->points_ref_im_sd,discretization,window_size,  \
                                                                                  epipolar_gradients,semidense_mapper-> initial_inv_depth_inEveryCamera_uncertainty, \
                                                                                  semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax,semidense_mapper->points_by_depth, \
                                                                                  semidense_mapper-> t_r_ref,semidense_mapper-> GX, semidense_mapper-> GY, semidense_mapper->num_cameras_mapping,\
                                                                                  semidense_mapper-> max_inv_depth_initial_seed,semidense_mapper-> min_inv_depth_initial_seed);
                                                if (optimize_previous_frame)
                                                {semidense_mapper->frames_previous_keyframe_processed++;}
                                                semidense_mapper->num_cameras_mapping++;

                                                float count_points_converged = 0;
                                                for (int ii=0;ii<semidense_mapper->points_convergence.rows;ii++)
                                                {
                                                    if (semidense_mapper->points_convergence.at<float>(ii,0) > 2)
                                                    {count_points_converged++;}
                                                }

                                                  if (semidense_mapper->num_cameras_mapping > semidense_mapper->num_cameras_mapping_th+1  &&\
                                                          100 * count_points_converged/ semidense_mapper->points_convergence.rows >  semidense_mapper -> minim_points_converged)
                                                 {semidense_mapper->do_var_mapping = 1;}
                                            }
                                    } // if used for mapping
                    } // for init end mapping
                }


                if (semidense_mapper->do_var_mapping > 0.5  )
                {
                         float count_points_converged = 0;
                         for (int i=0;i<semidense_mapper->points_convergence.rows;i++)
                         {
                             if (semidense_mapper->points_convergence.at<float>(i,0)>2)
                             {
                                 count_points_converged++;
                             }
                         }


                        keyframe_obj=init_mapping;
                        R1 = images.Im[reference_image]->R;
                        t1 = images.Im[reference_image]->t;
                        R2 = images.Im[keyframe_obj]->R;
                        t2 = images.Im[keyframe_obj]->t;
                        C1 = -R1.t()*t1;
                        C2 = -R2.t()*t2;
                        camera1 = C1.clone();


                        camera_translation = (fabs(C1.at<float>(0,0) - C2.at<float>(0,0)) +
                                              fabs(C1.at<float>(1,0) - C2.at<float>(1,0)) + \
                                              fabs(C1.at<float>(2,0) - C2.at<float>(2,0)) );
                        float camera_motion = camera_translation;
                        camera_translation = semidense_mapper->mean_value * semidense_mapper->translational_ratio_th_min;


                        semidense_mapper -> convergence = 0;
                        semidense_mapper->frames_previous_keyframe_processed = 0;
                        semidense_mapper->frames_previous_keyframe_used = 0;

                        cv::Mat deviation_inv_depth = cv::Mat::zeros(semidense_mapper-> initial_inv_depth_sd.rows,semidense_mapper-> initial_inv_depth_sd.cols,CV_32FC1);
                        for (unsigned int l = 0; l < discretization; l = l+1)
                        {
                            semidense_mapper->X_gx_ex.ph_error[l] = semidense_mapper->X_gx_ex.ph_error[l]/semidense_mapper->initial_inv_depth_inEveryCamera_largeParallax[l].rows;
                            semidense_mapper->X_gy_ey.ph_error[l] = semidense_mapper->X_gy_ey.ph_error[l]/semidense_mapper->initial_inv_depth_inEveryCamera_largeParallax[l].rows;
                        }

                        cv::Mat gray_image = images.Im[reference_image]->image_gray.clone();
                        cv::Mat depth_map_points_tracked = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);
                        cv::Mat variance_points_tracked = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);

                        depth_map_points_tracked = semidense_mapper -> depth_map_points_tracked;
                        variance_points_tracked = semidense_mapper -> variance_points_tracked;

                        vector<cv::Mat> init_inv_dephts_maps_scale(2);
                        init_inv_dephts_maps_scale[0] = cv::abs(semidense_mapper->depth_map.clone());
                        init_inv_dephts_maps_scale[1] = init_inv_dephts_maps_scale[0].clone()*0;


                        cv::Mat be_outlier= cv::Mat::zeros(semidense_mapper-> initial_inv_depth_sd.rows,semidense_mapper-> initial_inv_depth_sd.cols,CV_32FC1);
                        cv::Mat be_outlier_print = cv::Mat::zeros(semidense_mapper-> initial_inv_depth_sd.rows,semidense_mapper-> initial_inv_depth_sd.cols,CV_32FC1);
                        cv::Mat final_variances = cv::Mat::zeros(semidense_mapper-> initial_inv_depth_sd.rows,semidense_mapper-> initial_inv_depth_sd.cols,CV_32FC1);

                        semidense_mapper->X.ph_error[0] = semidense_mapper->X.ph_error[0]/semidense_mapper->num_cameras_mapping;

                        convergence_test(semidense_mapper,be_outlier,be_outlier_print,deviation_inv_depth,
                                         final_variances,inv_depth_disparity_th,inv_depth_disparity_print_th, camera_motion);

                        cv::Mat G_expanded = semidense_mapper->G_expanded.clone();
                        ///////////////////// Median REGULARIZATION////////////////////////////////////////
                        cv::Mat depths2regularize =  cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);

                        cv::Mat points_ref_im_sd(0,3, CV_32FC1);
                        cv::Mat be_outlier_aux(0,1, CV_32FC1);
                        cv::Mat final_variances_aux(0,1, CV_32FC1);
                        cv::Mat be_outlier_print_aux(0,1, CV_32FC1);
                        cv::Mat initial_inv_depth_sd(0,1, CV_32FC1);
                        cv::Mat deviation_inv_depth_aux(0,1, CV_32FC1);
                        cv::Mat stereo_baseline(0,1, CV_32FC1);
                        cv::Mat image_points_byFocal_sd(0,3, CV_32FC1);
                        cv::Mat point_i_sd(1,3, CV_32FC1);
                        cv::Mat point_ref_im_sd(1,3, CV_32FC1);

                        /// loopcloser
                        cv::Mat final_depth_map = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);
                        /// loopcloser



                        int cont_depths=0;

                        for (int i = corner; i < images.Im[reference_image]->image.rows-corner; i++)
                        {
                            for (int j = corner; j < images.Im[reference_image]->image.cols-corner; j++)
                            {
                                if (semidense_mapper->high_gradient_point.at<float>(i,j) > 100 && \
                                        (semidense_mapper->point_was_previously_estimated.at<float>(i,j) < 100 || \
                                         semidense_mapper->num_keyframes <  semidense_mapper -> init_keyframes +1 ))
                                {
                                    point_ref_im_sd.at<float>(0,0) = i;
                                    point_ref_im_sd.at<float>(0,1) = j;
                                    point_ref_im_sd.at<float>(0,2) = 1;
                                    points_ref_im_sd.push_back( point_ref_im_sd);

                                    point_i_sd.at<float>(0,0) = (images.Im[reference_image]->cx-j)/images.Im[reference_image]->fx;
                                    point_i_sd.at<float>(0,1) = (i-images.Im[reference_image]->cy)/images.Im[reference_image]->fy;
                                    point_i_sd.at<float>(0,2) = 1;
                                    image_points_byFocal_sd.push_back(point_i_sd);

                                    be_outlier_print_aux.push_back(be_outlier_print.at<float>(cont_depths,0));
                                    be_outlier_aux.push_back(be_outlier.at<float>(cont_depths,0));
                                    deviation_inv_depth_aux.push_back(deviation_inv_depth.at<float>(cont_depths,0));

                                    initial_inv_depth_sd.push_back(semidense_mapper-> initial_inv_depth_sd.at<float>(cont_depths,0));
                                    final_variances_aux.push_back(final_variances.at<float>(cont_depths,0));

                                    variance_points_tracked.at<float>(i,j) = final_variances.at<float>(cont_depths,0);

                                    depths2regularize.at<float>(i,j)= semidense_mapper-> initial_inv_depth_sd.at<float>(cont_depths,0);
                                    cont_depths++;

                                    if ( semidense_mapper -> num_keyframes > semidense_mapper -> init_keyframes && (depth_map_points_tracked.at<float>(i,j)) < 0 )
                                    {
                                       depths2regularize.at<float>(i,j)=  1/depth_map_points_tracked.at<float>(i,j);
                                       initial_inv_depth_sd.push_back( 1/depth_map_points_tracked.at<float>(i,j));
                                    }

                                    stereo_baseline.push_back(semidense_mapper->stereo_baseline.at<float>(cont_depths,0));
                                }

                                 if ( semidense_mapper->point_was_previously_estimated.at<float>(i,j) > 100 &&\
                                      semidense_mapper->high_gradient_point.at<float>(i,j) > 100 && \
                                      semidense_mapper->num_keyframes >  semidense_mapper -> init_keyframes )
                                 {

                                     point_ref_im_sd.at<float>(0,0) = i;
                                     point_ref_im_sd.at<float>(0,1) = j;
                                     point_ref_im_sd.at<float>(0,2) = 1;
                                     points_ref_im_sd.push_back( point_ref_im_sd);

                                     point_i_sd.at<float>(0,0) = (images.Im[reference_image]->cx-j)/images.Im[reference_image]->fx;
                                     point_i_sd.at<float>(0,1) = (i-images.Im[reference_image]->cy)/images.Im[reference_image]->fy;
                                     point_i_sd.at<float>(0,2) = 1;
                                     image_points_byFocal_sd.push_back(point_i_sd);

                                     float float_0 = 0;
                                     float float_1 = 1;

                                     final_variances_aux.push_back(variance_points_tracked.at<float>(i,j));

                                     be_outlier_print_aux.push_back(float_0);
                                     be_outlier_aux.push_back(float_0);
                                     deviation_inv_depth_aux.push_back(float_1);

                                     depths2regularize.at<float>(i,j)=  1/depth_map_points_tracked.at<float>(i,j);
                                     initial_inv_depth_sd.push_back( depths2regularize.at<float>(i,j));

                                     stereo_baseline.push_back(camera_motion);
                                }

                                if ( semidense_mapper -> num_keyframes >  semidense_mapper -> init_keyframes && (depth_map_points_tracked.at<float>(i,j)) < 0 )
                                {
                                     depths2regularize.at<float>(i,j)=  1/depth_map_points_tracked.at<float>(i,j);
                                }
                            }
                        }

                        be_outlier_print = be_outlier_print_aux;
                        be_outlier = be_outlier_aux;
                        semidense_mapper-> initial_inv_depth_sd = initial_inv_depth_sd;
                        deviation_inv_depth = deviation_inv_depth_aux;
                        semidense_mapper->points_ref_im_sd = points_ref_im_sd;
                        semidense_mapper->image_points_byFocal_sd = image_points_byFocal_sd;
                        final_variances = final_variances_aux;

                        cont_depths = 0;

                        cv::Mat baseline_convergence = cv::Mat::zeros(semidense_mapper-> initial_inv_depth_sd.rows,1,CV_32FC1);

                        #pragma omp parallel for num_threads(4)
                        for (cont_depths = 0; cont_depths <semidense_mapper-> initial_inv_depth_sd.rows;cont_depths++ )
                        {
                            if (be_outlier.at<float>(cont_depths,0) == 0)
                            {
                                    int j = round(semidense_mapper->points_ref_im_sd.at<float>(cont_depths,1));
                                    int i = round(semidense_mapper->points_ref_im_sd.at<float>(cont_depths,0));

                                    float max_depth = -1000;
                                    float min_depth = +1000;
                                    float mean_depths = 0;
                                    float cont_depths2reg = 0;

                                        for (int ii = i-regularization_size; ii < i+regularization_size+1; ii++)
                                        {
                                            for (int jj = j-regularization_size; jj < j+regularization_size+1; jj++)
                                            {
                                                if ( fabs(depths2regularize.at<float>(ii,jj)) > 0)
                                                {
                                                    if ( fabs(gray_image.at<float>(i,j)-gray_image.at<float>(ii,jj)) < 25)
                                                    {
                                                        if (depths2regularize.at<float>(ii,jj) < min_depth)
                                                        {
                                                            min_depth =  depths2regularize.at<float>(ii,jj);
                                                        }
                                                        if (depths2regularize.at<float>(ii,jj) > max_depth)
                                                        {
                                                            max_depth =  depths2regularize.at<float>(ii,jj);
                                                        }

                                                        mean_depths+=depths2regularize.at<float>(ii,jj);
                                                        cont_depths2reg ++;
                                                    }
                                                }
                                            }
                                        }


                                        if (cont_depths2reg > 1 )
                                        {
                                            if ((    ( max_depth/min_depth) < 0.96  )  )
                                            {
                                                be_outlier_print.at<float>(cont_depths,0) = 1;
                                                final_depth_map.at<float>(i,j) = 0;
                                            }

                                            if (fabs( min_depth-max_depth) /
                                                fabs(final_variances.at<float>(cont_depths,0)) >  spatial_threshold     )
                                            {
                                                be_outlier.at<float>(cont_depths,0) = 1;
                                                be_outlier_print.at<float>(cont_depths,0) = 1;
                                                final_depth_map.at<float>(i,j) = 0;
                                            }

                                           if( semidense_mapper->high_gradient_point.at<float>(i,j) > 100 && \
                                                   semidense_mapper->point_was_previously_estimated.at<float>(i,j) < 100)
                                           {
                                               semidense_mapper-> initial_inv_depth_sd.at<float>(cont_depths,0) = mean_depths / cont_depths2reg ;


                                               /// loopcloser
                                               if (be_outlier_print.at<float>(cont_depths,0) == 0)
                                               {
                                                   final_depth_map.at<float>(i,j) =  (mean_depths / cont_depths2reg);
                                               }
                                               else
                                               {
                                                   final_depth_map.at<float>(i,j) = 0;
                                               }
                                               /// loopcloser
                                           }
                                         } // cont_depths2reg > 2
                                        else
                                        {
                                            be_outlier.at<float>(cont_depths,0) = 1;
                                            be_outlier_print.at<float>(cont_depths,0) = 1;
                                        }

                                        if(fabs(stereo_baseline.at<float>(cont_depths,0)*semidense_mapper-> initial_inv_depth_sd.at<float>(cont_depths,0)) < 0.12 )
                                        {
                                             final_depth_map.at<float>(i,j) = 0;
                                             be_outlier_print.at<float>(cont_depths,0) = 1;
                                        }
                                        else
                                        {
                                            baseline_convergence.at<float>(cont_depths,0) = 1;
                                        }
                                        if (semidense_mapper->point_was_previously_estimated.at<float>(i,j) > 100)
                                        {
                                             baseline_convergence.at<float>(cont_depths,0) = 1;
                                        }
                            } // if be_outlier == 0
                     }
                     ////////////////////////// Median REGULARIZATION////////////////////////////////


                    float scale = 1;
                    cv::Mat inliers_matrix = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);
                    cv::Mat pixel_taken = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);
                    cv::Mat pixel_taken_print = cv::Mat::zeros(images.Im[reference_image]->image.rows,images.Im[reference_image]->image.cols,CV_32FC1);


                    cv::Mat points_aux2(0,9,CV_32FC1);
                    cv::Mat points_aux2_print(0,6,CV_32FC1);

                    cv::Mat additional_points_aux2(0,9,CV_32FC1);
                    cv::Mat additional_points_aux2_print(0,6,CV_32FC1);


                    float number_of_points_estimated = 0;
                    cv::Mat image_i = images.Im[reference_image]->image.clone();
                    semidense_mapper-> t_r_ref = cv::repeat(images.Im[reference_image]->t, 1, semidense_mapper-> initial_inv_depth_sd.rows).clone();

                    for (int l = 0; l < 2; l++)
                    {
                           cv::Mat seed_points_to_print;
                           cv::Mat initial_inv_depth1 = semidense_mapper-> initial_inv_depth_sd.clone();

                           if (l>0)
                           {
                                semidense_mapper-> image_points_byFocal_sd.copyTo(seed_points_to_print);

                                seed_points_to_print.colRange(0,1).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) = semidense_mapper-> image_points_byFocal_sd.colRange(0,1).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) / (initial_inv_depth1*scale);
                                seed_points_to_print.colRange(1,2).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) = semidense_mapper-> image_points_byFocal_sd.colRange(1,2).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) / (initial_inv_depth1*scale);
                                seed_points_to_print.colRange(2,3).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) = semidense_mapper-> image_points_byFocal_sd.colRange(2,3).rowRange(0,semidense_mapper-> image_points_byFocal_sd.rows) / (initial_inv_depth1*scale);

                                seed_points_to_print = seed_points_to_print.t();
                                seed_points_to_print = images.Im[reference_image]->R.t() * (seed_points_to_print - semidense_mapper-> t_r_ref);
                                seed_points_to_print = seed_points_to_print.t();
                           }

                            for (int i= 0; i<semidense_mapper->points_ref_im_sd.rows;i++)
                            {
                                if (l > 0 && be_outlier.at<float>(i,0) < 0.5)
                                {
                                    float n_x_ref = semidense_mapper->points_ref_im_sd.at<float>(i,1);
                                    float n_y_ref =semidense_mapper->points_ref_im_sd.at<float>(i,0);

                                    int count_inlier_neighbours = 0;

                                    if (n_y_ref > 1 && n_y_ref < image_i.rows-2 && n_x_ref > 1 && n_x_ref < image_i.cols-2  )
                                    {
                                        for (int n_x_ref_aux = n_x_ref-1; n_x_ref_aux <= n_x_ref+1; n_x_ref_aux++)
                                        {
                                            for (int n_y_ref_aux = n_y_ref-1; n_y_ref_aux <= n_y_ref+1; n_y_ref_aux++)
                                            {
                                                if (inliers_matrix.at<float>(n_y_ref_aux,n_x_ref_aux) > 0.5 || G_expanded.at<float>(n_y_ref_aux,n_x_ref_aux) > 5)
                                                {
                                                    count_inlier_neighbours++;
                                                }
                                            }
                                        }
                                    }

                                    if (be_outlier.at<float>(i,0) < 0.5 && count_inlier_neighbours > 1 )
                                    {
                                        cv::Mat points_aux(1,points_aux2.cols,CV_32FC1);
                                        points_aux.at<float>(0,0) = seed_points_to_print.at<float>(i,0);
                                        points_aux.at<float>(0,1) = seed_points_to_print.at<float>(i,1);
                                        points_aux.at<float>(0,2) = seed_points_to_print.at<float>(i,2);

                                        float n_x_ref = semidense_mapper->points_ref_im_sd.at<float>(i,1);
                                        float n_y_ref = semidense_mapper->points_ref_im_sd.at<float>(i,0);

                                        float n_y_ref_aux = n_y_ref;
                                        float n_x_ref_aux = n_x_ref;

                                        int color1 = gray_image.at<float>(n_y_ref,n_x_ref);

                                        points_aux.at<float>(0,3) = color1;
                                        points_aux.at<float>(0,4) = color1;
                                        points_aux.at<float>(0,5) = color1;
                                        points_aux.at<float>(0,6) = final_variances.at<float>(i,0);
                                        points_aux.at<float>(0,7) = baseline_convergence.at<float>(i,0);
                                        points_aux.at<float>(0,8) = 1;

                                        if (pixel_taken .at<float>(n_y_ref,n_x_ref) < 1.5)
                                        {
                                            points_aux2.push_back(points_aux);
                                            pixel_taken.at<float>(n_y_ref,n_x_ref) = 2;
                                        }
                                        semidense_mapper -> G_expanded.at<float>(round(n_y_ref),round(n_x_ref)) = 10;


                                        number_of_points_estimated++;

                                        if (  be_outlier_print.at<float>(i,0)<0.5 )
                                        {
                                            int color1 = image_i.at<cv::Vec3b>(n_y_ref_aux,n_x_ref_aux)[2];
                                            int color2 = image_i.at<cv::Vec3b>(n_y_ref_aux,n_x_ref_aux)[1];
                                            int color3 = image_i.at<cv::Vec3b>(n_y_ref_aux,n_x_ref_aux)[0];
                                            points_aux.at<float>(0,3) = color1;
                                            points_aux.at<float>(0,4) = color2;
                                            points_aux.at<float>(0,5) = color3;

                                            semidense_mapper->map_points.push_back(points_aux);
                                            semidense_mapper->local_map_points.push_back(points_aux);

                                            if (pixel_taken_print.at<float>(n_y_ref,n_x_ref) < 1.5 && semidense_mapper->point_was_previously_estimated.at<float>(n_y_ref,n_x_ref) > 100 &&\
                                                    semidense_mapper->high_gradient_point.at<float>(n_y_ref,n_x_ref) > 100)
                                            {
                                                points_aux2_print.push_back(points_aux);
                                                pixel_taken_print.at<float>(n_y_ref,n_x_ref) = 2;
                                                final_depth_map.at<float>(n_y_ref,n_x_ref) = semidense_mapper-> initial_inv_depth_sd.at<float>(i,0);
                                            }
                                        }

                                        for (int n_x_ref_aux = n_x_ref-1; n_x_ref_aux<=n_x_ref+1;n_x_ref_aux++)
                                        {
                                            for (int n_y_ref_aux = n_y_ref-1; n_y_ref_aux<=n_y_ref+1;n_y_ref_aux++)
                                            {
                                                if (pixel_taken.at<float>(n_y_ref_aux,n_x_ref_aux) < 0.5  )
                                                {
                                                        int color1 = gray_image.at<float>(n_y_ref_aux,n_x_ref_aux);

                                                        points_aux.at<float>(0,3) = color1;
                                                        points_aux.at<float>(0,4) = color1;
                                                        points_aux.at<float>(0,5) = color1;

                                                        cv::Mat point_i_sd(1,3, CV_32FC1);
                                                        point_i_sd.at<float>(0,0) = ((images.Im[reference_image]->cx-n_x_ref_aux)/images.Im[reference_image]->fx)/(initial_inv_depth1.at<float>(i,0)*scale);
                                                        point_i_sd.at<float>(0,1) = ((n_y_ref_aux-images.Im[reference_image]->cy)/images.Im[reference_image]->fy)/(initial_inv_depth1.at<float>(i,0)*scale);
                                                        point_i_sd.at<float>(0,2) = 1 / (initial_inv_depth1.at<float>(i,0)*scale);

                                                        points_aux.at<float>(0,0) = point_i_sd.at<float>(0,0);
                                                        points_aux.at<float>(0,1) = point_i_sd.at<float>(0,1);
                                                        points_aux.at<float>(0,2) = point_i_sd.at<float>(0,2);

                                                        additional_points_aux2.push_back(points_aux);
                                                        pixel_taken.at<float>(n_y_ref_aux,n_x_ref_aux) = 1;
                                                } /// pixel taken
                                            } /// loop n_y_ref_Aux
                                        }  /// loop n_x_ref_Aux
                                    }
                                } // if l > 0
                                else
                                {
                                    float n_x_ref = semidense_mapper->points_ref_im_sd.at<float>(i,1);
                                    float n_y_ref = semidense_mapper->points_ref_im_sd.at<float>(i,0);
                                    if ( be_outlier.at<float>(i,0) < 0.5)
                                    {
                                        inliers_matrix.at<float>(n_y_ref,n_x_ref) = 1;
                                    }

                                    if ( be_outlier_print.at<float>(i,0) < 0.5 )
                                    {
                                        init_inv_dephts_maps_scale[1].at<float>(static_cast<int>(n_y_ref),static_cast<int>(n_x_ref)) =
                                                1/fabs(semidense_mapper-> initial_inv_depth_sd.at<float>(i,0));
                                    }
                                }
                        } // for i

                        pixel_taken = inliers_matrix.clone();
                        pixel_taken_print = inliers_matrix.clone();

                        if (num_keyframes > semidense_mapper->init_keyframes)
                        {
                            int size_y = init_inv_dephts_maps_scale[0].rows;
                            int size_x = init_inv_dephts_maps_scale[0].cols;
                            cv::Mat scales(0,1,CV_32FC1);
                            for (int ii = 0; ii < size_y;ii++)
                            {
                                for (int jj = 0; jj < size_x;jj++)
                                {
                                    if ( init_inv_dephts_maps_scale[1].at<float>(ii,jj) > 0 && init_inv_dephts_maps_scale[0].at<float>(ii,jj) > 0)
                                    {
                                        float scale_ratio = init_inv_dephts_maps_scale[0].at<float>(ii,jj)  / init_inv_dephts_maps_scale[1].at<float>(ii,jj);
                                        if (fabs(scale_ratio -1 ) < 0.03)
                                        {scales.push_back(scale_ratio);}
                                    }
                                }
                            }

                            if (l==0)
                            {
                                if (scales.rows > 1000)
                                {
                                    cv::Mat sorted_scales;
                                    cv::sort(cv::abs(scales),sorted_scales,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
                                    scale = 1/sorted_scales.at<float>(round(sorted_scales.rows/2),0);

                                    if (scale > 1.005 || scale < 0.995)
                                    {
                                        scale = 1;
                                    }
                                }
                            }
                        }
                    } // l


                    if (semidense_mapper->num_keyframes > semidense_mapper->init_keyframes)
                    {
                       semidense_mapper->percentage_converged_points += 1.0*( be_outlier.rows - cv::sum(be_outlier)[0] )/ be_outlier.rows;
                       //cout << "Converged_points ->  " <<  semidense_mapper->percentage_converged_points / (semidense_mapper -> num_keyframes -semidense_mapper->init_keyframes)<< endl;
                    }


                    cv::Mat points_no_augmented = points_aux2.clone();

                    cv::Mat t_r_ref ;

                     if ( additional_points_aux2.rows > 100)
                    {
                     /// ADD ADITIONAL (NEIGHBOURING) POINTS TO THE MAP
                        cv::Mat aux_additional_points_aux2 = additional_points_aux2.colRange(0,3);
                        t_r_ref =   cv::repeat(images.Im[reference_image]->t, 1, additional_points_aux2.rows).clone();
                        aux_additional_points_aux2 = images.Im[reference_image]->R.t() * (aux_additional_points_aux2.t() - t_r_ref);
                        aux_additional_points_aux2 = aux_additional_points_aux2.t();
                        for (int i = 0 ; i < additional_points_aux2.rows;i++)
                        {
                            additional_points_aux2.at<float>(i,0) = aux_additional_points_aux2.at<float>(i,0) ;
                            additional_points_aux2.at<float>(i,1) = aux_additional_points_aux2.at<float>(i,1) ;
                            additional_points_aux2.at<float>(i,2) = aux_additional_points_aux2.at<float>(i,2) ;
                        }
                        points_aux2.push_back(additional_points_aux2);
                    }


                    if ( additional_points_aux2_print.rows > 100)
                    {
                        cv::Mat aux_additional_points_aux2_print = additional_points_aux2_print.colRange(0,3);
                        t_r_ref =   cv::repeat(images.Im[reference_image]->t, 1, additional_points_aux2_print.rows).clone();
                        aux_additional_points_aux2_print = images.Im[reference_image]->R.t() * (aux_additional_points_aux2_print.t() - t_r_ref);
                        aux_additional_points_aux2_print = aux_additional_points_aux2_print.t();
                        for (int i = 0 ; i < additional_points_aux2_print.rows;i++)
                        {
                            additional_points_aux2_print.at<float>(i,0) = aux_additional_points_aux2_print.at<float>(i,0) ;
                            additional_points_aux2_print.at<float>(i,1) = aux_additional_points_aux2_print.at<float>(i,1) ;
                            additional_points_aux2_print.at<float>(i,2) = aux_additional_points_aux2_print.at<float>(i,2) ;
                        }

                        points_aux2_print.push_back(additional_points_aux2_print);
                    }

                    cv::Mat point_cloud_toprint = points_aux2_print.clone();

                    if(semidense_tracker->use_kinect == 1)
                    {
                            cv::Mat depth_frame = semidense_mapper->image_depth_keyframe.clone();

                            for (int i = 0; i<depth_frame.rows; i++)
                            {
                                for (int j = 0; j<depth_frame.cols; j++)
                                {
                                    if (isnan(depth_frame.at<float>(i,j))  )
                                    {
                                        depth_frame.at<float>(i,j) = 0;
                                    }else{
                                        if(fabs(depth_frame.at<float>(i,j)) > 0 )
                                        final_depth_map.at<float>(i,j) = 1/depth_frame.at<float>(i,j);
                                    }
                                }
                            }

                            cv::Mat point_i(1,3,CV_32FC1);
                            cv::Mat color_i(1,3,CV_32FC1);
                            cv::Mat point_i_total(0,3,CV_32FC1);
                            cv::Mat color_i_total(0,3,CV_32FC1);

                            for (int i = 0; i < images.Im[reference_image]->image.rows; i++)
                            {
                                for (int j = 0; j < images.Im[reference_image]->image.cols; j++)
                                {
                                    if (fabs(depth_frame.at<float>(i,j)) > 0 && fabs(depth_frame.at<float>(i,j))< 2.25 )
                                    {
                                        point_i.at<float>(0,0) = depth_frame.at<float>(i,j)*(images.Im[reference_image]->cx-j)/(images.Im[reference_image]->fx);
                                        point_i.at<float>(0,1) = depth_frame.at<float>(i,j)*(i-images.Im[reference_image]->cy)/(images.Im[reference_image]->fy);
                                        point_i.at<float>(0,2) = depth_frame.at<float>(i,j);
                                        point_i_total.push_back(point_i);

                                        color_i.at<float>(0,0) = image_i.at<cv::Vec3b>(i,j)[2];
                                        color_i.at<float>(0,1) = image_i.at<cv::Vec3b>(i,j)[1];
                                        color_i.at<float>(0,2) = image_i.at<cv::Vec3b>(i,j)[0];
                                        color_i_total.push_back(color_i);
                                    }
                                }
                            }

                            if (point_i_total.rows > 100)
                            {
                                cv::Mat t_r_ref =   cv::repeat(images.Im[reference_image]->t, 1, point_i_total.rows).clone();
                                point_i_total = images.Im[reference_image]->R.t() * (point_i_total.t() - t_r_ref);
                                point_i_total = point_i_total.t();

                                cv::Mat points_i_total_with_color(point_i_total.rows,6,CV_32FC1);

                                for (int i = 0 ; i < point_i_total.rows;i++)
                                {
                                    points_i_total_with_color.at<float>(i,0) = point_i_total.at<float>(i,0) ;
                                    points_i_total_with_color.at<float>(i,1) = point_i_total.at<float>(i,1) ;
                                    points_i_total_with_color.at<float>(i,2) = point_i_total.at<float>(i,2) ;
                                    points_i_total_with_color.at<float>(i,3) = color_i_total.at<float>(i,0) ;
                                    points_i_total_with_color.at<float>(i,4) = color_i_total.at<float>(i,1) ;
                                    points_i_total_with_color.at<float>(i,5) = color_i_total.at<float>(i,2) ;
                                }
                                point_cloud_toprint = points_i_total_with_color.clone();
                            }
                   }  // if use_kinect


                    /// ADD ADITIONAL (NEIGHBOURING) POINTS TO THE MAP

                    int pyramid_levels = semidense_mapper->get_points_new_map().size();
                    pimages->Im[reference_image]->accurate_sd_map =  semidense_mapper->local_map_points.clone();
                    ///// MAXImo 20000 POINTS
                    vector<cv::Mat> points_new_map_aux(pyramid_levels);

                    for (int i = 0; i<points_aux2.rows;i++)
                    {
                        float points_limit = points_aux2.rows+2;
                        for (int j=pyramid_levels-1; j>-1;j--)
                        {
                            if (((rand() % 1000000 ) / 1000000.0) <  (points_limit / points_aux2.rows))
                            {
                                points_new_map_aux[j].push_back(points_aux2.row(i));
                            }
                            points_limit /= 3;
                        }
                    }

                    points_new_map_aux[pyramid_levels-1] = points_no_augmented.clone();
                    semidense_mapper->set_points_new_map(points_new_map_aux);


                   if(!semidense_mapper->reusing_map && num_keyframes > semidense_mapper->init_keyframes)
                   {
                        /// loopcloser -> look for loop closures and optimize the pose-graph if enough loop closures are found
                        semidense_tracker->loopcloser_obj.compute_keyframe( images.Im[reference_image]->R,\
                                                                            images.Im[reference_image]->t,\
                                                                            images.Im[reference_image]->image,\
                                                                            semidense_mapper->num_keyframes,\
                                                                            final_depth_map,
                                                                            point_cloud_toprint,points_new_map_aux,\
                                                                            images.Im[reference_image]->fx,\
                                                                            images.Im[reference_image]->fy,\
                                                                            images.Im[reference_image]->cx,\
                                                                            images.Im[reference_image]->cy,\
                                                                            images.Im[reference_image]->stamps,\
                                                                            semidense_mapper->num_potential_points);
                         /// loopcloser -> look for loop closures and optimize the pose-graph if enough loop closures are found
                   }

                    for (int l = 0; l<semidense_mapper->X.ph_error.size(); l++)
                    {
                        semidense_mapper->X.ph_error[l].release();
                        semidense_mapper->X_gx_ex.ph_error[l].release();
                        semidense_mapper->X_gy_ey.ph_error[l].release();
                        semidense_mapper->point_limits_for_sd.ph_error[l].release();
                    }

                    //////////////////////////////////////////////////// JOIN CLOSEST MAPS and prepare initialize dense mapper
                    find_closest_keyframes(semidense_mapper,Map,semidense_tracker);
                    join_last_keyframes(pimages,pimages_previous_keyframe,dense_mapper,semidense_mapper);
                    ///////////////////////////////////////////////////////////////

                    semidense_mapper->do_initialization_tracking = 1;


                    if (semidense_mapper->do_var_mapping > 0.5)
                    {
                            semidense_mapper->point_limits_for_sd.ph_error.clear();
                            semidense_mapper->do_var_mapping =0;
                            semidense_mapper->num_cameras_mapping = 0;
                            semidense_mapper->num_keyframes++;
                            semidense_mapper->do_init_semi=1;
                    }

                   semidense_mapper -> points3D_toprint[num_keyframes]=points_aux2_print.clone();
                   semidense_mapper->set_map_points_print(semidense_mapper->map_points);

                } // do_Var_mapping > 0.5

                /// RELEASE MEMORY
                for (int j = 0; j< images.getNumberOfImages(); j = j+1)
                {
                    delete images.Im[j];
                    images.Im[j] = NULL;
                }
                images.Im.clear();
                semidense_mapper->last_frame_mapped =  semidense_mapper->last_frame_tracked;
                /// RELEASE MEMORY
}


void copy_first_and_last_images(Images_class &images, Images_class &images_map,float &mean_value,float &translational_ratio_th_min)
{
    int images_size  = images.getNumberOfImages()-1;
    for (int l = 0 ; l < images_size; l = l+1)
    {

        cv::Mat R1 = images.Im[0]->R;
        cv::Mat t1 = images.Im[0]->t;
        cv::Mat R2 = images.Im[l]->R;
        cv::Mat t2 = images.Im[l]->t;
        cv::Mat C1 = -R1.t()*t1;
        cv::Mat C2 = -R2.t()*t2;

        float translational_ratio = (fabs(C1.at<float>(0,0) - C2.at<float>(0,0)) +\
                                     fabs(C1.at<float>(1,0) - C2.at<float>(1,0)) + \
                                     fabs(C1.at<float>(2,0) - C2.at<float>(2,0)) )  / fabs(mean_value);

        if (l < 1 ||(translational_ratio > translational_ratio_th_min && images.Im[l]-> is_used_for_mapping ==0))
        {
            images_map.computeImage();
            int images_map_size = images_map.getNumberOfImages()-1;

            images_map.Im[images_map_size ]->image = images.Im[l]->image.clone();
            images_map.Im[images_map_size ]->R = images.Im[l]->R.clone();
            images_map.Im[images_map_size ]->image_gray=images.Im[l]->image_gray.clone();
            images_map.Im[images_map_size ]->t = images.Im[l]->t.clone();
            images_map.Im[images_map_size ]->t_r = images.Im[l]->t_r.clone();
            images_map.Im[images_map_size ]->fx = images.Im[l]->fx;
            images_map.Im[images_map_size ]->fy = images.Im[l]->fy;
            images_map.Im[images_map_size ]->cx = images.Im[l]->cx;
            images_map.Im[images_map_size ]->cy = images.Im[l]->cy;
            images_map.Im[images_map_size]->error = images.Im[l]->error;
            images_map.Im[images_map_size]->stamps = images.Im[l]->stamps;
            images_map.Im[images_map_size]->num_keyframes = images.Im[l]-> num_keyframes;
            images_map.Im[images_map_size]->image_number = images.Im[l]-> image_number;

            images_map.Im[images_map_size]->is_used_for_mapping = images.Im[l]-> is_used_for_mapping;
            images.Im[l]-> is_used_for_mapping = 1;

            if (translational_ratio > translational_ratio_th_min) break;
        }
    }
}


void get_photometric_errors_matrix_sd_exhaustive(SemiDenseMapping *semidense_mapper, Images_class  &images,  float camera_motion,cv::Mat &inv_depths, photometric_term &X,\
                                      photometric_term &X_gx_ex, photometric_term &X_gy_ey, int reference_image, cv::Mat &initial_inv_depth , int image_to_be_added, \
                                      photometric_term &all_points,cv::Mat &points_ref_im_sd,int discretization, \
                                      int window_size, cv::Mat &epipolar_gradients, vector<cv::Mat> &initial_inv_depth_inEveryCamera_uncertainty,
                                      vector<cv::Mat> &initial_inv_depth_inEveryCamera_largeParallax, cv::Mat &points_by_depth, cv::Mat &t_r_ref, \
                                      cv::Mat &GX, cv::Mat &GY, int &num_cameras_mapping, cv::Mat &max_inv_depth_initial_seed, cv::Mat &min_inv_depth_initial_seed)
{
            cv::Mat points_i2_sd;
            cv::Mat inv_depths_vector = points_by_depth.clone();
            inv_depths_vector = inv_depths.at<float>(0,0);
            cv::Mat inv_depths_vector_end = points_by_depth.clone();
            inv_depths_vector_end = inv_depths.at<float>(discretization-1,0);
            float sigma_times = 2;

            #pragma omp parallel for num_threads(4)
            for (int i = 0; i < inv_depths_vector.cols; i++)
            {
                if (initial_inv_depth_inEveryCamera_largeParallax[i].rows > 1)
                {
                    float uncertainty;
                    uncertainty = initial_inv_depth_inEveryCamera_uncertainty[i].at<float>(initial_inv_depth_inEveryCamera_uncertainty[i].rows-1,0);
                    float inv_depth_down_initial_seed = max_inv_depth_initial_seed.at<float>(i,0)+sigma_times*uncertainty;
                    float inv_depth_up_initial_seed = min_inv_depth_initial_seed.at<float>(i,0)-sigma_times*uncertainty;

                    inv_depths_vector.at<float>(0,i) = inv_depth_down_initial_seed;
                    inv_depths_vector.at<float>(1,i) = inv_depth_down_initial_seed;
                    inv_depths_vector.at<float>(2,i) = inv_depth_down_initial_seed;

                    inv_depths_vector_end.at<float>(0,i) = inv_depth_up_initial_seed;
                    inv_depths_vector_end.at<float>(1,i) = inv_depth_up_initial_seed;
                    inv_depths_vector_end.at<float>(2,i) = inv_depth_up_initial_seed;
                }
            }


            cv::Mat inv_depths_vector_init = inv_depths_vector.clone();


            /// LINEAR RELATION BETWEEN INVERSE DEPTH AND 3D point
            if (num_cameras_mapping == 0)
            {
                semidense_mapper->points_convergence = cv::Mat::zeros( inv_depths_vector.cols,1,CV_32FC1);
                semidense_mapper->stereo_baseline = cv::Mat::zeros( inv_depths_vector.cols,1,CV_32FC1);

                points_i2_sd = points_by_depth / inv_depths_vector ;
                points_i2_sd = images.Im[reference_image]->R.t() * (points_i2_sd - t_r_ref);
                all_points.ph_error[0] = points_i2_sd.clone();

                cv::Mat init_points3D = points_i2_sd.clone();
                points_i2_sd = points_by_depth / inv_depths_vector_end;
                points_i2_sd = images.Im[reference_image]->R.t() * (points_i2_sd - t_r_ref);
                all_points.ph_error[discretization-1] = points_i2_sd.clone();

                cv::Mat end_points3D = points_i2_sd.clone();
                semidense_mapper->linear_relation_btw_3D_and_inv_depth1   = (end_points3D-init_points3D)/(1/inv_depths_vector_end - 1/inv_depths_vector);
                semidense_mapper->linear_relation_btw_3D_and_inv_depth2   = end_points3D - semidense_mapper->linear_relation_btw_3D_and_inv_depth1.mul(1/inv_depths_vector_end);
            }
            else
            {
                all_points.ph_error[0] = (1/inv_depths_vector).mul(semidense_mapper->linear_relation_btw_3D_and_inv_depth1 ) +
                        semidense_mapper->linear_relation_btw_3D_and_inv_depth2;

                all_points.ph_error[discretization-1] = (1/inv_depths_vector_end).mul(semidense_mapper->linear_relation_btw_3D_and_inv_depth1 ) +
                        semidense_mapper->linear_relation_btw_3D_and_inv_depth2;
            }

            all_points.ph_error[10] = ((1/inv_depths_vector_end + 1/inv_depths_vector)/2).mul(semidense_mapper->linear_relation_btw_3D_and_inv_depth1 ) +
                    semidense_mapper->linear_relation_btw_3D_and_inv_depth2;

            /// LINEAR RELATION BETWEEN INVERSE DEPTH AND 3D point
            ///
            ///
            cv::Mat errores;

            for (unsigned int l = 0; l < discretization; l++)
            {
                errores = cv::Mat::zeros(initial_inv_depth.rows, 1, CV_32FC1);
                if (X.ph_error[l].empty())
                {
                    errores.copyTo(X.ph_error[l]);
                    errores.copyTo(X_gx_ex.ph_error[l]);
                    errores.copyTo(X_gy_ey.ph_error[l]);
                }
            }

            errores = cv::Mat::zeros(initial_inv_depth.rows, 1, CV_32FC1) + 1000;
            cv::Mat t_r = cv::repeat(images.Im[image_to_be_added]->t, 1, initial_inv_depth.rows).clone();

            cv::Mat image_o_gray = images.Im[image_to_be_added]->image_gray.clone();
            cv::Mat image_i_gray = images.Im[reference_image]->image_gray.clone();


            if (num_cameras_mapping == 0)
            {
                GX = cv::abs(gradientX(image_i_gray,1));
                GY = cv::abs(gradientY(image_i_gray,1));

                cv::Mat GX2,GY2;
                cv::pow(GX,2,GX2);
                cv::pow(GY,2,GY2);
                cv::Mat G =(GX2) + (GY2);
                cv::pow(G,0.5,G);

                G = G + 0.001;

                cv::divide(GX, G,GX,1);
                cv::divide(GY, G,GY,1);
            }

            cv::Mat points_i_2;
            cv::Mat points_o;

            all_points.ph_error[0].copyTo(points_i_2);
            points_o  = images.Im[image_to_be_added]->R  * (points_i_2) + t_r;
            points_o.colRange(0,points_o.cols).rowRange(0,1) = points_o.colRange(0,points_o.cols).rowRange(0,1) * images.Im[image_to_be_added]->fx;
            points_o.colRange(0,points_o.cols).rowRange(1,2) = points_o.colRange(0,points_o.cols).rowRange(1,2) * images.Im[image_to_be_added]->fy;
            /////
            cv::Mat inv_X3_init = 1 / points_o.colRange(0,points_o.cols).rowRange(2,3);
            //////
            cv::Mat z_repeat =  cv::repeat(points_o.colRange(0,points_o.cols).rowRange(2,3),3,1);
            cv::divide(points_o, z_repeat,points_o,1);

            points_o.colRange(0,points_o.cols).rowRange(0,1) =  images.Im[image_to_be_added]->cx-points_o.colRange(0,points_o.cols).rowRange(0,1) ;
            points_o.colRange(0,points_o.cols).rowRange(1,2) =  points_o.colRange(0,points_o.cols).rowRange(1,2) + images.Im[image_to_be_added]->cy;

            /////
            cv::Mat xvalues_init = points_o.colRange(0,points_o.cols).rowRange(0,1).clone();
            cv::Mat yvalues_init = points_o.colRange(0,points_o.cols).rowRange(1,2).clone();
            //////
            epipolar_gradients = points_o.rowRange(0,2).clone();



            all_points.ph_error[discretization-1].copyTo(points_i_2);
            points_o  = images.Im[image_to_be_added]->R  * (points_i_2) + t_r;
            points_o.colRange(0,points_o.cols).rowRange(0,1) = points_o.colRange(0,points_o.cols).rowRange(0,1) * images.Im[image_to_be_added]->fx;
            points_o.colRange(0,points_o.cols).rowRange(1,2) = points_o.colRange(0,points_o.cols).rowRange(1,2) * images.Im[image_to_be_added]->fy;
            /////
            cv::Mat inv_X3_end = 1 /  points_o.colRange(0,points_o.cols).rowRange(2,3);
            //////
            z_repeat =  cv::repeat(points_o.colRange(0,points_o.cols).rowRange(2,3),3,1);
            cv::divide(points_o, z_repeat,points_o,1);
            points_o.colRange(0,points_o.cols).rowRange(0,1) =  images.Im[image_to_be_added]->cx-points_o.colRange(0,points_o.cols).rowRange(0,1) ;
            points_o.colRange(0,points_o.cols).rowRange(1,2) =  points_o.colRange(0,points_o.cols).rowRange(1,2) + images.Im[image_to_be_added]->cy;

            ///////////////////////////////


            cv::Mat xvalues_end = points_o.colRange(0,points_o.cols).rowRange(0,1).clone();
            cv::Mat yvalues_end = points_o.colRange(0,points_o.cols).rowRange(1,2).clone();


            /// EPIPOLAR DIRECTION
            epipolar_gradients = cv::abs(epipolar_gradients - points_o.rowRange(0,2));

            cv::Mat epipolar_gradientX2,epipolar_gradientY2;
            cv::Mat epipolar_gradientX = cv::abs(epipolar_gradients.colRange(0,points_o.cols).rowRange(0,1));
            cv::pow(epipolar_gradientX,2,epipolar_gradientX2);
            cv::Mat epipolar_gradientY = cv::abs(epipolar_gradients.colRange(0,points_o.cols).rowRange(1,2));
            cv::pow(epipolar_gradientY,2,epipolar_gradientY2);
            //cv::Mat epipolar_gradientT = epipolar_gradientX+epipolar_gradientY;

            cv::Mat epipolar_gradientT = epipolar_gradientX2 + epipolar_gradientY2;
            cv::pow(epipolar_gradientT,0.5,epipolar_gradientT);

            cv::divide( epipolar_gradientX, epipolar_gradientT, epipolar_gradientX,1);
            cv::divide( epipolar_gradientY, epipolar_gradientT, epipolar_gradientY,1);
            /// EPIPOLAR DIRECTION

            /// calculation of linear relation between 3D points and 2D pixels
            cv::Mat linear_relation1 = (1/inv_X3_end-1/inv_X3_init) / (1/inv_depths_vector_end.rowRange(0,1) - 1/inv_depths_vector_init.rowRange(0,1));
            cv::Mat linear_relation2 = 1/inv_X3_init - linear_relation1.mul(1/inv_depths_vector_init.rowRange(0,1));

            cv::Mat linear_relation3 = (xvalues_end-xvalues_init)/ (inv_X3_end-inv_X3_init);
            cv::Mat linear_relation4 = xvalues_init - linear_relation3.mul(inv_X3_init);

            //cv::Mat linear_relation5 = (yvalues_end-yvalues_init) / (inv_X3_end-inv_X3_init);
            //cv::Mat linear_relation6 = yvalues_init -linear_relation5.mul(inv_X3_init);
            /// calculation of linear relation between 3D points and 2D pixels

            int window_size_end =  window_size+1;

            cv::Mat xvalues_final = xvalues_init.clone();
            cv::Mat yvalues_final = yvalues_init.clone();


            cv::Mat errores_mas_uno = cv::Mat::zeros(initial_inv_depth.rows,1,CV_32FC1) + 1;
            cv::Mat errores_menos_uno = cv::Mat::zeros(initial_inv_depth.rows,1,CV_32FC1) + 1;
            cv::Mat gradient_by_epipolar_x = cv::Mat::zeros(initial_inv_depth.rows,1,CV_32FC1);
            cv::Mat gradient_by_epipolar_y = cv::Mat::zeros(initial_inv_depth.rows,1,CV_32FC1);
            cv::Mat xvalues_final_menos_uno = xvalues_final-1;
            cv::Mat xvalues_final_mas_uno = xvalues_final+1;

            int image_cols = images.Im[reference_image]->image.cols;
            int image_rows = images.Im[reference_image]->image.rows;



            int patch_step_size = 2 ;


            //patch_step_size = 1;


            cv::Mat GX_copy = GX.clone();
            cv::Mat GY_copy = GY.clone();

            #pragma omp parallel for num_threads(4)
            for (int i = 0; i < initial_inv_depth.rows; i++)
            {
                float X_gx_ex_aux = 0.0;
                float X_gy_ey_aux = 0.0;
                int   n_x_ref = points_ref_im_sd.at<float>(i,1);
                int   n_y_ref = points_ref_im_sd.at<float>(i,0);

                X_gx_ex_aux = GX_copy.at<float>(n_y_ref,n_x_ref)*epipolar_gradientX.at<float>(0,i);
                X_gy_ey_aux = GY_copy.at<float>(n_y_ref,n_x_ref)*epipolar_gradientY.at<float>(0,i);
                gradient_by_epipolar_x.at<float>(i,0) += X_gx_ex_aux;
                gradient_by_epipolar_y.at<float>(i,0) += X_gy_ey_aux;

                if (semidense_mapper->points_convergence.at<float>(i,0)  > 2) continue;

                float num_epipolar_search = 0;
                bool epipolar_inside_image = true;

                float xvalues_init1 = xvalues_init.at<float>(0,i);
                float yvalues_init1 = yvalues_init.at<float>(0,i);
                float xvalues_end1 = xvalues_end.at<float>(0,i);
                float yvalues_end1 = yvalues_end.at<float>(0,i);


                if (fabs(static_cast<int>(xvalues_init.at<float>(0,i) - xvalues_end.at<float>(0,i)) )  >
                        fabs(static_cast<int>(yvalues_init.at<float>(0,i) - yvalues_end.at<float>(0,i)) )  )
                {
                    num_epipolar_search  = fabs((xvalues_init1 - xvalues_end1) );
                }
                else
                {
                    num_epipolar_search  =  fabs((yvalues_init1 - yvalues_end1) );
                }


                float slope_x = (xvalues_end1 - xvalues_init1)  / num_epipolar_search;
                float slope_y = (yvalues_end1 - yvalues_init1)  / num_epipolar_search;
                cv::Mat interpolated_values_ref = cv::Mat::zeros(window_size*2+5,1,CV_32FC1);



                float l_init = 0;
                float l_end = num_epipolar_search +1 ;
                float l_opt = (l_init+l_end)/2;

               if (xvalues_init1 > 0 && xvalues_init1 < image_cols-1 && xvalues_end1 > 0 && xvalues_end1 < image_cols-1 &&
                        yvalues_init1 > 0 && yvalues_init1 < image_rows-1 && yvalues_end1 > 0 && yvalues_end1 < image_rows-1)
                {xvalues_end.at<float>(0,i);
                    l_init = 0;
                    l_end = num_epipolar_search+1;
                }
                else
                {
                    float l_init_x = l_init;
                    float l_init_y = l_init;
                    float l_end_x = l_end;
                    float l_end_y = l_end;

                    int count_intersections = 0;

                    cv::Mat intersections(0,1,CV_32FC1);

                    l_init_x = -xvalues_init1 / slope_x;
                    if (l_init_x > l_init && l_init_x < l_end)
                    {
                        count_intersections++;
                        intersections.push_back(l_init_x);
                    }

                    l_init_y = -yvalues_init1 / slope_y;
                    if (l_init_y > l_init && l_init_y < l_end)
                    {
                        count_intersections++;
                        intersections.push_back(l_init_y);
                    }

                    l_end_x = (image_cols-xvalues_init1) / slope_x;
                    if (l_end_x > l_init && l_end_x < l_end)
                    {
                        count_intersections++;
                        intersections.push_back(l_end_x);
                    }

                    l_end_y = (image_rows-yvalues_init1) / slope_y;
                    if (l_end_y > l_init && l_end_y < l_end)
                    {
                        count_intersections++;
                        intersections.push_back(l_end_y);
                    }

                    if (count_intersections == 1)
                    {
                                if (xvalues_init1 > 0 && xvalues_init1 < image_cols-1 && yvalues_init1 > 0 && yvalues_init1 < image_rows-1)
                                {
                                    intersections.push_back(l_init);
                                }
                                else
                                {
                                    intersections.push_back(l_end);
                                }
                                count_intersections++;
                    }

                    if (count_intersections == 2)
                    {
                        if (intersections.at<float>(0,0) < intersections.at<float>(1,0))
                        {
                            l_init = intersections.at<float>(0,0);
                            l_end = intersections.at<float>(1,0);
                        }
                        else
                        {
                            l_end = intersections.at<float>(0,0);
                            l_init = intersections.at<float>(1,0);
                        }
                    }
                 }





               cv::Mat vector_of_errors = cv::Mat::zeros(image_rows,1,CV_32FC1);
               int counter_epipolar_evaluations = 0;
               int counter_epipolar_evaluations_opt = 1;

               cv::Mat interpolated_values_o = cv::Mat::zeros(round(l_end - l_init+30),1,CV_32FC1);

                if (  epipolar_inside_image && l_end > l_init && l_end - l_init < image_i_gray.rows/3)
                {
                    float l = l_init-1;
                    l_end++;
                    while(l <= l_end)
                    {
                         int counter_interpolated_values_ref = 0;
                         float n_x_aux =   (xvalues_init1 + l*slope_x);
                         float n_y_aux =   (yvalues_init1 + l*slope_y);
                         float n_x =  n_x_aux;
                         float n_y =   n_y_aux;

                         if (n_x > window_size*2 && n_x < image_cols-(window_size+1)*2 && n_y > window_size*2 && n_y < image_rows-(window_size+1)*2\
                            && n_x_ref > window_size*2 && n_x_ref< image_cols-(window_size+1)*2 && n_y_ref >window_size*2 && n_y_ref < image_rows-(window_size+1)*2)
                         {
                              float error = 0;

                              int epipolar_counter = l-l_init+window_size;

                              for (int mm=-window_size;mm < window_size_end;mm = mm+patch_step_size)
                              {
                                       n_x =   (xvalues_init1 + (l+mm)*slope_x);
                                       n_y =   (yvalues_init1 + (l+mm)*slope_y);

                                       float r;

                                       if (interpolated_values_o.at<float>(round(epipolar_counter+mm),0) == 0)
                                       {
                                           float x_2 = n_x;
                                           float y_2 = n_y;
                                           if(x_2 > window_size && x_2 < image_cols-(window_size+1) &&
                                                   y_2 > window_size && y_2 < image_rows-(window_size+1))
                                           bilinear_interpolation(image_o_gray,x_2, y_2,r);
                                           interpolated_values_o.at<float>(round(epipolar_counter+mm),0) = r;
                                       }
                                       else
                                       {
                                           r = interpolated_values_o.at<float>(round(epipolar_counter+mm),0);
                                       }

                                       float r3;

                                       if (interpolated_values_ref.at<float>(counter_interpolated_values_ref,0) == 0)
                                       {
                                            float x_2 = n_x_ref+ n_x-n_x_aux;
                                            float y_2 = n_y_ref+ n_y-n_y_aux;
                                            if(x_2 > window_size && x_2 < image_cols-(window_size+1) &&
                                                    y_2 > window_size && y_2 < image_rows-(window_size+1))
                                            bilinear_interpolation(image_i_gray,x_2, y_2,r3);
                                            interpolated_values_ref.at<float>(counter_interpolated_values_ref,0) = r3;
                                       }
                                       else
                                       {
                                            r3 =  interpolated_values_ref.at<float>(counter_interpolated_values_ref,0);
                                       }
                                       counter_interpolated_values_ref++;
                                       error   += fabs(r3 -r);
                                 }

                                  if (error < errores.at<float>(i,0))
                                  {
                                      //int n_y_int = n_y_aux;
                                      //int n_x_int = n_x_aux;
                                      ////if (edges.at<float>((n_y_int),(n_x_int)) == 0 || edges.at<float>((n_y_int+1),(n_x_int)) == 0 \
                                                || edges.at<float>((n_y_int),(n_x_int+1) )== 0  || edges.at<float>((n_y_int+1),(n_x_int+1) == 0 ))
                                      //{
                                          l_opt = l;
                                          errores.at<float>(i,0) = error;
                                          counter_epipolar_evaluations_opt = counter_epipolar_evaluations;
                                      //}
                                  }
                                  vector_of_errors.at<float>(counter_epipolar_evaluations,0) = error;
                                  counter_epipolar_evaluations++;
                       }
                        l++;
                    }// epipolar search

                    l = l_opt;
                    float pos_x =    xvalues_init1 + l*slope_x;
                    float pos_y =    yvalues_init1 + l*slope_y;
                    int n_x =   round(pos_x);
                    int n_y =   round(pos_y);
                    if (n_x > window_size*2 && n_x < image_cols-(window_size+1)*2 && n_y > window_size*2 && n_y < image_rows-(window_size+1)*2\
                       && n_x_ref > window_size*2 && n_x_ref< image_cols-(window_size+1)*2 && n_y_ref >window_size*2 && n_y_ref < image_rows-(window_size+1)*2)
                    {

                           xvalues_final.at<float>(0,i) = pos_x;
                           yvalues_final.at<float>(0,i) = pos_y;

                           float error_mas_1 = 0;
                           float error_menos_1 = 0;
                           float pos_x_mas_uno= xvalues_init1 + (l_opt+1)*slope_x;
                           error_mas_1 = vector_of_errors.at<float>(counter_epipolar_evaluations_opt+1,0);
                           float pos_x_menos_uno = xvalues_init1 + (l_opt-1)*slope_x;
                           error_menos_1 = vector_of_errors.at<float>(counter_epipolar_evaluations_opt-1,0);

                           xvalues_final_mas_uno.at<float>(0,i) = pos_x_mas_uno;
                           xvalues_final_menos_uno.at<float>(0,i) = pos_x_menos_uno;
                           errores_mas_uno.at<float>(i,0) = error_mas_1;
                           errores_menos_uno.at<float>(i,0) = error_menos_1;
                      }
                }
            } // points

            cv::Mat inv_depths_opt =linear_relation1.mul(xvalues_final-linear_relation4) / (linear_relation3 - linear_relation2.mul(xvalues_final-linear_relation4));
            cv::Mat inv_depths_opt_1 =linear_relation1.mul(xvalues_final_mas_uno-linear_relation4) / (linear_relation3 - linear_relation2.mul(xvalues_final_mas_uno-linear_relation4));



            cv::Mat inv_depths_opt_aux = inv_depths_opt.clone();
            cv::Mat inv_depths_opt_mas_uno =linear_relation1.mul(xvalues_final_mas_uno-linear_relation4) / (linear_relation3 - linear_relation2.mul(xvalues_final_mas_uno-linear_relation4));
            cv::Mat inv_depths_opt_menos_uno =linear_relation1.mul(xvalues_final_menos_uno - linear_relation4) / (linear_relation3 - linear_relation2.mul(xvalues_final_menos_uno-linear_relation4));

            inv_depths_opt_aux = inv_depths_opt_aux.t();
            inv_depths_opt_mas_uno = inv_depths_opt_mas_uno.t();
            inv_depths_opt_menos_uno = inv_depths_opt_menos_uno.t();

            cv::Mat update_num = (errores_mas_uno - errores_menos_uno) / (inv_depths_opt_mas_uno - inv_depths_opt_menos_uno );
            cv::Mat update_den1 = (errores_mas_uno - errores) / (inv_depths_opt_mas_uno - inv_depths_opt_aux  );
            cv::Mat update_den2 = (errores - errores_menos_uno) / (inv_depths_opt_aux - inv_depths_opt_menos_uno );
            cv::Mat update_den = (update_den2 - update_den1) / (inv_depths_opt_mas_uno - inv_depths_opt_aux  );

           cv::Mat update = update_num / update_den;
           for (int i = 0; i< inv_depths_opt_aux.rows;i++)
           {
               if (fabs(inv_depths_opt_menos_uno.at<float>(i,0)-inv_depths_opt_mas_uno.at<float>(i,0)) > fabs(update.at<float>(i,0)))
               {
                   inv_depths_opt_aux.at<float>(i,0) += update.at<float>(i,0);
               }
           }
           inv_depths_opt_aux = inv_depths_opt_aux.t();
           inv_depths_opt     = inv_depths_opt_aux.clone();


            inv_depths_opt = inv_depths_opt.t();
            inv_depths_opt_1 = inv_depths_opt_1.t();

            cv::Mat uncertainty_1 = cv::abs(inv_depths_opt - inv_depths_opt_1);


            float patch_size = 0;
            for (int mm=-window_size;mm < window_size_end;mm = mm+patch_step_size)
            {
                //for (int nn = -window_size; nn < window_size_end; nn=nn+patch_step_size)
                {
                    patch_size++;
                }
            }
             errores = errores / patch_size;



             #pragma omp parallel for num_threads(4)
            for (int i = 0; i < initial_inv_depth.rows; i++)
            {
                 /// //1.2*semidense_mapper->num_cameras_mapping_th_aux * semidense_mapper-> translational_ratio_th_min_aux
              if (  (fabs(camera_motion*inv_depths_opt.at<float>(i,0)) > 0.12|| errores.at<float>(i,0) > 25) && num_cameras_mapping > semidense_mapper->num_cameras_mapping_th_aux/2 + 1  ) \
               {semidense_mapper->points_convergence.at<float>(i,0) += 1;}

                if (inv_depths_opt.at<float>(i,0) < 0  && errores.at<float>(i,0) < 25 && fabs(camera_motion*inv_depths_opt.at<float>(i,0)) < 10.4  && fabs(camera_motion*inv_depths_opt.at<float>(i,0)) > 0.005)
                {
                    if ( semidense_mapper->points_convergence.at<float>(i,0)  <= 2 &&   gradient_by_epipolar_y.at<float>(i,0) +  gradient_by_epipolar_x.at<float>(i,0) > 0.30)
                    {
                            if (inv_depths_opt.at<float>(i,0) < inv_depths.at<float>(0,0)*1.05 && inv_depths_opt.at<float>(i,0) > inv_depths.at<float>(discretization-1,0)*0.95  && errores.at<float>(i,0) < 25   )
                            {
                                         X_gx_ex.ph_error[0].at<float>(i,0) += gradient_by_epipolar_x.at<float>(i,0);
                                         X_gy_ey.ph_error[0].at<float>(i,0) += gradient_by_epipolar_y.at<float>(i,0);

                                        semidense_mapper->stereo_baseline.at<float>(i,0) = camera_motion;
                                        if (inv_depths_opt.at<float>(i,0) > max_inv_depth_initial_seed.at<float>(i,0))
                                        {
                                             max_inv_depth_initial_seed.at<float>(i,0) = inv_depths_opt.at<float>(i,0);
                                        }
                                        if (inv_depths_opt.at<float>(i,0) < min_inv_depth_initial_seed.at<float>(i,0))
                                        {
                                             min_inv_depth_initial_seed.at<float>(i,0) = inv_depths_opt.at<float>(i,0);
                                        }

                                        initial_inv_depth_inEveryCamera_largeParallax[i].push_back(inv_depths_opt.at<float>(i,0));
                                        initial_inv_depth_inEveryCamera_uncertainty[i].push_back(uncertainty_1.at<float>(i,0));
                            }
                   }
                }
          }
}



void convergence_test(SemiDenseMapping *semidense_mapper,cv::Mat &be_outlier,
                        cv::Mat &be_outlier_print,cv::Mat &deviation_inv_depth,
                        cv::Mat &final_variances, float inv_depth_disparity_th,
                        float inv_depth_disparity_print_th,float camera_motion)
{

    float counter_converged_points = 0;

    int minim_images  = 2;
    int minim_prev_and_post_images  = 1;



    minim_prev_and_post_images  = 0;
    minim_images  = semidense_mapper->num_cameras_mapping_th_aux / 2;

     #pragma omp parallel for num_threads(4)
     for (int i=0; i<semidense_mapper-> initial_inv_depth_sd.rows; i++)
     {

        if (semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i].rows >= minim_images )
        {

            if (semidense_mapper -> num_keyframes > semidense_mapper->init_keyframes)
            {
                int rows_to_delete = semidense_mapper -> initial_inv_depth_inEveryCamera_largeParallax[i].rows/2;
                semidense_mapper -> initial_inv_depth_inEveryCamera_largeParallax[i] = semidense_mapper -> initial_inv_depth_inEveryCamera_largeParallax[i].rowRange(rows_to_delete, semidense_mapper -> initial_inv_depth_inEveryCamera_largeParallax[i].rows);
                semidense_mapper -> initial_inv_depth_inEveryCamera_uncertainty[i] = semidense_mapper -> initial_inv_depth_inEveryCamera_uncertainty[i].rowRange(rows_to_delete, semidense_mapper -> initial_inv_depth_inEveryCamera_uncertainty[i].rows);
            }


            semidense_mapper-> initial_inv_depth_sd.at<float>(i,0) = semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i].at<float>(semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i].rows-1,0);


            cv::Mat sorted_inv_depths;
            cv::sort(semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i],sorted_inv_depths,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

            cv::Mat sorted_index ;
            cv::sortIdx(semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i],sorted_index,CV_SORT_EVERY_COLUMN +CV_SORT_ASCENDING);

            cv::Mat sorted_variances = semidense_mapper -> initial_inv_depth_inEveryCamera_uncertainty[i].clone();

            for (int l = 0; l < sorted_variances.rows; l++)
            {
                sorted_variances.at<float>(l,0) = semidense_mapper ->  initial_inv_depth_inEveryCamera_uncertainty[i].at<float>(sorted_index.at<int>(l,0),0);
            }


            deviation_inv_depth.at<float>(i,0) = (semidense_mapper->X_gy_ey.ph_error[0].at<float>(i,0)  +\
                                                semidense_mapper->X_gx_ex.ph_error[0].at<float>(i,0));

            int outliers_to_eliminate = round(0.1*sorted_inv_depths.rows);
            sorted_inv_depths= sorted_inv_depths.rowRange(outliers_to_eliminate,sorted_inv_depths.rows-outliers_to_eliminate);

             semidense_mapper-> initial_inv_depth_sd.at<float>(i,0) = sorted_inv_depths.at<float>(sorted_inv_depths.rows/2,0);

            final_variances.at<float>(i,0) = cv::mean(sorted_variances.rowRange(outliers_to_eliminate,sorted_variances.rows-outliers_to_eliminate) )[0];


            be_outlier.at<float>(i,0) = 1;
            be_outlier_print.at<float>(i,0) = 1;


            if ( fabs(sorted_inv_depths.at<float>(0,0) - sorted_inv_depths.at<float>(sorted_inv_depths.rows-1,0))  / final_variances.at<float>(i,0) < inv_depth_disparity_th)
            {
                     be_outlier.at<float>(i,0) = 0;

                    if ( fabs(sorted_inv_depths.at<float>(0,0) - sorted_inv_depths.at<float>(sorted_inv_depths.rows-1,0))  / final_variances.at<float>(i,0) < inv_depth_disparity_print_th )
                    {
                        be_outlier_print.at<float>(i,0) = 0;
                    }
            }
        }
        else
        {
             if (semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i].rows > 1 )
             {
                 cv::Mat sorted_inv_depths;
                 cv::sort(semidense_mapper-> initial_inv_depth_inEveryCamera_largeParallax[i],sorted_inv_depths,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);
                 semidense_mapper-> initial_inv_depth_sd.at<float>(i,0) = sorted_inv_depths.at<float>(round(sorted_inv_depths.rows/2),0);
             }

            be_outlier.at<float>(i,0) = 1;
            be_outlier_print.at<float>(i,0) = 1;
        }
        if (be_outlier_print.at<float>(i,0) == 0)
        counter_converged_points++;
    } // for
}


void find_closest_keyframes(SemiDenseMapping *semidense_mapper,MapShared *Map,SemiDenseTracking *semidense_tracker)
{
    vector<cv::Mat> points_last_keyframes(semidense_tracker->pyramid_levels);

    int c = (int)(semidense_mapper->num_keyframes) / semidense_tracker->local_maps_number;
    int pos_map = (semidense_mapper->num_keyframes) - semidense_tracker->local_maps_number * c;

    if (semidense_mapper->num_keyframes > semidense_mapper->init_keyframes-1)
    {
        for (int j = 0; j <semidense_tracker->pyramid_levels;j++)
        {
           semidense_tracker->set_local_maps(semidense_mapper->get_points_new_map()[j],pos_map*semidense_tracker->pyramid_levels+j);
        }
        //// FIX IT
        semidense_tracker->set_poses_local_maps(-Map->get_R().t()*Map->get_t(),pos_map);
    }
    int local_maps_number_real = 0;
    cv::Mat sorted_distances_local_maps(1,0,CV_32FC1);
    for ( int jj =0;jj< semidense_tracker->local_maps_number;jj=jj+1)
    {
        if ( semidense_tracker->get_poses_local_maps()[jj].rows > 0)
        {
            local_maps_number_real++;
            cv::Mat distance_local_map_aux = semidense_tracker->get_poses_local_maps()[jj]  + Map->get_R().t()*Map->get_t();
            cv::pow(distance_local_map_aux,2,distance_local_map_aux);
            semidense_tracker->set_distances_local_maps(sqrt(cv::sum(distance_local_map_aux)[0]),jj);
            sorted_distances_local_maps.push_back(semidense_tracker->get_distances_local_maps()[jj]);
        }
    }

    cv::sort(sorted_distances_local_maps,sorted_distances_local_maps,CV_SORT_EVERY_COLUMN + CV_SORT_ASCENDING);

    float limit_distance=1000;
    if (semidense_mapper->num_keyframes > semidense_mapper->init_keyframes-1)
    {
        limit_distance  = 1.0001*sorted_distances_local_maps.at<float>(semidense_tracker->local_maps_close_number,0);
    }

    for ( int jj =0;jj< semidense_tracker->local_maps_number;jj=jj+1)
    {
        if (semidense_tracker->get_distances_local_maps()[jj]< limit_distance && semidense_tracker->get_distances_local_maps()[jj] > 0)
        {
            for (int ii = 0; ii<semidense_tracker->pyramid_levels;ii++)
            {
                points_last_keyframes[ii].push_back(semidense_tracker->get_local_maps()[jj*semidense_tracker->pyramid_levels+ii]);
            }
        }
    }

    semidense_tracker->set_points_last_keyframes(points_last_keyframes);
}


void join_last_keyframes(Images_class *images,Images_class *images_previous_keyframe,\
                      DenseMapping *dense_mapper,SemiDenseMapping *semidense_mapper)
{
    int c = (int)(semidense_mapper->num_keyframes) / dense_mapper->points3D4spx.size();
    int pos_map = (semidense_mapper->num_keyframes) - dense_mapper->points3D4spx.size() * c;

    dense_mapper->points3D4spx[pos_map]=semidense_mapper->local_map_points.clone();
    cv::Mat local_map_points_init(0,6,CV_32FC1);
    semidense_mapper->local_map_points = local_map_points_init.clone();

    copy_images_dense(*images,*dense_mapper);
    filter_images(*dense_mapper,4);

    if (semidense_mapper->num_keyframes >  semidense_mapper -> init_keyframes && dense_mapper->get_do_dense() < 0.5)
    {
        cv::Mat local_map_points_aux =  dense_mapper->points3D4spx[0].clone();
        for (int i = 1 ; i < dense_mapper->points3D4spx.size();i++)
        {
            local_map_points_aux.push_back(dense_mapper->points3D4spx[i]);
        }

        dense_mapper->set_last3Dpoints(local_map_points_aux);
        if (dense_mapper->get_last3Dpoints().rows > 1000)
        {dense_mapper->set_do_dense(1);}
    }

    copy_images(*images,*images_previous_keyframe);
    filter_images(*images_previous_keyframe,4);


    semidense_mapper->frames_previous_keyframe_processed = 0;
    semidense_mapper->frames_previous_keyframe_used = 0;
}


void copy_images_dense(Images_class &images, Images_class &images_map)
{
    int images_size  = images.getNumberOfImages()-1;


    for (int l = 0 ; l < images_size-1; l = l+1)
    {
        images_map.computeImage();
        int images_map_size = images_map.getNumberOfImages()-1;

        images_map.Im[images_map_size ]->image = images.Im[l]->image.clone();
        images_map.Im[images_map_size ]->R = images.Im[l]->R.clone();
        images_map.Im[images_map_size ]->image_gray=images.Im[l]->image_gray.clone();
        images_map.Im[images_map_size ]->t = images.Im[l]->t.clone();
        images_map.Im[images_map_size ]->t_r = images.Im[l]->t_r.clone();
        images_map.Im[images_map_size ]->fx = images.Im[l]->fx;
        images_map.Im[images_map_size ]->fy = images.Im[l]->fy;
        images_map.Im[images_map_size ]->cx = images.Im[l]->cx;
        images_map.Im[images_map_size ]->cy = images.Im[l]->cy;
        images_map.Im[images_map_size]->error = images.Im[l]->error;
        images_map.Im[images_map_size]->stamps = images.Im[l]->stamps;
        images_map.Im[images_map_size]->num_keyframes = images.Im[l]-> num_keyframes;
        images_map.Im[images_map_size]->accurate_sd_map =  images.Im[l]-> accurate_sd_map;
    }
}
template <typename T>
void filter_images( T &images_dense, int num_keyframes )
{
    int num_images = images_dense.getNumberOfImages()-1;
    int last_keyframe = images_dense.Im[images_dense.getNumberOfImages()-2]->num_keyframes;

    for (int ii = num_images; ii> -0.5; ii--)
    {
       if (images_dense.Im[ii]->num_keyframes < (last_keyframe-num_keyframes) )
        {
          delete images_dense.Im[ii];
          images_dense.Im.erase(images_dense.Im.begin()+ii);
       }
    }
}

void copy_images(Images_class &images, Images_class &images_map)
{
    int images_size  = images.getNumberOfImages();

    int step = 1;

    for (int l = 0 ; l < images_size-1; l = l+step)
    {
        images_map.computeImage();
        int images_map_size = images_map.getNumberOfImages()-1;

        images_map.Im[images_map_size ]->image = images.Im[l]->image.clone();
        images_map.Im[images_map_size ]->R = images.Im[l]->R.clone();
        images_map.Im[images_map_size ]->image_gray=images.Im[l]->image_gray.clone();
        images_map.Im[images_map_size ]->t = images.Im[l]->t.clone();
        images_map.Im[images_map_size ]->t_r = images.Im[l]->t_r.clone();
        images_map.Im[images_map_size ]->fx = images.Im[l]->fx;
        images_map.Im[images_map_size ]->fy = images.Im[l]->fy;
        images_map.Im[images_map_size ]->cx = images.Im[l]->cx;
        images_map.Im[images_map_size ]->cy = images.Im[l]->cy;
        images_map.Im[images_map_size]->error = images.Im[l]->error;
        images_map.Im[images_map_size]->stamps = images.Im[l]-> stamps;
        images_map.Im[images_map_size]->num_keyframes = images.Im[l]-> num_keyframes;
        images_map.Im[images_map_size]->accurate_sd_map =  images.Im[l]-> accurate_sd_map;
        images_map.Im[images_map_size]->image_number =  images.Im[l]-> image_number;
    }
}
