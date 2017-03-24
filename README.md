# RGBDTAM:

RGBDTAM is a SLAM system that estimates a dense reconstruction of a scene in real-time and on a CPU using RGB-D cameras. 

Related Publication:
[1] Alejo Concha, Javier Civera. RGBDTAM: A cost-effective and accurate RGBD Tracking and Mapping System. https://arxiv.org/pdf/1703.00754.pdf

Video of the results that you should expect:
https://youtu.be/sc-hqtJtHD4

 
# License

RGBDTAM is licensed under the GNU General Public License Version 3 (GPLv3), please see http://www.gnu.org/licenses/gpl.html.

For commercial purposes, please contact the authors.


# Disclaimer

This site and the code provided here are under active development. Even though we try to only release working high quality code, this version might still contain some issues. Please use it with caution.

# Dependencies

ROS:

We have tested RGBDTAM in Ubuntu 14.04 with ROS Indigo.

To install ROS (indigo) use the following command:

     sudo apt-get install ros-indigo-desktop
     
Or check the following link if you have any issue:

    http://wiki.ros.org/indigo/Installation/Ubuntu
     

PCL library for visualization:

     version >= 1.7.2
     
BOOST library to launch the different threads:
    
     sudo apt-get install libboost-all-dev 
     
CHOLMOD, BLAS, LAPACK and Eigen3 to compile g2o:

     sudo apt-get install libsuitesparse-dev
     sudo apt-get install libblas-dev
     sudo apt-get install liblapack-dev
     sudo apt-get install libeigen3-dev   

Vocabulary used for loop closure and relocalization:
    
We have used the vocabulary created by ORB-SLAM authors. Please, download the vocabulary from this link "https://github.com/raulmur/ORB_SLAM/blob/master/Data/ORBvoc.txt.tar.gz" and place it in "ThirdParty/DBoW2/ORBvoc.txt"

    
# Compilation


1-) Run the script build_thirdparty.sh to compile the ThirdParty directories (segmentation, DBoW2 and g20).
       
    sh build_thirdparty.sh


2-) Compile rgbdtam
      
    catkin_make --pkg rgbdtam

# Usage

Launch rgbdtam from your 'catkin_workspace' folder:
     
    cd root/catkin_workspace 
    rosrun rgbdtam rgbdtam
    
Notice that the location of rgbdtam should be the following:

    root/catkin_workspace/src/rgbdtam

Launch the visualizer of the current frame

    rosrun image_view image_view image:=/rgbdtam/camera/image


You can use a sequence from the TUM dataset to test the algorithm:

    rosbag play sequence.bag

Do not move the visualizer point of view until the sequence has finished (only zoom in/out), or it will get lost otherwise. We will fix this issue in the near future.


There are three parameters that you have to modify (before executing a sequence) in rgbdtam/src/data.yml:

1-) Intrinsic parameters of the camera:

'cameraMatrix'

'distCoeffs'

2-) Camera topic

'camera_path'


# Parameters

There are a few tuneable parameters that you can modify in rgbdtam/src/data.yml:


1-) Number of frames for mapping

num_cameras_mapping_th: [int]. Number of frames that you want to use to estimate the depth maps. Default: 10.

2-) Minimum parallax required for mapping

translational_ratio_th_min: [double]. Minimum parallax to insert a keyframe. Default: 0.01. Typical values [0.03-0.15].

3-) Depth - RGB offset

depth_rgb_offset: [double]. Offset between the timestamps of depth and rgb images. Default: -0.020 (TUM dataset).

4-) Do relocalization

use_relocalization: [bool]. If true, the system will try to detect when it is lost and will try to relocalize.

# Contact

If you have any issue compiling/running rgbdtam or you would like to know anything about the code, please contact the authors:

     Alejo Concha -> aconchabelenguer@gmail.com

     Javier Civera -> jcivera@unizar.es
