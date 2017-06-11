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

1-) Download rgbdtam. Rename the main folder 'rgbdtam-master' and name it 'rgbdtam' instead.


2-) Run the script build_thirdparty.sh to compile the ThirdParty directories (segmentation, DBoW2 and g20).
       
    sh build_thirdparty.sh


3-) Compile rgbdtam
      
    catkin_make rgbdtam -j3

# Usage with a '.bag' sequence

Launch rgbdtam from your 'catkin_workspace' folder:
     
    cd root/catkin_workspace 
    rosrun rgbdtam rgbdtam
    

Launch the visualizer of the current frame

    rosrun image_view image_view image:=/rgbdtam/camera/image


Now you need a sequence or a depth camera. For example, download this sequence (ROS bag format) from the TUM dataset: http://vision.in.tum.de/rgbd/dataset/freiburg3/rgbd_dataset_freiburg3_long_office_household.bag.

Use the following command to remove the unnecessary topics from the sequence

    rosbag filter rgbd_dataset_freiburg3_long_office_household.bag new_office_household.bag "topic=='/camera/rgb/image_color' or topic == '/camera/depth/image'"
    
Play the new sequence
    
    rosbag play new_office_household.bag
    

Do not move the visualizer point of view until the sequence has finished (only zoom in/out), or it will get lost otherwise. We will fix this issue in the near future. You can also see the reconstrucction with meshlab after the sequence has finished. Open the following file 'rgbdtam/src/results_depth_maps/reconstruction_after_optimization_total.ply


There are four parameters that you have to modify (before executing a sequence) in rgbdtam/src/data.yml:

1-) Intrinsic parameters of the camera:

'cameraMatrix'

'distCoeffs'

2-) Camera and depth camera topics

'camera_path'

'depth_camera_path'

# Usage with ASUS

Run rgbdtam with the rectified rgb and depth images.

    rosrun rgbdtam rgbdtam /camera/rgb/image_color:=/camera/rgb/image_rect_color /camera/depth/image:=/camera/depth/image_rect

In your oppenni2.launch file set depth_ragistration to true:

arg name="depth_registration" default="true" 

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

5-) Minimize geometric error

use_depth_tracking: [bool]. If false, geometric error will not be minimized (only the photometric error will be minimized). It is recommended to minimize also the geometric error unless RGBDTAM does not work in real-time in your computer. Default: True.

6-) Maximum depth to print

depth_range_print: [float]. If higher far away points will also be displayed. Default: 1.7 (meters).

# Contact

If you have any issue compiling/running rgbdtam or you would like to know anything about the code, please contact the authors:

     Alejo Concha -> aconchabelenguer@gmail.com

     Javier Civera -> jcivera@unizar.es
