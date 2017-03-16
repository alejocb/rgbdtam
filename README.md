# RGBDTAM:

RGBDTAM is a SLAM algorithm that estimates a dense reconstruction of a scene in real-time on a CPU using monocular or RGB-D cameras. We are currently testing the code and solving compiling issues, it should be ready in the following days/weeks.

Related Publication:
[1] Alejo Concha, Javier Civera. RGBDTAM: A cost-effective and accurate RGBD Tracking and Mapping System. https://arxiv.org/pdf/1703.00754.pdf

Video of the results that you should expect in the example sequences:
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

Vocabulary used for loop closure and relocalization:
    
     We have used the vocabulary created by ORB-SLAM authors. Please, download the vocabulary from this link "www.github.com/raulmur/ORBvoc.txt.tar.gz" and place it in "ThirdParty/DBoW2/ORBvoc.txt"


# Installation

     git clone  https://github.com/alejocb/rgbdtam.git
    
# Compilation





1-) Efficient Graph-Based Image Segmentation. P. Felzenszwalb, D. Huttenlocher. International Journal of Computer Vision, Vol. 59, No. 2, September 2004

cd root/catkin_workspace/src/rgbdtam/ThirdParty/segment
make




2-) DBow2

cd ThirdParty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j



3-) g2o

cd ThirdParty/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j


4-) catkin_make --pkg rgbdtam

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

There are two parameters that you have to modify (before executing a sequence) in rgbdtam/src/data.yml:

1-) Intrinsic parameters of the camera:

'cameraMatrix'

'distCoeffs'

2-) Camera topic

camera_path:"/image_raw"



Update the the 'camera_path', 'cameraMatrix' and 'distCoeffs'  in the file rgbdtam/src/data.yml  


  

# Parameters

There are a few tuneable parameters that you can modify in rgbdtam/src/data.yml:

 

1-) Superpixel calculation

calculate_superpixels: [bool] If 1 it will calculate 3D superpixels.

2-) Number of frames for mapping

num_cameras_mapping_th: [int]. Number of frames that you want to use to estimate the depth maps. Default: 9.

3-) Minimum parallax required for mapping

translational_ratio_th_min: [double]. Minimum parallax to insert a keyframe. Default: 0.075. Typical values [0.03-0.15].

4-) Degenerated cases in 3D superpixel matching

limit_ratio_sing_val: [double]. This threshold deals with the degenerated cases in 3D superpixel calculation. Smaller values -> less outliers. Default: 100. Typical values [10-1000].

5-) Minimum normalized residual threshold required.

limit_normalized_residual: [double]. This threshold accounts for the minimum error required in superpixel calculation. Smaller values -> less outliers. Default: 0.30. Typical values [0.05-0.50].

6-) Minimum number of matches of 3D superpixels in multiple views to achieve multiview consistency.

matchings_active_search: [int]. Number of matches required of the 3D superpixel in multiple views. Larger values -> less outliers. Default: 3. Typical values [0-4].

7-) Kinect Initialization: 1
kinect_initialization: [bool] If 1 it will use the kinect for initialization.

# Contact

If you have any issue compiling/running rgbdtam or you would like to know anything about the code, please contact the authors:

     Alejo Concha -> aconchabelenguer@gmail.com

     Javier Civera -> jcivera@unizar.es
