cmake_minimum_required(VERSION 2.8.3)
project(rgbdtam)
add_definitions(-w -Wall -std=c++11 -Werror=return-type ${PCL_DEFINITIONS})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake)
set(CMAKE_BUILD_TYPE RelWithDebInfo)


LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(cmake_modules REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs image_transport cv_bridge tf)

find_package(PCL 1.7.2 REQUIRED)

set(CMAKE_BUILD_TYPE Debug)
catkin_package(CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge tf)

set(DENSE_SOURCE_FILES     
		src/SemiDenseMapping.cpp
                src/DenseMapping.cpp
                src/SemiDenseTracking.cpp
                src/vo_system.cpp
                src/superpixel.cpp
                src/loopcloser.cpp
                src/posegraph_Optimizer.cpp
                src/main.cpp
)

set(DENSE_HEADER_FILES  
		include/rgbdtam/SemiDenseMapping.h
                include/rgbdtam/DenseMapping.h
                include/rgbdtam/SemiDenseTracking.h
                include/rgbdtam/vo_system.h
                include/rgbdtam/superpixel.h
                include/rgbdtam/keyframe.h
                include/rgbdtam/posegraph_Optimizer.h
                include/rgbdtam/loopcloser.h
 )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Cholmod REQUIRED)

include_directories(
${PROJECT_SOURCE_DIR}
${EIGEN3_INCLUDE_DIR}
${CHOLMOD_INCLUDE_DIR}
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/ThirdParty/DBoW2/DBoW2
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


find_package(OpenMP)
if(OPENMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()


option(OPTIMIZATION "Enable optimization" TRUE)
if(OPTIMIZATION)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
message("OPTIMIZATION FLAG ACTIVATED")
else()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
message("NO OPTIMIZATION FLAGS")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra -pedantic  -Wcast-qual -Wpointer-arith -Winit-self -Wswitch-default -Wmissing-include-dirs -Wold-style-cast -Wnon-virtual-dtor -Wshadow -Wno-unknown-pragmas")


link_directories(${PCL_LIBRARY_DIRS})
###########
## Build ##
###########
add_executable(rgbdtam ${DENSE_HEADER_FILES} ${DENSE_SOURCE_FILES})
target_link_libraries(rgbdtam ${OpenCV_LIBS} ${catkin_LIBRARIES}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PCL_LIBRARIES}
cholmod
${PROJECT_SOURCE_DIR}/ThirdParty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2o_core.so
${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2o_solver_cholmod.so
${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2o_solver_dense.so
${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2o_stuff.so
${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2o_types_sba.so
${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2o_types_sim3.so
${PROJECT_SOURCE_DIR}/ThirdParty/g2o/lib/libg2o_types_slam3d.so
)
 install(PROGRAMS
  scripts/run_bag
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
