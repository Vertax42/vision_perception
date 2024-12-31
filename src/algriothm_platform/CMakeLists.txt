cmake_minimum_required(VERSION 3.0.2)
project(algriothm_platform)


set(CMAKE_BUILD_TYPE "Debug")

add_compile_options(-std=c++14 )
set(CMAKE_CXX_FLAGS "-std=c++14" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) 
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
## add_definitions(-Wall -Werror)

## Find catkin macros and librariesa
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy  
  message_filters
  sensor_msgs
  std_msgs
  geometry_msgs  
  tf
  message_generation
  eigen_conversions
  cv_bridge
  image_transport
)

## Find system libraries



find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
# add_definitions(${PCL_DEFINITIONS}
#                 ${OpenCV_DEFINITIONS})

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include/common_tools include/vision_tools
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport
  DEPENDS OpenCV PCL
  )


###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
                    include  
                    ${catkin_INCLUDE_DIRS}
                    ${EIGEN3_INCLUDE_DIR}
                    ${PCL_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
                    ./include/common_tools/include
                    ./include/vision_tools/include
                    ./third_party/ikd_tree
                    ./third_party/jtjson
                    )

add_library(circle_detector SHARED 
  ./include/vision_tools/circle_detector.cpp
  ./include/vision_tools/vision_common.cpp
  ./include/vision_tools/image.cpp
  ./include/common_tools/utils_common.cpp
  )

# add_dependencies(circle_detector ${circle_detector_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})



target_link_libraries(circle_detector
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  )

add_executable(${PROJECT_NAME}_interface
  src/algriothm_platform_interface.cpp
  include/common_tools/log4z.cpp
  )

target_link_libraries(${PROJECT_NAME}_interface
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  circle_detector
  )

target_compile_definitions(${PROJECT_NAME}_interface PRIVATE ${OpenCV_DEFINITIONS} ${PCL_DEFINITIONS})

link_directories(${PCL_LIBRARY_DIRS}
                 ${OpenCV_LIBRARY_DIRS}
                 )  


#############
## Install ##
#############
