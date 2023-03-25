cmake_minimum_required(VERSION 3.16.0)
project(loam_horizon)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14 -fext-numeric-literals")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
find_package(ament_cmake_auto REQUIRED)
# automatically link the dependency according to the xml (without find_package)
ament_auto_find_build_dependencies()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS})


# 生成目标文件
ament_auto_add_executable(scanRegistration src/scanRegistration.cpp)
target_link_libraries(scanRegistration ${PCL_LIBRARIES})

ament_auto_add_executable(laserOdometry src/laserOdometry.cpp)
target_link_libraries(laserOdometry ${PCL_LIBRARIES} ${CERES_LIBRARIES})

ament_auto_add_executable(laserMapping src/laserMapping.cpp)
target_link_libraries(laserMapping ${PCL_LIBRARIES} ${CERES_LIBRARIES})

ament_auto_add_executable(livox_repub src/livox_repub.cpp)
target_link_libraries(livox_repub ${PCL_LIBRARIES} ${OpenCV_LIBS})

ament_auto_add_executable(imu_process src/imu_processor/data_process_node.cpp src/imu_processor/data_process.cpp
  src/imu_processor/gyr_int.cpp)
target_link_libraries(imu_process ${PCL_LIBRARIES} ${OpenCV_LIBS})

# replace the export, install and ament_package command
ament_auto_package()
