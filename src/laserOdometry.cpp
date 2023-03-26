// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               Livox@gmail.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <mutex>
#include <queue>

#include "lidarFactor.hpp"
#include "loam_horizon/common.h"
#include "loam_horizon/tic_toc.h"

#define DISTORTION 0 // Low-speed scene, without distortion correction

int corner_correspondence = 0, plane_correspondence = 0;

constexpr double SCAN_PERIOD = 0.1;
constexpr double DISTANCE_SQ_THRESHOLD = 25;
constexpr double NEARBY_SCAN = 2.5;

int skipFrameNum = 5;
bool systemInited = false;

uint64_t timeCornerPointsSharp = 0;
uint64_t timeCornerPointsLessSharp = 0;
uint64_t timeSurfPointsFlat = 0;
uint64_t timeSurfPointsLessFlat = 0;
uint64_t timeLaserCloudFullRes = 0;

pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerLast(
  new pcl::KdTreeFLANN<PointType>());
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfLast(
  new pcl::KdTreeFLANN<PointType>());

pcl::PointCloud<PointType>::Ptr cornerPointsSharp(
  new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr cornerPointsLessSharp(
  new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsFlat(
  new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr surfPointsLessFlat(
  new pcl::PointCloud<PointType>());

pcl::PointCloud<PointType>::Ptr laserCloudCornerLast(
  new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudSurfLast(
  new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudFullRes(
  new pcl::PointCloud<PointType>());

int laserCloudCornerLastNum = 0;
int laserCloudSurfLastNum = 0;

// Transformation from current frame to world frame
Eigen::Quaterniond q_w_curr(1, 0, 0, 0);
Eigen::Vector3d t_w_curr(0, 0, 0);

// q_curr_last(x, y, z, w), t_curr_last
double para_q[4] = {0, 0, 0, 1};
double para_t[3] = {0, 0, 0};

Eigen::Map<Eigen::Quaterniond> q_last_curr(para_q);
Eigen::Map<Eigen::Vector3d> t_last_curr(para_t);

std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cornerSharpBuf;
std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> cornerLessSharpBuf;
std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> surfFlatBuf;
std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> surfLessFlatBuf;
std::queue<sensor_msgs::msg::PointCloud2::ConstSharedPtr> fullPointsBuf;
std::mutex mBuf;

// undistort lidar point
void TransformToStart(PointType const *const pi, PointType *const po) {
  // interpolation ratio
  double s;
  if (DISTORTION)
    s = (pi->intensity - int(pi->intensity)) * 10;
  else
    s = 1.0;
  // s = 1;
  Eigen::Quaterniond q_point_last =
    Eigen::Quaterniond::Identity().slerp(s, q_last_curr);
  Eigen::Vector3d t_point_last = s * t_last_curr;
  Eigen::Vector3d point(pi->x, pi->y, pi->z);
  Eigen::Vector3d un_point = q_point_last * point + t_point_last;

  po->x = un_point.x();
  po->y = un_point.y();
  po->z = un_point.z();
  po->intensity = pi->intensity;
}

// transform all lidar points to the start of the next frame

void TransformToEnd(PointType const *const pi, PointType *const po) {
  // undistort point first
  PointType un_point_tmp;
  TransformToStart(pi, &un_point_tmp);

  Eigen::Vector3d un_point(un_point_tmp.x, un_point_tmp.y, un_point_tmp.z);
  Eigen::Vector3d point_end = q_last_curr.inverse() * (un_point - t_last_curr);

  po->x = point_end.x();
  po->y = point_end.y();
  po->z = point_end.z();

  // Remove distortion time info
  po->intensity = pi->intensity;
}

void laserCloudSharpHandler(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cornerPointsSharp2) {
  mBuf.lock();
  cornerSharpBuf.push(cornerPointsSharp2);
  mBuf.unlock();
}

void laserCloudLessSharpHandler(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cornerPointsLessSharp2) {
  mBuf.lock();
  cornerLessSharpBuf.push(cornerPointsLessSharp2);
  mBuf.unlock();
}

void laserCloudFlatHandler(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &surfPointsFlat2) {
  mBuf.lock();
  surfFlatBuf.push(surfPointsFlat2);
  mBuf.unlock();
}

void laserCloudLessFlatHandler(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &surfPointsLessFlat2) {
  mBuf.lock();
  surfLessFlatBuf.push(surfPointsLessFlat2);
  mBuf.unlock();
}

// receive all point cloud
void laserCloudFullResHandler(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr &laserCloudFullRes2) {
  mBuf.lock();
  fullPointsBuf.push(laserCloudFullRes2);
  mBuf.unlock();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("laserOdometry");
  skipFrameNum = node->declare_parameter("mapping_skip_frame", 2); // Horizon has 6 scan lines
  printf("Mapping %d Hz \n", 10 / skipFrameNum);

  auto subCornerPointsSharp = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/laser_cloud_sharp", 100, laserCloudSharpHandler);
  auto subCornerPointsLessSharp = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/laser_cloud_less_sharp", 100, laserCloudLessSharpHandler);
  auto subSurfPointsFlat = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/laser_cloud_flat", 100, laserCloudFlatHandler);
  auto subSurfPointsLessFlat = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/laser_cloud_less_flat", 100, laserCloudLessFlatHandler);
  auto subLaserCloudFullRes = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/velodyne_cloud_2", 100, laserCloudFullResHandler);
  auto pubLaserCloudCornerLast = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/laser_cloud_corner_last", 100);
  auto pubLaserCloudSurfLast = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/laser_cloud_surf_last", 100);
  auto pubLaserCloudFullRes = node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/velodyne_cloud_3", 100);
  auto pubLaserOdometry = node->create_publisher<nav_msgs::msg::Odometry>(
    "/laser_odom_to_init", 100);
  auto pubLaserPath = node->create_publisher<nav_msgs::msg::Path>(
    "/laser_odom_path", 100);

  nav_msgs::msg::Path laserPath;

  int frameCount = 0;
  rclcpp::Rate rate(100);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);

    if (!cornerSharpBuf.empty() && !cornerLessSharpBuf.empty() &&
        !surfFlatBuf.empty() && !surfLessFlatBuf.empty() &&
        !fullPointsBuf.empty()) {
      timeCornerPointsSharp = rclcpp::Time(cornerSharpBuf.front()->header.stamp).nanoseconds();
      timeCornerPointsLessSharp = rclcpp::Time(
        cornerLessSharpBuf.front()->header.stamp).nanoseconds();
      timeSurfPointsFlat = rclcpp::Time(surfFlatBuf.front()->header.stamp).nanoseconds();
      timeSurfPointsLessFlat = rclcpp::Time(surfLessFlatBuf.front()->header.stamp).nanoseconds();
      timeLaserCloudFullRes = rclcpp::Time(fullPointsBuf.front()->header.stamp).nanoseconds();

      if (timeCornerPointsSharp != timeLaserCloudFullRes ||
          timeCornerPointsLessSharp != timeLaserCloudFullRes ||
          timeSurfPointsFlat != timeLaserCloudFullRes ||
          timeSurfPointsLessFlat != timeLaserCloudFullRes) {
        printf("unsync messeage!");
        // ROS_BREAK(); todo：无等价API
      }

      mBuf.lock();
      cornerPointsSharp->clear();
      pcl::fromROSMsg(*cornerSharpBuf.front(), *cornerPointsSharp);
      cornerSharpBuf.pop();

      cornerPointsLessSharp->clear();
      pcl::fromROSMsg(*cornerLessSharpBuf.front(), *cornerPointsLessSharp);
      cornerLessSharpBuf.pop();

      surfPointsFlat->clear();
      pcl::fromROSMsg(*surfFlatBuf.front(), *surfPointsFlat);
      surfFlatBuf.pop();

      surfPointsLessFlat->clear();
      pcl::fromROSMsg(*surfLessFlatBuf.front(), *surfPointsLessFlat);
      surfLessFlatBuf.pop();

      laserCloudFullRes->clear();
      pcl::fromROSMsg(*fullPointsBuf.front(), *laserCloudFullRes);
      fullPointsBuf.pop();
      mBuf.unlock();

      TicToc t_whole;
      // initializing
      if (!systemInited) {
        systemInited = true;
        std::cout << "Initialization finished \n";
      } else {
        int cornerPointsSharpNum = cornerPointsSharp->points.size();
        int surfPointsFlatNum = surfPointsFlat->points.size();

        TicToc t_opt;
        for (size_t opti_counter = 0; opti_counter < 2; ++opti_counter) {
          corner_correspondence = 0;
          plane_correspondence = 0;

          // ceres::LossFunction *loss_function = NULL;
          ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
          ceres::LocalParameterization *q_parameterization =
            new ceres::EigenQuaternionParameterization();
          ceres::Problem::Options problem_options;

          ceres::Problem problem(problem_options);
          problem.AddParameterBlock(para_q, 4, q_parameterization);
          problem.AddParameterBlock(para_t, 3);

          PointType pointSel;
          std::vector<int> pointSearchInd;
          std::vector<float> pointSearchSqDis;

          TicToc t_data;
          for (int i = 0; i < cornerPointsSharpNum; ++i) {
            TransformToStart(&(cornerPointsSharp->points[i]), &pointSel);
            kdtreeCornerLast->nearestKSearch(pointSel, 5, pointSearchInd,
                                             pointSearchSqDis);

            if (pointSearchSqDis[4] < DISTANCE_SQ_THRESHOLD) {
              std::vector<Eigen::Vector3d> nearCorners;
              Eigen::Vector3d center(0, 0, 0);
              for (int j = 0; j < 5; j++) {
                Eigen::Vector3d tmp(
                  laserCloudCornerLast->points[pointSearchInd[j]].x,
                  laserCloudCornerLast->points[pointSearchInd[j]].y,
                  laserCloudCornerLast->points[pointSearchInd[j]].z);
                center = center + tmp;
                nearCorners.push_back(tmp);
              }
              center = center / 5.0;

              Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();
              for (int j = 0; j < 5; j++) {
                Eigen::Matrix<double, 3, 1> tmpZeroMean =
                  nearCorners[j] - center;
                covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
              }

              Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

              // if is indeed line feature
              // note Eigen library sort eigenvalues in increasing order
              Eigen::Vector3d unit_direction = saes.eigenvectors().col(2);
              // Eigen::Vector3d curr_point(pointOri.x, pointOri.y, pointOri.z);
              if (saes.eigenvalues()[2] > 3 * saes.eigenvalues()[1]) {
                Eigen::Vector3d point_on_line = center;
                Eigen::Vector3d last_point_a, last_point_b;
                last_point_a = 0.1 * unit_direction + point_on_line;
                last_point_b = -0.1 * unit_direction + point_on_line;

                Eigen::Vector3d curr_point(cornerPointsSharp->points[i].x,
                                           cornerPointsSharp->points[i].y,
                                           cornerPointsSharp->points[i].z);

                double s;
                if (DISTORTION)
                  s = (cornerPointsSharp->points[i].intensity - int(cornerPointsSharp->points[i].intensity)) * 10;
                else
                  s = 1.0;

                // printf(" Edge s------ %f  \n", s);
                ceres::CostFunction *cost_function = LidarEdgeFactor::Create(
                  curr_point, last_point_a, last_point_b, s);
                problem.AddResidualBlock(cost_function, loss_function, para_q,
                                         para_t);
                corner_correspondence++;
              }
            }
          }

          // find correspondence for plane features
          for (int i = 0; i < surfPointsFlatNum; ++i) {
            TransformToStart(&(surfPointsFlat->points[i]), &pointSel);
            kdtreeSurfLast->nearestKSearch(pointSel, 5, pointSearchInd,
                                           pointSearchSqDis);

            Eigen::Matrix<double, 5, 3> matA0;
            Eigen::Matrix<double, 5, 1> matB0 =
              -1 * Eigen::Matrix<double, 5, 1>::Ones();
            if (pointSearchSqDis[4] < DISTANCE_SQ_THRESHOLD) {
              for (int j = 0; j < 5; j++) {
                matA0(j, 0) = laserCloudSurfLast->points[pointSearchInd[j]].x;
                matA0(j, 1) = laserCloudSurfLast->points[pointSearchInd[j]].y;
                matA0(j, 2) = laserCloudSurfLast->points[pointSearchInd[j]].z;
                // printf(" pts %f %f %f ", matA0(j, 0), matA0(j, 1), matA0(j,
                // 2));
              }
              // find the norm of plane
              Eigen::Vector3d norm = matA0.colPivHouseholderQr().solve(matB0);
              double negative_OA_dot_norm = 1 / norm.norm();
              norm.normalize();

              // Here n(pa, pb, pc) is unit norm of plane
              bool planeValid = true;
              for (int j = 0; j < 5; j++) {
                // if OX * n > 0.2, then plane is not fit well
                if (fabs(norm(0) *
                         laserCloudSurfLast->points[pointSearchInd[j]].x +
                         norm(1) *
                         laserCloudSurfLast->points[pointSearchInd[j]].y +
                         norm(2) *
                         laserCloudSurfLast->points[pointSearchInd[j]].z +
                         negative_OA_dot_norm) > 0.02) {
                  planeValid = false;
                  break;
                }
              }

              if (planeValid) {
                Eigen::Vector3d curr_point(surfPointsFlat->points[i].x,
                                           surfPointsFlat->points[i].y,
                                           surfPointsFlat->points[i].z);
                Eigen::Vector3d last_point_a(
                  laserCloudSurfLast->points[pointSearchInd[0]].x,
                  laserCloudSurfLast->points[pointSearchInd[0]].y,
                  laserCloudSurfLast->points[pointSearchInd[0]].z);
                Eigen::Vector3d last_point_b(
                  laserCloudSurfLast->points[pointSearchInd[2]].x,
                  laserCloudSurfLast->points[pointSearchInd[2]].y,
                  laserCloudSurfLast->points[pointSearchInd[2]].z);
                Eigen::Vector3d last_point_c(
                  laserCloudSurfLast->points[pointSearchInd[4]].x,
                  laserCloudSurfLast->points[pointSearchInd[4]].y,
                  laserCloudSurfLast->points[pointSearchInd[4]].z);

                double s;
                if (DISTORTION)
                  s = (surfPointsFlat->points[i].intensity - int(surfPointsFlat->points[i].intensity)) * 10;
                else
                  s = 1.0;
                // printf(" Plane s------ %f  \n", s);
                ceres::CostFunction *cost_function = LidarPlaneFactor::Create(
                  curr_point, last_point_a, last_point_b, last_point_c, s);
                problem.AddResidualBlock(cost_function, loss_function, para_q,
                                         para_t);
                plane_correspondence++;
              }
            }
            //}
          }
          // printf("coner_correspondance %d, plane_correspondence %d \n",
          // corner_correspondence, plane_correspondence);
          // printf("data association time %f ms \n", t_data.toc());

          if ((corner_correspondence + plane_correspondence) < 10) {
            printf(
              "less correspondence! "
              "*************************************************\n");
          }

          TicToc t_solver;
          ceres::Solver::Options options;
          options.linear_solver_type = ceres::DENSE_QR;
          options.max_num_iterations = 20;
          options.minimizer_progress_to_stdout = false;
          ceres::Solver::Summary summary;
          ceres::Solve(options, &problem, &summary);
          // printf("solver time %f ms \n", t_solver.toc());
        }
        // printf("optimization twice time %f \n", t_opt.toc());

        t_w_curr = t_w_curr + q_w_curr * t_last_curr;
        q_w_curr = q_w_curr * q_last_curr;
        // std::cout << "t_w_curr: " << t_w_curr.transpose() << std::endl;
      }

      TicToc t_pub;

      // publish odometry
      nav_msgs::msg::Odometry laserOdometry;
      laserOdometry.header.frame_id = "livox_frame";
      laserOdometry.child_frame_id = "laser_odom";
      laserOdometry.header.stamp = rclcpp::Time(static_cast<int64_t>(timeSurfPointsLessFlat));
      laserOdometry.pose.pose.orientation.x = q_w_curr.x();
      laserOdometry.pose.pose.orientation.y = q_w_curr.y();
      laserOdometry.pose.pose.orientation.z = q_w_curr.z();
      laserOdometry.pose.pose.orientation.w = q_w_curr.w();
      laserOdometry.pose.pose.position.x = t_w_curr.x();
      laserOdometry.pose.pose.position.y = t_w_curr.y();
      laserOdometry.pose.pose.position.z = t_w_curr.z();
      pubLaserOdometry->publish(laserOdometry);

      geometry_msgs::msg::PoseStamped laserPose;
      laserPose.header = laserOdometry.header;
      laserPose.pose = laserOdometry.pose.pose;
      laserPath.header.stamp = laserOdometry.header.stamp;
      laserPath.poses.push_back(laserPose);
      laserPath.header.frame_id = "livox_frame";
      pubLaserPath->publish(laserPath);

      // transform corner features and plane features to the scan end point
      if (DISTORTION) {
        int cornerPointsLessSharpNum = cornerPointsLessSharp->points.size();
        for (int i = 0; i < cornerPointsLessSharpNum; i++) {
          TransformToEnd(&cornerPointsLessSharp->points[i],
                         &cornerPointsLessSharp->points[i]);
        }

        int surfPointsLessFlatNum = surfPointsLessFlat->points.size();
        for (int i = 0; i < surfPointsLessFlatNum; i++) {
          TransformToEnd(&surfPointsLessFlat->points[i],
                         &surfPointsLessFlat->points[i]);
        }

        int laserCloudFullResNum = laserCloudFullRes->points.size();
        for (int i = 0; i < laserCloudFullResNum; i++) {
          TransformToEnd(&laserCloudFullRes->points[i],
                         &laserCloudFullRes->points[i]);
        }
      }

      pcl::PointCloud<PointType>::Ptr laserCloudTemp = cornerPointsLessSharp;
      cornerPointsLessSharp = laserCloudCornerLast;
      laserCloudCornerLast = laserCloudTemp;

      laserCloudTemp = surfPointsLessFlat;
      surfPointsLessFlat = laserCloudSurfLast;
      laserCloudSurfLast = laserCloudTemp;

      laserCloudCornerLastNum = laserCloudCornerLast->points.size();
      laserCloudSurfLastNum = laserCloudSurfLast->points.size();

      // std::cout << "the size of corner last is " << laserCloudCornerLastNum
      // << ", and the size of surf last is " << laserCloudSurfLastNum << '\n';

      kdtreeCornerLast->setInputCloud(laserCloudCornerLast);
      kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

      if (frameCount % skipFrameNum == 0) {
        frameCount = 0;

        sensor_msgs::msg::PointCloud2 laserCloudCornerLast2;
        pcl::toROSMsg(*cornerPointsSharp, laserCloudCornerLast2);
        laserCloudCornerLast2.header.stamp = rclcpp::Time(timeSurfPointsLessFlat);
        laserCloudCornerLast2.header.frame_id = "aft_mapped";
        pubLaserCloudCornerLast->publish(laserCloudCornerLast2);

        sensor_msgs::msg::PointCloud2 laserCloudSurfLast2;
        pcl::toROSMsg(*surfPointsFlat, laserCloudSurfLast2);
        laserCloudSurfLast2.header.stamp = rclcpp::Time(timeSurfPointsLessFlat);
        laserCloudSurfLast2.header.frame_id = "aft_mapped";
        pubLaserCloudSurfLast->publish(laserCloudSurfLast2);

        sensor_msgs::msg::PointCloud2 laserCloudFullRes3;
        pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3);
        laserCloudFullRes3.header.stamp = rclcpp::Time(timeSurfPointsLessFlat);
        laserCloudFullRes3.header.frame_id = "aft_mapped";
        pubLaserCloudFullRes->publish(laserCloudFullRes3);
      }
      // printf("publication time %f ms \n", t_pub.toc());
      // printf("whole laserOdometry time %f ms \n \n", t_whole.toc());
      if (t_whole.toc() > 100) {
        RCLCPP_WARN(rclcpp::get_logger("laserOdometry"), "odometry process over 100ms");
      }

      frameCount++;
    }
    rate.sleep();
  }
  return 0;
}
