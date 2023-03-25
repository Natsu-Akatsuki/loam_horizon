#include "imu_processor/data_process.h"
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cmath>
#include "rcpputils/asserts.hpp"
#include <pcl/kdtree/kdtree_flann.h>
#include <opencv2/opencv.hpp>

using Sophus::SE3d;
using Sophus::SO3d;

pcl::PointCloud<pcl::PointXYZINormal>::Ptr laserCloudtmp(
  new pcl::PointCloud<pcl::PointXYZINormal>());

ImuProcess::ImuProcess(rclcpp::Node::SharedPtr nh)
  : nh_(nh), b_first_frame_(true), last_lidar_(nullptr), last_imu_(nullptr) {
  Eigen::Quaterniond q(1, 0, 0, 0);
  Eigen::Vector3d t(0, 0, 0);
  T_i_l = Sophus::SE3d(q, t);
}

ImuProcess::~ImuProcess() {}

void ImuProcess::Reset() {

  b_first_frame_ = true;
  last_lidar_ = nullptr;
  last_imu_ = nullptr;

  gyr_int_.Reset(-1, nullptr);

  cur_pcl_in_.reset(new PointCloudXYZI());
  cur_pcl_un_.reset(new PointCloudXYZI());
}

void ImuProcess::IntegrateGyr(
  const std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> &v_imu) {
  /// Reset gyr integrator
  gyr_int_.Reset(rclcpp::Time(last_lidar_->header.stamp).seconds(), last_imu_);
  /// And then integrate all the imu measurements
  for (const auto &imu: v_imu) {
    gyr_int_.Integrate(imu);
  }
  ROS_INFO("integrate rotation angle [x, y, z]: [%.2f, %.2f, %.2f]",
           gyr_int_.GetRot().angleX() * 180.0 / M_PI,
           gyr_int_.GetRot().angleY() * 180.0 / M_PI,
           gyr_int_.GetRot().angleZ() * 180.0 / M_PI);
}

void ImuProcess::UndistortPcl(const PointCloudXYZI::Ptr &pcl_in_out,
                              double dt_be, const Sophus::SE3d &Tbe) {
  const Eigen::Vector3d &tbe = Tbe.translation();
  Eigen::Vector3d rso3_be = Tbe.so3().log();
  for (auto &pt: pcl_in_out->points) {
    int ring = int(pt.intensity);
    float dt_bi = pt.intensity - ring;

    if (dt_bi == 0) laserCloudtmp->push_back(pt);
    double ratio_bi = dt_bi / dt_be;
    /// Rotation from i-e
    double ratio_ie = 1 - ratio_bi;

    Eigen::Vector3d rso3_ie = ratio_ie * rso3_be;
    SO3d Rie = SO3d::exp(rso3_ie);

    /// Transform to the 'end' frame, using only the rotation
    /// Note: Compensation direction is INVERSE of Frame's moving direction
    /// So if we want to compensate a point at timestamp-i to the frame-e
    /// P_compensate = R_ei * Pi + t_ei
    Eigen::Vector3d tie = ratio_ie * tbe;
    // Eigen::Vector3d tei = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_pt_i(pt.x, pt.y, pt.z);
    Eigen::Vector3d v_pt_comp_e = Rie.inverse() * (v_pt_i - tie);

    /// Undistorted point
    pt.x = v_pt_comp_e.x();
    pt.y = v_pt_comp_e.y();
    pt.z = v_pt_comp_e.z();
  }
}

void ImuProcess::Process(const MeasureGroup &meas) {
  rcpputils::assert_true(!meas.imu.empty());
  rcpputils::assert_true(meas.lidar != nullptr);
  RCLCPP_DEBUG(rclcpp::get_logger("data_process"), "Process lidar at time: %.4f, %lu imu msgs from %.4f to %.4f",
               rclcpp::Time(meas.lidar->header.stamp).seconds(), meas.imu.size(),
               rclcpp::Time(meas.imu.front()->header.stamp).seconds(),
               rclcpp::Time(meas.imu.back()->header.stamp).seconds());

  auto pcl_in_msg = meas.lidar;

  if (b_first_frame_) {
    /// The very first lidar frame

    /// Reset
    Reset();

    /// Record first lidar, and first useful imu
    last_lidar_ = pcl_in_msg;
    last_imu_ = meas.imu.back();

    RCLCPP_WARN(rclcpp::get_logger("data_process"), "The very first lidar frame");

    /// Do nothing more, return
    b_first_frame_ = false;
    return;
  }

  /// Integrate all input imu message
  IntegrateGyr(meas.imu);

  /// Compensate lidar points with IMU rotation
  //// Initial pose from IMU (with only rotation)
  SE3d T_l_c(gyr_int_.GetRot(), Eigen::Vector3d::Zero());
  dt_l_c_ =
    rclcpp::Time(pcl_in_msg->header.stamp).seconds() - rclcpp::Time(last_lidar_->header.stamp).seconds();
  //// Get input pcl
  pcl::fromROSMsg(*pcl_in_msg, *cur_pcl_in_);

  /// Undistort points

  Sophus::SE3d T_l_be = T_i_l.inverse() * T_l_c * T_i_l;
  pcl::copyPointCloud(*cur_pcl_in_, *cur_pcl_un_);
  UndistortPcl(cur_pcl_un_, dt_l_c_, T_l_be);

  {
    static auto pub_UndistortPcl = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_undistort", 100);
    sensor_msgs::msg::PointCloud2 pcl_out_msg;
    pcl::toROSMsg(*laserCloudtmp, pcl_out_msg);
    pcl_out_msg.header = pcl_in_msg->header;
    pcl_out_msg.header.frame_id = "camera_init";
    pub_UndistortPcl->publish(pcl_out_msg);
    laserCloudtmp->clear();
  }

  {
    static auto pub_UndistortPcl = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_undistort", 100);
    sensor_msgs::msg::PointCloud2 pcl_out_msg;
    pcl::toROSMsg(*cur_pcl_un_, pcl_out_msg);
    pcl_out_msg.header = pcl_in_msg->header;
    pcl_out_msg.header.frame_id = "camera_init";
    pub_UndistortPcl->publish(pcl_out_msg);
  }

  {
    static auto pub_UndistortPcl = nh_->create_publisher<sensor_msgs::msg::PointCloud2>("/livox_undistort", 100);
    sensor_msgs::msg::PointCloud2 pcl_out_msg;
    pcl::toROSMsg(*cur_pcl_in_, pcl_out_msg);
    pcl_out_msg.header = pcl_in_msg->header;
    pcl_out_msg.header.frame_id = "camera_init";
    pub_UndistortPcl->publish(pcl_out_msg);
  }

  /// Record last measurements
  last_lidar_ = pcl_in_msg;
  last_imu_ = meas.imu.back();
  cur_pcl_in_.reset(new PointCloudXYZI());
  cur_pcl_un_.reset(new PointCloudXYZI());
}
