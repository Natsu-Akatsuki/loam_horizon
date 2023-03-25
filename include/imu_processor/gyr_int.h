#ifndef LOAM_HORIZON_GYR_INT_H
#define LOAM_HORIZON_GYR_INT_H
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include "sophus/so3.hpp"

class GyrInt {
 public:
  GyrInt();
  void Integrate(const sensor_msgs::msg::Imu::ConstSharedPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::msg::Imu::ConstSharedPtr &lastimu);

  const Sophus::SO3d GetRot() const;

 private:
  // Sophus::SO3d r_;
  /// last_imu_ is
  double start_timestamp_;
  sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::vector<sensor_msgs::msg::Imu::ConstSharedPtr> v_imu_;
  std::vector<Sophus::SO3d> v_rot_;
};

#endif  // LOAM_HORIZON_GYR_INT_H
