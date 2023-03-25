#ifndef LOAM_HORIZON_DATA_PROCESS_H
#define LOAM_HORIZON_DATA_PROCESS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <fstream>
#include "gyr_int.h"
#include "loam_horizon/common.h"
#include "sophus/se3.hpp"

struct MeasureGroup {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar;
    std::vector<sensor_msgs::msg::Imu::ConstPtr> imu;
};

class ImuProcess {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ImuProcess(rclcpp::Node::SharedPtr node);

    ~ImuProcess();

    void Process(const MeasureGroup &meas);

    void Reset();

    void IntegrateGyr(const std::vector<sensor_msgs::msg::Imu::ConstPtr> &v_imu);

    void UndistortPcl(const PointCloudXYZI::Ptr &pcl_in_out, double dt_be,
                      const Sophus::SE3d &Tbe);

    void set_T_i_l(Eigen::Quaterniond &q, Eigen::Vector3d &t) {
      T_i_l = Sophus::SE3d(q, t);
    }

    std::shared_ptr<rclcpp::Node> nh_;

private:
    /// Whether is the first frame, init for first frame
    bool b_first_frame_ = true;

    //// Input pointcloud
    PointCloudXYZI::Ptr cur_pcl_in_;
    //// Undistorted pointcloud
    PointCloudXYZI::Ptr cur_pcl_un_;

    double dt_l_c_;

    /// Transform form lidar to imu
    Sophus::SE3d T_i_l;
    //// For timestamp usage
    sensor_msgs::msg::PointCloud2::ConstSharedPtr last_lidar_;
    sensor_msgs::msg::Imu::ConstSharedPtr last_imu_;

    /// For gyroscope integration
    GyrInt gyr_int_;
};

#endif  // LOAM_HORIZON_DATA_PROCESS_H
