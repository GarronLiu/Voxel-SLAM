#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gnss_comm/gnss_utility.hpp>

class GNSSProcess
{
public:
  struct Frame
  {
    double timestamp = -1.0;
    double lidar_timestamp = -1.0;
    int window_index = -1;
    Eigen::Vector3d local_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d local_v = Eigen::Vector3d::Zero();
    std::vector<gnss_comm::ObsPtr> obs;
    std::vector<gnss_comm::EphemBasePtr> ephems;
  };

  struct Options
  {
    int min_obs = 10;
    int min_frames = 4;
    double min_horizontal_velocity = 0.3;
  };

  struct Result
  {
    Eigen::Matrix<double, 7, 1> rough_xyzt = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Matrix<double, 7, 1> refined_xyzt = Eigen::Matrix<double, 7, 1>::Zero();
    Eigen::Vector3d anchor_ecef = Eigen::Vector3d::Zero();
    Eigen::Matrix3d R_ecef_enu = Eigen::Matrix3d::Identity();
    double yaw_enu_local = 0.0;
    double receiver_clock_drift = 0.0;
    std::vector<double> receiver_clock_biases;
    std::vector<int> aligned_frame_indices;
    std::vector<Frame> aligned_frames;
  };

  enum class Status
  {
    kSuccess,
    kInsufficientFrames,
    kInsufficientMotion,
    kMissingIonosphereParameters,
    kCoarseLocalizationFailed,
    kYawAlignmentFailed,
    kAnchorRefinementFailed,
    kNoObservedConstellation
  };

  static Status initialize(std::deque<Frame> &alignment_frames,
                           const std::vector<double> &iono_params,
                           const Options &options,
                           Result &result);

  static const char *statusMessage(Status status);
};
