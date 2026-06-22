#include "GNSSProcess.h"

#include <cmath>

#include <Eigen/Geometry>
#include <Eigen/Cholesky>
#include <ros/ros.h>
#include <gnss_comm/gnss_spp.hpp>

namespace
{
class Initializer
{
public:
  Initializer(const std::vector<GNSSProcess::Frame> &frames,
              const std::vector<double> &iono_params)
      : frames_(frames), iono_params_(iono_params)
  {
    sat_states_.reserve(frames_.size());
    for(const auto &frame : frames_)
    {
      measurement_count_ += frame.obs.size();
      sat_states_.push_back(gnss_comm::sat_states(frame.obs, frame.ephems));
    }
  }

  bool coarseLocalization(Eigen::Matrix<double, 7, 1> &rough_xyzt) const
  {
    std::vector<gnss_comm::ObsPtr> observations;
    std::vector<gnss_comm::EphemBasePtr> ephemerides;
    for(const auto &frame : frames_)
    {
      observations.insert(observations.end(), frame.obs.begin(), frame.obs.end());
      ephemerides.insert(ephemerides.end(), frame.ephems.begin(), frame.ephems.end());
    }

    rough_xyzt = gnss_comm::psr_pos(observations, ephemerides, iono_params_);
    if(rough_xyzt.head<3>().norm() == 0.0 || !rough_xyzt.allFinite())
      return false;

    for(uint32_t k = 0; k < 4; ++k)
      if(std::fabs(rough_xyzt(k + 3)) < 1.0) rough_xyzt(k + 3) = 0.0;
    return true;
  }

  bool alignYaw(const std::vector<Eigen::Vector3d> &local_velocities,
                const Eigen::Vector3d &rough_anchor_ecef,
                double &yaw, double &clock_drift) const
  {
    if(local_velocities.size() != frames_.size() || measurement_count_ == 0)
      return false;

    yaw = 0.0;
    clock_drift = 0.0;
    const Eigen::Matrix3d R_ecef_enu = gnss_comm::ecef2rotation(rough_anchor_ecef);
    uint32_t iteration = 0;
    double increment_norm = 1.0;
    while(iteration < kMaxIterations && increment_norm > kConvergenceEpsilon)
    {
      Eigen::MatrixXd jacobian(measurement_count_, 2);
      Eigen::VectorXd residual(measurement_count_);
      jacobian.setZero();
      jacobian.col(1).setOnes();
      residual.setZero();

      const Eigen::Matrix3d R_enu_local(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
      Eigen::Matrix3d yaw_jacobian;
      yaw_jacobian << -std::sin(yaw), -std::cos(yaw), 0.0,
                       std::cos(yaw), -std::sin(yaw), 0.0,
                       0.0,            0.0,           0.0;

      uint32_t row = 0;
      for(size_t i = 0; i < frames_.size(); ++i)
      {
        Eigen::Vector4d ecef_velocity;
        ecef_velocity.head<3>() = R_ecef_enu * R_enu_local * local_velocities[i];
        ecef_velocity(3) = clock_drift;
        Eigen::VectorXd epoch_residual;
        Eigen::MatrixXd epoch_jacobian;
        gnss_comm::dopp_res(ecef_velocity, rough_anchor_ecef, frames_[i].obs,
                            sat_states_[i], epoch_residual, epoch_jacobian);
        jacobian.block(row, 0, frames_[i].obs.size(), 1) =
            epoch_jacobian.leftCols(3) * R_ecef_enu * yaw_jacobian * local_velocities[i];
        residual.segment(row, frames_[i].obs.size()) = epoch_residual;
        row += frames_[i].obs.size();
      }

      const Eigen::Vector2d increment =
          -(jacobian.transpose() * jacobian).ldlt().solve(jacobian.transpose() * residual);
      if(!increment.allFinite()) return false;
      yaw += increment(0);
      clock_drift += increment(1);
      increment_norm = increment.norm();
      ++iteration;
    }

    if(iteration >= kMaxIterations && increment_norm > kConvergenceEpsilon)
      return false;
    while(yaw > M_PI) yaw -= 2.0 * M_PI;
    while(yaw < -M_PI) yaw += 2.0 * M_PI;
    ROS_INFO("GNSS yaw alignment converged in %u iterations, dx_norm=%.6f", iteration, increment_norm);
    return true;
  }

  bool refineAnchor(const std::vector<Eigen::Vector3d> &local_positions,
                    double yaw, double clock_drift,
                    const Eigen::Matrix<double, 7, 1> &rough_xyzt,
                    Eigen::Matrix<double, 7, 1> &refined_xyzt) const
  {
    if(local_positions.size() != frames_.size() || measurement_count_ == 0)
      return false;

    const Eigen::Matrix3d R_enu_local(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d anchor = rough_xyzt.head<3>();
    Eigen::Vector4d clock_bias = rough_xyzt.tail<4>();
    std::vector<uint32_t> unobserved_systems;
    for(uint32_t k = 0; k < 4; ++k)
      if(rough_xyzt(3 + k) == 0.0) unobserved_systems.push_back(k);

    uint32_t iteration = 0;
    double increment_norm = 1.0;
    while(iteration < kMaxIterations && increment_norm > kConvergenceEpsilon)
    {
      Eigen::MatrixXd jacobian(measurement_count_ + unobserved_systems.size(), 7);
      Eigen::VectorXd residual(measurement_count_ + unobserved_systems.size());
      jacobian.setZero();
      residual.setZero();
      uint32_t row = 0;
      const Eigen::Matrix3d R_ecef_local = gnss_comm::ecef2rotation(anchor) * R_enu_local;
      for(size_t i = 0; i < frames_.size(); ++i)
      {
        Eigen::Matrix<double, 7, 1> state;
        state.head<3>() = R_ecef_local * local_positions[i] + anchor;
        state.tail<4>() = clock_bias + clock_drift * frameTimeDelta(i) * Eigen::Vector4d::Ones();
        Eigen::VectorXd epoch_residual;
        Eigen::MatrixXd epoch_jacobian;
        std::vector<Eigen::Vector2d> atmospheric_delay, satellite_azel;
        gnss_comm::psr_res(state, frames_[i].obs, sat_states_[i], iono_params_,
                           epoch_residual, epoch_jacobian, atmospheric_delay, satellite_azel);
        jacobian.middleRows(row, frames_[i].obs.size()) = epoch_jacobian;
        residual.segment(row, frames_[i].obs.size()) = epoch_residual;
        row += frames_[i].obs.size();
      }
      for(uint32_t system : unobserved_systems) jacobian(row++, system + 3) = 1.0;

      const Eigen::Matrix<double, 7, 1> increment =
          -(jacobian.transpose() * jacobian).ldlt().solve(jacobian.transpose() * residual);
      if(!increment.allFinite()) return false;
      anchor += increment.head<3>();
      clock_bias += increment.tail<4>();
      increment_norm = increment.norm();
      ++iteration;
    }

    if(iteration >= kMaxIterations && increment_norm > kConvergenceEpsilon)
      return false;
    refined_xyzt.head<3>() = anchor;
    refined_xyzt.tail<4>() = clock_bias;
    ROS_INFO("GNSS anchor refinement converged in %u iterations, dx_norm=%.6f", iteration, increment_norm);
    return true;
  }

private:
  double frameTimeDelta(size_t index) const
  {
    return frames_.empty() || index >= frames_.size() ? 0.0
        : frames_[index].timestamp - frames_.front().timestamp;
  }

  static constexpr uint32_t kMaxIterations = 10;
  static constexpr double kConvergenceEpsilon = 1e-5;
  const std::vector<GNSSProcess::Frame> &frames_;
  const std::vector<double> &iono_params_;
  size_t measurement_count_ = 0;
  std::vector<std::vector<gnss_comm::SatStatePtr>> sat_states_;
};
}  // namespace

GNSSProcess::Status GNSSProcess::initialize(std::deque<Frame> &alignment_frames,
                                            const std::vector<double> &iono_params,
                                            const Options &options,
                                            Result &result)
{
  Result candidate;
  for(const auto &frame : alignment_frames)
  {
    if(frame.obs.size() < static_cast<size_t>(options.min_obs)) continue;
    candidate.aligned_frames.push_back(frame);
    candidate.aligned_frame_indices.push_back(frame.window_index);
  }
  if(candidate.aligned_frames.size() < static_cast<size_t>(options.min_frames))
    return Status::kInsufficientFrames;

  Eigen::Vector2d average_horizontal_velocity = Eigen::Vector2d::Zero();
  std::vector<Eigen::Vector3d> local_positions, local_velocities;
  for(const auto &frame : candidate.aligned_frames)
  {
    local_positions.push_back(frame.local_p);
    local_velocities.push_back(frame.local_v);
    average_horizontal_velocity += frame.local_v.head<2>().cwiseAbs();
  }
  average_horizontal_velocity /= static_cast<double>(candidate.aligned_frames.size());
  if(average_horizontal_velocity.norm() < options.min_horizontal_velocity)
    return Status::kInsufficientMotion;
  if(iono_params.size() != 8) return Status::kMissingIonosphereParameters;

  Initializer initializer(candidate.aligned_frames, iono_params);
  if(!initializer.coarseLocalization(candidate.rough_xyzt))
  {
    alignment_frames.pop_front();
    return Status::kCoarseLocalizationFailed;
  }
  if(!initializer.alignYaw(local_velocities, candidate.rough_xyzt.head<3>(),
                           candidate.yaw_enu_local, candidate.receiver_clock_drift))
  {
    alignment_frames.pop_front();
    return Status::kYawAlignmentFailed;
  }
  if(!initializer.refineAnchor(local_positions, candidate.yaw_enu_local,
                               candidate.receiver_clock_drift, candidate.rough_xyzt,
                               candidate.refined_xyzt))
  {
    alignment_frames.pop_front();
    return Status::kAnchorRefinementFailed;
  }

  uint32_t observed_system = 4;
  for(uint32_t k = 0; k < 4; ++k)
    if(candidate.rough_xyzt(3 + k) != 0.0) { observed_system = k; break; }
  if(observed_system == 4) return Status::kNoObservedConstellation;

  candidate.receiver_clock_biases.assign(candidate.aligned_frames.size() * 4, 0.0);
  for(size_t i = 0; i < candidate.aligned_frames.size(); ++i)
  {
    const double dt = candidate.aligned_frames[i].timestamp - candidate.aligned_frames.front().timestamp;
    for(uint32_t k = 0; k < 4; ++k)
    {
      const double base = candidate.rough_xyzt(3 + k) == 0.0
          ? candidate.refined_xyzt(3 + observed_system) : candidate.refined_xyzt(3 + k);
      candidate.receiver_clock_biases[i * 4 + k] = base + candidate.receiver_clock_drift * dt;
    }
  }
  candidate.anchor_ecef = candidate.refined_xyzt.head<3>();
  candidate.R_ecef_enu = gnss_comm::ecef2rotation(candidate.anchor_ecef);
  result = std::move(candidate);
  return Status::kSuccess;
}

const char *GNSSProcess::statusMessage(Status status)
{
  switch(status)
  {
    case Status::kInsufficientFrames: return "GNSS alignment waits for enough valid frames.";
    case Status::kInsufficientMotion: return "GNSS alignment waits for velocity excitation.";
    case Status::kMissingIonosphereParameters: return "GNSS alignment waits for valid ionosphere parameters.";
    case Status::kCoarseLocalizationFailed: return "GNSS coarse pseudorange localization failed.";
    case Status::kYawAlignmentFailed: return "GNSS Doppler yaw alignment failed.";
    case Status::kAnchorRefinementFailed: return "GNSS anchor refinement failed.";
    case Status::kNoObservedConstellation: return "GNSS initialization found no observed constellation.";
    case Status::kSuccess: return "GNSS initialization succeeded.";
  }
  return "Unknown GNSS initialization status.";
}
