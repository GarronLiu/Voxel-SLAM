#pragma once

#include "tools.hpp"
#include "ekf_imu.hpp"
#include "voxel_map.hpp"
#include "feature_point.hpp"
#include "loop_refine.hpp"
#include <cmath>
#include <map>
#include <mutex>
#include <utility>
#include <vector>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <malloc.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <gnss_comm/gnss_ros.hpp>
#include <gnss_comm/gnss_spp.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <malloc.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include "BTC.h"

using namespace std;

ros::Publisher pub_scan, pub_cmap, pub_init, pub_pmap;
ros::Publisher pub_test, pub_prev_path, pub_curr_path;
ros::Publisher pub_gnss_fix_local, pub_gnss_spp_local;
ros::Subscriber sub_imu, sub_pcl;
ros::Subscriber sub_gnss_ephem, sub_gnss_glo_ephem, sub_gnss_meas, sub_gnss_iono_params;
ros::Subscriber sub_gnss_pvt;

template <typename T>
void pub_pl_func(T &pl, ros::Publisher &pub)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "camera_init";
  output.header.stamp = ros::Time::now();
  pub.publish(output);
}

mutex mBuf;
mutex mPvtBuf;
Features feat;
deque<sensor_msgs::Imu::Ptr> imu_buf;
deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
deque<double> time_buf;
deque<gnss_comm::PVTSolutionPtr> gnss_pvt_buf;

class GnssCache
{
public:
  struct ProcessedMeasurements
  {
    double timestamp = -1;
    double lidar_timestamp = -1;
    int window_index = -1;
    Eigen::Vector3d local_p = Eigen::Vector3d::Zero();
    Eigen::Vector3d local_v = Eigen::Vector3d::Zero();
    vector<gnss_comm::ObsPtr> obs;
    vector<gnss_comm::EphemBasePtr> ephems;
  };

  void inputEphem(const gnss_comm::EphemBasePtr &ephem)
  {
    if(!ephem) return;

    lock_guard<mutex> lock(mtx_);
    double toe = gnss_comm::time2sec(ephem->toe);
    if(sat2time_index_.count(ephem->sat) != 0 && sat2time_index_[ephem->sat].count(toe) != 0)
      return;

    vector<gnss_comm::EphemBasePtr> &sat_ephems = sat2ephem_[ephem->sat];
    sat_ephems.push_back(ephem);
    sat2time_index_[ephem->sat][toe] = sat_ephems.size() - 1;
  }

  void inputMeasurements(vector<gnss_comm::ObsPtr> meas)
  {
    if(meas.empty()) return;

    lock_guard<mutex> lock(mtx_);
    latest_meas_time_ = gnss_comm::time2sec(meas.front()->time);
    meas_buf_.push_back(std::move(meas));
  }

  bool inputIonoParams(double ts, const vector<double> &iono_params)
  {
    if(iono_params.size() != 8) return false;

    lock_guard<mutex> lock(mtx_);
    latest_iono_time_ = ts;
    latest_iono_params_ = iono_params;
    return true;
  }

  bool emptyMeasurements()
  {
    lock_guard<mutex> lock(mtx_);
    return meas_buf_.empty();
  }

  bool popFrontMeasurements(vector<gnss_comm::ObsPtr> &meas)
  {
    lock_guard<mutex> lock(mtx_);
    if(meas_buf_.empty()) return false;

    meas = std::move(meas_buf_.front());
    meas_buf_.pop_front();
    return true;
  }

  bool matchMeasurementsToLidarFrame(double lidar_time, int window_index,
                                     const Eigen::Vector3d &local_p,
                                     const Eigen::Vector3d &local_v,
                                     double max_delay,
                                     double gnss_local_time_diff,
                                     double psr_std_thres, double dopp_std_thres,
                                     uint32_t track_num_thres,
                                     bool use_elevation_filter,
                                     const Eigen::Vector3d &receiver_ecef,
                                     double elevation_thres_deg,
                                     ProcessedMeasurements &processed)
  {
    lock_guard<mutex> lock(mtx_);
    processed = ProcessedMeasurements();

    while(!meas_buf_.empty())
    {
      vector<gnss_comm::ObsPtr> &front_meas = meas_buf_.front();
      if(front_meas.empty())
      {
        meas_buf_.pop_front();
        continue;
      }
      double obs_time = gnss_comm::time2sec(front_meas.front()->time);
      double obs_local_time = obs_time - gnss_local_time_diff;
      if(obs_local_time > lidar_time + max_delay)
        return false;

      vector<gnss_comm::ObsPtr> valid_meas;
      vector<gnss_comm::EphemBasePtr> valid_ephems;
      filterMeasurements(front_meas, psr_std_thres, dopp_std_thres,
                         track_num_thres, use_elevation_filter,
                         receiver_ecef, elevation_thres_deg,
                         valid_meas, valid_ephems);
      meas_buf_.pop_front();

      if(fabs(obs_local_time - lidar_time) <= max_delay && !valid_meas.empty())
      {
        processed.timestamp = obs_time;
        processed.lidar_timestamp = lidar_time;
        processed.window_index = window_index;
        processed.local_p = local_p;
        processed.local_v = local_v;
        processed.obs = std::move(valid_meas);
        processed.ephems = std::move(valid_ephems);
        return true;
      }
    }

    return false;
  }

  size_t measurementSize()
  {
    lock_guard<mutex> lock(mtx_);
    return meas_buf_.size();
  }

  size_t ephemSatelliteSize()
  {
    lock_guard<mutex> lock(mtx_);
    return sat2ephem_.size();
  }

  double latestMeasurementTime()
  {
    lock_guard<mutex> lock(mtx_);
    return latest_meas_time_;
  }

  double latestIonoTime()
  {
    lock_guard<mutex> lock(mtx_);
    return latest_iono_time_;
  }

  vector<double> latestIonoParams()
  {
    lock_guard<mutex> lock(mtx_);
    return latest_iono_params_;
  }

  deque<vector<gnss_comm::ObsPtr>> measurementSnapshot()
  {
    lock_guard<mutex> lock(mtx_);
    return meas_buf_;
  }

  map<uint32_t, vector<gnss_comm::EphemBasePtr>> ephemSnapshot()
  {
    lock_guard<mutex> lock(mtx_);
    return sat2ephem_;
  }

private:
  void filterMeasurements(const vector<gnss_comm::ObsPtr> &meas,
                          double psr_std_thres, double dopp_std_thres,
                          uint32_t track_num_thres,
                          bool use_elevation_filter,
                          const Eigen::Vector3d &receiver_ecef,
                          double elevation_thres_deg,
                          vector<gnss_comm::ObsPtr> &valid_meas,
                          vector<gnss_comm::EphemBasePtr> &valid_ephems)
  {
    
    for(const gnss_comm::ObsPtr &obs: meas)
    {
      uint32_t sys = gnss_comm::satsys(obs->sat, NULL);
      if(sys != SYS_GPS && sys != SYS_GLO && sys != SYS_GAL && sys != SYS_BDS)
        continue;

      if(sat2ephem_.count(obs->sat) == 0 || obs->freqs.empty())
        continue;

      int freq_idx = -1;
      gnss_comm::L1_freq(obs, &freq_idx);
      if(freq_idx < 0) continue;

      if(freq_idx >= obs->psr_std.size() || freq_idx >= obs->dopp_std.size())
        continue;

      gnss_comm::EphemBasePtr best_ephem;
      if(!findBestEphem(obs, best_ephem))
        continue;

      if(use_elevation_filter)
      {
        Eigen::Vector3d sat_ecef;
        if(sys == SYS_GLO)
          sat_ecef = gnss_comm::geph2pos(obs->time, std::dynamic_pointer_cast<gnss_comm::GloEphem>(best_ephem), NULL);
        else
          sat_ecef = gnss_comm::eph2pos(obs->time, std::dynamic_pointer_cast<gnss_comm::Ephem>(best_ephem), NULL);

        double azel[2] = {0, M_PI/2.0};
        gnss_comm::sat_azel(receiver_ecef, sat_ecef, azel);
        if(azel[1] < elevation_thres_deg * M_PI / 180.0)
          continue;
      }

      if(obs->psr_std[freq_idx] > psr_std_thres ||
         obs->dopp_std[freq_idx] > dopp_std_thres)
      {
        sat_track_status_[obs->sat] = 0;
        continue;
      }

      sat_track_status_[obs->sat]++;
      if(sat_track_status_[obs->sat] < track_num_thres)
        continue;

      valid_meas.push_back(obs);
      valid_ephems.push_back(best_ephem);
    }
  }

  bool findBestEphem(const gnss_comm::ObsPtr &obs, gnss_comm::EphemBasePtr &best_ephem)
  {
    if(sat2time_index_.count(obs->sat) == 0)
      return false;

    double obs_time = gnss_comm::time2sec(obs->time);
    double ephem_time = EPH_VALID_SECONDS;
    size_t ephem_index = 0;
    bool found = false;
    for(const auto &ti: sat2time_index_[obs->sat])
    {
      double dt = fabs(ti.first - obs_time);
      if(dt < ephem_time)
      {
        ephem_time = dt;
        ephem_index = ti.second;
        found = true;
      }
    }

    if(!found || ephem_time >= EPH_VALID_SECONDS)
      return false;

    best_ephem = sat2ephem_[obs->sat][ephem_index];
    return true;
  }

  mutex mtx_;
  deque<vector<gnss_comm::ObsPtr>> meas_buf_;
  map<uint32_t, vector<gnss_comm::EphemBasePtr>> sat2ephem_;
  map<uint32_t, map<double, size_t>> sat2time_index_;
  map<uint32_t, uint32_t> sat_track_status_;
  vector<double> latest_iono_params_;
  double latest_meas_time_ = -1;
  double latest_iono_time_ = -1;
};

GnssCache gnss_cache;

class VoxelGnssInitializer
{
public:
  VoxelGnssInitializer(const vector<GnssCache::ProcessedMeasurements> &gnss_frames,
                       const vector<double> &iono_params)
      : gnss_frames_(gnss_frames), iono_params_(iono_params)
  {
    num_all_meas_ = 0;
    all_sat_states_.clear();
    all_sat_states_.reserve(gnss_frames_.size());
    for(const auto &frame: gnss_frames_)
    {
      num_all_meas_ += frame.obs.size();
      all_sat_states_.push_back(gnss_comm::sat_states(frame.obs, frame.ephems));
    }
  }

  bool coarseLocalization(Eigen::Matrix<double, 7, 1> &rough_xyzt)
  {
    rough_xyzt.setZero();
    vector<gnss_comm::ObsPtr> accum_obs;
    vector<gnss_comm::EphemBasePtr> accum_ephems;
    for(const auto &frame: gnss_frames_)
    {
      accum_obs.insert(accum_obs.end(), frame.obs.begin(), frame.obs.end());
      accum_ephems.insert(accum_ephems.end(), frame.ephems.begin(), frame.ephems.end());
    }

    rough_xyzt = gnss_comm::psr_pos(accum_obs, accum_ephems, iono_params_);
    if(rough_xyzt.head<3>().norm() == 0)
      return false;

    for(uint32_t k=0; k<4; k++)
    {
      if(fabs(rough_xyzt(k+3)) < 1)
        rough_xyzt(k+3) = 0;
    }
    return true;
  }

  bool yawAlignment(const vector<Eigen::Vector3d> &local_vs,
                    const Eigen::Vector3d &rough_anchor_ecef,
                    double &aligned_yaw, double &rcv_ddt)
  {
    if(local_vs.size() != gnss_frames_.size() || num_all_meas_ == 0)
      return false;

    aligned_yaw = 0;
    rcv_ddt = 0;
    double est_yaw = 0;
    double est_rcv_ddt = 0;
    Eigen::Matrix3d rough_R_ecef_enu = gnss_comm::ecef2rotation(rough_anchor_ecef);

    uint32_t iter = 0;
    double dx_norm = 1.0;
    while(iter < MAX_ITERATION && dx_norm > CONVERGENCE_EPSILON)
    {
      Eigen::MatrixXd G(num_all_meas_, 2);
      Eigen::VectorXd b(num_all_meas_);
      G.setZero();
      G.col(1).setOnes();
      b.setZero();

      Eigen::Matrix3d R_enu_local(Eigen::AngleAxisd(est_yaw, Eigen::Vector3d::UnitZ()));
      Eigen::Matrix3d d_yaw;
      d_yaw << -sin(est_yaw), -cos(est_yaw), 0,
                cos(est_yaw), -sin(est_yaw), 0,
                0,             0,            0;

      uint32_t counter = 0;
      for(uint32_t i=0; i<gnss_frames_.size(); i++)
      {
        Eigen::Matrix<double, 4, 1> ecef_vel_ddt;
        ecef_vel_ddt.head<3>() = rough_R_ecef_enu * R_enu_local * local_vs[i];
        ecef_vel_ddt(3) = est_rcv_ddt;

        Eigen::VectorXd epoch_res;
        Eigen::MatrixXd epoch_J;
        gnss_comm::dopp_res(ecef_vel_ddt, rough_anchor_ecef, gnss_frames_[i].obs,
                            all_sat_states_[i], epoch_res, epoch_J);
        G.block(counter, 0, gnss_frames_[i].obs.size(), 1) =
            epoch_J.leftCols(3) * rough_R_ecef_enu * d_yaw * local_vs[i];
        b.segment(counter, gnss_frames_[i].obs.size()) = epoch_res;
        counter += gnss_frames_[i].obs.size();
      }

      Eigen::Matrix2d H = G.transpose() * G;
      Eigen::Vector2d g = G.transpose() * b;
      Eigen::Vector2d dx = -H.ldlt().solve(g);
      if(!dx.allFinite()) return false;

      est_yaw += dx(0);
      est_rcv_ddt += dx(1);
      dx_norm = dx.norm();
      iter++;
    }

    if(iter >= MAX_ITERATION && dx_norm > CONVERGENCE_EPSILON)
      return false;

    aligned_yaw = est_yaw;
    while(aligned_yaw > M_PI) aligned_yaw -= 2.0*M_PI;
    while(aligned_yaw < -M_PI) aligned_yaw += 2.0*M_PI;
    rcv_ddt = est_rcv_ddt;
    return true;
  }

  bool anchorRefinement(const vector<Eigen::Vector3d> &local_ps,
                        double aligned_yaw, double aligned_ddt,
                        const Eigen::Matrix<double, 7, 1> &rough_xyzt,
                        Eigen::Matrix<double, 7, 1> &refined_xyzt)
  {
    if(local_ps.size() != gnss_frames_.size() || num_all_meas_ == 0)
      return false;

    refined_xyzt.setZero();
    Eigen::Matrix3d R_enu_local(Eigen::AngleAxisd(aligned_yaw, Eigen::Vector3d::UnitZ()));
    Eigen::Vector3d refine_anchor = rough_xyzt.head<3>();
    Eigen::Vector4d refine_dt = rough_xyzt.tail<4>();

    vector<uint32_t> unobserved_sys;
    for(uint32_t k=0; k<4; k++)
    {
      if(rough_xyzt(3+k) == 0)
        unobserved_sys.push_back(k);
    }

    uint32_t iter = 0;
    double dx_norm = 1.0;
    while(iter < MAX_ITERATION && dx_norm > CONVERGENCE_EPSILON)
    {
      Eigen::MatrixXd G(num_all_meas_ + unobserved_sys.size(), 7);
      Eigen::VectorXd b(num_all_meas_ + unobserved_sys.size());
      G.setZero();
      b.setZero();

      uint32_t counter = 0;
      Eigen::Matrix3d R_ecef_enu = gnss_comm::ecef2rotation(refine_anchor);
      Eigen::Matrix3d R_ecef_local = R_ecef_enu * R_enu_local;
      for(uint32_t i=0; i<gnss_frames_.size(); i++)
      {
        Eigen::Matrix<double, 7, 1> ecef_xyz_dt;
        ecef_xyz_dt.head<3>() = R_ecef_local * local_ps[i] + refine_anchor;
        ecef_xyz_dt.tail<4>() = refine_dt + aligned_ddt * frameTimeDelta(i) * Eigen::Vector4d::Ones();

        Eigen::VectorXd epoch_res;
        Eigen::MatrixXd epoch_J;
        vector<Eigen::Vector2d> atmos_delay, sv_azel;
        gnss_comm::psr_res(ecef_xyz_dt, gnss_frames_[i].obs, all_sat_states_[i],
                           iono_params_, epoch_res, epoch_J, atmos_delay, sv_azel);
        G.middleRows(counter, gnss_frames_[i].obs.size()) = epoch_J;
        b.segment(counter, gnss_frames_[i].obs.size()) = epoch_res;
        counter += gnss_frames_[i].obs.size();
      }

      for(uint32_t k: unobserved_sys)
      {
        G(counter, k+3) = 1.0;
        counter++;
      }

      Eigen::Matrix<double, 7, 7> H = G.transpose() * G;
      Eigen::Matrix<double, 7, 1> g = G.transpose() * b;
      Eigen::Matrix<double, 7, 1> dx = -H.ldlt().solve(g);
      if(!dx.allFinite()) return false;

      refine_anchor += dx.head<3>();
      refine_dt += dx.tail<4>();
      dx_norm = dx.norm();
      iter++;
    }

    if(iter >= MAX_ITERATION && dx_norm > CONVERGENCE_EPSILON)
      return false;

    refined_xyzt.head<3>() = refine_anchor;
    refined_xyzt.tail<4>() = refine_dt;
    return true;
  }

private:
  const vector<GnssCache::ProcessedMeasurements> &gnss_frames_;
  const vector<double> &iono_params_;
  uint32_t num_all_meas_ = 0;
  vector<vector<gnss_comm::SatStatePtr>> all_sat_states_;

  static constexpr uint32_t MAX_ITERATION = 10;
  static constexpr double CONVERGENCE_EPSILON = 1e-5;

  double frameTimeDelta(size_t frame_index) const
  {
    if(gnss_frames_.empty() || frame_index >= gnss_frames_.size())
      return 0.0;

    return gnss_frames_[frame_index].timestamp - gnss_frames_.front().timestamp;
  }
};

double imu_last_time = -1;
int point_notime = 0;
double last_pcl_time = -1;

void imu_handler(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  static int flag = 1;
  if(flag)
  {
    flag = 0;
    printf("Time0: %lf\n", msg_in->header.stamp.toSec());
  }

  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  // For Hilti 2022 exp03
  // double t0 = 1646320760 + 255.5;
  // double t1 = 1646320760 + 256.2;
  // double tc = msg->header.stamp.toSec();
  // if(tc > t0 && tc < t1)
  //   msg->linear_acceleration.z = -9.7;

  mBuf.lock();
  imu_last_time = msg->header.stamp.toSec();
  imu_buf.push_back(msg);
  mBuf.unlock();
}

void gnss_ephem_handler(const gnss_comm::GnssEphemMsgConstPtr &msg_in)
{
  gnss_comm::EphemPtr ephem = gnss_comm::msg2ephem(msg_in);
  gnss_cache.inputEphem(ephem);
}

void gnss_glo_ephem_handler(const gnss_comm::GnssGloEphemMsgConstPtr &msg_in)
{
  gnss_comm::GloEphemPtr glo_ephem = gnss_comm::msg2glo_ephem(msg_in);
  gnss_cache.inputEphem(glo_ephem);
}

void gnss_meas_handler(const gnss_comm::GnssMeasMsgConstPtr &msg_in)
{
  vector<gnss_comm::ObsPtr> gnss_meas = gnss_comm::msg2meas(msg_in);
  gnss_cache.inputMeasurements(std::move(gnss_meas));
}

void gnss_pvt_handler(const gnss_comm::GnssPVTSolnMsgConstPtr &msg_in)
{
  gnss_comm::PVTSolutionPtr pvt = gnss_comm::msg2pvt(msg_in);
  if(!pvt || !pvt->valid_fix)
    return;
  lock_guard<mutex> lock(mPvtBuf);
  gnss_pvt_buf.push_back(pvt);
  while(gnss_pvt_buf.size() > 200)
    gnss_pvt_buf.pop_front();
}

void gnss_iono_params_handler(const gnss_comm::StampedFloat64ArrayConstPtr &msg_in)
{
  vector<double> iono_params(msg_in->data.begin(), msg_in->data.end());
  if(!gnss_cache.inputIonoParams(msg_in->header.stamp.toSec(), iono_params))
  {
    ROS_WARN_THROTTLE(5.0, "Ignore GNSS iono params with invalid size: %lu", msg_in->data.size());
    return;
  }
}

template<class T>
void pcl_handler(T &msg)
{
  pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
  double t0 = feat.process(msg, *pl_ptr);

  if(pl_ptr->empty())
  {
    PointType ap; 
    ap.x = 0; ap.y = 0; ap.z = 0; 
    ap.intensity = 0; ap.curvature = 0;
    pl_ptr->push_back(ap);
    ap.curvature = 0.09;
    pl_ptr->push_back(ap);
  }

  sort(pl_ptr->begin(), pl_ptr->end(), [](PointType &x, PointType &y)
  {
    return x.curvature < y.curvature;
  });
  while(pl_ptr->back().curvature > 0.11)
    pl_ptr->points.pop_back();

  mBuf.lock();
  time_buf.push_back(t0);
  pcl_buf.push_back(pl_ptr);
  mBuf.unlock();
}

bool sync_packages(pcl::PointCloud<PointType>::Ptr &pl_ptr, deque<sensor_msgs::Imu::Ptr> &imus, IMUEKF &p_imu)
{
  static bool pl_ready = false;

  if(!pl_ready)
  {
    {
      lock_guard<mutex> lock(mBuf);
      if(pcl_buf.empty()) return false;

      pl_ptr = pcl_buf.front();
      p_imu.pcl_beg_time = time_buf.front();
      pcl_buf.pop_front(); time_buf.pop_front();
    }

    p_imu.pcl_end_time = p_imu.pcl_beg_time + pl_ptr->back().curvature;

    if(point_notime)
    {
      if(last_pcl_time < 0)
      {
        last_pcl_time = p_imu.pcl_beg_time;
        return false;
      }

      p_imu.pcl_end_time = p_imu.pcl_beg_time;
      p_imu.pcl_beg_time = last_pcl_time;
      last_pcl_time = p_imu.pcl_end_time;
    }

    pl_ready = true;
  }

  {
    lock_guard<mutex> lock(mBuf);
    if(!pl_ready || imu_buf.empty() || imu_last_time <= p_imu.pcl_end_time) return false;
  }

  {
    lock_guard<mutex> lock(mBuf);
    while(!imu_buf.empty() && imu_buf.front()->header.stamp.toSec() < p_imu.pcl_end_time)
    {
      imus.push_back(imu_buf.front());
      imu_buf.pop_front();
    }
  }

  {
    lock_guard<mutex> lock(mBuf);
    if(imu_buf.empty())
    {
      printf("imu buf empty\n"); exit(0);
    }
  }

  pl_ready = false;

  if(imus.size() > 4)
    return true;
  else
    return false;
}

bool sync_packages_append_pvt(
    pcl::PointCloud<PointType>::Ptr &pl_ptr,
    deque<sensor_msgs::Imu::Ptr> &imus,
    IMUEKF &p_imu,
    gnss_comm::PVTSolutionPtr &matched_pvt,
    double gnss_local_time_diff, double gnss_max_delay, bool gnss_enable)
{
  static bool pl_ready = false;

  if(!pl_ready)
  {
    {
      lock_guard<mutex> lock(mBuf);
      if(pcl_buf.empty()) return false;

      pl_ptr = pcl_buf.front();
      p_imu.pcl_beg_time = time_buf.front();
      pcl_buf.pop_front(); time_buf.pop_front();
    }

    p_imu.pcl_end_time = p_imu.pcl_beg_time + pl_ptr->back().curvature;

    if(point_notime)
    {
      if(last_pcl_time < 0)
      {
        last_pcl_time = p_imu.pcl_beg_time;
        return false;
      }

      p_imu.pcl_end_time = p_imu.pcl_beg_time;
      p_imu.pcl_beg_time = last_pcl_time;
      last_pcl_time = p_imu.pcl_end_time;
    }

    pl_ready = true;
  }

  {
    lock_guard<mutex> lock(mBuf);
    if(!pl_ready || imu_buf.empty() || imu_last_time <= p_imu.pcl_end_time) return false;
  }

  // 假设GNSS PVT的时间戳已对齐到整秒，且其时间与LiDAR扫描结束时间接近（在gnss_max_delay范围内），
  // 则将该PVT附加到当前LiDAR扫描。注意：若没有满足条件的GNSS PVT消息，处理将阻塞，
  // 系统会暂停LiDAR-IMU融合，直到收到合格的GNSS PVT为止。
  const double scan_end = max(p_imu.pcl_beg_time, p_imu.pcl_end_time);
  const double target_pvt_utc = round(scan_end);
  const double round_time_error = fabs(scan_end - target_pvt_utc);
  const bool lidar_end_near_round_time = round_time_error <= gnss_max_delay;
  if(gnss_enable && lidar_end_near_round_time)
  {
    lock_guard<mutex> lock(mPvtBuf);
    while(!gnss_pvt_buf.empty())
    {
      gnss_comm::PVTSolutionPtr &front_pvt = gnss_pvt_buf.front();
      if(!front_pvt)
      {
        gnss_pvt_buf.pop_front();
        continue;
      }

      double pvt_time = gnss_comm::time2sec(front_pvt->time);
      double pvt_utc = pvt_time - gnss_local_time_diff;
      if(pvt_utc >= target_pvt_utc - gnss_max_delay)
        break;

      gnss_pvt_buf.pop_front();
    }

    if(gnss_pvt_buf.empty())
      return false;
  }

  {
    lock_guard<mutex> lock(mBuf);
    while(!imu_buf.empty() && imu_buf.front()->header.stamp.toSec() < p_imu.pcl_end_time)
    {
      imus.push_back(imu_buf.front());
      imu_buf.pop_front();
    }
  }

  {
    lock_guard<mutex> lock(mBuf);
    if(imu_buf.empty())
    {
      printf("imu buf empty\n"); exit(0);
    }
  }

  pl_ready = false;

  // Phase 4: attach one PVT epoch to the current LiDAR scan if available.
  matched_pvt.reset();
  if(gnss_enable && lidar_end_near_round_time)
  {
    lock_guard<mutex> lock(mPvtBuf);
    while(!gnss_pvt_buf.empty())
    {
      gnss_comm::PVTSolutionPtr &front_pvt = gnss_pvt_buf.front();
      if(!front_pvt)
      {
        gnss_pvt_buf.pop_front();
        continue;
      }

      double pvt_time = gnss_comm::time2sec(front_pvt->time);
      double pvt_utc = pvt_time - gnss_local_time_diff;

      if(pvt_utc > target_pvt_utc + gnss_max_delay)
        break;

      if(fabs(pvt_utc - target_pvt_utc) <= gnss_max_delay)
      {
        matched_pvt = front_pvt;
        gnss_pvt_buf.pop_front();
        break;
      }

      gnss_pvt_buf.pop_front();
    }
  }

  if(imus.size() > 4)
    return true;
  else
    return false;
}


double dept_err, beam_err;
void calcBodyVar(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &var) 
{
  if (pb[2] == 0)
    pb[2] = 0.0001;
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  var = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
};

// Compute the variance of the each point
void var_init(IMUST &ext, pcl::PointCloud<PointType> &pl_cur, PVecPtr pptr, double dept_err, double beam_err)
{
  int plsize = pl_cur.size();
  pptr->clear();
  pptr->resize(plsize);
  for(int i=0; i<plsize; i++)
  {
    PointType &ap = pl_cur[i];
    pointVar &pv = pptr->at(i);
    pv.pnt << ap.x, ap.y, ap.z;
    calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
    pv.pnt = ext.R * pv.pnt + ext.p;
    pv.var = ext.R * pv.var * ext.R.transpose();
  }
}

void pvec_update(PVecPtr pptr, IMUST &x_curr, PLV(3) &pwld)
{
  Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
  Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);

  for(pointVar &pv: *pptr)
  {
    Eigen::Matrix3d phat = hat(pv.pnt);
    pv.var = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
    pwld.push_back(x_curr.R * pv.pnt + x_curr.p);
  }
}

// Read the alidarstate.txt
void read_lidarstate(string filename, vector<ScanPose*> &bl_tem)
{
  ifstream file(filename);
  if(!file.is_open())
  {
    printf("Error: %s not found\n", filename.c_str());
    exit(0);
  }

  string lineStr, str;
  vector<double> nums;
  while(getline(file, lineStr))
  {
    nums.clear();
    stringstream ss(lineStr);
    while(getline(ss, str, ' '))
      nums.push_back(stod(str));
    
    IMUST xx;
    xx.t = nums[0];
    xx.p << nums[1], nums[2], nums[3];
    xx.R = Eigen::Quaterniond(nums[7], nums[4], nums[5], nums[6]).matrix();

    if(nums.size() >= 20)
    {
      xx.v << nums[8], nums[9], nums[10];
      xx.bg << nums[11], nums[12], nums[13];
      xx.ba << nums[14], nums[15], nums[16];
      xx.g << nums[17], nums[18], nums[19];
    }

    ScanPose* blp = new ScanPose(xx, nullptr);
    bl_tem.push_back(blp);

    if(nums.size() >= 26)
      for(int i=0; i<6; i++) 
        blp->v6[i] = nums[i + 20];
  }
}

double get_memory()
{
  ifstream infile("/proc/self/status");
  double mem = -1;
  string lineStr, str;
  while(getline(infile, lineStr))
  {
    stringstream ss(lineStr);
    bool is_find = false;
    while(ss >> str)
    {
      if(str == "VmRSS:")
      {
        is_find = true; continue;
      }

      if(is_find) mem = stod(str);
      break;
    }
    if(is_find) break;
  }
  return mem / (1048576);
}

void icp_check(pcl::PointCloud<PointType> &pl_src, pcl::PointCloud<PointType> &pl_tar, ros::Publisher &pub_src, ros::Publisher &pub_tar, pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform, IMUST &xx)
{
  pcl::PointCloud<PointType> pl1, pl2;
  for(PointType ap: pl_src.points)
  {
    Eigen::Vector3d v(ap.x, ap.y, ap.z);
    v = loop_transform.second * v + loop_transform.first;
    v = xx.R * v + xx.p;
    ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
    pl1.push_back(ap);
  }
  for(PointType ap: pl_tar.points)
  {
    Eigen::Vector3d v(ap.x, ap.y, ap.z);
    v = xx.R * v + xx.p;
    ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
    pl2.push_back(ap);
  }
  pub_pl_func(pl1, pub_src); pub_pl_func(pl2, pub_tar);
}
