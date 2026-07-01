/*
 * BSD 3-Clause License

 *  Copyright (c) 2025, Dongjiao He
 *  All rights reserved.
 *
 *  Author: Dongjiao HE <hdj65822@connect.hku.hk>
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Universitaet Bremen nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once
#include "GNSS_Initialization.h"
#include "GNSS_Assignment.h"

#include <deque>
#include <mutex>
#include <pcl/registration/icp.h>
using namespace gnss_comm;

#define WINDOW_SIZE (10) // should be 0

class GNSSProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct IeskfNormalEquation
  {
    Eigen::Matrix<double, 6, 6> hessian =
        Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 1> gradient =
        Eigen::Matrix<double, 6, 1>::Zero();
    int tdcp_accepted = 0;
    int doppler_accepted = 0;
    int chi_square_rejected = 0;
    double clock_drift_correction = 0.0;
    bool valid = false;
  };

  enum class IeskfPrepareStatus
  {
    NotRequested,
    Ready,
    GnssDisabled,
    InsufficientObservations,
    EphemerisMismatch,
    SatelliteStateFailure,
    InsufficientCarrierPhase,
    NoPreviousEpoch,
    InvalidTimeGap,
    InsufficientCommonCarrierPhase,
    InvalidReceiverState
  };

  struct IeskfFilterDiagnostics
  {
    IeskfPrepareStatus prepare_status = IeskfPrepareStatus::NotRequested;
    int raw_observations = 0;
    int carrier_phase_valid = 0;
    int common_carrier_phase = 0;
    double epoch_time_gap = 0.0;
    double maximum_time_gap = 0.0;
    int doppler_candidates = 0;
    int doppler_invalid = 0;
    int doppler_gross_rejected = 0;
    int tdcp_candidates = 0;
    int tdcp_geometry_rejected = 0;
    int tdcp_elevation_rejected = 0;
    int tdcp_gross_rejected = 0;
    int chi_square_rejected = 0;
  };

  struct PvtResult
  {
    double timestamp = -1.0;
    Eigen::Vector3d ecef = Eigen::Vector3d::Zero();
    Eigen::Vector3d variance_enu = Eigen::Vector3d::Ones();
  };

  GNSSProcess();
  ~GNSSProcess();
  
  void Reset();
  void inputIonoParams(double ts, const std::vector<double> &iono_params);
  void inputGNSSTimeDiff(const double t_diff);
  void processGNSS(const std::vector<ObsPtr> &gnss_meas, IMUST &state);
  bool GNSSLIAlign();
  void inputpvt(double ts, double lat, double lon, double alt,
                double h_acc, double v_acc, int float_sol, int diff_sol);
  bool matchClosestPvt(double local_timestamp, double max_time_error,
                       PvtResult &result);
  void inputlla(double ts, double lat, double lon, double alt);
  Eigen::Vector3d local2enu(Eigen::Matrix3d enu_rot, Eigen::Vector3d anc, Eigen::Vector3d &pos);
  std::map<sat_first, std::map<uint32_t, double[6]>> sat2cp; // 
  std::vector<ObsPtr> gnss_meas_buf[WINDOW_SIZE+1]; //
  std::vector<double> psr_meas_hatch_filter;
  std::vector<EphemBasePtr> gnss_ephem_buf[WINDOW_SIZE+1]; // 
  Eigen::Matrix3d rot_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d omg_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d pos_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d pos_ecef_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d vel_window[WINDOW_SIZE+1]; //
  Eigen::Vector3d Tex_imu_r;
  std::vector<double> pvt_time;
  std::vector<double> lla_time;
  std::vector<Eigen::Vector3d> pvt_holder;
  std::vector<int> diff_holder;
  std::vector<int> float_holder;
  std::vector<Eigen::Vector3d> lla_holder;
  std::queue<std::vector<ObsPtr>> gnss_msg;

  bool invalid_lidar = false;
  // double dt[4];
  // double ddt; 
  size_t id_accumulate = 0; // 
  size_t frame_delete = 0; // 

  int frame_num = 0; // 
  double last_gnss_time = 0.0; //
  double first_gnss_time = 0.0; //
  double gnss_sample_period = 0.1;

  double diff_t_gnss_local = 0.0;
  // double gnss_cp_std_threshold = 30;
  double gnss_cp_time_threshold = 30;
  Eigen::Vector3d ecef_pos, first_xyz_ecef_pvt, first_xyz_ecef_lla, first_lla_pvt, first_lla_lla;
  Eigen::Matrix3d Rot_gnss_init = Eigen::Matrix3d::Identity();
  bool gnss_ready = false;
  int frame_count = 0; //
  int delete_thred = 0;

  int wind_size = WINDOW_SIZE;
  size_t min_obs = 4;
  bool current_gnss_factor_enabled = false;
  double current_gnss_time = -1.0;
  int norm_vec_num = 0;
  bool nolidar = false;
  bool nolidar_cur = false;
  std::vector<Eigen::Vector3d> norm_vec_holder;
  // double para_yaw_enu_local[1];
  double para_rcv_dt[(WINDOW_SIZE+1)*4] = {0}; //
  double para_rcv_ddt[WINDOW_SIZE+1] = {0}; //
  Eigen::Vector3d anc_ecef, gravity_init;
  Eigen::Vector3d anc_local = Eigen::Vector3d::Zero();
  Eigen::Matrix3d R_ecef_enu;
  double yaw_enu_local = 0.0;
  
  bool prepareTdcpDopplerIeskf(const IMUST &state);
  bool buildTdcpDopplerIeskfNormal(
      const IMUST &state,
      const Eigen::Matrix<double, 6, 6> &position_velocity_cov,
      bool apply_chi_square_test,
      IeskfNormalEquation &normal) const;
  const char *ieskfPrepareStatusString() const;
  void commitTdcpDopplerIeskf(const IMUST &state);
  void resetTdcpDopplerIeskf();
  mutable IeskfFilterDiagnostics ieskf_filter_diagnostics;
  bool last_ieskf_update_valid = false;
  bool last_ieskf_degeneracy_aided = false;
  IMUST state_const_;
  IMUST state_const_last;
  double relative_sqrt_info = 10;
  double min_hor_vel = 0.0;
  double cp_weight = 1.0;
  Eigen::Matrix<double, 24, 24> sqrt_lidar;
  double odo_weight1 = 1.0;
  double odo_weight2 = 1.0;
  double odo_weight3 = 1.0;
  // Eigen::Matrix<double, 3, 3> rot_weight = Eigen::Matrix3d::Identity();
  double odo_weight4 = 2.0;
  double odo_weight5 = 2.0;
  double odo_weight6 = 2.0;
  GNSSAssignment* p_assign = new GNSSAssignment();
  private:
    struct TdcpIeskfSatCache
    {
      double cp_m = 0.0;
      double cp_std_m = 0.05;
      Eigen::Vector3d sat_pos = Eigen::Vector3d::Zero();
      gtime_t ttx;
      double sat_corr_m = 0.0;
    };

    bool extractL1CarrierPhase(
        const ObsPtr &obs, double &cp_m, double &cp_std_m) const;
    Eigen::Vector3d antennaEcef(const IMUST &state) const;

    const ObsPtr obs;
    const EphemBasePtr ephem;
    const std::vector<double> iono_paras;
    int freq_idx = 0;
    double freq = 0.0;
    Eigen::Vector3d sv_pos;
    Eigen::Vector3d sv_vel;
    double svdt, svddt, tgd;
    double pr_uura, dp_uura;
    Eigen::Matrix3d rot_pos;
    double tdcp_ieskf_prev_time = 0.0;
    double tdcp_ieskf_current_time = 0.0;
    bool tdcp_ieskf_prev_valid = false;
    bool tdcp_ieskf_current_valid = false;
    Eigen::Vector3d tdcp_ieskf_prev_receiver_ecef = Eigen::Vector3d::Zero();
    std::map<uint32_t, TdcpIeskfSatCache> tdcp_ieskf_prev_sats;
    std::map<uint32_t, TdcpIeskfSatCache> tdcp_ieskf_current_sats;
    std::vector<SatStatePtr> tdcp_ieskf_current_states;
    mutable std::mutex pvt_cache_mutex;
    std::deque<PvtResult> pvt_cache;
};

// # endif
