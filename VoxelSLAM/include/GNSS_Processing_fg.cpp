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

#include "GNSS_Processing_fg.h"

#include <limits>

GNSSProcess::GNSSProcess()
    : diff_t_gnss_local(0.0)
{
  Reset();
}

GNSSProcess::~GNSSProcess() {}

void GNSSProcess::Reset() 
{
  ROS_WARN("Reset GNSSProcess");
  std::map<sat_first, std::map<uint32_t, double[6]>> empty_map_c;
  sat2cp.swap(empty_map_c);
  // sat2time_index.swap(empty_map_i);
  // sat2ephem.swap(empty_map_e);
  for (size_t i = 0; i < WINDOW_SIZE+1; i++)
  {
    std::vector<ObsPtr> empty_vec_o;
    std::vector<EphemBasePtr> empty_vec_e;
    gnss_meas_buf[i].swap(empty_vec_o);
    gnss_ephem_buf[i].swap(empty_vec_e);
  }
  std::map<uint32_t, uint32_t> empty_map_t;
  std::map<uint32_t, double> empty_map_st;
  p_assign->sat_track_status.swap(empty_map_t);
  p_assign->sat_track_time.swap(empty_map_st);
  p_assign->sat_track_last_time.swap(empty_map_st);
  p_assign->hatch_filter_meas.swap(empty_map_st);
  p_assign->last_cp_meas.swap(empty_map_st);
  p_assign->sum_d = 0;
  p_assign->sum_d2 = 0;
  last_gnss_time = 0.0;
  first_gnss_time = 0.0;
  frame_count = 0;
  invalid_lidar = false;
  current_gnss_factor_enabled = false;
  current_gnss_time = -1.0;
  resetTdcpDopplerIeskf();
  {
    std::lock_guard<std::mutex> lock(pvt_cache_mutex);
    pvt_cache.clear();
  }
  Rot_gnss_init.setIdentity();
  gnss_ready = false;
}

void GNSSProcess::inputIonoParams(double ts, const std::vector<double> &iono_params) 
{
  if (iono_params.size() != 8)    return;

  // update ionosphere parameters
  std::vector<double> empty_vec_d;
  p_assign->latest_gnss_iono_params.swap(empty_vec_d);
  std::copy(iono_params.begin(), iono_params.end(), std::back_inserter(p_assign->latest_gnss_iono_params));
}

void GNSSProcess::inputpvt(double ts, double lat, double lon, double alt,
                           double h_acc, double v_acc,
                           int float_sol, int diff_sol)
{
  Eigen::Vector3d lla;
  lla << lat, lon, alt;
  if (pvt_time.empty())
  {
    first_lla_pvt = lla;
    first_xyz_ecef_pvt = geo2ecef(lla);
    // first_lla_pvt << 22.609671,114.017229,98.401000;
    // first_xyz_ecef_pvt << -2397684.725162,5380932.949883,2436910.600325;
    printf("first ecef xyz 3:%f,%f,%f\n",first_xyz_ecef_pvt(0),first_xyz_ecef_pvt(1),first_xyz_ecef_pvt(2));
    printf("first lla:%f,%f,%f\n",first_lla_pvt(0),first_lla_pvt(1),first_lla_pvt(2));
  }
  Eigen::Vector3d xyz_ecef = geo2ecef(lla);
  Eigen::Vector3d xyz_enu = ecef2enu(first_lla_pvt, xyz_ecef - first_xyz_ecef_pvt);
  pvt_time.push_back(ts);
  pvt_holder.push_back(xyz_enu);
  diff_holder.push_back(diff_sol);
  float_holder.push_back(float_sol);

  PvtResult result;
  result.timestamp = ts - diff_t_gnss_local;
  result.ecef = xyz_ecef;
  const double horizontal_sigma =
      std::max(0.05, std::isfinite(h_acc) ? h_acc : 10.0);
  const double vertical_sigma =
      std::max(0.05, std::isfinite(v_acc) ? v_acc : 20.0);
  result.variance_enu <<
      horizontal_sigma * horizontal_sigma,
      horizontal_sigma * horizontal_sigma,
      vertical_sigma * vertical_sigma;
  {
    std::lock_guard<std::mutex> lock(pvt_cache_mutex);
    pvt_cache.push_back(result);
    while (pvt_cache.size() > 200)
      pvt_cache.pop_front();
  }
}

bool GNSSProcess::matchClosestPvt(
    double local_timestamp, double max_time_error, PvtResult &result)
{
  std::lock_guard<std::mutex> lock(pvt_cache_mutex);
  while (!pvt_cache.empty() &&
         pvt_cache.front().timestamp < local_timestamp - max_time_error)
    pvt_cache.pop_front();
  if (pvt_cache.empty())
    return false;

  auto best = pvt_cache.end();
  double best_error = max_time_error;
  for (auto iter = pvt_cache.begin(); iter != pvt_cache.end(); ++iter)
  {
    const double time_error =
        std::fabs(iter->timestamp - local_timestamp);
    if (time_error <= best_error)
    {
      best = iter;
      best_error = time_error;
    }
    if (iter->timestamp > local_timestamp + max_time_error)
      break;
  }
  if (best == pvt_cache.end())
    return false;

  result = *best;
  pvt_cache.erase(pvt_cache.begin(), std::next(best));
  return true;
}

void GNSSProcess::inputlla(double ts, double lat, double lon, double alt) // 
{
  Eigen::Vector3d lla;
  lla << lat, lon, alt;
  if (lla_time.empty())
  {
    first_lla_lla = lla;
    first_xyz_ecef_lla = geo2ecef(lla);
  }
  Eigen::Vector3d xyz_ecef = geo2ecef(lla);
  Eigen::Vector3d xyz_enu = ecef2enu(first_lla_lla, xyz_ecef - first_xyz_ecef_lla);
  lla_time.push_back(ts);
  lla_holder.push_back(xyz_enu);
}

Eigen::Vector3d GNSSProcess::local2enu(Eigen::Matrix3d R_enu_local_, Eigen::Vector3d anc, Eigen::Vector3d &pos)
{
  Eigen::Vector3d enu_pos;
  Eigen::Vector3d pos_r = pos;
  // Eigen::Vector3d lla_pos = ecef2geo(first_xyz_enu_pvt);
  enu_pos = ecef2enu(first_lla_pvt, pos_r - first_xyz_ecef_pvt);
  return enu_pos;
}

void GNSSProcess::inputGNSSTimeDiff(const double t_diff) // 
{
    diff_t_gnss_local = t_diff;
}

void GNSSProcess::processGNSS(const std::vector<ObsPtr> &gnss_meas, IMUST &state)
{
  std::vector<double>().swap(psr_meas_hatch_filter);
  std::vector<ObsPtr> valid_meas;
  std::vector<EphemBasePtr> valid_ephems;
  valid_ephems.clear();
  valid_meas.clear();
  current_gnss_factor_enabled = false;
  current_gnss_time = gnss_meas.empty() ? -1.0 : time2sec(gnss_meas.front()->time);
  if (gnss_meas.empty())  
  {
    if (gnss_ready)
    {
      std::vector<ObsPtr> empty_vec_o;
      std::vector<EphemBasePtr> empty_vec_e;
      gnss_meas_buf[0].swap(empty_vec_o);
      gnss_ephem_buf[0].swap(empty_vec_e);
    }
    return;
  }

  // The old graph path updated ecef_pos from its optimized anchor before
  // observation filtering.  The TDCP/Doppler IESKF does not own that graph,
  // so derive the antenna position directly from the current LIO state.
  if (gnss_ready)
    ecef_pos = antennaEcef(state);

  p_assign->processGNSSBase(gnss_meas, psr_meas_hatch_filter, valid_meas, valid_ephems, gnss_ready, ecef_pos, last_gnss_time);

  current_gnss_factor_enabled = valid_meas.size() >= min_obs;
  if (!current_gnss_factor_enabled)
  {
    ROS_WARN_THROTTLE(5.0, "GNSS raw factors skipped: valid_obs=%lu min_obs=%lu gnss_ready=%d",
                      valid_meas.size(), min_obs, gnss_ready);
    if (!gnss_ready)
      return;
  }
  
  if (!gnss_ready)
  {
    {
      rot_window[frame_count] = state.R; //.normalized().toRotationMatrix();
      pos_window[frame_count] = state.p;
      vel_window[frame_count] = state.v;
      omg_window[frame_count] = state.omg;
    }
    gnss_meas_buf[frame_count] = valid_meas; 
    gnss_ephem_buf[frame_count] = valid_ephems;
    frame_count ++;
    gnss_ready = GNSSLIAlign();
    if (gnss_ready)
    {
      ROS_INFO("GNSS Initialization is done");
      state_const_ = state;
      state_const_last = state;
    }
  }
  else
  {  
    gnss_meas_buf[0] = valid_meas; 
    gnss_ephem_buf[0] = valid_ephems;
  }
}

bool GNSSProcess::GNSSLIAlign()
{
  if (frame_count < wind_size + 1) return false;
  
  for (uint32_t i = 0; i < wind_size; i++)
  {
    if (time2sec(gnss_meas_buf[i+1][0]->time) - time2sec(gnss_meas_buf[i][0]->time) > 15 * gnss_sample_period) // need IMU to prop
    {
      // if (frame_count == wind_size + 1)
      // {
        for (uint32_t j = i+1; j < wind_size+1; ++j)
        {
          gnss_meas_buf[j-i-1] = gnss_meas_buf[j];
          gnss_ephem_buf[j-i-1] = gnss_ephem_buf[j];
          rot_window[j-i-1] = rot_window[j];
          pos_window[j-i-1] = pos_window[j];
          vel_window[j-i-1] = vel_window[j];
          omg_window[j-i-1] = omg_window[j];
        }
        frame_count -= i+1;
        for (uint32_t j = frame_count; j < wind_size+1; ++j) // wind_size-i
        {
          std::vector<ObsPtr> empty_vec_o;
          std::vector<EphemBasePtr> empty_vec_e;
          gnss_meas_buf[j].swap(empty_vec_o);
          gnss_ephem_buf[j].swap(empty_vec_e); 
        }             
      // }
      return false;
    }
  }

  auto drop_oldest_gnss_frame = [&]()
  {
    for (uint32_t i = 0; i < static_cast<uint32_t>(wind_size); ++i)
    {
      gnss_meas_buf[i] = gnss_meas_buf[i+1];
      gnss_ephem_buf[i] = gnss_ephem_buf[i+1];
      rot_window[i] = rot_window[i+1];
      pos_window[i] = pos_window[i+1];
      vel_window[i] = vel_window[i+1];
      omg_window[i] = omg_window[i+1];
      pos_ecef_window[i] = pos_ecef_window[i+1];
      for (uint32_t k = 0; k < 4; ++k)
        para_rcv_dt[4*i+k] = para_rcv_dt[4*(i+1)+k];
    }
    frame_count = wind_size;
    std::vector<ObsPtr>().swap(gnss_meas_buf[frame_count]);
    std::vector<EphemBasePtr>().swap(gnss_ephem_buf[frame_count]);
  };

  std::vector<std::vector<ObsPtr>> curr_gnss_meas_buf;
  std::vector<std::vector<EphemBasePtr>> curr_gnss_ephem_buf;
  curr_gnss_meas_buf.reserve(wind_size+1);
  curr_gnss_ephem_buf.reserve(wind_size+1);
  for (uint32_t i = 0; i < (wind_size+1); ++i)
  {
    curr_gnss_meas_buf.push_back(gnss_meas_buf[i]);
    curr_gnss_ephem_buf.push_back(gnss_ephem_buf[i]);
  }

  GNSSLIInitializer gnss_li_initializer(curr_gnss_meas_buf, curr_gnss_ephem_buf, p_assign->latest_gnss_iono_params);

  // 1. get a rough global location
  Eigen::Matrix<double, 7, 1> rough_xyzt;
  rough_xyzt.setZero();
  if (!gnss_li_initializer.coarse_localization(rough_xyzt))
  {
    std::cerr << "Fail to obtain a coarse location.\n";
    drop_oldest_gnss_frame();
    return false;
  }

  uint32_t observed_dt_idx = 4;
  for (uint32_t k = 0; k < 4; ++k)
  {
    if (fabs(rough_xyzt(3+k)) > 0)
    {
      observed_dt_idx = k;
      break;
    }
  }
  if (observed_dt_idx == 4)
  {
    std::cerr << "Fail to quick init anchor point.\n";
    drop_oldest_gnss_frame();
    return false;
  }

  std::vector<Eigen::Vector3d> local_ps;
  std::vector<Eigen::Vector3d> local_vs;
  std::vector<Eigen::Matrix3d> local_rots;
  std::vector<Eigen::Vector3d> local_omegas;
  local_ps.reserve(wind_size+1);
  local_vs.reserve(wind_size+1);
  std::vector<std::vector<ObsPtr>> yaw_gnss_meas_buf;
  std::vector<std::vector<EphemBasePtr>> yaw_gnss_ephem_buf;
  std::vector<Eigen::Vector3d> yaw_local_vs;
  std::vector<Eigen::Matrix3d> yaw_local_rots;
  std::vector<Eigen::Vector3d> yaw_local_omegas;
  uint32_t yaw_meas_num = 0;
  for (uint32_t i = 0; i < (wind_size+1); ++i)
  {
    local_ps.push_back(pos_window[i]);
    local_vs.push_back(vel_window[i]);
    local_rots.push_back(rot_window[i]);
    local_omegas.push_back(omg_window[i]);
    const double hor_vel = local_vs.back().head<2>().norm();
    if (min_hor_vel <= 0.0 || hor_vel >= min_hor_vel)
    {
      yaw_gnss_meas_buf.push_back(gnss_meas_buf[i]);
      yaw_gnss_ephem_buf.push_back(gnss_ephem_buf[i]);
      yaw_local_vs.push_back(local_vs.back());
      yaw_local_rots.push_back(local_rots.back());
      yaw_local_omegas.push_back(local_omegas.back());
      yaw_meas_num += gnss_meas_buf[i].size();
    }
  }

  // 2. align local frame yaw in ENU with Doppler velocity residuals
  yaw_enu_local = 0.0;
  para_rcv_ddt[0] = 0.0;
  if (yaw_local_vs.size() < 2 || yaw_meas_num < 2)
  {
    ROS_WARN("GNSS yaw initialization waits for enough moving frames: usable_frames=%zu usable_obs=%u min_hor_vel=%.3f",
             yaw_local_vs.size(), yaw_meas_num, min_hor_vel);
    drop_oldest_gnss_frame();
    return false;
  }
  GNSSLIInitializer yaw_gnss_li_initializer(yaw_gnss_meas_buf, yaw_gnss_ephem_buf, p_assign->latest_gnss_iono_params);
  const Eigen::Vector3d lever_arm_initial = Tex_imu_r;
  if (!yaw_gnss_li_initializer.yaw_alignment(
          yaw_local_vs, yaw_local_rots, yaw_local_omegas,
          rough_xyzt.head<3>(), yaw_enu_local, para_rcv_ddt[0], Tex_imu_r))
  {
    std::cerr << "Fail to initialize yaw offset.\n";
    drop_oldest_gnss_frame();
    return false;
  }

  // 3. refine the anchor point and receiver clock bias with pseudorange residuals
  Eigen::Matrix<double, 7, 1> refined_xyzt;
  refined_xyzt.setZero();
  std::vector<Eigen::Vector3d> antenna_local_ps;
  antenna_local_ps.reserve(local_ps.size());
  for (size_t i = 0; i < local_ps.size(); ++i)
    antenna_local_ps.push_back(local_ps[i] + local_rots[i] * Tex_imu_r);
  if (!gnss_li_initializer.anchor_refinement(
          antenna_local_ps, yaw_enu_local, para_rcv_ddt[0],
          rough_xyzt, refined_xyzt))
  {
    std::cerr << "Fail to refine anchor point.\n";
    drop_oldest_gnss_frame();
    return false;
  }

  const Eigen::Matrix3d R_ecef_local =
      ecef2rotation(refined_xyzt.head<3>()) * Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()).matrix();
  anc_local = antenna_local_ps[0];
  anc_ecef = refined_xyzt.head<3>() - R_ecef_local * anc_local;
  R_ecef_enu = R_ecef_local;

  const double first_time = time2sec(gnss_meas_buf[0][0]->time);
  for (uint32_t i = 0; i < (wind_size+1); ++i)
  {
    const double dt = time2sec(gnss_meas_buf[i][0]->time) - first_time;
    for (uint32_t k = 0; k < 4; ++k)
    {
      const double base_dt = rough_xyzt(3+k) == 0.0 ? refined_xyzt(3+observed_dt_idx) : refined_xyzt(3+k);
      para_rcv_dt[4*i+k] = base_dt + para_rcv_ddt[0] * dt;
    }
  }

  std::cout << "GNSS coarse localization: " << rough_xyzt.transpose() << std::endl;
  std::cout << "GNSS yaw alignment: " << yaw_enu_local << ", ddt: " << para_rcv_ddt[0] << std::endl;
  ROS_INFO(
      "GNSS optimized lever arm [m]: %.6f %.6f %.6f, delta: %.6f %.6f %.6f",
      Tex_imu_r.x(), Tex_imu_r.y(), Tex_imu_r.z(),
      Tex_imu_r.x() - lever_arm_initial.x(),
      Tex_imu_r.y() - lever_arm_initial.y(),
      Tex_imu_r.z() - lever_arm_initial.z());
  std::cout << "GNSS refined anchor: " << refined_xyzt.transpose() << std::endl;

    last_gnss_time = time2sec(gnss_meas_buf[wind_size][0]->time);
    first_gnss_time = time2sec(gnss_meas_buf[wind_size][0]->time);

  for (uint32_t k_ = 1; k_ < wind_size+1; k_++)
  {
    std::vector<ObsPtr>().swap(gnss_meas_buf[k_]);
    std::vector<EphemBasePtr>().swap(gnss_ephem_buf[k_]);
  }
  return true;
}

bool GNSSProcess::extractL1CarrierPhase(
    const ObsPtr &obs_, double &cp_m, double &cp_std_m) const
{
  if (!obs_)
    return false;

  int l1_idx = -1;
  const double l1_freq = L1_freq(obs_, &l1_idx);
  if (l1_idx < 0 || l1_freq <= 0.0 ||
      l1_idx >= static_cast<int>(obs_->cp.size()) ||
      l1_idx >= static_cast<int>(obs_->cp_std.size()))
    return false;
  if (!std::isfinite(obs_->cp[l1_idx]) ||
      std::fabs(obs_->cp[l1_idx]) < 1e-6)
    return false;
  if (l1_idx < static_cast<int>(obs_->status.size()) &&
      (obs_->status[l1_idx] & 0x02) == 0)
    return false;
  if (l1_idx < static_cast<int>(obs_->LLI.size()) &&
      obs_->LLI[l1_idx] != 0)
    return false;

  const double wavelength = LIGHT_SPEED / l1_freq;
  cp_m = obs_->cp[l1_idx] * wavelength;
  cp_std_m = 0.05;
  if (std::isfinite(obs_->cp_std[l1_idx]) && obs_->cp_std[l1_idx] > 0.0)
    cp_std_m = std::max(0.02, obs_->cp_std[l1_idx] * wavelength);
  return std::isfinite(cp_m) && std::isfinite(cp_std_m) && cp_std_m <= 1.0;
}

Eigen::Vector3d GNSSProcess::antennaEcef(const IMUST &state) const
{
  return anc_ecef + R_ecef_enu * (state.p + state.R * Tex_imu_r);
}

void GNSSProcess::resetTdcpDopplerIeskf()
{
  tdcp_ieskf_prev_time = 0.0;
  tdcp_ieskf_current_time = 0.0;
  tdcp_ieskf_prev_valid = false;
  tdcp_ieskf_current_valid = false;
  tdcp_ieskf_prev_receiver_ecef.setZero();
  tdcp_ieskf_prev_sats.clear();
  tdcp_ieskf_current_sats.clear();
  tdcp_ieskf_current_states.clear();
  last_ieskf_update_valid = false;
  last_ieskf_degeneracy_aided = false;
  ieskf_filter_diagnostics = IeskfFilterDiagnostics();
}

bool GNSSProcess::prepareTdcpDopplerIeskf(const IMUST &state)
{
  ieskf_filter_diagnostics = IeskfFilterDiagnostics();
  tdcp_ieskf_current_valid = false;
  tdcp_ieskf_current_sats.clear();
  tdcp_ieskf_current_states.clear();
  last_ieskf_update_valid = false;
  last_ieskf_degeneracy_aided = false;

  const std::vector<ObsPtr> &current_obs = gnss_meas_buf[0];
  const std::vector<EphemBasePtr> &current_ephems = gnss_ephem_buf[0];
  ieskf_filter_diagnostics.raw_observations =
      static_cast<int>(current_obs.size());
  if (!gnss_ready)
  {
    ieskf_filter_diagnostics.prepare_status =
        IeskfPrepareStatus::GnssDisabled;
    return false;
  }
  if (!current_gnss_factor_enabled || current_obs.size() < min_obs)
  {
    ieskf_filter_diagnostics.prepare_status =
        IeskfPrepareStatus::InsufficientObservations;
    return false;
  }
  if (current_obs.size() != current_ephems.size())
  {
    ieskf_filter_diagnostics.prepare_status =
        IeskfPrepareStatus::EphemerisMismatch;
    return false;
  }

  tdcp_ieskf_current_states = sat_states(current_obs, current_ephems);
  if (tdcp_ieskf_current_states.size() != current_obs.size())
  {
    ieskf_filter_diagnostics.prepare_status =
        IeskfPrepareStatus::SatelliteStateFailure;
    return false;
  }

  for (size_t i = 0; i < current_obs.size(); ++i)
  {
    const SatStatePtr &sat_state = tdcp_ieskf_current_states[i];
    if (!sat_state || !sat_state->pos.allFinite() ||
        sat_state->pos.norm() < 1.0)
      continue;

    double cp_m = 0.0;
    double cp_std_m = 0.05;
    if (!extractL1CarrierPhase(current_obs[i], cp_m, cp_std_m))
      continue;

    TdcpIeskfSatCache cache;
    cache.cp_m = cp_m;
    cache.cp_std_m = cp_std_m;
    cache.sat_pos = sat_state->pos;
    cache.ttx = sat_state->ttx;
    cache.sat_corr_m = -sat_state->dt * LIGHT_SPEED;
    tdcp_ieskf_current_sats[current_obs[i]->sat] = cache;
  }
  ieskf_filter_diagnostics.carrier_phase_valid =
      static_cast<int>(tdcp_ieskf_current_sats.size());

  tdcp_ieskf_current_time = time2sec(current_obs.front()->time);
  tdcp_ieskf_current_valid =
      std::isfinite(tdcp_ieskf_current_time) &&
      tdcp_ieskf_current_sats.size() >= 4;
  if (!tdcp_ieskf_current_valid)
  {
    ieskf_filter_diagnostics.prepare_status =
        IeskfPrepareStatus::InsufficientCarrierPhase;
    return false;
  }
  if (!tdcp_ieskf_prev_valid)
  {
    ieskf_filter_diagnostics.prepare_status =
        IeskfPrepareStatus::NoPreviousEpoch;
    return false;
  }

  const double dt = tdcp_ieskf_current_time - tdcp_ieskf_prev_time;
  const double max_dt = std::min(
      gnss_cp_time_threshold, std::max(2.5 * gnss_sample_period, 0.5));
  ieskf_filter_diagnostics.epoch_time_gap = dt;
  ieskf_filter_diagnostics.maximum_time_gap = max_dt;
  if (dt <= 0.0 || dt > max_dt)
  {
    ieskf_filter_diagnostics.prepare_status =
        IeskfPrepareStatus::InvalidTimeGap;
    return false;
  }

  size_t common_satellites = 0;
  for (const auto &sat : tdcp_ieskf_current_sats)
    if (tdcp_ieskf_prev_sats.count(sat.first) > 0)
      ++common_satellites;
  ieskf_filter_diagnostics.common_carrier_phase =
      static_cast<int>(common_satellites);
  if (common_satellites < 4)
  {
    ieskf_filter_diagnostics.prepare_status =
        IeskfPrepareStatus::InsufficientCommonCarrierPhase;
    return false;
  }
  if (!antennaEcef(state).allFinite())
  {
    ieskf_filter_diagnostics.prepare_status =
        IeskfPrepareStatus::InvalidReceiverState;
    return false;
  }
  ieskf_filter_diagnostics.prepare_status = IeskfPrepareStatus::Ready;
  return true;
}

bool GNSSProcess::buildTdcpDopplerIeskfNormal(
    const IMUST &state,
    const Eigen::Matrix<double, 6, 6> &position_velocity_cov,
    bool apply_chi_square_test,
    IeskfNormalEquation &normal) const
{
  normal = IeskfNormalEquation();
  ieskf_filter_diagnostics.doppler_candidates = 0;
  ieskf_filter_diagnostics.doppler_invalid = 0;
  ieskf_filter_diagnostics.doppler_gross_rejected = 0;
  ieskf_filter_diagnostics.tdcp_candidates = 0;
  ieskf_filter_diagnostics.tdcp_geometry_rejected = 0;
  ieskf_filter_diagnostics.tdcp_elevation_rejected = 0;
  ieskf_filter_diagnostics.tdcp_gross_rejected = 0;
  ieskf_filter_diagnostics.chi_square_rejected = 0;
  if (!tdcp_ieskf_current_valid || !tdcp_ieskf_prev_valid ||
      tdcp_ieskf_current_states.size() != gnss_meas_buf[0].size())
    return false;

  const double dt = tdcp_ieskf_current_time - tdcp_ieskf_prev_time;
  const Eigen::Vector3d receiver_ecef = antennaEcef(state);
  Eigen::Matrix3d omega_skew;
  omega_skew << SKEW_SYM_MATRX(state.omg);
  const Eigen::Vector3d antenna_velocity_local =
      state.v + state.R * omega_skew * Tex_imu_r;

  Eigen::Matrix<double, 4, 1> ecef_velocity_clock;
  ecef_velocity_clock.head<3>() = R_ecef_enu * antenna_velocity_local;
  ecef_velocity_clock(3) = para_rcv_ddt[0];

  Eigen::VectorXd doppler_residual;
  Eigen::MatrixXd doppler_jacobian;
  dopp_res(ecef_velocity_clock, receiver_ecef, gnss_meas_buf[0],
           tdcp_ieskf_current_states, doppler_residual, doppler_jacobian);

  constexpr double chi_square_threshold = 6.635;
  auto accept_residual = [&](double residual,
                             const Eigen::Matrix<double, 1, 6> &jacobian,
                             double sigma)
  {
    if (!apply_chi_square_test)
      return true;
    const double innovation_variance =
        std::max(1e-8,
                 sigma * sigma +
                 (jacobian * position_velocity_cov *
                  jacobian.transpose())(0, 0));
    return residual * residual / innovation_variance <=
           chi_square_threshold;
  };

  struct ResidualCandidate
  {
    double residual = 0.0;
    double sigma = 1.0;
    double clock_jacobian = 0.0;
    Eigen::Matrix<double, 1, 6> state_jacobian =
        Eigen::Matrix<double, 1, 6>::Zero();
    bool is_tdcp = false;
  };
  std::vector<ResidualCandidate> candidates;
  candidates.reserve(2 * gnss_meas_buf[0].size());

  for (size_t i = 0; i < gnss_meas_buf[0].size(); ++i)
  {
    if (i >= static_cast<size_t>(doppler_residual.size()) ||
        i >= static_cast<size_t>(doppler_jacobian.rows()))
    {
      ++ieskf_filter_diagnostics.doppler_invalid;
      continue;
    }

    int l1_idx = -1;
    const double l1_freq = L1_freq(gnss_meas_buf[0][i], &l1_idx);
    if (l1_idx < 0 || l1_freq <= 0.0 ||
        l1_idx >= static_cast<int>(gnss_meas_buf[0][i]->dopp_std.size()))
    {
      ++ieskf_filter_diagnostics.doppler_invalid;
      continue;
    }
    ++ieskf_filter_diagnostics.doppler_candidates;
    const double sigma = std::max(
        0.05,
        gnss_meas_buf[0][i]->dopp_std[l1_idx] *
        LIGHT_SPEED / l1_freq);
    const double residual = doppler_residual(i);
    if (!std::isfinite(residual) ||
        std::fabs(residual) > std::max(10.0, 8.0 * sigma))
    {
      ++ieskf_filter_diagnostics.doppler_gross_rejected;
      continue;
    }

    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian.setZero();
    jacobian.block<1, 3>(0, 3) =
        doppler_jacobian.block<1, 3>(i, 0) * R_ecef_enu;
    ResidualCandidate candidate;
    candidate.residual = residual;
    candidate.sigma = sigma;
    candidate.clock_jacobian = doppler_jacobian(i, 3);
    candidate.state_jacobian = jacobian;
    candidates.push_back(candidate);
  }

  const vector<double> &iono = p_assign->latest_gnss_iono_params;
  for (const auto &sat : tdcp_ieskf_current_sats)
  {
    const auto previous = tdcp_ieskf_prev_sats.find(sat.first);
    if (previous == tdcp_ieskf_prev_sats.end())
      continue;
    ++ieskf_filter_diagnostics.tdcp_candidates;

    const TdcpIeskfSatCache &current_cp = sat.second;
    const TdcpIeskfSatCache &previous_cp = previous->second;
    const Eigen::Vector3d current_los =
        current_cp.sat_pos - receiver_ecef;
    const Eigen::Vector3d previous_los =
        previous_cp.sat_pos - tdcp_ieskf_prev_receiver_ecef;
    if (current_los.norm() < 1.0 || previous_los.norm() < 1.0)
    {
      ++ieskf_filter_diagnostics.tdcp_geometry_rejected;
      continue;
    }

    double current_azel[2] = {0.0, M_PI / 2.0};
    double previous_azel[2] = {0.0, M_PI / 2.0};
    sat_azel(receiver_ecef, current_cp.sat_pos, current_azel);
    sat_azel(tdcp_ieskf_prev_receiver_ecef,
             previous_cp.sat_pos, previous_azel);
    if (current_azel[1] < p_assign->gnss_elevation_threshold * M_PI / 180.0 ||
        previous_azel[1] < p_assign->gnss_elevation_threshold * M_PI / 180.0)
    {
      ++ieskf_filter_diagnostics.tdcp_elevation_rejected;
      continue;
    }

    const double current_sagnac = EARTH_OMG_GPS *
        (current_cp.sat_pos.x() * receiver_ecef.y() -
         current_cp.sat_pos.y() * receiver_ecef.x()) / LIGHT_SPEED;
    const double previous_sagnac = EARTH_OMG_GPS *
        (previous_cp.sat_pos.x() * tdcp_ieskf_prev_receiver_ecef.y() -
         previous_cp.sat_pos.y() * tdcp_ieskf_prev_receiver_ecef.x()) /
        LIGHT_SPEED;
    const Eigen::Vector3d current_lla = ecef2geo(receiver_ecef);
    const Eigen::Vector3d previous_lla =
        ecef2geo(tdcp_ieskf_prev_receiver_ecef);
    const double current_trop = calculate_trop_delay(
        current_cp.ttx, current_lla, current_azel);
    const double previous_trop = calculate_trop_delay(
        previous_cp.ttx, previous_lla, previous_azel);
    const double current_iono = iono.size() == 8 ?
        calculate_ion_delay(
            current_cp.ttx, iono, current_lla, current_azel) : 0.0;
    const double previous_iono = iono.size() == 8 ?
        calculate_ion_delay(
            previous_cp.ttx, iono, previous_lla, previous_azel) : 0.0;

    const double predicted_delta =
        current_los.norm() - previous_los.norm() +
        current_sagnac - previous_sagnac +
        current_cp.sat_corr_m - previous_cp.sat_corr_m +
        current_trop - previous_trop -
        current_iono + previous_iono +
        para_rcv_ddt[0] * dt;
    const double measured_delta = current_cp.cp_m - previous_cp.cp_m;
    const double residual = predicted_delta - measured_delta;
    const double sigma = std::max(
        0.05, std::sqrt(
            current_cp.cp_std_m * current_cp.cp_std_m +
            previous_cp.cp_std_m * previous_cp.cp_std_m));
    if (!std::isfinite(residual) ||
        std::fabs(residual) > std::max(10.0, 8.0 * sigma))
    {
      ++ieskf_filter_diagnostics.tdcp_gross_rejected;
      continue;
    }

    Eigen::Matrix<double, 1, 6> jacobian;
    jacobian.setZero();
    jacobian.block<1, 3>(0, 0) =
        -current_los.normalized().transpose() * R_ecef_enu;
    ResidualCandidate candidate;
    candidate.residual = residual;
    candidate.sigma = sigma;
    candidate.clock_jacobian = dt;
    candidate.state_jacobian = jacobian;
    candidate.is_tdcp = true;
    candidates.push_back(candidate);
  }

  // Receiver clock drift is common to all TDCP and Doppler residuals but is
  // not part of the LIO state. Remove its robust epoch-wise estimate before
  // per-satellite gating, then marginalize it from the accepted normal system.
  std::vector<double> clock_corrections;
  clock_corrections.reserve(candidates.size());
  for (const ResidualCandidate &candidate : candidates)
  {
    if (std::fabs(candidate.clock_jacobian) > 1e-8)
      clock_corrections.push_back(
          -candidate.residual / candidate.clock_jacobian);
  }
  if (clock_corrections.empty())
    return false;

  const size_t median_index = clock_corrections.size() / 2;
  std::nth_element(clock_corrections.begin(),
                   clock_corrections.begin() + median_index,
                   clock_corrections.end());
  normal.clock_drift_correction = clock_corrections[median_index];

  Eigen::Matrix<double, 6, 6> state_hessian =
      Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 1> state_gradient =
      Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> state_clock_hessian =
      Eigen::Matrix<double, 6, 1>::Zero();
  double clock_hessian = 0.0;
  double clock_gradient = 0.0;
  for (const ResidualCandidate &candidate : candidates)
  {
    const double corrected_residual =
        candidate.residual +
        candidate.clock_jacobian * normal.clock_drift_correction;
    if (!accept_residual(corrected_residual,
                         candidate.state_jacobian,
                         candidate.sigma))
    {
      ++normal.chi_square_rejected;
      continue;
    }

    const double weight = 1.0 / (candidate.sigma * candidate.sigma);
    state_hessian += weight *
        candidate.state_jacobian.transpose() * candidate.state_jacobian;
    state_gradient -= weight *
        candidate.state_jacobian.transpose() * corrected_residual;
    state_clock_hessian += weight *
        candidate.state_jacobian.transpose() * candidate.clock_jacobian;
    clock_hessian += weight *
        candidate.clock_jacobian * candidate.clock_jacobian;
    clock_gradient -= weight *
        candidate.clock_jacobian * corrected_residual;
    if (candidate.is_tdcp)
      ++normal.tdcp_accepted;
    else
      ++normal.doppler_accepted;
  }

  normal.valid =
      normal.tdcp_accepted >= 4 &&
      normal.doppler_accepted >= static_cast<int>(min_obs) &&
      clock_hessian > 1e-12;
  if (normal.valid)
  {
    normal.hessian = state_hessian -
        state_clock_hessian * state_clock_hessian.transpose() /
        clock_hessian;
    normal.gradient = state_gradient -
        state_clock_hessian * (clock_gradient / clock_hessian);
  }
  ieskf_filter_diagnostics.chi_square_rejected =
      normal.chi_square_rejected;
  return normal.valid;
}

const char *GNSSProcess::ieskfPrepareStatusString() const
{
  switch (ieskf_filter_diagnostics.prepare_status)
  {
    case IeskfPrepareStatus::NotRequested: return "not-requested";
    case IeskfPrepareStatus::Ready: return "ready";
    case IeskfPrepareStatus::GnssDisabled: return "gnss-disabled";
    case IeskfPrepareStatus::InsufficientObservations: return "insufficient-observations";
    case IeskfPrepareStatus::EphemerisMismatch: return "ephemeris-mismatch";
    case IeskfPrepareStatus::SatelliteStateFailure: return "satellite-state-failure";
    case IeskfPrepareStatus::InsufficientCarrierPhase: return "insufficient-carrier-phase";
    case IeskfPrepareStatus::NoPreviousEpoch: return "no-previous-epoch";
    case IeskfPrepareStatus::InvalidTimeGap: return "invalid-time-gap";
    case IeskfPrepareStatus::InsufficientCommonCarrierPhase: return "insufficient-common-carrier";
    case IeskfPrepareStatus::InvalidReceiverState: return "invalid-receiver-state";
  }
  return "unknown";
}

void GNSSProcess::commitTdcpDopplerIeskf(const IMUST &state)
{
  if (!tdcp_ieskf_current_valid)
    return;

  tdcp_ieskf_prev_time = tdcp_ieskf_current_time;
  tdcp_ieskf_prev_receiver_ecef = antennaEcef(state);
  tdcp_ieskf_prev_sats = tdcp_ieskf_current_sats;
  tdcp_ieskf_prev_valid = !tdcp_ieskf_prev_sats.empty();
  tdcp_ieskf_current_valid = false;
  tdcp_ieskf_current_sats.clear();
  tdcp_ieskf_current_states.clear();
}