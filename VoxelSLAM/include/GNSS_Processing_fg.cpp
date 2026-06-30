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

namespace {
bool valuesContainKey(const gtsam::Values &values, gtsam::Key key)
{
  return values.exists(key);
}
}

GNSSProcess::GNSSProcess()
    : diff_t_gnss_local(0.0)
{
  Reset();
  // initNoises();
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
  p_assign->change_ext = 1;
  std::map<uint32_t, uint32_t> empty_map_t;
  std::map<uint32_t, double> empty_map_st;
  p_assign->sat_track_status.swap(empty_map_t);
  p_assign->sat_track_time.swap(empty_map_st);
  p_assign->sat_track_last_time.swap(empty_map_st);
  p_assign->hatch_filter_meas.swap(empty_map_st);
  p_assign->last_cp_meas.swap(empty_map_st);
  p_assign->gtSAMgraph.resize(0); 
  p_assign->initialEstimate.clear();
  p_assign->isamCurrentEstimate.clear();
  p_assign->sum_d = 0;
  p_assign->sum_d2 = 0;
  // p_assign->hatch_filter_meas = 0;
  // p_assign->last_cp = 0;
  // index_delete = 0;
  frame_delete = 0;
  p_assign->factor_id_frame.clear();
  id_accumulate = 0;
  frame_num = 0;
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
  p_assign->process_feat_num = 0;
  gnss_ready = false;
  {
    pre_integration->repropagate(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  }

  gtsam::ISAM2Params parameters;
  parameters.relinearizeThreshold = 0.1;
  parameters.relinearizeSkip = 5; // may matter? improtant!
  p_assign->isam = gtsam::ISAM2(parameters);
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
  if (!nolidar)
  {
    enu_pos = R_enu_local_ * pos; // - anc_local); // 

    // Eigen::Matrix3d R_ecef_enu_ = ecef2rotation(anc);
    // Eigen::Vector3d ecef_pos_ = anc + R_ecef_enu_ * enu_pos;
    Eigen::Vector3d ecef_pos_ = anc + enu_pos;
    // Eigen::Vector3d lla_pos = ecef2geo(first_xyz_enu_pvt);
    enu_pos = ecef2enu(first_lla_pvt, ecef_pos_ - first_xyz_ecef_pvt);
  }
  else
  {
    Eigen::Vector3d pos_r = pos;
    // Eigen::Vector3d lla_pos = ecef2geo(first_xyz_enu_pvt);
    enu_pos = ecef2enu(first_lla_pvt, pos_r - first_xyz_ecef_pvt);
  }
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

  if (gnss_ready)
  {
    Eigen::Vector3d pos_gnss = state.p + state.R * Tex_imu_r; // .normalized()
    updateGNSSStatistics(pos_gnss);
  }
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

void GNSSProcess::runISAM2opt(void) //
{
  gtsam::FactorIndices delete_factor;
  gtsam::FactorIndices().swap(delete_factor);

  if (gnss_ready)
  {
    bool delete_happen = false;
    if (frame_num - frame_delete > delete_thred) // (graph_whole1.size() - index_delete > 4000)
    {
      delete_happen = true;
    while (frame_num - frame_delete > delete_thred) // (graph_whole1.size() - index_delete > 3000)
    { 
      if (!p_assign->factor_id_frame.empty())       
      {
        // if (frame_delete > 0)
        {
        for (size_t i = 0; i < p_assign->factor_id_frame[0].size(); i++)
        {
          {
            delete_factor.push_back(p_assign->factor_id_frame[0][i]);
          }
        }
        // index_delete += p_assign->factor_id_frame[0].size();
        }
      
        p_assign->factor_id_frame.pop_front();
        frame_delete ++;
      }
      if (p_assign->factor_id_frame.empty()) break;
    }
    }

    if (delete_happen)
    {
      p_assign->delete_variables(nolidar, frame_delete, frame_num, id_accumulate, delete_factor);
    }
    else
    {
      p_assign->isam.update(p_assign->gtSAMgraph, p_assign->initialEstimate);
      p_assign->gtSAMgraph.resize(0); // will the initialEstimate change?
      p_assign->initialEstimate.clear();
      for (int i = 0; i < 3; ++i)
        p_assign->isam.update();
    }
  }
  else
  {
    p_assign->isam.update(p_assign->gtSAMgraph, p_assign->initialEstimate);
    p_assign->gtSAMgraph.resize(0); // will the initialEstimate change?
    p_assign->initialEstimate.clear();
    for (int i = 0; i < 3; ++i)
      p_assign->isam.update();
  }
  p_assign->isamCurrentEstimate = p_assign->isam.calculateEstimate();
  
  if (nolidar) // || invalid_lidar)
  {
    pre_integration->repropagate(p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(6),
                                p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(9));
  }
  std::map<sat_first, std::map<uint32_t, double[6]>>::iterator it_old;
  if (!sat2cp.empty())
  {
    it_old = sat2cp.begin();
    while (it_old->first.frame_num < frame_delete)
    {
      std::map<uint32_t, double[6]>().swap(it_old->second);
      size_t del_size = sat2cp.erase(it_old->first);
      if (sat2cp.empty()) break;
      it_old = sat2cp.begin();
    }
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

    SetInit();
    frame_num = 1; // frame_count;
    last_gnss_time = time2sec(gnss_meas_buf[wind_size][0]->time);
    first_gnss_time = time2sec(gnss_meas_buf[wind_size][0]->time);
    // printf("first gnss time: %f", first_gnss_time);
  // }

  for (uint32_t k_ = 1; k_ < wind_size+1; k_++)
  {
    std::vector<ObsPtr>().swap(gnss_meas_buf[k_]);
    std::vector<EphemBasePtr>().swap(gnss_ephem_buf[k_]);
  }
  runISAM2opt();
  return true;
}

void GNSSProcess::updateGNSSStatistics(Eigen::Vector3d &pos) // delete
{
  if (!nolidar)
  {
    Eigen::Vector3d anc_cur;
    Eigen::Matrix3d R_enu_local_;
    // if (frame_num == 1)
    // {
    //   anc_cur = anc_ecef;
    //   R_enu_local_ = R_ecef_enu * Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()) * Rot_gnss_init;
    // }
    // else
    {
      anc_cur = p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0));
      R_enu_local_ = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix();
    }
    Eigen::Vector3d enu_pos = R_enu_local_ * pos; // - anc_local);
    // R_ecef_enu = ecef2rotation(anc_cur);
    // ecef_pos = anc_cur + R_ecef_enu * enu_pos;
    ecef_pos = anc_cur + enu_pos;
  }
  else
  {
    ecef_pos = pos;
  }
}

void GNSSProcess::GnssPsrDoppMeas(const ObsPtr &obs_, const EphemBasePtr &ephem_) 
{
  freq = L1_freq(obs_, &freq_idx);
  LOG_IF(FATAL, freq < 0) << "No L1 observation found.";

  uint32_t sys = satsys(obs_->sat, NULL);
  double tof = obs_->psr[freq_idx] / LIGHT_SPEED;
  gtime_t sv_tx = time_add(obs_->time, -tof);

  if (sys == SYS_GLO)
  {
      GloEphemPtr glo_ephem = std::dynamic_pointer_cast<GloEphem>(ephem_);
      svdt = geph2svdt(sv_tx, glo_ephem);
      sv_tx = time_add(sv_tx, -svdt);
      sv_pos = geph2pos(sv_tx, glo_ephem, &svdt);
      sv_vel = geph2vel(sv_tx, glo_ephem, &svddt);
      tgd = 0.0;
      pr_uura = 2.0 * (obs_->psr_std[freq_idx]/0.16);
      dp_uura = 2.0 * (obs_->dopp_std[freq_idx]/0.256);
  }
  else
  {
      EphemPtr eph = std::dynamic_pointer_cast<Ephem>(ephem_);
      svdt = eph2svdt(sv_tx, eph); // used in eva
      sv_tx = time_add(sv_tx, -svdt);
      sv_pos = eph2pos(sv_tx, eph, &svdt); // used in eva
      sv_vel = eph2vel(sv_tx, eph, &svddt); // used in eva
      tgd = eph->tgd[0];
      if (sys == SYS_GAL)
      {
          pr_uura = (eph->ura - 2.0) * (obs_->psr_std[freq_idx]/0.16);
          dp_uura = (eph->ura - 2.0) * (obs_->dopp_std[freq_idx]/0.256);
      }
      else
      {
          pr_uura = (eph->ura - 1.0) * (obs_->psr_std[freq_idx]/0.16);
          dp_uura = (eph->ura - 1.0) * (obs_->dopp_std[freq_idx]/0.256);
      }
  }
  LOG_IF(FATAL, pr_uura <= 0) << "pr_uura is " << pr_uura; // get those parameters mainly, both used in eva
  LOG_IF(FATAL, dp_uura <= 0) << "dp_uura is " << dp_uura;
  // relative_sqrt_info = 10;
}

void GNSSProcess::SvPosCals(const ObsPtr &obs_, const EphemBasePtr &ephem_) 
{
  freq = L1_freq(obs_, &freq_idx);
  LOG_IF(FATAL, freq < 0) << "No L1 observation found.";

  uint32_t sys = satsys(obs_->sat, NULL);
  double tof = obs_->psr[freq_idx] / LIGHT_SPEED;
  gtime_t sv_tx = time_add(obs_->time, -tof);

  if (sys == SYS_GLO)
  {
      GloEphemPtr glo_ephem = std::dynamic_pointer_cast<GloEphem>(ephem_);
      svdt = geph2svdt(sv_tx, glo_ephem);
      sv_tx = time_add(sv_tx, -svdt);
      sv_pos = geph2pos(sv_tx, glo_ephem, &svdt);
      sv_vel = geph2vel(sv_tx, glo_ephem, &svddt);
  }
  else
  {
      EphemPtr eph = std::dynamic_pointer_cast<Ephem>(ephem_);
      svdt = eph2svdt(sv_tx, eph); // used in eva
      sv_tx = time_add(sv_tx, -svdt);
      sv_pos = eph2pos(sv_tx, eph, &svdt); // used in eva
      sv_vel = eph2vel(sv_tx, eph, &svddt); // used in eva
  }
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

bool GNSSProcess::Evaluate(IMUST &state)
{
  if (gnss_meas_buf[0].empty() && current_gnss_time < 0.0) // ||gnss_meas_buf[0].size() < 4) //  
  {
    // cout << "no valid gnss" << endl;
    return false;
  }

  double time_current = current_gnss_time >= 0.0 ? current_gnss_time : time2sec(gnss_meas_buf[0][0]->time);
  double delta_t = time_current - last_gnss_time;

  gtsam::Rot3 rel_rot; // = gtsam::Rot3(pre_integration->delta_q);
  gtsam::Point3 rel_pos, pos, ba, bg, acc, omg;
  gtsam::Vector3 rel_vel, vel; 
  Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
  if (!nolidar) // && !invalid_lidar)
  {
    // Eigen::Matrix3d last_rot = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(0)).matrix(); // state_const_.rot; // 
    // // cout << "check time period" << pre_integration->sum_dt << ";" << time_current - last_gnss_time <<  endl;
    // Eigen::Vector3d last_pos = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(0)).segment<3>(0); // state_.pos; // 
    // Eigen::Vector3d last_vel = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(0)).segment<3>(3); // state_.vel; //
    // Eigen::Vector3d cur_grav = state.rot.transpose() * state.gravity; //  
    rot = state.R; //.normalized().toRotationMatrix(); last_rot.transpose() *
    // rel_rot = gtsam::Rot3(last_rot.transpose() * state.rot.normalized().toRotationMatrix());
    pos = state.p; // - anc_local; // last_rot.transpose() * (state.pos - last_pos - last_vel * delta_t - 0.5 * state.gravity * delta_t * delta_t);  - last_pos 
    // (state.pos - last_pos);
    vel = state.v; // last_rot.transpose() * (state.vel - last_vel - state.gravity * delta_t); // (state.vel - last_vel);  - last_vel
    ba = state.ba;
    bg = state.bg;
    acc = state.acc;
    omg = state.omg;
  }
  else
  {
    ba = state.ba;
    bg = state.bg;
    rel_rot = gtsam::Rot3(pre_integration->delta_q);
    rel_pos = pre_integration->delta_p;
    rel_vel = pre_integration->delta_v; 
  }

  auto graphHasKey = [&](gtsam::Key key) {
    return valuesContainKey(p_assign->initialEstimate, key) ||
           valuesContainKey(p_assign->isamCurrentEstimate, key);
  };
  if((!nolidar && (graphHasKey(A(frame_num)) || graphHasKey(R(frame_num)) ||
                  graphHasKey(O(frame_num)) || graphHasKey(G(frame_num)))) ||
     (nolidar && (graphHasKey(F(frame_num)) || graphHasKey(R(frame_num)))) ||
     graphHasKey(B(frame_num)) || graphHasKey(C(frame_num)))
  {
    ROS_WARN("GNSS graph variable already exists before raw update: frame=%d. Skip to avoid duplicate insertion.",
             frame_num);
    return false;
  }
  
  if (!nolidar) // && invalid_lidar)
  {
    Eigen::Matrix<double, 6, 1> init_vel_bias_vector_imu;
    init_vel_bias_vector_imu.block<3,1>(0,0) = state.p; // - anc_local;
    init_vel_bias_vector_imu.block<3,1>(3,0) = state.v;
    Eigen::Matrix<double, 12, 1> init_others_vector_imu;
    init_others_vector_imu.block<3,1>(0,0) = state.omg;
    init_others_vector_imu.block<3,1>(3,0) = state.acc;
    init_others_vector_imu.block<3,1>(6,0) = state.bg;
    init_others_vector_imu.block<3,1>(9,0) = state.ba;
    p_assign->initialEstimate.insert(A(frame_num), gtsam::Vector6(init_vel_bias_vector_imu));
    p_assign->initialEstimate.insert(G(frame_num), gtsam::Vector3(state.g));
    p_assign->initialEstimate.insert(O(frame_num), gtsam::Vector12(init_others_vector_imu));
    p_assign->initialEstimate.insert(R(frame_num), gtsam::Rot3(state.R));  // .normalized().toRotationMatrix()
  }
  else
  {
    Eigen::Matrix<double, 12, 1> init_vel_bias_vector;
    init_vel_bias_vector.block<3,1>(0,0) = state.p;
    init_vel_bias_vector.block<3,1>(3,0) = state.v;
    init_vel_bias_vector.block<3,1>(6,0) = state.ba;
    init_vel_bias_vector.block<3,1>(9,0) = state.bg;
    p_assign->initialEstimate.insert(F(frame_num), gtsam::Vector12(init_vel_bias_vector));
    p_assign->initialEstimate.insert(R(frame_num), gtsam::Rot3(state.R)); // .normalized().toRotationMatrix()
  }              
  // rot_pos = state.rot; //.normalized().toRotationMatrix();
  if (AddFactor(rel_rot, rel_pos, rel_vel, state.g, delta_t, time_current, ba, bg, pos, vel, acc, omg, rot))
  {
    frame_num ++;
    runISAM2opt();
    // auto ekfPosNoise = p_assign->isam.marginalCovariance(A(frame_num-1));
    // odo_weight = 60 / (ekfPosNoise(0,0) + ekfPosNoise(1,1) + ekfPosNoise(2,2));
  }
  else
  {
    return false;
  }

  // state.cov.block<3,3>(0, 0) = isam.marginalCovariance(R(frame_num-1));
  // state.cov.block<6,6>(3, 3) = isam.marginalCovariance(F(frame_num-1)).block<6, 6>(0, 0);
  
  if (nolidar)
  {
    state.R = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(frame_num-1)).matrix();
    // state.rot.normalize();
    state.p = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(0);
    state.v = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(3);
    state.ba = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(6);
    state.bg = p_assign->isamCurrentEstimate.at<gtsam::Vector12>(F(frame_num-1)).segment<3>(9);
    state.g = ecef2rotation(state.p) * gravity_init;
  }
  else
  {
    state_const_.R = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(frame_num-1)).matrix();
    state_const_.p = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(frame_num-1)).segment<3>(0); // + anc_local;
    state_const_.v = p_assign->isamCurrentEstimate.at<gtsam::Vector6>(A(frame_num-1)).segment<3>(3);
    state.g = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(P(0)).matrix().transpose() * ecef2rotation(p_assign->isamCurrentEstimate.at<gtsam::Vector3>(E(0))) * gravity_init;
  }    
  last_gnss_time = time_current;
  state_const_last = state;
  std::map<sat_first, std::map<uint32_t, double[6]>>::iterator it_old;
  if (!sat2cp.empty())
  {
    it_old = sat2cp.begin();
    while (time_current - it_old->first.timecur > gnss_cp_time_threshold)
    {
      std::map<uint32_t, double[6]>().swap(it_old->second);
      size_t del_size = sat2cp.erase(it_old->first);
      if (sat2cp.empty()) break;
      it_old = sat2cp.begin();
    }
  }
  return true;
}

bool GNSSProcess::AddFactor(gtsam::Rot3 rel_rot, gtsam::Point3 rel_pos, gtsam::Vector3 rel_vel, Eigen::Vector3d state_gravity, double delta_t, double time_current,
                Eigen::Vector3d ba, Eigen::Vector3d bg, Eigen::Vector3d pos, Eigen::Vector3d vel, Eigen::Vector3d acc, Eigen::Vector3d omg, Eigen::Matrix3d rot)
{
  double rcv_dt[4];
  bool rcv_sys[4];
  rcv_sys[0] = false; rcv_sys[1] = false; rcv_sys[2] = false; rcv_sys[3] = false;
  double rcv_ddt;
  invalid_lidar = false;
  if (!nolidar)
  {
    double weight_lid = 1;
    if (p_assign->process_feat_num < 10) 
    {
      weight_lid = 0;
      invalid_lidar = true;
    }
    else
    {
      if (norm_vec_num < 10)
      {
        invalid_lidar = true;
      }
      weight_lid = 2 * double(norm_vec_num) / double(p_assign->process_feat_num);
    }
    norm_vec_num = 0;
    p_assign->process_feat_num = 0;
    double weight_check1 =  sqrt_lidar(0, 0) < sqrt_lidar(1, 1) ? sqrt_lidar(0, 0) : sqrt_lidar(1, 1);
    weight_check1 = weight_check1 < sqrt_lidar(2, 2) ? weight_check1 : sqrt_lidar(2, 2); 
    double weight_check2 = sqrt_lidar(0, 0) > sqrt_lidar(1, 1) ? sqrt_lidar(0, 0) : sqrt_lidar(1, 1);
    weight_check2 = weight_check2 > sqrt_lidar(2, 2) ? weight_check2 : sqrt_lidar(2, 2); 
    if (weight_check2 / weight_check1 > 3)
    {
      sqrt_lidar *= 0.5 / weight_check1; // .block<3, 3>(0, 0)
    }
    else
    {
      double scale_weight = weight_lid > 0.5 ? weight_lid : 0.5;
      sqrt_lidar *= scale_weight / weight_check1; // .block<3, 3>(0, 0)
    }
  }
  // sqrt_lidar.block<6, 6>(3, 3) *= 2;
// for (size_t j = 3; j < 9; j++)
// {
//   sqrt_lidar(j, j) += 0.50;
// }
  // if ((delta_t > 15 * gnss_sample_period && nolidar) || (delta_t > 15 * gnss_sample_period && invalid_lidar && !nolidar))
  // {
    // Reset();
    // return false;
  // }
  if (nolidar_cur && !nolidar) nolidar_cur = false;
  const gtsam::Key prev_c_key = C(frame_num - 1);
  const gtsam::Key prev_b_key = B(frame_num - 1);
  const gtsam::Vector1 prev_rcv_ddt =
      p_assign->initialEstimate.exists(prev_c_key) ?
      p_assign->initialEstimate.at<gtsam::Vector1>(prev_c_key) :
      p_assign->isamCurrentEstimate.at<gtsam::Vector1>(prev_c_key);
  const gtsam::Vector4 prev_rcv_dt =
      p_assign->initialEstimate.exists(prev_b_key) ?
      p_assign->initialEstimate.at<gtsam::Vector4>(prev_b_key) :
      p_assign->isamCurrentEstimate.at<gtsam::Vector4>(prev_b_key);
  rcv_ddt = prev_rcv_ddt[0];
  rcv_dt[0] = prev_rcv_dt[0] + rcv_ddt * delta_t;
  rcv_dt[1] = prev_rcv_dt[1] + rcv_ddt * delta_t;
  rcv_dt[2] = prev_rcv_dt[2] + rcv_ddt * delta_t;
  rcv_dt[3] = prev_rcv_dt[3] + rcv_ddt * delta_t;

  const std::vector<ObsPtr> &curr_obs = gnss_meas_buf[0];
  const std::vector<EphemBasePtr> &curr_ephem = gnss_ephem_buf[0];

  // find best sat in the current gnss measurements
  std::map<sat_first, std::map<uint32_t, double[6]> >::reverse_iterator it;
  
  std::deque<uint32_t> pair_sat_copy;
  // std::deque<uint32_t>().swap(pair_sat_copy);

  std::deque<double> meas_sats, meas_sats_final;
  // std::deque<double> meas_cov_sats; //, meas_cov_sats_final;
  // std::deque<double> meas_time_sats, meas_time_sats_final;
  std::deque<int> meas_index_sats, meas_index_sats_final;
  std::deque<Eigen::Vector3d> meas_RTex_sats, meas_RTex_sats_final;
  std::deque<Eigen::Vector3d> meas_svpos_sats, meas_svpos_sats_final;
  
  for (uint32_t j = 0; j < curr_obs.size(); j++) //   && j < 10
  {
    std::map<uint32_t, double[6]>::iterator it_old; // t_old_best, 
    double meas;
    // double meas_cov;
    double meas_time;
    int meas_index;
    Eigen::Vector3d meas_svpos;
    Eigen::Vector3d RTex_sats;
    // Eigen::Vector3d best_svpos;
    // if (pair_sat.size() > 0)
    // if (curr_obs[j]->cp_std[freq_idx] < p_assign->gnss_cp_std_threshold)
    {
      // if (j == pair_sat.front())
      // if (curr_obs[j]->cp[freq_idx] > 10)
      {
        bool cp_found = false;
        for (it = sat2cp.rbegin(); it != sat2cp.rend(); it++)
        {
          it_old = it->second.find(curr_obs[j]->sat); // the same satellite
          // it_old_best = it->second.find(curr_obs[best_sat]->sat);
          if (it_old != it->second.end()) // && it_old_best != it->second.end())
          {
            if (it->first.timecur >= p_assign->sat_track_time[curr_obs[j]->sat])
            {
            // if ((time_current - it->first.timecur) / gnss_sample_period <= p_assign->sat_track_status[curr_obs[best_sat]->sat] - p_assign->gnss_track_num_threshold &&
            // if ((time_current - it->first.timecur) / gnss_sample_period <= p_assign->sat_track_status[curr_obs[j]->sat]) //- p_assign->gnss_track_num_threshold)
            if (time_current > p_assign->sat_track_time[curr_obs[j]->sat] && p_assign->sat_track_status[curr_obs[j]->sat] > 0) //- p_assign->gnss_track_num_threshold)
            {
              cp_found = true;
              meas = it_old->second[0]; // - it_old_best->second[1] + it_old->second[1]); it_old_best->second[0] -
              // meas_cov = (it_old->second[2] * it_old->second[2]); // it_old_best->second[2] * it_old_best->second[2] + 
              meas_time = it->first.timecur;
              meas_index = it->first.frame_num;
              RTex_sats << it->first.RTex[0], it->first.RTex[1], it->first.RTex[2];
              meas_svpos << it_old->second[3], it_old->second[4], it_old->second[5];
              // best_svpos << it_old_best->second[3], it_old_best->second[4], it_old_best->second[5];
              break;
            }
            }
          }
        }
      
        if (cp_found)
        {
          meas_sats.push_back(meas);
          // meas_cov_sats.push_back(meas_cov);
          // meas_time_sats.push_back(meas_time);
          meas_index_sats.push_back(meas_index);
          meas_svpos_sats.push_back(meas_svpos);
          meas_RTex_sats.push_back(RTex_sats);
          // meas_svpos_best.push_back(best_svpos);
          pair_sat_copy.push_back(j);
        }
        // pair_sat.pop_front();
      }
    }
  }

  std::map<uint32_t, double[6]> curr_cp_map;
  std::vector<double> meas_cp;
  // std::vector<double> cov_cp;
  std::vector<Eigen::Vector3d> sv_pos_pair, sat_svpos;
  double cov_cp_best, meas_cp_best; //, esti_cp_best, 
  // Eigen::Vector3d sv_pos_best;
  std::vector<size_t> factor_id_cur, sys_idx_cp;
  Eigen::Matrix3d omg_skew;
  omg_skew << SKEW_SYM_MATRX(omg);
  Eigen::Vector3d hat_omg_T = omg_skew * Tex_imu_r;
  const double receiver_hor_vel =
      nolidar ? std::numeric_limits<double>::infinity() :
      (vel + rot * hat_omg_T).head<2>().norm();
  const bool enough_obs_for_gnss_factors =
      current_gnss_factor_enabled && curr_obs.size() >= min_obs && curr_obs.size() == curr_ephem.size();
  const bool enough_motion_for_gnss_factors = min_hor_vel <= 0.0 || receiver_hor_vel >= min_hor_vel;
  const bool add_gnss_measurement_factors =
      enough_obs_for_gnss_factors && enough_motion_for_gnss_factors;
  if(!enough_obs_for_gnss_factors)
  {
    ROS_WARN_THROTTLE(2.0,
      "GNSS measurement factors skipped: valid_obs=%lu min_obs=%lu frame=%d",
      curr_obs.size(), min_obs, frame_num);
  }
  if(enough_obs_for_gnss_factors && !enough_motion_for_gnss_factors)
  {
    ROS_WARN_THROTTLE(2.0,
      "GNSS measurement factors skipped at low horizontal velocity: %.3f < %.3f m/s frame=%d",
      receiver_hor_vel, min_hor_vel, frame_num);
  }
  if (add_gnss_measurement_factors)
  {
  for (uint32_t j = 0; j < curr_obs.size(); j++) //   && j < 10
  {
    bool balance = false;
    if (j > curr_obs.size() / 2)
    {
      balance = true;
    }
    const uint32_t sys = satsys(curr_obs[j]->sat, NULL);
    const uint32_t sys_idx = gnss_comm::sys2idx.at(sys);
    GnssPsrDoppMeas(curr_obs[j], curr_ephem[j]); //, latest_gnss_iono_params);
    freq = L1_freq(curr_obs[j], &freq_idx); // save
    const double wavelength = LIGHT_SPEED / freq; // save
    if (curr_obs[j]->cp_std[freq_idx] < p_assign->gnss_cp_std_threshold)
    {
      if (curr_obs[j]->cp[freq_idx] * wavelength > 100)
      {
        curr_cp_map[curr_obs[j]->sat][0] = curr_obs[j]->cp[freq_idx] * wavelength + svdt * LIGHT_SPEED - tgd * LIGHT_SPEED;
        // curr_cp_map[curr_obs[j]->sat][2] = curr_obs[j]->cp_std[freq_idx] * 0.004;
        curr_cp_map[curr_obs[j]->sat][3] = sv_pos[0];
        curr_cp_map[curr_obs[j]->sat][4] = sv_pos[1];
        curr_cp_map[curr_obs[j]->sat][5] = sv_pos[2];
 
        if (pair_sat_copy.size() > 0)
        {
          for (size_t k = 0; k < pair_sat_copy.size(); k++)
          {
            if (j == pair_sat_copy[k])
            {
              meas_cp.push_back(curr_obs[j]->cp[freq_idx] * wavelength + svdt * LIGHT_SPEED - tgd * LIGHT_SPEED);
              sys_idx_cp.push_back(sys_idx);
              meas_sats_final.push_back(meas_sats[k]);
              // cov_cp.push_back(curr_obs[j]->cp_std[freq_idx] * curr_obs[j]->cp_std[freq_idx] * 0.004 * 0.004);
              // meas_cov_sats_final.push_back(meas_cov_sats[k]);
              sv_pos_pair.push_back(sv_pos);
              meas_svpos_sats_final.push_back(meas_svpos_sats[k]);
              // meas_time_sats_final.push_back(meas_time_sats[k]);
              meas_index_sats_final.push_back(meas_index_sats[k]);
              meas_RTex_sats_final.push_back(meas_RTex_sats[k]);
              break;
              // pair_sat_copy.pop_front();
            }
          }
        }
      }
    }
    /////////////////////////////////
    double values[28];
    values[0] = Tex_imu_r[0]; values[1] = Tex_imu_r[1]; values[2] = Tex_imu_r[2]; //values[3] = anc_local[0]; values[4] = anc_local[1]; values[5] = anc_local[2];
    values[3] = sv_pos[0]; values[4] = sv_pos[1]; values[5] = sv_pos[2]; values[6] = sv_vel[0]; values[7] = sv_vel[1]; values[8] = sv_vel[2];
    values[9] = svdt; values[10] = tgd; values[11] = svddt; values[12] = pr_uura; values[13] = dp_uura; values[14] = relative_sqrt_info; // psr_weight_adjust;
    values[15] = p_assign->latest_gnss_iono_params[0]; values[16] = p_assign->latest_gnss_iono_params[1]; values[17] = p_assign->latest_gnss_iono_params[2]; values[18] = p_assign->latest_gnss_iono_params[3]; 
    values[19] = p_assign->latest_gnss_iono_params[4]; values[20] = p_assign->latest_gnss_iono_params[5]; values[21] = p_assign->latest_gnss_iono_params[6]; values[22] = p_assign->latest_gnss_iono_params[7]; 
    values[23] = time_current; values[24] = freq; values[25] = psr_meas_hatch_filter[j]; values[26] = curr_obs[j]->dopp[freq_idx]; values[27] = 1.0; //curr_obs[j]->psr[freq_idx]; 
    rcv_sys[sys_idx] = true;

    if (!nolidar)
    { 
      Eigen::Vector3d RTex = rot * Tex_imu_r;
      values[0] = RTex[0]; values[1] = RTex[1]; values[2] = RTex[2];
      if (frame_num < delete_thred)
      {      
        p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorNoR(A(frame_num), B(frame_num), C(frame_num), E(0), P(0), balance, values, sys_idx, rot * hat_omg_T, p_assign->robustpsrdoppNoise_init));
      }
      else
      {
        p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorNoR(A(frame_num), B(frame_num), C(frame_num), E(0), P(0), balance, values, sys_idx, rot * hat_omg_T, p_assign->robustpsrdoppNoise));
      }
      // p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorNoR(A(frame_num), B(frame_num), C(frame_num), E(0), P(0), invalid_lidar, values, sys_idx, rot * hat_omg_T, p_assign->robustpsrdoppNoise));
      // p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactor(R(frame_num), A(frame_num), B(frame_num), C(frame_num), E(0), P(0), invalid_lidar, values, sys_idx, hat_omg_T, p_assign->robustpsrdoppNoise));
      // p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorPos(A(frame_num), B(frame_num), C(frame_num), E(0), P(0), invalid_lidar, values, sys_idx, rot_pos, hat_omg_T, p_assign->robustpsrdoppNoise));
    }
    else
    {   
      if (frame_num < delete_thred)
      {
        p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorNolidar(R(frame_num), F(frame_num), B(frame_num), C(frame_num), values, sys_idx, hat_omg_T, p_assign->robustpsrdoppNoise_init)); // not work
      }
      else
      {
        p_assign->gtSAMgraph.add(ligo::GnssPsrDoppFactorNolidar(R(frame_num), F(frame_num), B(frame_num), C(frame_num), values, sys_idx, hat_omg_T, p_assign->robustpsrdoppNoise)); // not work
      } 
    }
    factor_id_cur.push_back(id_accumulate);
    id_accumulate += 1;

  }
  }
  if (add_gnss_measurement_factors)
  {
    sat_first cur_key;
    cur_key.RTex = rot * Tex_imu_r;
    cur_key.timecur = time2sec(curr_obs[0]->time);
    cur_key.frame_num = frame_num;
    sat2cp[cur_key] = curr_cp_map;
  }
  // if (frame_num < delete_thred)
  // {
  //   p_assign->gtSAMgraph.add(ligo::DdtSmoothFactor(C(frame_num-1), C(frame_num), p_assign->ddtNoise_init));
  //   // p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector1>(C(frame_num), gtsam::Vector1(rcv_ddt), p_assign->ddtNoise));
  //   p_assign->gtSAMgraph.add(ligo::DtDdtFactor(B(frame_num-1), B(frame_num), C(frame_num-1), C(frame_num), rcv_sys, delta_t, p_assign->dtNoise_init)); // not work
  // }
  // else
  // {
    p_assign->gtSAMgraph.add(ligo::DdtSmoothFactor(C(frame_num-1), C(frame_num), p_assign->ddtNoise));
    // p_assign->gtSAMgraph.add(gtsam::PriorFactor<gtsam::Vector1>(C(frame_num), gtsam::Vector1(rcv_ddt), p_assign->ddtNoise));
  
  p_assign->gtSAMgraph.add(ligo::DtDdtFactor(B(frame_num-1), B(frame_num), C(frame_num-1), C(frame_num), rcv_sys, delta_t, p_assign->dtNoise)); // not work
  // }
  {
  // if (frame_num > 1)
  {
    p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate+1);
    p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate);
  }
  // else
  // {
    // p_assign->factor_id_frame.push_back(std::vector<size_t>(id_accumulate, id_accumulate+1));
  // }
  }
  // else
  // {
    // p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate+1);
    // p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate); 
  // }
  id_accumulate += 2;
  if (!nolidar)
  {
    bool no_weight = false;
    // if (frame_num < delete_thred)
    // {
      // p_assign->gtSAMgraph.add(ligo::GnssLioFactor(P(0), E(0), R(0), A(0), R(frame_num), A(frame_num), gravity_init, state_gravity, ba, bg, rot, sqrt_lidar, p_assign->odomaNoise)); //LioNoise)); // odomNoiseIMU));
    // }
    // else
    {
      // p_assign->gtSAMgraph.add(ligo::GnssLioHardFactorNoR(A(frame_num), ba, bg, sqrt_lidar, no_weight, p_assign->odomNoise)); //LioNoise)); // odomNoiseIMU));
      p_assign->gtSAMgraph.add(ligo::GnssLioFactor(P(0), E(0), R(frame_num), A(frame_num), O(frame_num), G(frame_num), gravity_init, state_gravity, pos, vel, rot, ba, bg, acc, omg, sqrt_lidar, p_assign->odomNoise)); //LioNoise)); // odomNoiseIMU));
    }
      // p_assign->gtSAMgraph.add(ligo::GnssLioHardFactor(R(frame_num), A(frame_num), ba, bg, rot, sqrt_lidar, no_weight, p_assign->odomNoise)); //LioNoise)); // odomNoiseIMU));
    factor_id_cur.push_back(id_accumulate);
    id_accumulate += 1;    
  }
  else
  {
    p_assign->gtSAMgraph.add(ligo::GnssLioFactorNolidar(R(frame_num-1), F(frame_num-1), R(frame_num), F(frame_num), rel_rot, rel_pos, rel_vel, 
                  state_gravity, delta_t, ba, bg, pre_integration, p_assign->odomNoiseIMU));
    p_assign->factor_id_frame[frame_num-1-frame_delete].push_back(id_accumulate);
    id_accumulate += 1;
  }
  p_assign->initialEstimate.insert(C(frame_num), gtsam::Vector1(rcv_ddt));
  p_assign->initialEstimate.insert(B(frame_num), gtsam::Vector4(rcv_dt[0], rcv_dt[1], rcv_dt[2], rcv_dt[3]));  
  for (uint32_t j = 0; j < meas_index_sats_final.size(); j++)
  {
    double values[11];
    values[0] = Tex_imu_r[0]; values[1] = Tex_imu_r[1]; values[2] = Tex_imu_r[2]; // values[3] = anc_local[0]; values[4] = anc_local[1]; values[5] = anc_local[2];
    values[3] = meas_svpos_sats_final[j][0]; values[4] = meas_svpos_sats_final[j][1]; values[5] = meas_svpos_sats_final[j][2]; 
    // values[9] = meas_svpos_sats_final[j][0]; values[10] = meas_svpos_sats_final[j][1]; values[11] = meas_svpos_sats_final[j][2];
    // values[12] = sv_pos_best[0]; values[13] = sv_pos_best[1]; values[14] = sv_pos_best[2]; 
    values[6] = sv_pos_pair[j][0]; values[7] = sv_pos_pair[j][1]; values[8] = sv_pos_pair[j][2];
    values[9] = meas_cp[j] - meas_sats_final[j]; values[10] = cp_weight; //_adjust; 
    // values[14] = rcv_dt[0] - p_assign->isamCurrentEstimate.at<gtsam::Vector4>(B(meas_index_sats_final[j]))[0] + dt_com;
    if (!nolidar)
    {
      Eigen::Vector3d RTex1 = rot * Tex_imu_r;
      values[0] = RTex1[0]; values[1] = RTex1[1]; values[2] = RTex1[2]; 
      if (frame_num < delete_thred)
      {
        p_assign->gtSAMgraph.add(ligo::GnssCpFactorNoR(E(0), P(0), A(meas_index_sats_final[j]), A(frame_num), B(meas_index_sats_final[j]), B(frame_num), sys_idx_cp[j], invalid_lidar, values, meas_RTex_sats_final[j], p_assign->robustcpNoise_init));
      }
      else
      {
        p_assign->gtSAMgraph.add(ligo::GnssCpFactorNoR(E(0), P(0), A(meas_index_sats_final[j]), A(frame_num), B(meas_index_sats_final[j]), B(frame_num), sys_idx_cp[j], invalid_lidar, values, meas_RTex_sats_final[j], p_assign->robustcpNoise));
      }
      // p_assign->gtSAMgraph.add(ligo::GnssCpFactorNoR(E(0), P(0), A(meas_index_sats_final[j]), A(frame_num), B(meas_index_sats_final[j]), B(frame_num), sys_idx_cp[j], invalid_lidar, values, meas_RTex_sats_final[j], p_assign->robustcpNoise));
    }
    else
    {
      if (frame_num < delete_thred)
      {
        p_assign->gtSAMgraph.add(ligo::GnssCpFactorNolidar(R(meas_index_sats_final[j]), F(meas_index_sats_final[j]), R(frame_num), F(frame_num), B(meas_index_sats_final[j]), B(frame_num), sys_idx_cp[j], values, p_assign->robustcpNoise_init)); // not work
      }
      else
      {// p_assign->gtSAMgraph.add(ligo::GnssCpFactorNolidar(R(meas_index_sats_final[j]), F(meas_index_sats_final[j]), R(frame_num), F(frame_num), sys_idx_cp[j], values, p_assign->robustcpNoise)); // not work
        p_assign->gtSAMgraph.add(ligo::GnssCpFactorNolidar(R(meas_index_sats_final[j]), F(meas_index_sats_final[j]), R(frame_num), F(frame_num), B(meas_index_sats_final[j]), B(frame_num), sys_idx_cp[j], values, p_assign->robustcpNoise)); // not work
      }// Eigen::Matrix3d rot_before = p_assign->isamCurrentEstimate.at<gtsam::Rot3>(R(meas_index_sats_final[j])).matrix();
      // p_assign->gtSAMgraph.add(ligo::GnssCpFactorNolidarPos(F(meas_index_sats_final[j]), F(frame_num), values, rot_before, rot_pos, p_assign->robustcpNoise)); // not work
    }
    // factor_id_cur.push_back(id_accumulate);
    p_assign->factor_id_frame[meas_index_sats_final[j]-frame_delete].push_back(id_accumulate);
    id_accumulate += 1;
  }

  {
    p_assign->factor_id_frame.push_back(factor_id_cur);
    std::vector<size_t>().swap(factor_id_cur);
  }
  // if (meas_index_sats_final.size() < 4)
  // {
  //   runISAM2opt();
  //   frame_num ++;
  //   return false;
  // }
  return true;
}

void GNSSProcess::SetInit()
{
  if (!nolidar)
  {
    // Eigen::Matrix3d R_enu_local_;
    // R_enu_local_ = R_ecef_enu; // * Rot_gnss_init; // * Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ()) 
    // prior factor 
    Eigen::Matrix<double, 6, 1> init_vel_bias_vector;
    Eigen::Matrix<double, 12, 1> init_others_vector;
    // init_vel_bias_vector.block<3,1>(0,0) = Rot_gnss_init.transpose() * pos_window[wind_size];
    init_vel_bias_vector.block<3,1>(0,0) = Eigen::Vector3d::Zero();
    init_vel_bias_vector.block<3,1>(3,0) = Eigen::Vector3d::Zero(); // vel_window[wind_size];
    init_others_vector.block<3,1>(0,0) = Eigen::Vector3d::Zero(); // vel_window[wind_size];
    init_others_vector.block<3,1>(3,0) = Eigen::Vector3d::Zero(); // vel_window[wind_size];
    init_others_vector.block<3,1>(6,0) = Eigen::Vector3d::Zero(); // vel_window[wind_size];
    init_others_vector.block<3,1>(9,0) = Eigen::Vector3d::Zero(); // vel_window[wind_size];
    // dt[0] = para_rcv_dt[wind_size*4]; dt[1] = para_rcv_dt[wind_size*4+1], dt[2] = para_rcv_dt[wind_size*4+2], dt[3] = para_rcv_dt[wind_size*4+3];
    // ddt = para_rcv_ddt[wind_size];
    p_assign->initialEstimate.insert(R(0), gtsam::Rot3(Rot_gnss_init)); //.transpose() * rot_window[wind_size]));
    p_assign->initialEstimate.insert(G(0), gtsam::Vector3(gravity_init)); //.transpose() * rot_window[wind_size]));
    // p_assign->initialEstimate.insert(F(0), gtsam::Vector12(init_vel_bias_vector));
    p_assign->initialEstimate.insert(A(0), gtsam::Vector6(init_vel_bias_vector));
    p_assign->initialEstimate.insert(O(0), gtsam::Vector12(init_others_vector));
    p_assign->initialEstimate.insert(B(0), gtsam::Vector4(para_rcv_dt[wind_size*4], para_rcv_dt[wind_size*4+1], para_rcv_dt[wind_size*4+2], para_rcv_dt[wind_size*4+3]));
    // p_assign->initialEstimate.insert(C(0), gtsam::Vector1(para_rcv_ddt[wind_size]));
    p_assign->initialEstimate.insert(C(0), gtsam::Vector1(para_rcv_ddt[0])); //(163.119147)); //(161.874045) 
    // p_assign->initialEstimate.insert(Y(0), gtsam::Vector1(yaw_enu_local));
    p_assign->initialEstimate.insert(E(0), gtsam::Vector3(anc_ecef[0], anc_ecef[1], anc_ecef[2]));
    // cout << anc_ecef.transpose() << endl;
    p_assign->initialEstimate.insert(P(0), gtsam::Rot3(R_ecef_enu));

    gtsam::PriorFactor<gtsam::Rot3> init_rot_ext(P(0), gtsam::Rot3(gtsam::Rot3(R_ecef_enu)), p_assign->priorextrotNoise);
    gtsam::PriorFactor<gtsam::Vector3> init_pos_ext(E(0), gtsam::Vector3(anc_ecef[0], anc_ecef[1], anc_ecef[2]), p_assign->priorextposNoise);
    gtsam::PriorFactor<gtsam::Vector4> init_dt(B(0), gtsam::Vector4(para_rcv_dt[wind_size*4], para_rcv_dt[wind_size*4+1], para_rcv_dt[wind_size*4+2], para_rcv_dt[wind_size*4+3]), p_assign->priordtNoise);
    // gtsam::PriorFactor<gtsam::Vector1> init_ddt(C(0), gtsam::Vector1(para_rcv_ddt[wind_size]), p_assign->priorddtNoise);
    gtsam::PriorFactor<gtsam::Vector1> init_ddt(C(0), gtsam::Vector1(para_rcv_ddt[0]), p_assign->priorddtNoise); // (161.874045) 163.119147
    gtsam::PriorFactor<gtsam::Rot3> init_rot_(R(0), gtsam::Rot3(Rot_gnss_init), p_assign->priorrotNoise);
    gtsam::PriorFactor<gtsam::Vector6> init_vel_(A(0), gtsam::Vector6(init_vel_bias_vector), p_assign->priorNoise); // priorposNoise);
    gtsam::PriorFactor<gtsam::Vector12> init_bias_(O(0), gtsam::Vector12(init_others_vector), p_assign->priorBiasNoise); // priorposNoise);
    gtsam::PriorFactor<gtsam::Vector3> init_grav_(G(0), gtsam::Vector3(gravity_init), p_assign->priorGravNoise);
    p_assign->gtSAMgraph.add(init_rot_ext);
    p_assign->gtSAMgraph.add(init_pos_ext);
    p_assign->gtSAMgraph.add(init_dt);
    p_assign->gtSAMgraph.add(init_ddt);
    p_assign->gtSAMgraph.add(init_rot_);
    p_assign->gtSAMgraph.add(init_vel_);
    p_assign->gtSAMgraph.add(init_bias_);
    p_assign->gtSAMgraph.add(init_grav_);
    p_assign->factor_id_frame.push_back(std::vector<size_t>{0, 1, 2, 3, 4, 5, 6, 7});
    id_accumulate += 8;
  }
  else
  {
  //   Eigen::Matrix3d R_enu_local_;
  //   R_enu_local_ = Eigen::AngleAxisd(yaw_enu_local, Eigen::Vector3d::UnitZ());
    // dt[0] = para_rcv_dt[wind_size*4], dt[1] = para_rcv_dt[wind_size*4+1], dt[2] = para_rcv_dt[wind_size*4+2], dt[3] = para_rcv_dt[wind_size*4+3];
    // ddt = para_rcv_ddt[wind_size];
    gtsam::PriorFactor<gtsam::Rot3> init_rot(R(0), gtsam::Rot3(R_ecef_enu * rot_window[wind_size]), p_assign->priorrotNoise); //  * R_enu_local_
    Eigen::Matrix<double, 12, 1> init_vel_bias_vector;
    init_vel_bias_vector.block<3,1>(0,0) = anc_ecef + R_ecef_enu * (pos_window[wind_size] - rot_window[wind_size] * Tex_imu_r); //  * R_enu_local_- pos_window[0]
    init_vel_bias_vector.block<3,1>(3,0) = R_ecef_enu * vel_window[wind_size]; // R_enu_local_ * 
    init_vel_bias_vector.block<6,1>(6,0) = Eigen::Matrix<double, 6, 1>::Zero();
    gtsam::PriorFactor<gtsam::Vector12> init_vel_bias(F(0), gtsam::Vector12(init_vel_bias_vector), p_assign->priorposNoise);
    // gtsam::PriorFactor<gtsam::Vector4> init_dt(B(0), gtsam::Vector4(para_rcv_dt[wind_size*4], para_rcv_dt[wind_size*4+1], para_rcv_dt[wind_size*4+2], para_rcv_dt[wind_size*4+3]), p_assign->priordtNoise);
    gtsam::PriorFactor<gtsam::Vector4> init_dt(B(0), gtsam::Vector4(para_rcv_dt[0], para_rcv_dt[0], para_rcv_dt[0], para_rcv_dt[0]), p_assign->priordtNoise);
    gtsam::PriorFactor<gtsam::Vector1> init_ddt(C(0), gtsam::Vector1(para_rcv_ddt[0]), p_assign->priorddtNoise); // para_rcv_ddt[wind_size]
    p_assign->gtSAMgraph.add(init_rot);
    p_assign->gtSAMgraph.add(init_vel_bias);
    p_assign->gtSAMgraph.add(init_dt);
    p_assign->gtSAMgraph.add(init_ddt);
    p_assign->factor_id_frame.push_back(std::vector<size_t>{0, 1, 2, 3}); //{i * 4, i * 4 + 1, i * 4  + 2, i * 4 + 3});
    p_assign->initialEstimate.insert(R(0), gtsam::Rot3(R_ecef_enu * rot_window[wind_size])); // R_enu_local_ * 
    p_assign->initialEstimate.insert(F(0), gtsam::Vector12(init_vel_bias_vector));
    // p_assign->initialEstimate.insert(B(0), gtsam::Vector4(para_rcv_dt[wind_size*4], para_rcv_dt[wind_size*4+1], para_rcv_dt[wind_size*4+2], para_rcv_dt[wind_size*4+3]));
    p_assign->initialEstimate.insert(B(0), gtsam::Vector4(para_rcv_dt[0], para_rcv_dt[0], para_rcv_dt[0], para_rcv_dt[0]));
    p_assign->initialEstimate.insert(C(0), gtsam::Vector1(para_rcv_ddt[0])); // para_rcv_ddt[wind_size]
    id_accumulate += 4;
  }
}
