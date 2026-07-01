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

#include "GNSS_Assignment.h"

GNSSAssignment::GNSSAssignment() : process_feat_num(0) {
    // fout_std.open((string(string(ROOT_DIR) + "Log/"+ "std.txt")), ios::out);
}

void GNSSAssignment::inputEphem(EphemBasePtr ephem_ptr) // 
{
    double toe = time2sec(ephem_ptr->toe);
    // if a new ephemeris comes
    if (sat2time_index.count(ephem_ptr->sat) == 0 || sat2time_index.at(ephem_ptr->sat).count(toe) == 0)
    {
        sat2ephem[ephem_ptr->sat].emplace_back(ephem_ptr);
        sat2time_index[ephem_ptr->sat].emplace(toe, sat2ephem.at(ephem_ptr->sat).size()-1);
    }
}

void GNSSAssignment::processGNSSBase(const std::vector<ObsPtr> &gnss_meas, std::vector<double> &psr_meas, std::vector<ObsPtr> &valid_meas, std::vector<EphemBasePtr> &valid_ephems, bool gnss_ready, Eigen::Vector3d ecef_pos, double last_gnss_time_process)
{
  std::vector<ObsPtr> backup_meas;
  std::vector<EphemBasePtr> backup_ephems;
  std::vector<double> backup_psr_meas;
  backup_meas.clear();
  backup_ephems.clear();
  backup_psr_meas.clear();
  const int n = 20;
  bool diff_angle[n];
  std::fill(diff_angle, diff_angle + n, false);
  for (auto obs : gnss_meas)
  {
    // filter according to system
    uint32_t sys = satsys(obs->sat, NULL);
    if (sys != SYS_GPS && sys != SYS_GLO && sys != SYS_GAL && sys != SYS_BDS)
        continue;
    size_t ephem_index = -1;
    EphemBasePtr best_ephem_cur;
    double obs_time = time2sec(obs->time);
    if (obs->freqs.empty())    continue;       // no valid signal measurement
    freq_idx_ = -1;
    double freq = L1_freq(obs, &freq_idx_); // L1_freq NEEDED
    if (freq_idx_ < 0)   continue;              // no L1 observation
    if (obs->CN0[freq_idx_] < gnss_cn0_threshold)  continue; // low signal strength
    // printf("cn0:%f\n", obs->CN0[freq_idx_]);
    // fout_std << setw(20) << obs_time << " " << sys << " " << obs->sat << " " << obs->psr_std[freq_idx_] << " " << obs->dopp_std[freq_idx_] * LIGHT_SPEED / freq << " " << obs->cp_std[freq_idx_] * LIGHT_SPEED / freq << endl;
    // printf("%f,%f,%f\n", obs->psr[freq_idx_], obs->dopp[freq_idx_] * LIGHT_SPEED / freq, LIGHT_SPEED / freq);
    // num_std++;
    // ave_std = double(num_std - 1) / double(num_std) * ave_std + 1 / double(num_std) * obs->psr_std[freq_idx_];
    double dis_integer = obs->cp[freq_idx_] * LIGHT_SPEED / freq - obs->psr[freq_idx_];
    if (gnss_ready)
    {
        if (obs->psr_std[freq_idx_]  > gnss_psr_std_threshold ||
            obs->dopp_std[freq_idx_] > gnss_dopp_std_threshold ||
            obs->cp_std[freq_idx_] > gnss_cp_std_threshold )
        {
            sat_track_status[obs->sat] = 0;
            continue;
        }
        else
        {
            if (sat_track_status.count(obs->sat) == 0)
            {
                sat_track_status[obs->sat] = 0;
                sat_track_time[obs->sat] = obs_time;
                sat_track_last_time[obs->sat] = obs_time;
                sum_d = dis_integer;
                sum_d2 = sum_d * sum_d;
                hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
                // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
                last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
            }
            else
            {
                if (sat_track_status[obs->sat] == 0)
                {
                    sat_track_status[obs->sat] = 0;
                    sat_track_time[obs->sat] = obs_time;
                    sat_track_last_time[obs->sat] = obs_time;
                    sum_d = dis_integer;
                    sum_d2 = sum_d * sum_d;
                    hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
                    // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
                    last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
                }
            }
            ++ sat_track_status[obs->sat];
        }
    }
    else
    {
        if (obs->psr_std[freq_idx_]  > gnss_psr_std_threshold||
            obs->dopp_std[freq_idx_] > gnss_dopp_std_threshold ||
            obs->cp_std[freq_idx_] > gnss_cp_std_threshold)
        {
            sat_track_status[obs->sat] = 0;
            continue;
        }
        else
        {
            if (sat_track_status.count(obs->sat) == 0)
            {
                sat_track_status[obs->sat] = 0;
                sat_track_last_time[obs->sat] = obs_time;
                sat_track_time[obs->sat] = obs_time;
                sum_d = dis_integer;
                sum_d2 = sum_d * sum_d;
                hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
                // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
                last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
            }
            else
            {
                if (sat_track_status[obs->sat] == 0)
                {
                    sat_track_status[obs->sat] = 0;
                    sat_track_time[obs->sat] = obs_time;
                    sat_track_last_time[obs->sat] = obs_time;
                    sum_d = dis_integer;
                    sum_d2 = sum_d * sum_d;
                    hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
                    // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
                    last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
                }
            }
            ++ sat_track_status[obs->sat];
            // sat_track_last_time[obs->sat] = obs_time;
        }
    }
    
    if (last_cp_meas[obs->sat] < 100)
    {
        sat_track_status[obs->sat] = 0;
        hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
        // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
    }
    else
    {
    if (obs_time - sat_track_last_time[obs->sat] > 15)
    {
        sat_track_status[obs->sat] = 1;
        sat_track_last_time[obs->sat] = obs_time;
        sat_track_time[obs->sat] = obs_time;
        sum_d = dis_integer; 
        sum_d2 = sum_d * sum_d;
        hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
        // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
        last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
    }
    else
    {
        if (sat_track_status[obs->sat] > 1) // problem!
        {
            // if (obs->status[freq_idx_])
            if (fabs(dis_integer) > 6 * sqrt(sum_d2 / sat_track_status[obs->sat] - sum_d * sum_d / sat_track_status[obs->sat] / sat_track_status[obs->sat])) // ?
            {
                sat_track_status[obs->sat] = 1;
                sat_track_last_time[obs->sat] = obs_time;
                sat_track_time[obs->sat] = obs_time;
                sum_d = dis_integer; 
                sum_d2 = sum_d * sum_d;
                hatch_filter_meas[obs->sat] = obs->psr[freq_idx_];
                // hatch_filter_noise[obs->sat] = obs->psr_std[freq_idx_];
                last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
            }
            else
            {
                sum_d += dis_integer;
                sum_d2 += dis_integer * dis_integer;
                sat_track_last_time[obs->sat] = obs_time;
                // if (last_cp_meas[obs->sat] > 10 && obs->cp[freq_idx_] * LIGHT_SPEED / freq > 10)
                {
                    double last_psr = hatch_filter_meas[obs->sat];
                    hatch_filter_meas[obs->sat] = 1 / double(sat_track_status[obs->sat]) * obs->psr[freq_idx_] + double(sat_track_status[obs->sat]-1)/double(sat_track_status[obs->sat]) 
                                        * (last_psr + obs->cp[freq_idx_] * LIGHT_SPEED / freq - last_cp_meas[obs->sat]); // obs->psr[freq_idx_];
                    obs->psr_std[freq_idx_] = std::sqrt(obs->psr_std[freq_idx_] * obs->psr_std[freq_idx_] / 2 + obs->cp_std[freq_idx_] * obs->cp_std[freq_idx_] * LIGHT_SPEED / freq * LIGHT_SPEED / freq);
                    // cout << "check after:" << hatch_filter_meas[obs->sat] << ";" << obs->psr[freq_idx_] << ";" << obs->cp[freq_idx_] * LIGHT_SPEED / freq << ";" << last_cp_meas[obs->sat] << endl;
                }
                last_cp_meas[obs->sat] = obs->cp[freq_idx_] * LIGHT_SPEED / freq;
            }
        }
    }
    }

      // if not got cooresponding ephemeris yet
      if (sat2ephem.count(obs->sat) == 0)
          continue;
      
      std::map<double, size_t> time2index = sat2time_index.at(obs->sat);
      double ephem_time = EPH_VALID_SECONDS;
      for (auto ti : time2index)
      {
          if (std::abs(ti.first - obs_time) < ephem_time)
          {
              ephem_time = std::abs(ti.first - obs_time);
              ephem_index = ti.second;
          }
      }
      std::map<double, size_t>().swap(time2index);
      if (ephem_time >= EPH_VALID_SECONDS)
      {
          cerr << "ephemeris not valid anymore\n";
          continue;
      }
      best_ephem_cur = sat2ephem.at(obs->sat).at(ephem_index);

      const EphemBasePtr &best_ephem = best_ephem_cur;
      // filter by tracking status
      LOG_IF(FATAL, freq_idx_ < 0) << "No L1 observation found.\n";
      
      // filter by elevation angle
      if (gnss_ready) // && !quick_it) // gnss initialization is completed, then filter the sat by elevation angle // need to be defined
      {
          Eigen::Vector3d sat_ecef;
          if (sys == SYS_GLO)
              sat_ecef = geph2pos(obs->time, std::dynamic_pointer_cast<GloEphem>(best_ephem), NULL);
          else
              sat_ecef = eph2pos(obs->time, std::dynamic_pointer_cast<Ephem>(best_ephem), NULL);
          double azel[2] = {0, M_PI/2.0};
        //   if (fabs((ecef_pos-sat_ecef).norm() - hatch_filter_meas[obs->sat]) > 3 * 1e6)
            //   continue;
          sat_azel(ecef_pos, sat_ecef, azel); // ecef_pos should be updated for this time step // coarse value is acceptable as well TODO
        //   std::cout << "check angle:" << azel[0] << ";" << azel[1] << std::endl;
          if (azel[1] < gnss_elevation_threshold*M_PI/180.0)
              continue;
          int angle_id = int(azel[0] / 0.314);
          if (diff_angle[angle_id])
          {
            backup_meas.push_back(obs);
            backup_ephems.push_back(best_ephem);
            backup_psr_meas.push_back(hatch_filter_meas[obs->sat]);
            continue;
          }
          diff_angle[angle_id] = true;
      }
      psr_meas.push_back(hatch_filter_meas[obs->sat]); // obs->psr[freq_idx_]); // 
    //   obs->psr[freq_idx_] = hatch_filter_meas[obs->sat];
      valid_meas.push_back(obs);
      valid_ephems.push_back(best_ephem);
    if (valid_meas.size() >= 15) break; // 
  }
  if (valid_meas.size() < 15) 
  {
    // for (int i = 0; i < backup_meas.size(); i++)
    // {
    //   valid_meas.push_back(backup_meas[i]);
    //   valid_ephems.push_back(backup_ephems[i]);
    //   psr_meas.push_back(backup_psr_meas[i]);
    //   if (valid_meas.size() >= 15) break;
    // }
  } //
}
