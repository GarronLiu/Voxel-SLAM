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

void GNSSAssignment::initNoises( void ) // maybe usable!
{
    gtsam::Vector priorrotNoiseVector3(3);
    priorrotNoiseVector3 << marg_noise, marg_noise, marg_noise; // prior_noise / 100, prior_noise / 100, prior_noise / 100;
    // priorrotNoiseVector3 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000;
    priorrotNoise = gtsam::noiseModel::Diagonal::Variances(priorrotNoiseVector3);

    gtsam::Vector priorposNoiseVector12(12);
    // priorposNoiseVector12 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000,
    //                         prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000;
    priorposNoiseVector12 << prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise,
                            prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorposNoise = gtsam::noiseModel::Diagonal::Variances(priorposNoiseVector12);

    // gtsam::Vector priorvelNoiseVector3(3);
    // priorvelNoiseVector3 << prior_noise, prior_noise, prior_noise;
    // priorvelNoise = gtsam::noiseModel::Diagonal::Variances(priorvelNoiseVector3);

    gtsam::Vector priorNoiseVector6(6);
    // priorNoiseVector6 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000; 
    priorNoiseVector6 << marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise; // prior_noise / 100, prior_noise / 100, prior_noise / 100, prior_noise / 100, prior_noise / 100, prior_noise / 100; 
    //, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector priorNoiseVector12(12);
    // priorNoiseVector6 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000; 
    priorNoiseVector12 << marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise; 
    //, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorBiasNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector12);

    gtsam::Vector priorNoiseVector3(3);
    // priorNoiseVector6 << prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000, prior_noise / 1000; 
    priorNoiseVector3 << marg_noise, marg_noise, marg_noise; // 0.01, 0.01, 0.01; //, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01; 
    //, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise;
    priorGravNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector3);
    
    gtsam::Vector priordtNoiseVector4(4);
    priordtNoiseVector4 << prior_noise, prior_noise, prior_noise, prior_noise;
    priordtNoise = gtsam::noiseModel::Diagonal::Variances(priordtNoiseVector4);

    // gtsam::Vector margExtNoiseVector4(4);
    // margExtNoiseVector4 << 1e-6, 1e-6, 1e-6, 1e-6;
    // margExtNoise = gtsam::noiseModel::Diagonal::Variances(margExtNoiseVector4);

    gtsam::Vector priorddtNoiseVector1(1);
    priorddtNoiseVector1 << prior_noise;  // / 10;
    priorddtNoise = gtsam::noiseModel::Diagonal::Variances(priorddtNoiseVector1);

    gtsam::Vector margrotNoiseVector3(3);
    margrotNoiseVector3 << prior_noise / 10, prior_noise / 10, prior_noise / 10;
    margrotNoise = gtsam::noiseModel::Diagonal::Variances(margrotNoiseVector3);

    gtsam::Vector margposNoiseVector6(6);
    margposNoiseVector6 << prior_noise, prior_noise, prior_noise, prior_noise, prior_noise, prior_noise; //, marg_noise, marg_noise, marg_noise; //, marg_noise, marg_noise, marg_noise;
    margposNoise = gtsam::noiseModel::Diagonal::Variances(margposNoiseVector6);

    gtsam::Vector odomaNoiseVector12(12);
    odomaNoiseVector12 <<grav_noise, grav_noise, grav_noise, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10, odo_noise / 10;
    // odomaNoiseVector12 << marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise;
    odomaNoise = gtsam::noiseModel::Diagonal::Variances(odomaNoiseVector12);

    gtsam::Vector priorextrotNoiseVector3(3);
    priorextrotNoiseVector3 << prior_noise, prior_noise, prior_noise; // 10, 10, 100; // 
    priorextrotNoise = gtsam::noiseModel::Diagonal::Variances(priorextrotNoiseVector3);

    gtsam::Vector margNoiseVector3(3);
    margNoiseVector3 << prior_noise / 10, prior_noise / 10, prior_noise / 10; //, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, 
                        // marg_noise, marg_noise;
    margNoise = gtsam::noiseModel::Diagonal::Variances(margNoiseVector3);

    gtsam::Vector margdtNoiseVector4(4);
    margdtNoiseVector4 << prior_noise, prior_noise, prior_noise, prior_noise;
    margdtNoise = gtsam::noiseModel::Diagonal::Variances(margdtNoiseVector4);

    gtsam::Vector priorextposNoiseVector3(3);
    priorextposNoiseVector3 << prior_noise, prior_noise, prior_noise;
    priorextposNoise = gtsam::noiseModel::Diagonal::Variances(priorextposNoiseVector3);

    gtsam::Vector margddtNoiseVector1(1);
    margddtNoiseVector1 << prior_noise; //, prior_noise, prior_noise; //prior_noise;
    margddtNoise = gtsam::noiseModel::Diagonal::Variances(margddtNoiseVector1);

    gtsam::Vector dtNoiseVector4(4);
    dtNoiseVector4 << dt_noise / 10, dt_noise / 10, dt_noise / 10, dt_noise / 10;
    dtNoise = gtsam::noiseModel::Diagonal::Variances(dtNoiseVector4);

    // gtsam::Vector dtNoiseVector4_init(4);
    // dtNoiseVector4_init << dt_noise * 0.1, dt_noise * 0.1, dt_noise * 0.1, dt_noise * 0.1;
    // dtNoise_init = gtsam::noiseModel::Diagonal::Variances(dtNoiseVector4_init);

    gtsam::Vector ddtNoiseVector1(1);
    ddtNoiseVector1 << ddt_noise;
    ddtNoise = gtsam::noiseModel::Diagonal::Variances(ddtNoiseVector1);

    // gtsam::Vector ddtNoiseVector1_init(1);
    // ddtNoiseVector1_init << ddt_noise * 0.1;
    // ddtNoise_init = gtsam::noiseModel::Diagonal::Variances(ddtNoiseVector1_init);

    gtsam::Vector odomNoiseVector27(27);
    odomNoiseVector27 << grav_noise, grav_noise, grav_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, 
                            marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise, marg_noise;
    odomNoise = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector27); // should be related to the time, maybe proportional
    // gtsam::Vector relatNoiseVector6(6);
    // relatNoiseVector6 << odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise; //, odo_noise, odo_noise, odo_noise;
    // relatNoise = gtsam::noiseModel::Diagonal::Variances(relatNoiseVector6);
    // gtsam::Vector odomNoiseVector3(3);
    // odomNoiseVector3 << odo_noise, odo_noise, odo_noise; //, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise;
    // odomNoise = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector3); // should be related to the imu noise
    gtsam::Vector odomNoiseVector15(15);
    odomNoiseVector15 << odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise,
                        odo_noise, odo_noise, odo_noise, odo_noise, odo_noise, odo_noise;
    odomNoiseIMU = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector15); // should be related to the imu noise
    // odomNoiseIMU = gtsam::noiseModel::Robust::Create(
    //                 gtsam::noiseModel::mEstimator::Cauchy::Create(10), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                 gtsam::noiseModel::Diagonal::Variances(odomNoiseVector15));

    // double psrNoiseScore = psr_dopp_noise; // constant is ok...
    // double doppNoiseScore = psr_dopp_noise; // constant is ok...
    gtsam::Vector robustpsrdoppNoiseVector2(2); // gtsam::Pose3 factor has 6 elements (6D)
    // gtsam::Vector robustpsrdoppNoiseVector2_init(2); // gtsam::Pose3 factor has 6 elements (6D)
    robustpsrdoppNoiseVector2 << psr_dopp_noise, psr_dopp_noise; // / 10;
    // robustpsrdoppNoiseVector2_init << psr_dopp_noise * 10, psr_dopp_noise * 10;
    // double cpNoiseScore = cp_noise; // 1e9
    gtsam::Vector robustcpNoiseVector1(1); // gps factor has 3 elements (xyz)
    // gtsam::Vector robustcpNoiseVector1_init(1); // gps factor has 3 elements (xyz)
    robustcpNoiseVector1 << cp_noise; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    // robustcpNoiseVector1_init << cp_noise * 10; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    if (outlier_rej)
    {
      robustpsrdoppNoise = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      gtsam::noiseModel::Diagonal::Variances(robustpsrdoppNoiseVector2));


      robustpsrdoppNoise_init = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres_init), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      gtsam::noiseModel::Diagonal::Variances(robustpsrdoppNoiseVector2));

      robustcpNoise = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      gtsam::noiseModel::Diagonal::Variances(robustcpNoiseVector1));

      robustcpNoise_init = gtsam::noiseModel::Robust::Create(
                      gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres_init), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                      gtsam::noiseModel::Diagonal::Variances(robustcpNoiseVector1));
    //   ddtNoise = gtsam::noiseModel::Robust::Create(
    //                   gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                   // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                   gtsam::noiseModel::Diagonal::Variances(ddtNoiseVector1));
    //   dtNoise = gtsam::noiseModel::Robust::Create(
    //                   gtsam::noiseModel::mEstimator::Cauchy::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                   // gtsam::noiseModel::mEstimator::Huber::Create(outlier_thres), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
    //                   gtsam::noiseModel::Diagonal::Variances(dtNoiseVector4));
    }
    else
    {
      robustpsrdoppNoise = gtsam::noiseModel::Diagonal::Variances(robustpsrdoppNoiseVector2);
      robustcpNoise = gtsam::noiseModel::Diagonal::Variances(robustcpNoiseVector1);
    }
    // testNoise = gtsam::noiseModel::Gaussian::Covariance()
    // robustpsrdoppNoise = gtsam::noiseModel::Diagonal::Variances(robustpsrdoppNoiseVector2);

} // initNoises

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

void GNSSAssignment::delete_variables(bool nolidar, size_t frame_delete, int frame_num, size_t &id_accumulate, gtsam::FactorIndices delete_factor)
{
    if (!nolidar)
    {
      if (frame_delete > 0)
      {
        if (frame_delete >= change_ext)
        {
            gtsam::noiseModel::Gaussian::shared_ptr updatedERNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(P(0))); // important
            gtsam::noiseModel::Gaussian::shared_ptr updatedEPNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(E(0))); // important
            gtsam::PriorFactor<gtsam::Rot3> init_ER(P(0),isamCurrentEstimate.at<gtsam::Rot3>(P(0)), updatedERNoise); // margrotNoise); // 
            gtsam::PriorFactor<gtsam::Vector3> init_EP(E(0),isamCurrentEstimate.at<gtsam::Vector3>(E(0)), updatedEPNoise); // margNoise); //
            gtSAMgraph.add(init_ER);
            gtSAMgraph.add(init_EP);
            // factor_id_frame[0].push_back(id_accumulate);
            // factor_id_frame[0].push_back(id_accumulate+1);
            factor_id_frame[frame_num - 1 - frame_delete].push_back(id_accumulate);
            factor_id_frame[frame_num - 1 - frame_delete].push_back(id_accumulate+1);
            id_accumulate += 2;
            change_ext = frame_num;
        }
        size_t j = 0;
        for (; j < marg_thred; j++)
        {
            // get updated noise before reset
            // gtsam::noiseModel::Gaussian::shared_ptr updatedRotNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(R(frame_delete+j))); // important
            // gtsam::noiseModel::Gaussian::shared_ptr updatedPosNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(A(frame_delete+j))); // important
            // gtsam::noiseModel::Gaussian::shared_ptr updatedPosNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(F(frame_delete+j))); // important
            // gtsam::noiseModel::Gaussian::shared_ptr updatedDtNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(B(frame_delete+j))); // important
            // gtsam::noiseModel::Gaussian::shared_ptr updatedDdtNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(C(frame_delete+j))); // important

            // gtsam::PriorFactor<gtsam::Rot3> init_rot(R(frame_delete+j),isamCurrentEstimate.at<gtsam::Rot3>(R(frame_delete+j)),updatedRotNoise); //  margrotNoise); // 
            // gtsam::PriorFactor<gtsam::Vector12> init_vel(F(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector12>(F(frame_delete+j)), updatedPosNoise); // margposNoise);
            // gtsam::PriorFactor<gtsam::Vector6> init_vel(A(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector6>(A(frame_delete+j)),updatedPosNoise); //  margposNoise); // 
            gtsam::PriorFactor<gtsam::Vector4> init_dt(B(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector4>(B(frame_delete+j)), margdtNoise); //  updatedDtNoise); // 
            gtsam::PriorFactor<gtsam::Vector1> init_ddt(C(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector1>(C(frame_delete+j)), margddtNoise); // updatedDdtNoise); // 
            // gtSAMgraph.add(init_rot);
            // gtSAMgraph.add(init_vel);
            gtSAMgraph.add(init_dt);
            gtSAMgraph.add(init_ddt);
            factor_id_frame[0].push_back(id_accumulate+(j)*2);
            factor_id_frame[0].push_back(id_accumulate+1+(j)*2);
            // factor_id_frame[0].push_back(id_accumulate+2+(j)*4);
            // factor_id_frame[0].push_back(id_accumulate+3+(j)*4);
        }
        // id_accumulate += (j-1) * 4;
        id_accumulate += j * 2;
      }
      isam.update(gtSAMgraph, initialEstimate);
      gtSAMgraph.resize(0); // will the initialEstimate change?
      initialEstimate.clear();
      isam.update(gtSAMgraph, initialEstimate, delete_factor);   
    }
    else
    {
      if (frame_delete > 0) // (frame_delete == 0)
      {
        size_t j = 0;
        // for (; j < 10; j++)
        for (; j < marg_thred; j++)
        {
            gtsam::noiseModel::Gaussian::shared_ptr updatedRotNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(R(frame_delete+j))); // important
            gtsam::noiseModel::Gaussian::shared_ptr updatedPosNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(F(frame_delete+j))); // important
            gtsam::noiseModel::Gaussian::shared_ptr updatedDtNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(B(frame_delete+j))); // important
            gtsam::noiseModel::Gaussian::shared_ptr updatedDdtNoise = gtsam::noiseModel::Gaussian::Covariance(isam.marginalCovariance(C(frame_delete+j))); // important

            gtsam::PriorFactor<gtsam::Rot3> init_rot(R(frame_delete+j),isamCurrentEstimate.at<gtsam::Rot3>(R(frame_delete+j)), updatedRotNoise); // margrotNoise);
            gtsam::PriorFactor<gtsam::Vector12> init_vel(F(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector12>(F(frame_delete+j)), updatedPosNoise); // margposNoise);
            gtsam::PriorFactor<gtsam::Vector4> init_dt(B(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector4>(B(frame_delete+j)), updatedDtNoise); // margdtNoise);
            gtsam::PriorFactor<gtsam::Vector1> init_ddt(C(frame_delete+j), isamCurrentEstimate.at<gtsam::Vector1>(C(frame_delete+j)), updatedDdtNoise); // margddtNoise); // could delete?
            gtSAMgraph.add(init_rot);
            gtSAMgraph.add(init_vel);
            gtSAMgraph.add(init_dt);
            gtSAMgraph.add(init_ddt);
            
            {
            factor_id_frame[0].push_back(id_accumulate+j*4);
            factor_id_frame[0].push_back(id_accumulate+1+j*4);
            factor_id_frame[0].push_back(id_accumulate+2+j*4);
            factor_id_frame[0].push_back(id_accumulate+3+j*4);
            // factor_id_frame[0].push_back(id_accumulate+4+j*4);
            }
        }
        id_accumulate += j * 4;
      }
      isam.update(gtSAMgraph, initialEstimate);
      gtSAMgraph.resize(0); // will the initialEstimate change?
      initialEstimate.clear();
      isam.update(gtSAMgraph, initialEstimate, delete_factor);
      // gtSAMgraph.resize(0); // will the initialEstimate change?
      // initialEstimate.clear();
      // isam.update();
    }
}  

double GNSSAssignment::str2double(const std::string &num_str)
{
    size_t D_pos = num_str.find("D");
    std::string tmp_str = num_str;
    if (D_pos != std::string::npos)
        tmp_str = tmp_str.replace(D_pos, 1, "e");
    return std::stod(tmp_str);
}


// int GNSSAssignment::satno_rtk(int sys, int prn)
// {
//     if (prn<=0) return 0;
//     switch (sys) {
//         case SYS_GPS:
//             if (prn<1||32<prn) return 0;
//             return prn-1+1;
//         case SYS_GLO:
//             if (prn<1||24<prn) return 0;
//             return 32+prn-1+1;
//         case SYS_GAL:
//             if (prn<1||30<prn) return 0;
//             return 32+24+prn-1+1;
//         case SYS_QZS:
//             if (prn<193||199<prn) return 0;
//             return 32+24+30+prn-193+1;
//         case SYS_BDS:
//             if (prn<1||35<prn) return 0;
//             return 32+24+30+7+prn-1+1;
//         case SYS_LEO:
//             if (prn<1||10<prn) return 0;
//             return 32+24+30+7+35+prn-1+1;
//         case SYS_SBS:
//             if (prn<120||140<prn) return 0;
//             return 32+24+30+7+35+10+prn-120+1;
//     }
//     return 0;
// }
