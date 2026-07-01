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
#include <vector>
#include <eigen3/Eigen/Dense>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_ros.hpp>
#include "tools.h"
#include <numeric>
#include <opencv2/core/eigen.hpp>
#include <fstream>
#include <queue>


using namespace gnss_comm;


struct sat_first
{
    double timecur;
    Eigen::Vector3d RTex;
    int frame_num;

    bool operator <(const sat_first &s) const
    {
        return (timecur < s.timecur);
    }
};

class GNSSAssignment
{
    public:
        GNSSAssignment();
        GNSSAssignment(const GNSSAssignment&) = delete;
        GNSSAssignment& operator=(const GNSSAssignment&) = delete;
        ~GNSSAssignment() {};
        // ofstream fout_std;


        double prior_noise = 0.01;
        double marg_noise = 0.01;
        double ddt_noise = 0.01;
        double dt_noise = 0.01;
        double odo_noise = 0.01;
        double grav_noise = 0.01;
        double psr_dopp_noise = 0.01;
        double psr_noise = 0.01;
        double dopp_noise = 0.01;
        double cp_noise = 0.01;

        double ave_std = 0.0;
        int num_std = 0;

        bool outlier_rej = false;
        double outlier_thres = 0.1;
        double outlier_thres_init = 10;
        uint32_t gnss_track_num_threshold = 0;
        int process_feat_num = 0;

        int marg_thred = 1;
        int change_ext = 1;
        std::deque<std::vector<size_t>> factor_id_frame; // 

        std::map<uint32_t, std::vector<EphemBasePtr>> sat2ephem;
        std::vector<double> latest_gnss_iono_params;
        bool ephem_from_rinex = false;
        bool obs_from_rinex = false;
        bool pvt_is_gt = true;
        std::map<uint32_t, std::map<double, size_t>> sat2time_index;
        void inputEphem(EphemBasePtr ephem_ptr);
        int freq_idx_ = 0;
        // int satno_rtk(int sys, int prn);
        double gnss_psr_std_threshold = 30.0;
        double gnss_dopp_std_threshold = 30.0;
        double gnss_cn0_threshold = 35.0;
        double sum_d = 0, sum_d2 = 0;
        double gnss_cp_std_threshold = 30;
        // double hatch_filter_meas = 0, last_cp = 0;
        bool cp_locked = false;
        std::map<uint32_t, uint32_t> sat_track_status; //
        std::map<uint32_t, double> sat_track_time; //
        std::map<uint32_t, double> sat_track_last_time; //
        std::map<uint32_t, double> hatch_filter_meas; //
        // std::map<uint32_t, double> hatch_filter_noise; //
        std::map<uint32_t, double> last_cp_meas; //
        double gnss_elevation_threshold = 30;
        void processGNSSBase(const std::vector<ObsPtr> &gnss_meas, std::vector<double> &psr_meas, std::vector<ObsPtr> &valid_meas, std::vector<EphemBasePtr> &valid_ephems, bool gnss_ready, Eigen::Vector3d ecef_pos, double last_gnss_time_process);
};
