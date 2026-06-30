/*
 * BSD 3-Clause License
 */

#ifndef GNSS_PSR_DOPP_FACTOR_FIXED_LIO_H_
#define GNSS_PSR_DOPP_FACTOR_FIXED_LIO_H_

#include <cmath>
#include <vector>

#include <Eigen/Dense>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gnss_comm/gnss_constant.hpp>
#include <gnss_comm/gnss_utility.hpp>

namespace ligo {

class GnssPsrDoppFactorFixedLio
    : public gtsam::NoiseModelFactor4<gtsam::Vector3, gtsam::Rot3, gtsam::Vector4, gtsam::Vector1>
{
  public:
    GnssPsrDoppFactorFixedLio(gtsam::Key anchor_key, gtsam::Key rot_key,
                              gtsam::Key dt_key, gtsam::Key ddt_key,
                              const Eigen::Vector3d &local_pos_,
                              const Eigen::Vector3d &local_vel_,
                              const double values_[24],
                              const int sys_idx_,
                              const gtsam::SharedNoiseModel &model)
        : gtsam::NoiseModelFactor4<gtsam::Vector3, gtsam::Rot3, gtsam::Vector4, gtsam::Vector1>(
              model, anchor_key, rot_key, dt_key, ddt_key),
          local_pos(local_pos_), local_vel(local_vel_), sys_idx(sys_idx_)
    {
      sv_pos << values_[0], values_[1], values_[2];
      sv_vel << values_[3], values_[4], values_[5];
      svdt = values_[6];
      tgd = values_[7];
      svddt = values_[8];
      pr_uura = values_[9];
      dp_uura = values_[10];
      relative_sqrt_info = values_[11];
      for(int i = 0; i < 8; ++i)
        latest_gnss_iono_params.push_back(values_[12+i]);
      time_current = values_[20];
      freq = values_[21];
      psr_measured = values_[22];
      dopp_measured = values_[23];
    }

    virtual ~GnssPsrDoppFactorFixedLio() {}

    gtsam::Vector evaluateError(const gtsam::Vector3 &anchor_ecef,
                                const gtsam::Rot3 &R_ecef_local,
                                const gtsam::Vector4 &dt,
                                const gtsam::Vector1 &ddt,
                                boost::optional<gtsam::Matrix&> H1 = boost::none,
                                boost::optional<gtsam::Matrix&> H2 = boost::none,
                                boost::optional<gtsam::Matrix&> H3 = boost::none,
                                boost::optional<gtsam::Matrix&> H4 = boost::none) const
    {
      const Eigen::Matrix3d R = R_ecef_local.matrix();
      const Eigen::Vector3d P_ecef = R * local_pos + anchor_ecef;
      const Eigen::Vector3d V_ecef = R * local_vel;

      double ion_delay = 0.0;
      double tro_delay = 0.0;
      double azel[2] = {0.0, M_PI/2.0};
      if(P_ecef.norm() > 0.0)
      {
        gnss_comm::sat_azel(P_ecef, sv_pos, azel);
        Eigen::Vector3d rcv_lla = gnss_comm::ecef2geo(P_ecef);
        tro_delay = gnss_comm::calculate_trop_delay(gnss_comm::sec2time(time_current), rcv_lla, azel);
        ion_delay = gnss_comm::calculate_ion_delay(gnss_comm::sec2time(time_current),
                                                   latest_gnss_iono_params, rcv_lla, azel);
      }

      const double sin_el = std::sin(azel[1]);
      const double sin_el_2 = sin_el * sin_el;
      const double pr_weight = sin_el_2 / pr_uura * relative_sqrt_info;
      const double dp_weight = sin_el_2 / dp_uura * relative_sqrt_info * 3.0;

      const Eigen::Vector3d rcv2sat_ecef = sv_pos - P_ecef;
      const Eigen::Vector3d rcv2sat_unit = rcv2sat_ecef.normalized();
      const double wavelength = LIGHT_SPEED / freq;
      const double psr_sagnac = EARTH_OMG_GPS *
          (sv_pos(0)*P_ecef(1) - sv_pos(1)*P_ecef(0)) / LIGHT_SPEED;
      const double psr_estimated = rcv2sat_ecef.norm() + psr_sagnac + dt[sys_idx] -
          svdt * LIGHT_SPEED + ion_delay + tro_delay + tgd * LIGHT_SPEED;

      const double dopp_sagnac = EARTH_OMG_GPS / LIGHT_SPEED *
          (sv_vel(0)*P_ecef(1) + sv_pos(0)*V_ecef(1) -
           sv_vel(1)*P_ecef(0) - sv_pos(1)*V_ecef(0));
      const double dopp_estimated = (sv_vel - V_ecef).dot(rcv2sat_unit) + ddt[0] +
          dopp_sagnac - svddt * LIGHT_SPEED;

      gtsam::Vector residual(2);
      residual[0] = (psr_estimated - psr_measured) * pr_weight;
      residual[1] = (dopp_estimated + dopp_measured * wavelength) * dp_weight;

      const double norm3 = std::pow(rcv2sat_ecef.norm(), 3);
      const double norm2 = rcv2sat_ecef.squaredNorm();
      Eigen::Matrix3d unit2rcv_pos;
      for(size_t i = 0; i < 3; ++i)
      {
        for(size_t k = 0; k < 3; ++k)
        {
          if(i == k)
            unit2rcv_pos(i, k) = (norm2 - rcv2sat_ecef(i)*rcv2sat_ecef(i)) / norm3;
          else
            unit2rcv_pos(i, k) = (-rcv2sat_ecef(i)*rcv2sat_ecef(k)) / norm3;
        }
      }
      unit2rcv_pos *= -1.0;

      if(H1)
      {
        (*H1) = gtsam::Matrix::Zero(2, 3);
        (*H1).block<1,3>(0,0) = -rcv2sat_unit.transpose() * pr_weight;
        (*H1).block<1,3>(1,0) = (sv_vel - V_ecef).transpose() * unit2rcv_pos * dp_weight;
      }
      if(H2)
      {
        (*H2) = gtsam::Matrix::Zero(2, 3);
        Eigen::Matrix3d d_pos;
        d_pos << 0.0, -local_pos[2], local_pos[1],
                 local_pos[2], 0.0, -local_pos[0],
                 -local_pos[1], local_pos[0], 0.0;
        Eigen::Matrix3d d_vel;
        d_vel << 0.0, -local_vel[2], local_vel[1],
                 local_vel[2], 0.0, -local_vel[0],
                 -local_vel[1], local_vel[0], 0.0;
        (*H2).block<1,3>(0,0) = rcv2sat_unit.transpose() * (R * d_pos) * pr_weight;
        (*H2).block<1,3>(1,0) = rcv2sat_unit.transpose() * (R * d_vel) * dp_weight -
            (sv_vel - V_ecef).transpose() * unit2rcv_pos * R * d_pos * dp_weight;
      }
      if(H3)
      {
        (*H3) = gtsam::Matrix::Zero(2, 4);
        (*H3)(0, sys_idx) = pr_weight;
      }
      if(H4)
      {
        (*H4) = gtsam::Matrix::Zero(2, 1);
        (*H4)(1, 0) = dp_weight;
      }
      return residual;
    }

  private:
    Eigen::Vector3d local_pos;
    Eigen::Vector3d local_vel;
    Eigen::Vector3d sv_pos;
    Eigen::Vector3d sv_vel;
    std::vector<double> latest_gnss_iono_params;
    double svdt = 0.0;
    double tgd = 0.0;
    double svddt = 0.0;
    double pr_uura = 1.0;
    double dp_uura = 1.0;
    double relative_sqrt_info = 1.0;
    double time_current = 0.0;
    double freq = 1.0;
    double psr_measured = 0.0;
    double dopp_measured = 0.0;
    int sys_idx = 0;
};

}  // namespace ligo

#endif
