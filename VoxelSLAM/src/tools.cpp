#include "tools.h"

#include <Eigen/Geometry>
#include <cmath>

const Eigen::Matrix3d I33(Eigen::Matrix3d::Identity());

Eigen::Matrix3d Exp(const Eigen::Vector3d &ang)
{
  double ang_norm = ang.norm();
  if (ang_norm >= 1e-11)
  {
    Eigen::Vector3d r_axis = ang / ang_norm;
    Eigen::Matrix3d K;
    K << SKEW_SYM_MATRX(r_axis);
    return I33 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }

  return I33;
}

Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt)
{
  double ang_vel_norm = ang_vel.norm();
  if (ang_vel_norm > 1e-7)
  {
    Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix3d K;
    K << SKEW_SYM_MATRX(r_axis);
    double r_ang = ang_vel_norm * dt;

    return I33 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  }

  return I33;
}

Eigen::Vector3d Log(const Eigen::Matrix3d &R)
{
  double theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Vector3d K(R(2, 1) - R(1, 2), R(0, 2) - R(2, 0), R(1, 0) - R(0, 1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

Eigen::Matrix3d hat(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d Omega;
  Omega << 0, -v(2), v(1),
           v(2), 0, -v(0),
           -v(1), v(0), 0;
  return Omega;
}

Eigen::Matrix3d jr(Eigen::Vector3d vec)
{
  double ang = vec.norm();

  if (ang < 1e-9)
  {
    return I33;
  }

  vec /= ang;
  double ra = std::sin(ang) / ang;
  return ra * I33 + (1 - ra) * vec * vec.transpose() - (1 - std::cos(ang)) / ang * hat(vec);
}

Eigen::Matrix3d jr_inv(const Eigen::Matrix3d &rotR)
{
  Eigen::AngleAxisd rot_vec(rotR);
  Eigen::Vector3d axi = rot_vec.axis();
  double ang = rot_vec.angle();

  if (ang < 1e-9)
  {
    return I33;
  }

  double ctt = ang / 2 / std::tan(ang / 2);
  return ctt * I33 + (1 - ctt) * axi * axi.transpose() + ang / 2 * hat(axi);
}

Eigen::Matrix3d Jacob_right_inv(Eigen::Vector3d &vec)
{
  Eigen::Matrix3d hat_v, res;
  hat_v << SKEW_SYM_MATRX(vec);
  if (vec.norm() > 1e-6)
  {
    res = Eigen::Matrix<double, 3, 3>::Identity() + 0.5 * hat_v +
          (1 - vec.norm() * std::cos(vec.norm() / 2) / 2 / std::sin(vec.norm() / 2)) *
              hat_v * hat_v / vec.squaredNorm();
  }
  else
  {
    res = Eigen::Matrix<double, 3, 3>::Identity();
  }
  return res;
}

void down_sampling_voxel(pcl::PointCloud<PointType> &pl_feat, double voxel_size)
{
  if(voxel_size < 0.001) return;

  unordered_map<VOXEL_LOC, PointType> feat_map;
  float loc_xyz[3];
  for(PointType &p_c : pl_feat.points)
  {
    for(int j=0; j<3; j++)
    {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if(loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter == feat_map.end())
    {
      PointType pp = p_c;
      pp.curvature = 1;
      feat_map[position] = pp;
    }
    else
    {
      PointType &pp = iter->second;
      pp.x = (pp.x * pp.curvature + p_c.x) / (pp.curvature + 1);
      pp.y = (pp.y * pp.curvature + p_c.y) / (pp.curvature + 1);
      pp.z = (pp.z * pp.curvature + p_c.z) / (pp.curvature + 1);
      pp.curvature += 1;
    }
  }

  pl_feat.clear();
  for(auto iter=feat_map.begin(); iter!=feat_map.end(); ++iter)
    pl_feat.push_back(iter->second);
  
}

void down_sampling_close(pcl::PointCloud<PointType> &pl_feat, double voxel_size)
{
  if(voxel_size < 0.001) return;

  unordered_map<VOXEL_LOC, pcl::PointCloud<PointType>::Ptr> feat_map;
  float loc_xyz[3];
  for(PointType &p_c: pl_feat.points)
  {
    for(int j=0; j<3; j++)
    {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if(loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter == feat_map.end())
    {
      pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>);
      pl_ptr->push_back(p_c);
      feat_map[position] = pl_ptr;
    }
    else
    {
      iter->second->push_back(p_c);
    }
  }

  pl_feat.clear();
  for(auto iter=feat_map.begin(); iter!=feat_map.end(); ++iter)
  {
    pcl::PointCloud<PointType>::Ptr pl_ptr = iter->second;

    PointType pb = pl_ptr->points[0];
    int plsize = pl_ptr->size();
    for(int i=1; i<plsize; i++)
    {
      PointType &pp = pl_ptr->points[i];
      pb.x += pp.x; pb.y += pp.y; pb.z += pp.z;
    }
    pb.x /= plsize; pb.y /=plsize; pb.z /= plsize;

    double ndis = 100;
    int mnum = 0;
    for(int i=0; i<plsize; i++)
    {
      PointType &pp = pl_ptr->points[i];
      double xx = pb.x - pp.x;
      double yy = pb.y - pp.y;
      double zz = pb.z - pp.z;
      double dis = xx*xx + yy*yy + zz*zz;
      if(dis < ndis)
      {
        mnum = i;
        ndis = dis;
      }
    }

    pl_feat.push_back(pl_ptr->points[mnum]);
  }

}

