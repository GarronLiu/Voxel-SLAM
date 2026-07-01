#pragma once

#include "tools.h"
#include "ekf_imu.hpp"
#include "voxel_map.hpp"
#include "feature_point.hpp"
#include "loop_refine.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <queue>
#include <sstream>
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
#include <gnss_comm/gnss_utility.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <malloc.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include "BTC.h"

#include "GNSS_Processing_fg.h"

using namespace std;

ros::Publisher pub_scan, pub_cmap, pub_init, pub_pmap;
ros::Publisher pub_test, pub_prev_path, pub_curr_path;
ros::Publisher pub_gnss_fix_local, pub_gnss_pvt_local, pub_gnss_spp_local, pub_gnss_tc_local;
ros::Publisher pub_lio_odom_enu;
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
mutex mGnssMeasBuf;
Features feat;
deque<sensor_msgs::Imu::Ptr> imu_buf;
deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
deque<double> time_buf;
deque<gnss_comm::PVTSolutionPtr> gnss_pvt_buf;
deque<vector<gnss_comm::ObsPtr>> gnss_meas_sync_buf;
queue<vector<gnss_comm::ObsPtr>> gnss_meas_buf;

std::shared_ptr<GNSSProcess> p_gnss;

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
  if(!p_gnss || !msg_in) return;
  gnss_comm::EphemPtr ephem = gnss_comm::msg2ephem(msg_in);
  p_gnss->p_assign->inputEphem(ephem);
}

void gnss_glo_ephem_handler(const gnss_comm::GnssGloEphemMsgConstPtr &msg_in)
{
  if(!p_gnss || !msg_in) return;
  gnss_comm::GloEphemPtr glo_ephem = gnss_comm::msg2glo_ephem(msg_in);
  p_gnss->p_assign->inputEphem(glo_ephem);
}

void gnss_meas_handler(const gnss_comm::GnssMeasMsgConstPtr &msg_in)
{
  vector<gnss_comm::ObsPtr> gnss_meas = gnss_comm::msg2meas(msg_in);
  if(gnss_meas.empty())
    return;
  //latest_gnss_time = time2sec(gnss_meas[0]->time);

  {
    lock_guard<mutex> lock(mGnssMeasBuf);
    gnss_meas_sync_buf.push_back(gnss_meas);
  }
  gnss_meas_buf.push(std::move(gnss_meas));
}

void gnss_pvt_handler(const gnss_comm::GnssPVTSolnMsgConstPtr &msg_in)
{
  if(!p_gnss || !msg_in) return;
  double ts = time2sec(gst2time(msg_in->time.week, msg_in->time.tow));
  p_gnss->inputpvt(ts, msg_in->latitude, msg_in->longitude, msg_in->altitude,
                   msg_in->h_acc, msg_in->v_acc,
                   msg_in->carr_soln, msg_in->diff_soln);
}

void gnss_iono_params_handler(const gnss_comm::StampedFloat64ArrayConstPtr &msg_in)
{
    if(!p_gnss || !msg_in) return;
    double ts = msg_in->header.stamp.toSec();
    std::vector<double> iono_params;
    std::copy(msg_in->data.begin(), msg_in->data.end(), std::back_inserter(iono_params));
    assert(iono_params.size() == 8);
    p_gnss->inputIonoParams(ts, iono_params);
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

bool sync_packages_append_GNSSRaw(
    pcl::PointCloud<PointType>::Ptr &pl_ptr,
    deque<sensor_msgs::Imu::Ptr> &imus,
    IMUEKF &p_imu,
    vector<gnss_comm::ObsPtr> &matched_gnss_raw,
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

  // GnssMeasMsg没有Header，各卫星观测属于同一历元，因此使用首个观测的GPS时间。
  // 在原始GNSS数据尚未推进到当前LiDAR扫描附近时，等待新的测量消息到达。
  const double scan_end = max(p_imu.pcl_beg_time, p_imu.pcl_end_time);
  const double target_meas_utc =
    round(scan_end / p_gnss->gnss_sample_period) * p_gnss->gnss_sample_period;
  const double round_time_error = fabs(scan_end - target_meas_utc);
  const bool lidar_end_near_round_time = round_time_error <= gnss_max_delay;
  if(gnss_enable && lidar_end_near_round_time)
    // if(gnss_enable)
  {
    lock_guard<mutex> lock(mGnssMeasBuf);
    while(!gnss_meas_sync_buf.empty())
    {
      vector<gnss_comm::ObsPtr> &front_meas = gnss_meas_sync_buf.front();
      if(front_meas.empty() || !front_meas.front())
      {
        gnss_meas_sync_buf.pop_front();
        continue;
      }

      double meas_time = gnss_comm::time2sec(front_meas.front()->time);
      double meas_local_time = meas_time - gnss_local_time_diff;
      if(meas_local_time >= scan_end - gnss_max_delay)
        break;

      gnss_meas_sync_buf.pop_front();
    }

    if(gnss_meas_sync_buf.empty())
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

  // Attach one raw GNSS measurement epoch to the current LiDAR scan if available.
  matched_gnss_raw.clear();
  if(gnss_enable && lidar_end_near_round_time)
  //  if(gnss_enable)
  {
    lock_guard<mutex> lock(mGnssMeasBuf);
    while(!gnss_meas_sync_buf.empty())
    {
      vector<gnss_comm::ObsPtr> &front_meas = gnss_meas_sync_buf.front();
      if(front_meas.empty() || !front_meas.front())
      {
        gnss_meas_sync_buf.pop_front();
        continue;
      }

      double meas_time = gnss_comm::time2sec(front_meas.front()->time);
      double meas_local_time = meas_time - gnss_local_time_diff;

      if(meas_local_time > scan_end + gnss_max_delay)
        break;

      if(fabs(meas_local_time - scan_end) <= gnss_max_delay)
      {
        matched_gnss_raw = std::move(front_meas);
        gnss_meas_sync_buf.pop_front();
        break;
      }

      gnss_meas_sync_buf.pop_front();
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
