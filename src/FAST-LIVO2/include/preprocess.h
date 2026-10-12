/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef PREPROCESS_H_
#define PREPROCESS_H_

#include "common_lib.h"
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false)

// ###################### 1. 几何特征分类与判定状态 ######################
// 点的几何类别；仅在 feature_enabled 路径中使用。
enum LiDARFeature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
};
// 相邻点方向：用于判断当前点的前后邻域。
enum Surround
{
  Prev,
  Next
};
// 相邻点连线/跳变的状态标记。
enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
};

// 单个点的局部几何信息，供 give_feature() 完成平面和边缘分类。
struct orgtype
{
  double range;
  double dista;
  double angle[2];
  double intersect;
  E_jump edj[2];
  LiDARFeature ftype;
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

// ###################### 2. 各雷达 ROS 点类型适配 ######################
// Velodyne PointCloud2 的字段定义与 PCL 注册。
namespace velodyne_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  float time;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(std::uint16_t, ring, ring))
// Ouster PointCloud2 的字段定义与 PCL 注册。
namespace ouster_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  std::uint32_t t;
  std::uint16_t reflectivity;
  uint8_t ring;
  std::uint16_t ambient;
  std::uint32_t range;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace ouster_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (std::uint32_t, t, t)(std::uint16_t, reflectivity,
                                                        reflectivity)(std::uint8_t, ring, ring)(std::uint16_t, ambient, ambient)(std::uint32_t, range, range))
// Hesai XT32 PointCloud2 的字段定义与 PCL 注册。
namespace xt32_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float intensity;
  double timestamp;
  std::uint16_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace xt32_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(double, timestamp, timestamp)(std::uint16_t, ring, ring))
// Hesai Pandar128 PointCloud2 的字段定义与 PCL 注册。
namespace Pandar128_ros
{
struct EIGEN_ALIGN16 Point
{
  PCL_ADD_POINT4D;
  float timestamp;
  uint8_t ring;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace Pandar128_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(Pandar128_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, timestamp, timestamp))
class Preprocess
{
public:
  // ###################### 1. 生命周期与对外预处理接口 ######################
  Preprocess();
  ~Preprocess();

  // 将 Livox CustomMsg 转为内部 PointCloudXYZI，并写入 pcl_out。
  void process(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  // 将其他雷达的 PointCloud2 转为内部 PointCloudXYZI，并写入 pcl_out。
  void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  // 直接设置特征提取、雷达类型、盲区距离和均匀抽点间隔。
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // ###################### 2. 当前帧点云缓存与扫描线组织 ######################
  // 原始转换点云、角点特征、最终输出的面/普通点云。
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  // 最多 128 条扫描线的点缓存与对应局部几何信息。
  PointCloudXYZI pl_buff[128];
  vector<orgtype> typess[128];

  // ###################### 3. YAML 预处理配置与运行开关 ######################
  // 雷达类型、均匀抽点间隔、扫描线数量和扫描频率。
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE;
  // 近距离盲区阈值及其平方，后者用于避免每点开方。
  double blind, blind_sqr;
  // 是否启用几何特征提取；输入消息是否已提供有效逐点时间。
  bool feature_enabled, given_offset_time;
  // 由 config/log.yaml 控制的输入/输出点数日志开关。
  bool log_input_point_count = false;
  bool log_output_point_count = false;

  // ###################### 4. 调试点云发布者 ######################
  // 可选发布完整点云、面点与角点；当前主流程默认不使用。
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_full;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_surf;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_corn;

private:
  // ###################### 5. 按雷达型号分派的消息处理函数 ######################
  // Livox AVIA / MID-360 的 CustomMsg 处理入口。
  void avia_handler(const livox_ros_driver2::msg::CustomMsg::SharedPtr &msg);
  // Ouster、Velodyne、Hesai XT32、Pandar128、L515 的 PointCloud2 处理入口。
  void oust64_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void xt32_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void Pandar128_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void l515_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

  // ###################### 6. 可选几何特征提取与调试工具 ######################
  // 基于扫描线局部几何关系，将点分类为平面、边缘等特征。
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  // 发布调试点云。
  void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);
  // 平面段判定、局部小平面判定与边缘跳变判定。
  int plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

  // ###################### 7. 特征提取阈值与临时计算变量 ######################
  // 邻域分组大小及距离阈值。
  int group_size;
  double disA, disB, inf_bound;
  // 平面/边缘判断的比例、角度与距离阈值。
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  // 相邻点坐标差的临时缓存。
  double vx, vy, vz;
};

// Preprocess 的共享智能指针类型，供 LIVMapper 持有和调用。
typedef std::shared_ptr<Preprocess> PreprocessPtr;

#endif // PREPROCESS_H_
