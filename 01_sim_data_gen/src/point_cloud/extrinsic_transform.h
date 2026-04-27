#pragma once

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <string>

namespace sim_data_gen {
namespace transform {

// 点云类型别名
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;

// 外参变换配置
struct ExtrinsicConfig {
    float rx_min = 0.0f;       // X轴俯仰角最小值（度）
    float rx_max = 80.0f;     // X轴俯仰角最大值（度）
    float rx_step = 20.0f;    // X轴采样间隔（度）

    float ry_min = 0.0f;     // Y轴翻滚角最小值（度）
    float ry_max = 80.0f;     // Y轴翻滚角最大值（度）
    float ry_step = 20.0f;    // Y轴采样间隔（度）

    float rz_min = 0.0f;      // Z轴方位角最小值（度）
    float rz_max = 360.0f;    // Z轴方位角最大值（度）
    float rz_step = 20.0f;    // Z轴方位角采样间隔（度）

    float tx = 0.0f;          // X方向平移（米）
    float ty = 0.0f;          // Y方向平移（米）
    float tz = 1.0f;          // Z方向平移（米）- 默认抬高1米模拟相机视角
};

// 单个姿态参数
struct Pose {
    float rx;  // X轴旋转（弧度）
    float ry;  // Y轴旋转（弧度）
    float rz;  // Z轴旋转（弧度）
    float tx;  // X方向平移
    float ty;  // Y方向平移
    float tz;  // Z方向平移

    int pose_id;      // 姿态编号
    std::string name; // 姿态名称，如 "rx0_ry0_rz0"
};

// 外参变换器
class ExtrinsicTransform {
public:
    // 构造函数
    explicit ExtrinsicTransform(const ExtrinsicConfig& config = ExtrinsicConfig{});

    // 生成所有姿态参数列表
    // @return 姿态参数向量
    std::vector<Pose> generatePoses() const;

    // 计算指定姿态的外参矩阵
    // @param pose 姿态参数
    // @return 4x4变换矩阵
    Eigen::Matrix4f computeExtrinsicMatrix(const Pose& pose) const;

    // 计算旋转矩阵（欧拉角转旋转矩阵）
    // @param rx, ry, rz 欧拉角（弧度）
    // @return 3x3旋转矩阵
    Eigen::Matrix3f eulerToRotation(float rx, float ry, float rz) const;

    // 应用外参变换到点云
    // @param input 输入点云
    // @param transform 4x4变换矩阵
    // @return 变换后的点云
    PointCloudXYZPtr applyTransform(const PointCloudXYZPtr& input,
                                   const Eigen::Matrix4f& transform) const;

    // 生成批量变换
    // @param input 输入点云
    // @param poses 姿态参数列表
    // @return 变换后的点云列表
    std::vector<PointCloudXYZPtr> batchTransform(const PointCloudXYZPtr& input,
                                                  const std::vector<Pose>& poses) const;

    // 获取姿态总数
    int getTotalPoses() const;

    // 获取配置
    const ExtrinsicConfig& getConfig() const;

private:
    ExtrinsicConfig config_;
    std::vector<Pose> poses_;

    // 初始化姿态列表
    void initPoses();

    // 度转弧度
    static constexpr float degToRad(float deg) { return deg * 0.017453292519943295f; }

    // 弧度转度
    static constexpr float radToDeg(float rad) { return rad * 57.29577951308232f; }
};

}  // namespace transform
}  // namespace sim_data_gen
