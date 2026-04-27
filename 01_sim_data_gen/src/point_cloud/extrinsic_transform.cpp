#include "extrinsic_transform.h"

#include <sstream>
#include <iomanip>
#include <pcl/common/transforms.h>

namespace sim_data_gen {
namespace transform {

ExtrinsicTransform::ExtrinsicTransform(const ExtrinsicConfig& config)
    : config_(config) {
    initPoses();
}

void ExtrinsicTransform::initPoses() {
    poses_.clear();

    int pose_id = 0;

    // X轴俯仰角遍历
    for (float rx_deg = config_.rx_min; rx_deg <= config_.rx_max + 0.001f; rx_deg += config_.rx_step) {
        // Y轴翻滚角遍历
        for (float ry_deg = config_.ry_min; ry_deg <= config_.ry_max + 0.001f; ry_deg += config_.ry_step) {
            // Z轴方位角遍历
            for (float rz_deg = config_.rz_min; rz_deg < config_.rz_max; rz_deg += config_.rz_step) {
                Pose pose;
                pose.rx = degToRad(rx_deg);
                pose.ry = degToRad(ry_deg);
                pose.rz = degToRad(rz_deg);
                pose.tx = config_.tx;
                pose.ty = config_.ty;
                pose.tz = config_.tz;
                pose.pose_id = pose_id;

                // 生成姿态名称
                std::ostringstream oss;
                oss << "rx" << static_cast<int>(rx_deg)
                    << "_ry" << static_cast<int>(ry_deg)
                    << "_rz" << static_cast<int>(rz_deg);
                pose.name = oss.str();

                poses_.push_back(pose);
                ++pose_id;
            }
        }
    }
}

std::vector<Pose> ExtrinsicTransform::generatePoses() const {
    return poses_;
}

Eigen::Matrix4f ExtrinsicTransform::computeExtrinsicMatrix(const Pose& pose) const {
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

    // 计算旋转矩阵
    Eigen::Matrix3f rotation = eulerToRotation(pose.rx, pose.ry, pose.rz);

    // 构建4x4变换矩阵
    transform.block<3, 3>(0, 0) = rotation;
    transform(0, 3) = pose.tx;
    transform(1, 3) = pose.ty;
    transform(2, 3) = pose.tz;

    return transform;
}

Eigen::Matrix3f ExtrinsicTransform::eulerToRotation(float rx, float ry, float rz) const {
    // ZYX欧拉角转旋转矩阵（航空惯例）
    // 旋转顺序：X轴 → Y轴 → Z轴
    float cx = std::cos(rx);
    float sx = std::sin(rx);
    float cy = std::cos(ry);
    float sy = std::sin(ry);
    float cz = std::cos(rz);
    float sz = std::sin(rz);

    Eigen::Matrix3f rot_x, rot_y, rot_z;

    // 绕X轴旋转
    rot_x << 1, 0, 0,
             0, cx, -sx,
             0, sx, cx;

    // 绕Y轴旋转
    rot_y << cy, 0, sy,
             0, 1, 0,
             -sy, 0, cy;

    // 绕Z轴旋转
    rot_z << cz, -sz, 0,
             sz, cz, 0,
             0, 0, 1;

    // 组合旋转：R = Rz * Ry * Rx
    return rot_z * rot_y * rot_x;
}

PointCloudXYZPtr ExtrinsicTransform::applyTransform(
    const PointCloudXYZPtr& input,
    const Eigen::Matrix4f& transform) const {

    auto output = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::transformPointCloud(*input, *output, transform);
    return output;
}

std::vector<PointCloudXYZPtr> ExtrinsicTransform::batchTransform(
    const PointCloudXYZPtr& input,
    const std::vector<Pose>& poses) const {

    std::vector<PointCloudXYZPtr> results;
    results.reserve(poses.size());

    for (const auto& pose : poses) {
        Eigen::Matrix4f transform = computeExtrinsicMatrix(pose);
        results.push_back(applyTransform(input, transform));
    }

    return results;
}

int ExtrinsicTransform::getTotalPoses() const {
    return static_cast<int>(poses_.size());
}

const ExtrinsicConfig& ExtrinsicTransform::getConfig() const {
    return config_;
}

}  // namespace transform
}  // namespace sim_data_gen
