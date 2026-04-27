#pragma once

#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "../utils/types.h"

namespace sim_data_gen {
namespace point_cloud {

/**
 * @brief 采样配置
 */
struct SamplingConfig {
    /** 每条边采样点数（不包括顶点），默认15 */
    int samples_per_edge = 15;

    /** 每平方毫米采样点数，默认0.1 */
    float points_per_mm2 = 0.1f;

    /** 是否启用单位标准化（米转毫米），默认false */
    bool enable_unit_standardization = false;

    /** 是否启用中心化，默认true */
    bool enable_centering = true;

    /** 是否启用PCA对齐，默认true */
    bool enable_pca_alignment = true;

    /** 目标点数，如果为0则使用完整采样 */
    int target_points = 0;
};

/**
 * @brief STL模型读取器
 *
 * 使用PCL读取STL/PLY格式的3D模型文件，并转换为点云
 * 支持基于面积的混合采样（顶点 + 边缘 + 内部）
 */
class STLReader {
public:
    /**
     * @brief 默认构造函数
     */
    STLReader();

    /**
     * @brief 读取STL文件并转换为点云（使用混合采样）
     * @param file_path STL/PLY文件路径
     * @param config 采样配置（可选）
     * @return 点云智能指针，读取失败返回nullptr
     */
    PointCloudXYZPtr read(const std::string& file_path, const SamplingConfig& config = SamplingConfig{});

    /**
     * @brief 读取STL文件并指定目标采样点数
     * @param file_path STL/PLY文件路径
     * @param target_points 期望的目标点数
     * @param config 其他采样配置（可选）
     * @return 点云智能指针
     */
    PointCloudXYZPtr readWithTargetPoints(const std::string& file_path, int target_points, const SamplingConfig& config = SamplingConfig{});

    /**
     * @brief 检查文件是否存在
     * @param file_path 文件路径
     * @return 是否存在
     */
    static bool fileExists(const std::string& file_path);

    /**
     * @brief 获取文件扩展名
     * @param file_path 文件路径
     * @return 扩展名（小写，不含点）
     */
    static std::string getExtension(const std::string& file_path);

    /**
     * @brief 获取上一个错误的描述
     * @return 错误信息
     */
    const std::string& getLastError() const;

private:
    std::string last_error_;

    // 内部实现：读取并混合采样
    PointCloudXYZPtr readWithSamplingImpl(const std::string& file_path, const SamplingConfig& config);
};

}  // namespace point_cloud
}  // namespace sim_data_gen
