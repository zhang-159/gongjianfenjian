#pragma once

/**
 * @file types.h
 * @brief 特征提取模块的类型定义
 */

#include "../../../libs/common_types.h"
#include <pcl/features/esf.h>
#include <pcl/features/our_cvfh.h>
#include <vector>
#include <string>

namespace feature_extraction {

// 导入公共类型到本模块命名空间
using common_types::PointXYZ;
using common_types::PointCloudXYZ;
using common_types::PointCloudXYZPtr;
using common_types::PointCloudXYZConstPtr;
using common_types::NormalCloud;
using common_types::NormalCloudPtr;
using common_types::NormalCloudConstPtr;

// Normal 类型通过 common_types 提供
using common_types::Normal;

// =============================================
// ESF特征类型（640维）
// =============================================
using ESFFeature = pcl::ESFSignature640;
using ESFFeatureCloud = pcl::PointCloud<ESFFeature>;
using ESFFeatureCloudPtr = pcl::PointCloud<ESFFeature>::Ptr;

// =============================================
// CVFH特征类型（345维）
// =============================================
using CVFHFeature = pcl::VFHSignature308;
using CVFHFeatureCloud = pcl::PointCloud<CVFHFeature>;
using CVFHFeatureCloudPtr = pcl::PointCloud<CVFHFeature>::Ptr;

// =============================================
// 特征向量类型
// =============================================
using FeatureVector = std::vector<float>;

// =============================================
// ESF特征提取配置
// =============================================
struct ESFConfig {
    int sample_points = 2048;  // 采样点数
};

// =============================================
// CVFH特征提取配置
// =============================================
struct CVFHConfig {
    float normal_radius = 0.02f;  // 法线估计搜索半径(m)
    float cvfh_radius = 0.03f;    // CVFH估计搜索半径(m)
    float axis_ratio = 0.98f;     // 主轴比率
    int min_points = 2048;         // 最小点数要求
};

// =============================================
// 特征提取结果
// =============================================
struct FeatureResult {
    FeatureVector esf_features;     // ESF特征向量(640维)
    FeatureVector cvfh_features;    // CVFH特征向量(345维)
    int input_point_count = 0;      // 输入点云点数
    std::string source_file;        // 源文件路径
};

}  // namespace feature_extraction
