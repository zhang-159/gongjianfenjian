/**
 * @file cvfh_recognizer.h
 * @brief OUR-CVFH 特征识别器 - 封装法线估计、OUR-CVFH 特征提取和 FAISS 检索
 *
 * OUR-CVFH 是 CVFH 的扩展版本，对没有稳定主导方向的点也能给出特征，
 * 在 PCL 中通过 pcl::OURCVFHEstimation 实现，输出 VFHSignature308 (308 维)。
 */

#pragma once

#include <memory>
#include <string>
#include <vector>
#include <optional>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "D:/vsxiangmu/gongjianfenjian/04_category_recognition/include/category_recognition.h"

namespace esf_database {
namespace recognition {

// 法线云类型别名
using NormalCloud = pcl::PointCloud<pcl::Normal>;
using NormalCloudPtr = pcl::PointCloud<pcl::Normal>::Ptr;

class CVFHRecognizer {
public:
    CVFHRecognizer();
    ~CVFHRecognizer();

    // OUR-CVFH 特征维度（PCL 中对应 VFHSignature308）
    static constexpr size_t kCVFHDimension = 308;

    // 特征提取阶段的常用参数（PCL 1.14.0 OURCVFH 实际 setter）
    struct ExtractParams {
        float normal_radius = 0.02f;          // 法线估计半径（米）
        int normal_min_neighbors = 5;         // 法线估计最小邻居数（保留兼容字段，PCL 默认 0）
        float cluster_tolerance = 0.05f;      // 对应 setClusterTolerance：聚类容差
        std::size_t min_points = 5;           // 对应 setMinPoints：聚类前最小点数（PCL 1.14 替代旧 setMinClusterSize）
        // 注：PCL 1.14 取消了 setMaxClusterSize，原 cluster_max_nodes 字段已删除
        float min_axis_value = 0.05f;         // 对应 setMinAxisValue：消歧轴最小值（替代旧 setAxisVarianceThreshold）
        float axis_ratio = 0.96f;             // 对应 setAxisRatio：主轴比阈值（替代旧 setMinAxisAngle）
        bool normalize_bins = true;           // 对应 setNormalizeBins：VFH bin 归一化（替代旧 setNormalizeAxes）
        float refine_clusters = 1.0f;         // 对应 setRefineClusters：聚类精化系数
    };

    /**
     * @brief 加载 FAISS 索引和 ID 映射
     */
    bool loadIndex(const std::string& index_path, const std::string& ids_path);

    /**
     * @brief 从原始点云提取 OUR-CVFH 特征（内部自动估计法线）
     * @param cloud 输入点云
     * @param params 提取参数
     * @return 308 维 OUR-CVFH 特征（多个聚类签名拼接后的结果）
     */
    std::optional<std::vector<float>> extractCVFH(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const ExtractParams& params = ExtractParams{});

    /**
     * @brief 仅估计点云法线（方便复用、调试或可视化）
     */
    NormalCloudPtr estimateNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        float normal_radius = 0.02f);

    /**
     * @brief 直接对已计算好的法线和点云提取 OUR-CVFH 特征
     */
    std::optional<std::vector<float>> extractCVFHWithNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const NormalCloudPtr& normals,
        const ExtractParams& params = ExtractParams{});

    /**
     * @brief 执行识别（带可配置阈值过滤）
     */
    RecognitionResult recognize(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        float distance_threshold = -1.0f);

    /**
     * @brief Top-K 检索（带可配置阈值过滤）
     */
    std::vector<RecognitionResult> recognizeTopK(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int k = 5,
        float distance_threshold = -1.0f);

    /**
     * @brief 使用结构化参数执行 Top-K 检索
     */
    std::vector<RecognitionResult> recognizeTopK(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const RecognizeOptions& opts);

    /** 索引是否已加载 */
    bool isIndexLoaded() const { return is_index_loaded_; }

private:
    bool loadIdMapping(const std::string& path);

    // 计算置信度：Top-2 距离 / Top-1 距离，越大越确定
    float computeConfidence(float dist1, float dist2) const;

    // 对 OUR-CVFH 多签名输出做拼接，返回拼接后的特征
    std::vector<float> flattenSignatureCloud(
        const pcl::PointCloud<pcl::VFHSignature308>::Ptr& signatures) const;

    // ID 映射项
    struct IdMappingItem {
        std::string category_id;
        std::string pose_id;
    };

    struct FaissIndexImpl;
    std::unique_ptr<FaissIndexImpl> faiss_impl_;
    std::vector<IdMappingItem> id_mapping_;
    bool is_index_loaded_ = false;
};

} // namespace recognition
} // namespace esf_database
