/**
 * @file esf_recognizer.h
 * @brief ESF特征识别器 - 封装ESF特征提取和FAISS检索
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

// ESF特征识别器类
class ESFRecognizer {
public:
    // 构造函数
    ESFRecognizer();

    // 析构函数
    ~ESFRecognizer();

    /**
     * @brief 加载FAISS索引和ID映射文件
     * @param index_path 索引文件路径 (.bin)
     * @param ids_path ID映射文件路径 (.txt)
     * @return 是否成功
     */
    bool loadIndex(const std::string& index_path, const std::string& ids_path);

    /**
     * @brief 提取ESF特征
     * @param cloud 输入点云
     * @return 640维ESF特征向量，失败返回空
     */
    std::optional<std::vector<float>> extractESF(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief 执行识别
     * @param cloud 输入点云
     * @return 识别结果
     */
    RecognitionResult recognize(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    /**
     * @brief 获取Top-K检索结果
     * @param cloud 输入点云
     * @param k 返回数量
     * @return Top-K结果
     */
    std::vector<RecognitionResult> recognizeTopK(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, int k = 5);

    /**
     * @brief 检查索引是否已加载
     */
    bool isIndexLoaded() const { return is_index_loaded_; }

    /**
     * @brief 获取ESF维度
     */
    static constexpr size_t kESFDimension = 640;

private:
    // 解析ID映射文件
    bool loadIdMapping(const std::string& path);

    // 计算置信度 (Top-1 / Top-2)
    float computeConfidence(float dist1, float dist2) const;

    // ID映射项: vector_id -> {category_id, pose_id}
    struct IdMappingItem {
        std::string category_id;  // 支持字符串ID (如 "one", "two")
        std::string pose_id;      // 姿态ID (如 "default", "rx0_ry0_rz0")
    };

    // FAISS索引实现
    struct FaissIndexImpl;
    std::unique_ptr<FaissIndexImpl> faiss_impl_;

    // ID映射: vector_id -> IdMappingItem
    std::vector<IdMappingItem> id_mapping_;

    // 索引加载标志
    bool is_index_loaded_ = false;
};

} // namespace recognition
} // namespace esf_database
