/**
 * @file category_recognition.h
 * @brief 类别判定模块公共类型定义
 */

#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>

namespace esf_database {
namespace recognition {

// 识别结果结构
struct RecognitionResult {
    std::string category_id;          // 类别ID，支持字符串 (如 "one") 或数字 (如 "0")
    std::string category_name;        // 类别名称
    float confidence = 0.0f;          // 置信度（Top-1/Top-2比值）
    std::string feature_type;         // 特征类型："ESF" / "CVFH" / "FUSED"

    // 调试信息
    std::vector<int> topk_ids;        // Top-K 向量ID列表
    std::vector<float> topk_scores;   // Top-K 距离列表
    std::string pose_id;             // 姿态ID

    bool isValid() const {
        return !category_id.empty() && confidence > 0.0f;
    }
};

// 搜索结果（FAISS检索结果）
struct SearchResult {
    int64_t id;          // 向量ID
    float distance;      // 距离（越小越相似）
};

// ID映射信息
struct IdMapping {
    int vector_id;           // FAISS向量ID
    std::string object_id;   // 工件ID（如 "one"）
    std::string pose_id;     // 姿态ID（如 "rx0_ry0_rz0"）
    std::string filename;     // 对应的特征文件名
};

// 识别配置
struct RecognitionConfig {
    float confidence_threshold = 1.5f;
    int top_k = 5;

    std::string esf_index_path;
    std::string esf_ids_path;
    std::string cvfh_index_path;
    std::string cvfh_ids_path;

    float voxel_leaf_size = 0.005f;
    int mean_k = 50;
    float std_dev_mul = 1.0f;
};

} // namespace recognition
} // namespace esf_database
