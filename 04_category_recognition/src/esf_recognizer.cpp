/**
 * @file esf_recognizer.cpp
 * @brief ESF特征识别器实现
 */

#include "D:/vsxiangmu/gongjianfenjian/04_category_recognition/include/esf_recognizer.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>

#include <pcl/features/esf.h>

// 包含FAISS头文件
#include <faiss/Index.h>
#include <faiss/IndexIVFFlat.h>
#include <faiss/index_io.h>

namespace esf_database {
namespace recognition {

// FAISS索引实现结构
struct ESFRecognizer::FaissIndexImpl {
    faiss::Index* index = nullptr;
    size_t dimension = 0;

    ~FaissIndexImpl() {
        delete index;
    }
};

ESFRecognizer::ESFRecognizer() 
    : faiss_impl_(std::make_unique<FaissIndexImpl>()) {
}

ESFRecognizer::~ESFRecognizer() = default;

bool ESFRecognizer::loadIndex(const std::string& index_path, 
                              const std::string& ids_path) {
    try {
        // 加载FAISS索引
        faiss_impl_->index = faiss::read_index(index_path.c_str());
        faiss_impl_->dimension = faiss_impl_->index->d;

        // 加载ID映射
        if (!loadIdMapping(ids_path)) {
            std::cerr << "[ESFRecognizer] 加载ID映射失败" << std::endl;
            return false;
        }

        is_index_loaded_ = true;
        std::cout << "[ESFRecognizer] 索引加载成功: " 
                  << faiss_impl_->index->ntotal << " 个向量" << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[ESFRecognizer] 加载索引异常: " << e.what() << std::endl;
        return false;
    }
}

bool ESFRecognizer::loadIdMapping(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[ESFRecognizer] 无法打开ID映射文件: " << path << std::endl;
        return false;
    }

    id_mapping_.clear();
    std::string line;

    // 跳过第一行（CSV头）
    if (!std::getline(file, line)) {
        std::cerr << "[ESFRecognizer] ID映射文件为空" << std::endl;
        return false;
    }

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> values;

        while (std::getline(ss, token, ',')) {
            values.push_back(token);
        }

        // 支持两种格式:
        // 格式1 (字符串): vector_id,object_id,pose_id,filename
        // 格式2 (数字):   vector_id,category_id,pose_id,filename
        if (values.size() >= 3) {
            IdMappingItem item;
            item.category_id = values[1];  // object_id 或 category_id
            item.pose_id = values[2];
            id_mapping_.push_back(item);
        }
    }

    file.close();
    std::cout << "[ESFRecognizer] 加载ID映射: " << id_mapping_.size() << " 条记录" << std::endl;
    return true;
}

std::optional<std::vector<float>> ESFRecognizer::extractESF(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    
    if (!cloud || cloud->empty()) {
        std::cerr << "[ESFRecognizer] 输入点云为空" << std::endl;
        return std::nullopt;
    }

    // 检查点云大小
    if (cloud->size() < 1024) {
        std::cerr << "[ESFRecognizer] 点云点数太少: " << cloud->size() 
                  << ", 建议至少1024点" << std::endl;
    }

    try {
        // ESF特征估计器
        pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
        esf.setInputCloud(cloud);

        pcl::PointCloud<pcl::ESFSignature640>::Ptr esf_features(
            new pcl::PointCloud<pcl::ESFSignature640>());
        esf.compute(*esf_features);

        if (esf_features->empty()) {
            std::cerr << "[ESFRecognizer] ESF特征提取失败" << std::endl;
            return std::nullopt;
        }

        // 提取特征值 (640维)
        std::vector<float> feature(640);
        for (size_t i = 0; i < 640; ++i) {
            feature[i] = esf_features->points[0].histogram[i];
        }

        return feature;
    }
    catch (const std::exception& e) {
        std::cerr << "[ESFRecognizer] ESF特征提取异常: " << e.what() << std::endl;
        return std::nullopt;
    }
}

float ESFRecognizer::computeConfidence(float dist1, float dist2) const {
    // 使用L2距离，距离越小相似度越高
    // 转换为置信度: 1.0 / (1.0 + dist1/dist2) 或直接使用比值
    if (dist2 < 1e-6f) {
        return dist1 < 1e-6f ? 1.0f : 0.0f;
    }
    return dist2 / dist1; // Top-2 / Top-1，距离比值越大越确定
}

RecognitionResult ESFRecognizer::recognize(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    float distance_threshold) {

    auto topk_results = recognizeTopK(cloud, 5, distance_threshold);
    if (topk_results.empty()) {
        return RecognitionResult{};
    }
    return topk_results[0];
}

std::vector<RecognitionResult> ESFRecognizer::recognizeTopK(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    int k,
    float distance_threshold) {

    RecognizeOptions opts;
    opts.top_k = k;
    opts.distance_threshold = distance_threshold;
    return recognizeTopK(cloud, opts);
}

std::vector<RecognitionResult> ESFRecognizer::recognizeTopK(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const RecognizeOptions& opts) {

    std::vector<RecognitionResult> results;
    const int k = opts.top_k;
    const float threshold = opts.distance_threshold;
    const bool enable_filter = (threshold > 0.0f);

    if (!is_index_loaded_) {
        std::cerr << "[ESFRecognizer] 索引未加载" << std::endl;
        return results;
    }

    if (k <= 0) {
        std::cerr << "[ESFRecognizer] k 必须为正数, 实际=" << k << std::endl;
        return results;
    }

    // 提取ESF特征
    auto feature_opt = extractESF(cloud);
    if (!feature_opt) {
        return results;
    }
    const auto& feature = *feature_opt;

    // FAISS检索
    std::vector<float> distances(k);
    std::vector<faiss::idx_t> indices(k);

    faiss_impl_->index->search(1, feature.data(), k, distances.data(), indices.data());

    // 记录原始检索数量（用于过滤统计）
    int raw_count = 0;
    for (int i = 0; i < k; ++i) {
        if (indices[i] >= 0) ++raw_count;
    }

    // 第一次过滤：去掉无效索引
    struct Candidate {
        int64_t vec_id;
        float distance;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(k);
    for (int i = 0; i < k; ++i) {
        if (indices[i] < 0) continue;
        candidates.push_back({indices[i], distances[i]});
    }

    // 第二次过滤：距离阈值
    int filtered_count = static_cast<int>(candidates.size());
    if (enable_filter) {
        std::vector<Candidate> kept;
        kept.reserve(candidates.size());
        for (const auto& c : candidates) {
            if (c.distance <= threshold) {
                kept.push_back(c);
            }
        }
        candidates.swap(kept);
        filtered_count = static_cast<int>(candidates.size());

        if (opts.verbose) {
            std::cout << "[ESFRecognizer] 阈值过滤: raw=" << raw_count
                      << ", dist<= " << threshold
                      << ", kept=" << filtered_count << std::endl;
        }
    }

    // 构建结果
    results.reserve(candidates.size());
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& c = candidates[i];

        RecognitionResult result;
        result.topk_ids.push_back(static_cast<int>(c.vec_id));
        result.topk_scores.push_back(c.distance);

        // ID映射
        size_t vec_id = static_cast<size_t>(c.vec_id);
        if (vec_id < id_mapping_.size()) {
            result.category_id = id_mapping_[vec_id].category_id;
            result.pose_id = id_mapping_[vec_id].pose_id;
        } else {
            result.category_id = std::to_string(vec_id);
            result.pose_id = "unknown";
        }

        // 计算置信度（基于过滤后 Top-1/Top-2 的距离比值）
        if (i == 0 && candidates.size() > 1) {
            result.confidence = computeConfidence(candidates[0].distance,
                                                  candidates[1].distance);
        } else if (i > 0) {
            result.confidence = computeConfidence(candidates[i].distance,
                                                  candidates[0].distance);
        } else {
            result.confidence = 1.0f / (1.0f + candidates[0].distance);
        }

        result.feature_type = "ESF";
        result.raw_topk_count = raw_count;
        result.filtered_topk_count = filtered_count;
        result.applied_threshold = enable_filter ? threshold : -1.0f;
        results.push_back(result);
    }

    return results;
}

} // namespace recognition
} // namespace esf_database
