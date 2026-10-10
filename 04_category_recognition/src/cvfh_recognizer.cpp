/**
 * @file cvfh_recognizer.cpp
 * @brief OUR-CVFH 特征识别器实现
 */

#include "D:/vsxiangmu/gongjianfenjian/04_category_recognition/include/cvfh_recognizer.h"

#include <fstream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <iostream>

#include <pcl/features/normal_3d.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <faiss/Index.h>
#include <faiss/index_io.h>

namespace esf_database {
namespace recognition {

// FAISS 索引内部实现
struct CVFHRecognizer::FaissIndexImpl {
    faiss::Index* index = nullptr;
    size_t dimension = 0;

    ~FaissIndexImpl() {
        delete index;
    }
};

CVFHRecognizer::CVFHRecognizer()
    : faiss_impl_(std::make_unique<FaissIndexImpl>()) {
}

CVFHRecognizer::~CVFHRecognizer() = default;

bool CVFHRecognizer::loadIndex(const std::string& index_path,
                                const std::string& ids_path) {
    try {
        faiss_impl_->index = faiss::read_index(index_path.c_str());
        faiss_impl_->dimension = faiss_impl_->index->d;

        if (faiss_impl_->dimension != kCVFHDimension) {
            std::cerr << "[CVFHRecognizer] 警告：索引维度=" << faiss_impl_->dimension
                      << " 与期望值 " << kCVFHDimension << " 不一致" << std::endl;
        }

        if (!loadIdMapping(ids_path)) {
            std::cerr << "[CVFHRecognizer] 加载ID映射失败" << std::endl;
            return false;
        }

        is_index_loaded_ = true;
        std::cout << "[CVFHRecognizer] 索引加载成功: "
                  << faiss_impl_->index->ntotal << " 个向量, 维度="
                  << faiss_impl_->dimension << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "[CVFHRecognizer] 加载索引异常: " << e.what() << std::endl;
        return false;
    }
}

bool CVFHRecognizer::loadIdMapping(const std::string& path) {
    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[CVFHRecognizer] 无法打开ID映射文件: " << path << std::endl;
        return false;
    }

    id_mapping_.clear();
    std::string line;

    // 跳过第一行表头
    if (!std::getline(file, line)) {
        std::cerr << "[CVFHRecognizer] ID映射文件为空" << std::endl;
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

        if (values.size() >= 3) {
            IdMappingItem item;
            item.category_id = values[1];
            item.pose_id = values[2];
            id_mapping_.push_back(item);
        }
    }

    file.close();
    std::cout << "[CVFHRecognizer] 加载ID映射: " << id_mapping_.size() << " 条记录" << std::endl;
    return true;
}

// ============================== 法线估计 ==============================

NormalCloudPtr CVFHRecognizer::estimateNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    float normal_radius) {

    if (!cloud || cloud->empty()) {
        std::cerr << "[CVFHRecognizer] 输入点云为空" << std::endl;
        return nullptr;
    }

    try {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>());
        ne.setSearchMethod(tree);

        if (normal_radius > 0.0f) {
            ne.setRadiusSearch(normal_radius);
        } else {
            ne.setKSearch(20);
        }

        NormalCloudPtr normals(new NormalCloud());
        ne.compute(*normals);

        if (normals->empty()) {
            std::cerr << "[CVFHRecognizer] 法线估计失败" << std::endl;
            return nullptr;
        }
        return normals;
    }
    catch (const std::exception& e) {
        std::cerr << "[CVFHRecognizer] 法线估计异常: " << e.what() << std::endl;
        return nullptr;
    }
}

// ============================== 特征提取 ==============================

std::vector<float> CVFHRecognizer::flattenSignatureCloud(
    const pcl::PointCloud<pcl::VFHSignature308>::Ptr& signatures) const {
    if (!signatures || signatures->empty()) {
        return {};
    }
    // 取 cluster 点数最大的那个签名（PCL 通过 signatures.points.size() 标识）
    // 这里简单取第一个，索引通常按 cluster 大小排序
    // 实际上 OUR-CVFH 每个 signature 的 histogram 长度均为 308
    const auto& sig = signatures->points[0];
    std::vector<float> feature(kCVFHDimension);
    for (size_t i = 0; i < kCVFHDimension; ++i) {
        feature[i] = sig.histogram[i];
    }
    return feature;
}

std::optional<std::vector<float>> CVFHRecognizer::extractCVFHWithNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const NormalCloudPtr& normals,
    const ExtractParams& params) {

    if (!cloud || cloud->empty() || !normals || normals->empty()) {
        std::cerr << "[CVFHRecognizer] 输入点云或法线为空" << std::endl;
        return std::nullopt;
    }

    try {
        pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> ourcvfh;
        ourcvfh.setInputCloud(cloud);
        ourcvfh.setInputNormals(normals);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
            new pcl::search::KdTree<pcl::PointXYZ>());
        ourcvfh.setSearchMethod(tree);

        ourcvfh.setClusterTolerance(params.cluster_tolerance);
        ourcvfh.setMinPoints(params.min_points);
        ourcvfh.setMinAxisValue(params.min_axis_value);
        ourcvfh.setAxisRatio(params.axis_ratio);
        ourcvfh.setNormalizeBins(params.normalize_bins);
        ourcvfh.setRefineClusters(params.refine_clusters);

        pcl::PointCloud<pcl::VFHSignature308>::Ptr signatures(
            new pcl::PointCloud<pcl::VFHSignature308>());
        ourcvfh.compute(*signatures);

        if (signatures->empty()) {
            std::cerr << "[CVFHRecognizer] OUR-CVFH特征提取失败(无签名)" << std::endl;
            return std::nullopt;
        }

        auto feature = flattenSignatureCloud(signatures);
        if (feature.empty()) {
            std::cerr << "[CVFHRecognizer] OUR-CVFH特征维度异常" << std::endl;
            return std::nullopt;
        }

        std::cout << "[CVFHRecognizer] OUR-CVFH提取: " << signatures->size()
                  << " 个签名, 选用第1个 (维度=" << feature.size() << ")" << std::endl;
        return feature;
    }
    catch (const std::exception& e) {
        std::cerr << "[CVFHRecognizer] OUR-CVFH特征提取异常: " << e.what() << std::endl;
        return std::nullopt;
    }
}

std::optional<std::vector<float>> CVFHRecognizer::extractCVFH(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const ExtractParams& params) {

    auto normals = estimateNormals(cloud, params.normal_radius);
    if (!normals) {
        return std::nullopt;
    }
    return extractCVFHWithNormals(cloud, normals, params);
}

// ============================== 识别 ==============================

float CVFHRecognizer::computeConfidence(float dist1, float dist2) const {
    if (dist2 < 1e-6f) {
        return dist1 < 1e-6f ? 1.0f : 0.0f;
    }
    return dist2 / dist1; // Top-2 / Top-1
}

RecognitionResult CVFHRecognizer::recognize(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    float distance_threshold) {
    auto topk_results = recognizeTopK(cloud, 5, distance_threshold);
    if (topk_results.empty()) {
        return RecognitionResult{};
    }
    return topk_results[0];
}

std::vector<RecognitionResult> CVFHRecognizer::recognizeTopK(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    int k,
    float distance_threshold) {
    RecognizeOptions opts;
    opts.top_k = k;
    opts.distance_threshold = distance_threshold;
    return recognizeTopK(cloud, opts);
}

std::vector<RecognitionResult> CVFHRecognizer::recognizeTopK(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const RecognizeOptions& opts) {

    std::vector<RecognitionResult> results;
    const int k = opts.top_k;
    const float threshold = opts.distance_threshold;
    const bool enable_filter = (threshold > 0.0f);

    if (!is_index_loaded_) {
        std::cerr << "[CVFHRecognizer] 索引未加载" << std::endl;
        return results;
    }

    if (k <= 0) {
        std::cerr << "[CVFHRecognizer] k 必须为正数, 实际=" << k << std::endl;
        return results;
    }

    // 提取 OUR-CVFH 特征（自动先估计法线）
    auto feature_opt = extractCVFH(cloud);
    if (!feature_opt) {
        return results;
    }
    const auto& feature = *feature_opt;

    // FAISS 检索
    std::vector<float> distances(k);
    std::vector<faiss::idx_t> indices(k);

    faiss_impl_->index->search(1, feature.data(), k,
                                distances.data(), indices.data());

    int raw_count = 0;
    for (int i = 0; i < k; ++i) {
        if (indices[i] >= 0) ++raw_count;
    }

    // 过滤无效索引
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
            std::cout << "[CVFHRecognizer] 阈值过滤: raw=" << raw_count
                      << ", dist<= " << threshold
                      << ", kept=" << filtered_count << std::endl;
        }
    }

    results.reserve(candidates.size());
    for (size_t i = 0; i < candidates.size(); ++i) {
        const auto& c = candidates[i];

        RecognitionResult result;
        result.topk_ids.push_back(static_cast<int>(c.vec_id));
        result.topk_scores.push_back(c.distance);

        size_t vec_id = static_cast<size_t>(c.vec_id);
        if (vec_id < id_mapping_.size()) {
            result.category_id = id_mapping_[vec_id].category_id;
            result.pose_id = id_mapping_[vec_id].pose_id;
        } else {
            result.category_id = std::to_string(vec_id);
            result.pose_id = "unknown";
        }

        if (i == 0 && candidates.size() > 1) {
            result.confidence = computeConfidence(candidates[0].distance,
                                                  candidates[1].distance);
        } else if (i > 0) {
            result.confidence = computeConfidence(candidates[i].distance,
                                                  candidates[0].distance);
        } else {
            result.confidence = 1.0f / (1.0f + candidates[0].distance);
        }

        result.feature_type = "CVFH";
        result.raw_topk_count = raw_count;
        result.filtered_topk_count = filtered_count;
        result.applied_threshold = enable_filter ? threshold : -1.0f;
        results.push_back(result);
    }

    return results;
}

} // namespace recognition
} // namespace esf_database
