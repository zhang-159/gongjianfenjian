/**
 * @file main.cpp
 * @brief 04类别判定模块测试程序 - ESF + CVFH 双重过滤演示
 *
 * 测试流程：
 * 1. 加载 ESF 索引和 CVFH 索引
 * 2. 加载测试点云
 * 3. ESF Top-K + 距离阈值过滤
 * 4. CVFH Top-K + 距离阈值过滤
 * 5. 对比结果（打印过滤前/后数量及最终类别）
 */

#include <iostream>
#include <memory>
#include <string>
#include <iomanip>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "D:/vsxiangmu/gongjianfenjian/04_category_recognition/include/category_recognition.h"
#include "D:/vsxiangmu/gongjianfenjian/04_category_recognition/include/esf_recognizer.h"
#include "D:/vsxiangmu/gongjianfenjian/04_category_recognition/include/cvfh_recognizer.h"

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr;

namespace rec = esf_database::recognition;

// 打印一行 Top-K 结果
void printTopK(const std::string& title,
               const std::vector<rec::RecognitionResult>& results) {
    std::cout << "\n--- " << title << " ---" << std::endl;
    if (results.empty()) {
        std::cout << "  (空结果，可能被阈值全部过滤掉)" << std::endl;
        return;
    }
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        std::cout << std::fixed << std::setprecision(4)
                  << "  Top-" << (i + 1)
                  << ": ID=" << r.category_id
                  << ", pose=" << r.pose_id
                  << ", dist=" << r.topk_scores[0]
                  << ", conf=" << r.confidence;
        if (r.applied_threshold > 0.0f) {
            std::cout << ", raw=" << r.raw_topk_count
                      << ", kept=" << r.filtered_topk_count
                      << ", thr=" << r.applied_threshold;
        }
        std::cout << std::endl;
    }
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "04 类别判定模块 - ESF + CVFH 双重过滤测试" << std::endl;
    std::cout << "========================================" << std::endl;

    // 路径配置
    const std::string kProjectRoot = "D:/vsxiangmu/gongjianfenjian";
    const std::string kEsfIndexPath = kProjectRoot + "/data/fingerprints/esf_index.bin";
    const std::string kEsfIdsPath   = kProjectRoot + "/data/fingerprints/esf_ids.txt";
    const std::string kCvfhIndexPath = kProjectRoot + "/data/fingerprints/cvfh_index.bin";
    const std::string kCvfhIdsPath   = kProjectRoot + "/data/fingerprints/cvfh_ids.txt";
    const std::string kTestCloudPath = kProjectRoot + "/data/pointclouds/raw/one_rx0_ry0_rz20.pcd";

    // 阈值参数（与 recognition_config.yaml 默认值一致）
    const float kEsfDistanceThreshold  = 0.45f;   // ESF L2 阈值（按需调整）
    const float kCvfhDistanceThreshold = 1.20f;   // CVFH L2 阈值（按需调整）
    const int   kTopK = 5;

    // 1. 创建识别器
    auto esf_recognizer   = std::make_unique<rec::ESFRecognizer>();
    auto cvfh_recognizer  = std::make_unique<rec::CVFHRecognizer>();

    // 2. 加载 ESF 索引
    std::cout << "\n[2.A] 加载 ESF FAISS 索引..." << std::endl;
    if (!esf_recognizer->loadIndex(kEsfIndexPath, kEsfIdsPath)) {
        std::cerr << "ESF 索引加载失败，请先运行 03_database_build 生成索引" << std::endl;
        return -1;
    }

    // 3. 加载 CVFH 索引
    std::cout << "\n[2.B] 加载 CVFH FAISS 索引..." << std::endl;
    if (!cvfh_recognizer->loadIndex(kCvfhIndexPath, kCvfhIdsPath)) {
        std::cerr << "CVFH 索引加载失败，请确认 03 已生成 cvfh_index.bin" << std::endl;
        return -1;
    }

    // 4. 加载测试点云
    std::cout << "\n[3] 加载测试点云: " << kTestCloudPath << std::endl;
    PointCloudXYZ test_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(kTestCloudPath, *test_cloud) < 0) {
        std::cerr << "错误: 点云加载失败!" << std::endl;
        return -1;
    }
    std::cout << "    点数: " << test_cloud->size() << std::endl;

    // 5. ESF Top-K（带阈值过滤）
    std::cout << "\n[4] ESF Top-" << kTopK
              << " (threshold=" << kEsfDistanceThreshold << ")..." << std::endl;
    rec::RecognizeOptions esf_opts;
    esf_opts.top_k = kTopK;
    esf_opts.distance_threshold = kEsfDistanceThreshold;
    auto esf_results = esf_recognizer->recognizeTopK(test_cloud, esf_opts);
    printTopK("ESF Top-K (filtered)", esf_results);

    // 6. CVFH Top-K（带阈值过滤）
    std::cout << "\n[5] CVFH Top-" << kTopK
              << " (threshold=" << kCvfhDistanceThreshold << ")..." << std::endl;
    rec::RecognizeOptions cvfh_opts;
    cvfh_opts.top_k = kTopK;
    cvfh_opts.distance_threshold = kCvfhDistanceThreshold;
    auto cvfh_results = cvfh_recognizer->recognizeTopK(test_cloud, cvfh_opts);
    printTopK("CVFH Top-K (filtered)", cvfh_results);

    // 7. 对比 Top-1
    std::cout << "\n========================================" << std::endl;
    std::cout << "Top-1 类别对比 (过滤后):" << std::endl;
    std::cout << "========================================" << std::endl;
    auto printTop1 = [](const std::string& tag,
                        const std::vector<rec::RecognitionResult>& r) {
        if (r.empty()) {
            std::cout << "  " << tag << ": (空结果 - 阈值过严)" << std::endl;
        } else {
            std::cout << "  " << tag << ": class=" << r[0].category_id
                      << ", pose=" << r[0].pose_id
                      << ", conf=" << std::fixed << std::setprecision(3)
                      << r[0].confidence << std::endl;
        }
    };
    printTop1("ESF ", esf_results);
    printTop1("CVFH", cvfh_results);

    std::cout << "\n测试完成!" << std::endl;
    return 0;
}
