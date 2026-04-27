/**
 * @file main.cpp
 * @brief 04类别判定模块测试程序
 * 
 * 测试流程：
 * 1. 加载 FAISS 索引
 * 2. 加载测试点云
 * 3. 提取 ESF 特征
 * 4. 执行检索并输出结果
 */

#include <iostream>
#include <memory>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include "D:/vsxiangmu/gongjianfenjian/04_category_recognition/include/esf_recognizer.h"
#include "D:/vsxiangmu/gongjianfenjian/04_category_recognition/include/category_recognition.h"

using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>::Ptr;

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "04 类别判定模块 - ESF识别测试" << std::endl;
    std::cout << "========================================" << std::endl;

    // 路径配置
    const std::string kProjectRoot = "D:/vsxiangmu/gongjianfenjian";
    const std::string kIndexPath = kProjectRoot + "/data/fingerprints/esf_index.bin";
    const std::string kIdsPath = kProjectRoot + "/data/fingerprints/esf_ids.txt";
    const std::string kTestCloudPath = kProjectRoot + "/data/pointclouds/raw/one_rx0_ry0_rz0.pcd";

    // 1. 创建ESF识别器
    std::cout << "\n[1] 创建ESF识别器..." << std::endl;
    auto recognizer = std::make_unique<esf_database::recognition::ESFRecognizer>();

    // 2. 加载索引
    std::cout << "\n[2] 加载FAISS索引..." << std::endl;
    std::cout << "    索引文件: " << kIndexPath << std::endl;
    std::cout << "    ID映射: " << kIdsPath << std::endl;
    
    if (!recognizer->loadIndex(kIndexPath, kIdsPath)) {
        std::cerr << "错误: 索引加载失败!" << std::endl;
        std::cerr << "请先运行03_database_build生成索引" << std::endl;
        return -1;
    }
    std::cout << "    索引加载成功!" << std::endl;

    // 3. 加载测试点云
    std::cout << "\n[3] 加载测试点云..." << std::endl;
    std::cout << "    点云文件: " << kTestCloudPath << std::endl;
    
    PointCloudXYZ test_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(kTestCloudPath, *test_cloud) < 0) {
        std::cerr << "错误: 点云加载失败!" << std::endl;
        return -1;
    }
    std::cout << "    点云加载成功! 点数: " << test_cloud->size() << std::endl;

    // 4. 执行识别
    std::cout << "\n[4] 执行ESF识别..." << std::endl;
    auto results = recognizer->recognizeTopK(test_cloud, 5);

    // 5. 输出结果
    std::cout << "\n========================================" << std::endl;
    std::cout << "识别结果 (Top-5):" << std::endl;
    std::cout << "========================================" << std::endl;
    
    for (size_t i = 0; i < results.size(); ++i) {
        const auto& r = results[i];
        std::cout << "  Top-" << (i + 1) << ": "
                  << "类别ID=" << r.category_id
                  << ", 姿态ID=" << r.pose_id
                  << ", 距离=" << r.topk_scores[0]
                  << ", 置信度=" << r.confidence
                  << std::endl;
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "测试完成!" << std::endl;

    return 0;
}
