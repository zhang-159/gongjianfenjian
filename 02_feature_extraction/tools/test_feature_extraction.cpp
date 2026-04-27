/**
 * @file test_feature_extraction.cpp
 * @brief ESF特征提取模块测试程序
 * 
 * 用法:
 *   test_feature_extraction.exe                 # 处理第一个文件
 *   test_feature_extraction.exe --all           # 处理所有文件
 *   test_feature_extraction.exe --file xxx.pcd  # 处理指定文件
 * 
 * 输入: D:/vsxiangmu/gongjianfenjian/data/pointclouds/raw/*.pcd
 * 输出: D:/vsxiangmu/gongjianfenjian/data/esf_features/train/*.esf
 */

#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <string>
#include <pcl/io/pcd_io.h>

#include "../src/features/esf_estimator.h"
#include "../src/utils/types.h"

namespace fs = std::filesystem;

using namespace feature_extraction;

// 路径配置 - 共享数据目录
const fs::path kDataDir = "D:/vsxiangmu/gongjianfenjian/data";
const fs::path kInputPointCloudDir = kDataDir / "pointclouds/raw";
const fs::path kOutputEsfDir = kDataDir / "esf_features/train";

// 保存ESF特征到二进制文件
bool saveEsfFeature(const FeatureVector& feature, const fs::path& output_path) {
    try {
        fs::create_directories(output_path.parent_path());
        
        std::ofstream ofs(output_path, std::ios::binary);
        if (!ofs.is_open()) {
            std::cerr << "[ERROR] 无法创建文件: " << output_path << std::endl;
            return false;
        }
        
        size_t dim = feature.size();
        ofs.write(reinterpret_cast<const char*>(&dim), sizeof(size_t));
        ofs.write(reinterpret_cast<const char*>(feature.data()), 
                   dim * sizeof(float));
        ofs.close();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 保存失败: " << e.what() << std::endl;
        return false;
    }
}

// 处理单个文件
bool processSingleFile(const fs::path& pcd_file, ESFEstimator& estimator, 
                       int& success_count, int& fail_count) {
    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "处理文件: " << pcd_file.filename() << std::endl;
    
    try {
        // 读取点云
        PointCloudXYZPtr cloud(new PointCloudXYZ);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file.string(), *cloud) < 0) {
            std::cerr << "[ERROR] 加载点云失败" << std::endl;
            fail_count++;
            return false;
        }
        
        if (cloud->empty()) {
            std::cerr << "[WARN] 点云为空，跳过" << std::endl;
            fail_count++;
            return false;
        }
        
        std::cout << "  点数: " << cloud->size() << std::endl;
        
        // 提取ESF特征
        auto start_time = std::chrono::high_resolution_clock::now();
        FeatureVector esf_feature = estimator.extract(cloud);
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time - start_time);
        
        std::cout << "  特征维度: " << esf_feature.size() << ", 耗时: " 
                  << duration.count() << " ms" << std::endl;
        
        // 保存ESF特征
        std::string output_name = pcd_file.stem().string() + ".esf";
        fs::path output_path = kOutputEsfDir / output_name;
        
        if (saveEsfFeature(esf_feature, output_path)) {
            std::cout << "  [OK] 已保存: " << output_name << std::endl;
            success_count++;
            return true;
        } else {
            fail_count++;
            return false;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 处理失败: " << e.what() << std::endl;
        fail_count++;
        return false;
    }
}

// 打印帮助信息
void printHelp(const char* prog_name) {
    std::cout << "\n用法:" << std::endl;
    std::cout << "  " << prog_name << "                 # 处理第一个文件" << std::endl;
    std::cout << "  " << prog_name << " --all           # 处理所有文件" << std::endl;
    std::cout << "  " << prog_name << " --file <name>   # 处理指定文件" << std::endl;
    std::cout << "  " << prog_name << " --help          # 显示帮助" << std::endl;
    std::cout << "\n路径配置:" << std::endl;
    std::cout << "  输入: " << kInputPointCloudDir << std::endl;
    std::cout << "  输出: " << kOutputEsfDir << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "ESF特征提取模块测试" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 解析命令行参数
    enum class Mode { First, All, Single };
    Mode mode = Mode::First;
    std::string specific_file;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp(argv[0]);
            return 0;
        } else if (arg == "--all") {
            mode = Mode::All;
        } else if (arg == "--file" && i + 1 < argc) {
            mode = Mode::Single;
            specific_file = argv[++i];
        } else {
            std::cerr << "[ERROR] 未知参数: " << arg << std::endl;
            printHelp(argv[0]);
            return 1;
        }
    }
    
    std::cout << "\n路径配置:" << std::endl;
    std::cout << "  输入目录: " << kInputPointCloudDir << std::endl;
    std::cout << "  输出目录: " << kOutputEsfDir << std::endl;
    
    // 创建ESF特征提取器
    ESFEstimator estimator;
    std::cout << "\n[OK] ESFEstimator 创建成功" << std::endl;
    std::cout << "特征描述: " << ESFEstimator::getDescription() << std::endl;
    std::cout << "特征维度: " << ESFEstimator::dimension() << std::endl;
    
    // 确保输出目录存在
    fs::create_directories(kOutputEsfDir);
    
    try {
        // 检查输入目录
        if (!fs::exists(kInputPointCloudDir)) {
            std::cerr << "\n[ERROR] 输入目录不存在: " << kInputPointCloudDir << std::endl;
            return 1;
        }
        
        // 收集所有PCD文件
        std::vector<fs::path> pcd_files;
        for (const auto& entry : fs::directory_iterator(kInputPointCloudDir)) {
            if (entry.is_regular_file() && 
                entry.path().extension() == ".pcd") {
                pcd_files.push_back(entry.path());
            }
        }
        
        if (pcd_files.empty()) {
            std::cerr << "\n[ERROR] 输入目录中没有PCD文件" << std::endl;
            return 1;
        }
        
        // 根据模式处理
        int success_count = 0;
        int fail_count = 0;
        
        if (mode == Mode::First) {
            // 模式1: 只处理第一个文件
            std::cout << "\n[模式1] 处理第一个文件" << std::endl;
            processSingleFile(pcd_files[0], estimator, success_count, fail_count);
            
        } else if (mode == Mode::All) {
            // 模式2: 处理所有文件
            std::cout << "\n[模式2] 批量处理所有 " << pcd_files.size() << " 个文件" << std::endl;
            auto total_start = std::chrono::high_resolution_clock::now();
            
            for (size_t i = 0; i < pcd_files.size(); ++i) {
                std::cout << "\n[" << (i + 1) << "/" << pcd_files.size() << "]";
                processSingleFile(pcd_files[i], estimator, success_count, fail_count);
            }
            
            auto total_end = std::chrono::high_resolution_clock::now();
            auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                total_end - total_start);
            
            std::cout << "\n========================================" << std::endl;
            std::cout << "批量处理完成!" << std::endl;
            std::cout << "  成功: " << success_count << std::endl;
            std::cout << "  失败: " << fail_count << std::endl;
            std::cout << "  总耗时: " << total_duration.count() << " ms" << std::endl;
            
        } else if (mode == Mode::Single) {
            // 模式3: 处理指定文件
            std::cout << "\n[模式3] 处理指定文件: " << specific_file << std::endl;
            
            fs::path target_file = kInputPointCloudDir / specific_file;
            if (!fs::exists(target_file)) {
                target_file = kInputPointCloudDir / (specific_file + ".pcd");
            }
            
            if (!fs::exists(target_file)) {
                std::cerr << "[ERROR] 文件不存在: " << target_file << std::endl;
                return 1;
            }
            
            processSingleFile(target_file, estimator, success_count, fail_count);
        }
        
        std::cout << "\n========================================" << std::endl;
        std::cout << "ESF特征提取测试完成!" << std::endl;
        std::cout << "========================================" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] 测试失败: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
