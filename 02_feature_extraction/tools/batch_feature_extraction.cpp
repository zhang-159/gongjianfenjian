/**
 * @file batch_feature_extraction.cpp
 * @brief 批量特征提取工具 - 同时提取ESF和CVFH特征
 * 
 * 用法:
 *   batch_feature_extraction.exe                 # 处理所有文件
 *   batch_feature_extraction.exe --esf-only      # 仅提取ESF
 *   batch_feature_extraction.exe --cvfh-only     # 仅提取CVFH
 *   batch_feature_extraction.exe --file xxx.pcd  # 处理指定文件
 *   batch_feature_extraction.exe --help           # 显示帮助
 * 
 * 输入: data/pointclouds/raw/*.pcd
 * 输出: 
 *   - data/esf_features/train/*.esf
 *   - data/cvfh_features/train/*.cvfh
 */

#include <iostream>
#include <fstream>
#include <chrono>
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>

#include "../src/features/esf_estimator.h"
#include "../src/features/cvfh_estimator.h"
#include "../src/utils/types.h"

namespace fs = std::filesystem;

using namespace feature_extraction;

// 路径配置 - 共享数据目录
const fs::path kDataDir = "D:/vsxiangmu/gongjianfenjian/data";
const fs::path kInputPointCloudDir = kDataDir / "pointclouds/raw";
const fs::path kOutputEsfDir = kDataDir / "esf_features/train";
const fs::path kOutputCvfhDir = kDataDir / "cvfh_features/train";

// ==================== 文件保存函数 ====================

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

// 保存CVFH特征到二进制文件
bool saveCvfhFeature(const FeatureVector& feature, const fs::path& output_path) {
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

// ==================== 处理函数 ====================

struct ProcessResult {
    bool esf_success = false;
    bool cvfh_success = false;
    int point_count = 0;
    int esf_time_ms = 0;
    int cvfh_time_ms = 0;
};

// 处理单个文件
ProcessResult processSingleFile(const fs::path& pcd_file,
                               ESFEstimator& esf_estimator,
                               CVFHEstimator& cvfh_estimator,
                               bool extract_esf,
                               bool extract_cvfh) {
    ProcessResult result;
    
    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "处理文件: " << pcd_file.filename() << std::endl;
    
    try {
        // 读取点云
        PointCloudXYZPtr cloud(new PointCloudXYZ);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file.string(), *cloud) < 0) {
            std::cerr << "[ERROR] 加载点云失败" << std::endl;
            return result;
        }
        
        if (cloud->empty()) {
            std::cerr << "[WARN] 点云为空，跳过" << std::endl;
            return result;
        }
        
        result.point_count = cloud->size();
        std::cout << "  点数: " << result.point_count << std::endl;
        
        // 提取ESF特征
        if (extract_esf) {
            auto start_time = std::chrono::high_resolution_clock::now();
            FeatureVector esf_feature = esf_estimator.extract(cloud);
            auto end_time = std::chrono::high_resolution_clock::now();
            result.esf_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time).count();
            
            std::cout << "  ESF特征: " << esf_feature.size() << "维, 耗时: " 
                      << result.esf_time_ms << " ms" << std::endl;
            
            // 保存ESF特征
            std::string output_name = pcd_file.stem().string() + ".esf";
            fs::path output_path = kOutputEsfDir / output_name;
            
            if (saveEsfFeature(esf_feature, output_path)) {
                std::cout << "  [OK] ESF已保存: " << output_name << std::endl;
                result.esf_success = true;
            }
        }
        
        // 提取CVFH特征
        if (extract_cvfh) {
            auto start_time = std::chrono::high_resolution_clock::now();
            FeatureVector cvfh_feature = cvfh_estimator.extract(cloud);
            auto end_time = std::chrono::high_resolution_clock::now();
            result.cvfh_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                end_time - start_time).count();
            
            std::cout << "  CVFH特征: " << cvfh_feature.size() << "维, 耗时: " 
                      << result.cvfh_time_ms << " ms" << std::endl;
            
            // 保存CVFH特征
            std::string output_name = pcd_file.stem().string() + ".cvfh";
            fs::path output_path = kOutputCvfhDir / output_name;
            
            if (saveCvfhFeature(cvfh_feature, output_path)) {
                std::cout << "  [OK] CVFH已保存: " << output_name << std::endl;
                result.cvfh_success = true;
            }
        }
        
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 处理失败: " << e.what() << std::endl;
    }
    
    return result;
}

// ==================== 帮助信息 ====================

void printHelp(const char* prog_name) {
    std::cout << "\n用法:" << std::endl;
    std::cout << "  " << prog_name << "                  # 处理所有文件（提取ESF+CVFH）" << std::endl;
    std::cout << "  " << prog_name << " --esf-only       # 仅提取ESF特征" << std::endl;
    std::cout << "  " << prog_name << " --cvfh-only      # 仅提取CVFH特征" << std::endl;
    std::cout << "  " << prog_name << " --file <name>    # 处理指定文件" << std::endl;
    std::cout << "  " << prog_name << " --help           # 显示帮助" << std::endl;
    std::cout << "\n路径配置:" << std::endl;
    std::cout << "  输入: " << kInputPointCloudDir << std::endl;
    std::cout << "  ESF输出: " << kOutputEsfDir << std::endl;
    std::cout << "  CVFH输出: " << kOutputCvfhDir << std::endl;
}

// ==================== 主函数 ====================

int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "批量特征提取工具" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 解析命令行参数
    enum class Mode { All, EsfOnly, CvfhOnly, Single };
    Mode mode = Mode::All;
    std::string specific_file;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp(argv[0]);
            return 0;
        } else if (arg == "--esf-only") {
            mode = Mode::EsfOnly;
        } else if (arg == "--cvfh-only") {
            mode = Mode::CvfhOnly;
        } else if (arg == "--file" && i + 1 < argc) {
            mode = Mode::Single;
            specific_file = argv[++i];
        } else {
            std::cerr << "[ERROR] 未知参数: " << arg << std::endl;
            printHelp(argv[0]);
            return 1;
        }
    }
    
    // 确定提取模式
    bool extract_esf = (mode == Mode::All || mode == Mode::EsfOnly);
    bool extract_cvfh = (mode == Mode::All || mode == Mode::CvfhOnly);
    
    std::cout << "\n路径配置:" << std::endl;
    std::cout << "  输入目录: " << kInputPointCloudDir << std::endl;
    if (extract_esf) std::cout << "  ESF输出目录: " << kOutputEsfDir << std::endl;
    if (extract_cvfh) std::cout << "  CVFH输出目录: " << kOutputCvfhDir << std::endl;
    
    // 创建特征提取器
    ESFEstimator esf_estimator;
    CVFHEstimator cvfh_estimator;
    
    std::cout << "\n[OK] 特征提取器创建成功" << std::endl;
    if (extract_esf) {
        std::cout << "  ESF: " << ESFEstimator::dimension() << "维" << std::endl;
    }
    if (extract_cvfh) {
        std::cout << "  CVFH: " << CVFHEstimator::dimension() << "维" << std::endl;
    }
    
    // 确保输出目录存在
    if (extract_esf) fs::create_directories(kOutputEsfDir);
    if (extract_cvfh) fs::create_directories(kOutputCvfhDir);
    
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
        
        // 按文件名排序，确保顺序一致
        std::sort(pcd_files.begin(), pcd_files.end());
        
        // 统计
        int total_files = static_cast<int>(pcd_files.size());
        int success_count = 0;
        int fail_count = 0;
        int total_esf_time = 0;
        int total_cvfh_time = 0;
        
        auto total_start = std::chrono::high_resolution_clock::now();
        
        if (mode == Mode::Single) {
            // 处理指定文件
            std::cout << "\n[模式] 处理指定文件: " << specific_file << std::endl;
            
            fs::path target_file = kInputPointCloudDir / specific_file;
            if (!fs::exists(target_file)) {
                target_file = kInputPointCloudDir / (specific_file + ".pcd");
            }
            
            if (!fs::exists(target_file)) {
                std::cerr << "[ERROR] 文件不存在: " << target_file << std::endl;
                return 1;
            }
            
            auto result = processSingleFile(target_file, esf_estimator, cvfh_estimator,
                                            extract_esf, extract_cvfh);
            if (result.esf_success || result.cvfh_success) {
                success_count++;
            } else {
                fail_count++;
            }
            total_esf_time += result.esf_time_ms;
            total_cvfh_time += result.cvfh_time_ms;
            
        } else {
            // 批量处理
            std::string mode_str = (mode == Mode::All) ? "ESF+CVFH" :
                                   (mode == Mode::EsfOnly) ? "ESF" : "CVFH";
            std::cout << "\n[模式] 批量处理 " << mode_str 
                      << ", 共 " << total_files << " 个文件" << std::endl;
            
            for (int i = 0; i < total_files; ++i) {
                std::cout << "\n[" << (i + 1) << "/" << total_files << "]";
                
                auto result = processSingleFile(pcd_files[i], esf_estimator, cvfh_estimator,
                                                extract_esf, extract_cvfh);
                if (result.esf_success || result.cvfh_success) {
                    success_count++;
                } else {
                    fail_count++;
                }
                total_esf_time += result.esf_time_ms;
                total_cvfh_time += result.cvfh_time_ms;
            }
        }
        
        auto total_end = std::chrono::high_resolution_clock::now();
        auto total_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            total_end - total_start).count();
        
        // 输出汇总
        std::cout << "\n========================================" << std::endl;
        std::cout << "批量处理完成!" << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "统计结果:" << std::endl;
        std::cout << "  总文件数: " << total_files << std::endl;
        std::cout << "  成功: " << success_count << std::endl;
        std::cout << "  失败: " << fail_count << std::endl;
        std::cout << "----------------------------------------" << std::endl;
        std::cout << "耗时统计:" << std::endl;
        std::cout << "  总耗时: " << total_duration << " ms" << std::endl;
        if (extract_esf) {
            std::cout << "  ESF总耗时: " << total_esf_time << " ms" << std::endl;
            std::cout << "  ESF平均: " << (success_count > 0 ? total_esf_time / success_count : 0) << " ms/文件" << std::endl;
        }
        if (extract_cvfh) {
            std::cout << "  CVFH总耗时: " << total_cvfh_time << " ms" << std::endl;
            std::cout << "  CVFH平均: " << (success_count > 0 ? total_cvfh_time / success_count : 0) << " ms/文件" << std::endl;
        }
        std::cout << "========================================" << std::endl;
        std::cout << "批量特征提取完成!" << std::endl;
        std::cout << "========================================" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "\n[ERROR] 测试失败: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
