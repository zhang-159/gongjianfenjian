/**
 * @file build_faiss_index.cpp
 * @brief FAISS指纹库构建工具
 * 
 * 用法:
 *   build_faiss_index.exe              # 构建所有索引
 *   build_faiss_index.exe --esf-only   # 仅构建ESF索引
 *   build_faiss_index.exe --cvfh-only # 仅构建CVFH索引
 *   build_faiss_index.exe --rebuild   # 强制重建
 *   build_faiss_index.exe --help       # 显示帮助
 * 
 * 输入: 
 *   - data/esf_features/train/*.esf
 *   - data/cvfh_features/train/*.cvfh
 * 
 * 输出:
 *   - data/fingerprints/esf_index.bin
 *   - data/fingerprints/cvfh_index.bin
 *   - data/fingerprints/esf_ids.txt
 *   - data/fingerprints/cvfh_ids.txt
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <filesystem>
#include <vector>
#include <string>
#include <regex>
#include <chrono>
#include <cctype>

#include "../src/faiss_index.h"

namespace fs = std::filesystem;

// ==================== 路径配置 ====================

const fs::path kProjectRoot = "D:/vsxiangmu/gongjianfenjian";
const fs::path kDataDir = kProjectRoot / "data";
const fs::path kInputEsfDir = kDataDir / "esf_features/train";
const fs::path kInputCvfhDir = kDataDir / "cvfh_features/train";
const fs::path kOutputFingerprintsDir = kDataDir / "fingerprints";

// ESF维度
const size_t kEsfDimension = 640;
// CVFH维度  
const size_t kCvfhDimension = 345;

// IVF聚类数 (根据数据量动态调整)
size_t kNlist = 4096;

// 根据训练数据量动态调整nlist
size_t computeNlist(size_t train_count, size_t min_nlist = 32) {
    // 经验公式: nlist = sqrt(n) 或 4*sqrt(n)，但不超过4096
    size_t suggested = static_cast<size_t>(std::sqrt(static_cast<double>(train_count)) * 4);
    suggested = std::max(suggested, min_nlist);
    suggested = std::min(suggested, kNlist);
    return suggested;
}

// ==================== 数据结构 ====================

struct FeatureFileInfo {
    fs::path file_path;
    std::string object_id;   // 工件ID (如 "one")
    std::string pose_id;     // 姿态ID (如 "rx0_ry0_rz0")
    int64_t vector_id;       // FAISS向量ID (0,1,2,...)
};

struct FeatureVector {
    std::vector<float> data;
    int64_t vector_id;
    std::string object_id;
    std::string pose_id;
};

// ==================== 文件解析函数 ====================

// 从文件名解析工件ID和姿态ID
// 格式: object_rx0_ry0_rz0.esf -> object_id="object", pose_id="rx0_ry0_rz0"
bool parseFeatureFilename(const fs::path& file_path, 
                          std::string& object_id, 
                          std::string& pose_id) {
    std::string filename = file_path.stem().string();
    
    // 查找第一个下划线位置
    size_t underscore_pos = filename.find('_');
    if (underscore_pos == std::string::npos) {
        object_id = filename;
        pose_id = "default";
        return true;
    }
    
    object_id = filename.substr(0, underscore_pos);
    pose_id = filename.substr(underscore_pos + 1);
    
    return true;
}

// 读取二进制特征文件
bool readFeatureFile(const fs::path& file_path, std::vector<float>& features) {
    try {
        std::ifstream ifs(file_path, std::ios::binary);
        if (!ifs.is_open()) {
            std::cerr << "[ERROR] 无法打开文件: " << file_path << std::endl;
            return false;
        }
        
        // 读取维度
        size_t dim;
        ifs.read(reinterpret_cast<char*>(&dim), sizeof(size_t));
        
        // 读取特征数据
        features.resize(dim);
        ifs.read(reinterpret_cast<char*>(features.data()), dim * sizeof(float));
        
        ifs.close();
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] 读取失败: " << e.what() << std::endl;
        return false;
    }
}

// ==================== 索引构建函数 ====================

// 收集目录中的所有特征文件
std::vector<FeatureFileInfo> collectFeatureFiles(const fs::path& dir, 
                                                  const std::string& extension) {
    std::vector<FeatureFileInfo> files;
    
    if (!fs::exists(dir)) {
        std::cerr << "[WARN] 目录不存在: " << dir << std::endl;
        return files;
    }
    
    for (const auto& entry : fs::directory_iterator(dir)) {
        if (entry.is_regular_file() && entry.path().extension() == extension) {
            FeatureFileInfo info;
            info.file_path = entry.path();
            
            if (parseFeatureFilename(info.file_path, info.object_id, info.pose_id)) {
                files.push_back(info);
            }
        }
    }
    
    // 按文件名排序
    std::sort(files.begin(), files.end(), 
              [](const FeatureFileInfo& a, const FeatureFileInfo& b) {
                  return a.file_path < b.file_path;
              });
    
    return files;
}

// 构建单个索引
bool buildIndex(const std::string& index_name,
                const std::vector<FeatureFileInfo>& files,
                const fs::path& output_index_path,
                const fs::path& output_ids_path,
                size_t dimension) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "构建 " << index_name << " 索引" << std::endl;
    std::cout << "========================================" << std::endl;
    
    if (files.empty()) {
        std::cerr << "[ERROR] 没有找到特征文件" << std::endl;
        return false;
    }
    
    std::cout << "找到 " << files.size() << " 个特征文件" << std::endl;
    
    // 创建FAISS索引
    // 动态调整nlist，数据量小时使用更小的值
    size_t actual_nlist = computeNlist(files.size());
    std::cout << "索引nlist: " << actual_nlist << std::endl;
    
    auto index = std::make_unique<esf_database::database::FaissIndex>(
        dimension, actual_nlist, esf_database::database::FaissIndex::MetricType::L2);
    
    // 第一遍：读取所有特征并收集训练数据
    std::cout << "\n[1/3] 读取特征文件..." << std::endl;
    std::vector<FeatureVector> all_features;
    all_features.reserve(files.size());
    
    int64_t vector_id = 0;
    for (const auto& file : files) {
        std::vector<float> features;
        if (readFeatureFile(file.file_path, features)) {
            FeatureVector fv;
            fv.data = std::move(features);
            fv.vector_id = vector_id;
            fv.object_id = file.object_id;
            fv.pose_id = file.pose_id;
            all_features.push_back(std::move(fv));
            vector_id++;
        }
    }
    
    if (all_features.empty()) {
        std::cerr << "[ERROR] 没有成功读取任何特征" << std::endl;
        return false;
    }
    
    std::cout << "成功读取 " << all_features.size() << " 个特征向量" << std::endl;
    
    // 根据数据量决定是否训练
    if (files.size() < 32 || files.size() < actual_nlist) {
        // 数据量太小，跳过训练（使用Flat索引）
        std::cout << "\n[2/3] 数据量较小，使用Flat索引（无需训练）..." << std::endl;
    } else {
        // 数据量足够，进行训练
        size_t train_count = std::min(all_features.size(), (size_t)1000);
        std::vector<float> train_data(dimension * train_count);
        
        for (size_t i = 0; i < train_count; i++) {
            std::memcpy(train_data.data() + i * dimension, 
                        all_features[i].data.data(), 
                        dimension * sizeof(float));
        }
        
        // 训练索引
        std::cout << "\n[2/3] 训练索引 (训练数据: " << train_count << " 个向量)..." << std::endl;
        auto train_start = std::chrono::high_resolution_clock::now();
        
        if (!index->train(train_data.data(), train_count)) {
            std::cerr << "[ERROR] 索引训练失败" << std::endl;
            return false;
        }
        
        auto train_end = std::chrono::high_resolution_clock::now();
        auto train_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            train_end - train_start).count();
        std::cout << "训练完成，耗时: " << train_ms << " ms" << std::endl;
    }
    
    // 添加向量到索引
    std::cout << "\n[3/3] 添加向量到索引..." << std::endl;
    auto add_start = std::chrono::high_resolution_clock::now();
    
    int add_success = 0;
    for (const auto& fv : all_features) {
        if (index->add(fv.data.data(), fv.vector_id)) {
            add_success++;
        } else {
            std::cerr << "[ERROR] 添加向量 " << fv.vector_id << " 失败" << std::endl;
        }
    }
    
    auto add_end = std::chrono::high_resolution_clock::now();
    auto add_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        add_end - add_start).count();
    std::cout << "添加完成，成功: " << add_success << " 个，耗时: " << add_ms << " ms" << std::endl;
    
    // 保存索引
    std::cout << "\n保存索引到: " << output_index_path << std::endl;
    fs::create_directories(output_index_path.parent_path());
    
    if (!index->save(output_index_path.string())) {
        std::cerr << "[ERROR] 保存索引失败" << std::endl;
        return false;
    }
    
    // 保存ID映射文件
    std::cout << "保存ID映射到: " << output_ids_path << std::endl;
    std::ofstream id_file(output_ids_path);
    if (!id_file.is_open()) {
        std::cerr << "[ERROR] 无法创建ID映射文件" << std::endl;
        return false;
    }
    
    // 写入CSV头
    id_file << "vector_id,object_id,pose_id,filename\n";
    
    for (size_t i = 0; i < all_features.size(); i++) {
        const auto& fv = all_features[i];
        id_file << fv.vector_id << ","
                << fv.object_id << ","
                << fv.pose_id << ","
                << files[i].file_path.filename().string() << "\n";
    }
    id_file.close();
    
    std::cout << "\n[OK] " << index_name << " 索引构建完成!" << std::endl;
    std::cout << "  索引大小: " << index->size() << " 个向量" << std::endl;
    
    return true;
}

// ==================== 帮助信息 ====================

void printHelp(const char* prog_name) {
    std::cout << "\n用法:" << std::endl;
    std::cout << "  " << prog_name << "                  # 构建所有索引 (ESF + CVFH)" << std::endl;
    std::cout << "  " << prog_name << " --esf-only       # 仅构建ESF索引" << std::endl;
    std::cout << "  " << prog_name << " --cvfh-only      # 仅构建CVFH索引" << std::endl;
    std::cout << "  " << prog_name << " --help           # 显示帮助" << std::endl;
    std::cout << "\n输入路径:" << std::endl;
    std::cout << "  ESF:   " << kInputEsfDir << std::endl;
    std::cout << "  CVFH:  " << kInputCvfhDir << std::endl;
    std::cout << "\n输出路径:" << std::endl;
    std::cout << "  " << kOutputFingerprintsDir << std::endl;
    std::cout << "\n文件名格式:" << std::endl;
    std::cout << "  object_rx0_ry0_rz0.esf" << std::endl;
    std::cout << "  object: 工件ID" << std::endl;
    std::cout << "  rx0_ry0_rz0: 姿态描述" << std::endl;
}

// ==================== 主函数 ====================
// [暂时禁用main函数，避免与04模块冲突，后续开发完成后取消注释]
/*
int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "FAISS指纹库构建工具" << std::endl;
    std::cout << "========================================" << std::endl;
    
    // 解析命令行参数
    enum class Mode { All, EsfOnly, CvfhOnly };
    Mode mode = Mode::All;
    
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printHelp(argv[0]);
            return 0;
        } else if (arg == "--esf-only") {
            mode = Mode::EsfOnly;
        } else if (arg == "--cvfh-only") {
            mode = Mode::CvfhOnly;
        } else {
            std::cerr << "[ERROR] 未知参数: " << arg << std::endl;
            printHelp(argv[0]);
            return 1;
        }
    }
    
    // 打印路径配置
    std::cout << "\n路径配置:" << std::endl;
    std::cout << "  项目根目录: " << kProjectRoot << std::endl;
    std::cout << "  ESF特征: " << kInputEsfDir << std::endl;
    std::cout << "  CVFH特征: " << kInputCvfhDir << std::endl;
    std::cout << "  索引输出: " << kOutputFingerprintsDir << std::endl;
    
    // 确保输出目录存在
    fs::create_directories(kOutputFingerprintsDir);
    
    // 开始计时
    auto total_start = std::chrono::high_resolution_clock::now();
    
    bool success = true;
    
    // 构建ESF索引
    if (mode == Mode::All || mode == Mode::EsfOnly) {
        auto esf_files = collectFeatureFiles(kInputEsfDir, ".esf");
        
        success = buildIndex("ESF", 
                            esf_files,
                            kOutputFingerprintsDir / "esf_index.bin",
                            kOutputFingerprintsDir / "esf_ids.txt",
                            kEsfDimension);
    }
    
    // 构建CVFH索引
    if (mode == Mode::All || mode == Mode::CvfhOnly) {
        auto cvfh_files = collectFeatureFiles(kInputCvfhDir, ".cvfh");
        
        if (success || mode == Mode::CvfhOnly) {
            success = buildIndex("CVFH", 
                                cvfh_files,
                                kOutputFingerprintsDir / "cvfh_index.bin",
                                kOutputFingerprintsDir / "cvfh_ids.txt",
                                kCvfhDimension);
        }
    }
    
    // 结束计时
    auto total_end = std::chrono::high_resolution_clock::now();
    auto total_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        total_end - total_start).count();
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "构建完成! 总耗时: " << total_ms << " ms" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return success ? 0 : 1;
}
*/
