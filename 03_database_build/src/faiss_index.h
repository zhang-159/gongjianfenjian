/**
 * @file faiss_index.h
 * @brief FAISS索引封装类
 * 
 * 支持 ESF(640维) 和 CVFH(345维) 特征库的构建与检索
 */

#pragma once

#include <string>
#include <vector>
#include <memory>
#include <optional>

namespace esf_database {
namespace database {

// 搜索结果结构
struct SearchResult {
    int64_t id;       // 向量ID
    float distance;   // 距离
};

// FAISS索引封装类
class FaissIndex {
public:
    // 索引类型
    enum class MetricType {
        L2,     // 欧氏距离
        IP      // 内积（余弦相似度）
    };

    /**
     * @brief 构造函数
     * @param dimension 特征维度
     * @param nlist IVF列表数
     * @param metric 距离度量类型
     */
    FaissIndex(size_t dimension, size_t nlist = 4096, MetricType metric = MetricType::L2);
    
    /**
     * @brief 析构函数
     */
    ~FaissIndex();

    // 删除拷贝构造
    FaissIndex(const FaissIndex&) = delete;
    FaissIndex& operator=(const FaissIndex&) = delete;

    // 允许移动
    FaissIndex(FaissIndex&&) noexcept;
    FaissIndex& operator=(FaissIndex&&) noexcept;

    /**
     * @brief 训练索引
     * @param train_data 训练数据 (n_vectors x dimension)
     * @param count 向量数量
     * @return 是否成功
     */
    bool train(const float* train_data, size_t count);

    /**
     * @brief 添加向量
     * @param data 向量数据
     * @param id 向量ID
     * @return 是否成功
     */
    bool add(const float* data, int64_t id);

    /**
     * @brief 批量添加向量
     * @param data 向量数据 (n_vectors x dimension)
     * @param ids 向量ID列表
     * @param count 向量数量
     * @return 是否成功
     */
    bool addBatch(const float* data, const int64_t* ids, size_t count);

    /**
     * @brief 搜索Top-K最近邻
     * @param query 查询向量
     * @param k 返回数量
     * @return 搜索结果
     */
    std::vector<SearchResult> search(const float* query, size_t k);

    /**
     * @brief 保存索引到文件
     * @param path 文件路径
     * @return 是否成功
     */
    bool save(const std::string& path);

    /**
     * @brief 从文件加载索引
     * @param path 文件路径
     * @return 是否成功
     */
    bool load(const std::string& path);

    /**
     * @brief 获取索引维度
     */
    size_t dimension() const { return dimension_; }

    /**
     * @brief 获取向量数量
     */
    size_t size() const;

    /**
     * @brief 是否已训练
     */
    bool isTrained() const { return is_trained_; }

    /**
     * @brief 设置搜索探查数
     * @param nprobe 探查的列表数
     */
    void setNprobe(size_t nprobe);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    
    size_t dimension_;
    size_t nlist_;
    MetricType metric_;
    bool is_trained_ = false;
};

} // namespace database
} // namespace esf_database
