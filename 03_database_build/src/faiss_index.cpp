/**
 * @file faiss_index.cpp
 * @brief FAISS索引封装类实现
 */

#include "faiss_index.h"
#include <faiss/Index.h>
#include <faiss/IndexIVFFlat.h>
#include <faiss/IndexFlat.h>
#include <faiss/IndexIDMap.h>
#include <faiss/index_io.h>
#include <iostream>
#include <cstring>
#include <stdexcept>

namespace esf_database {
namespace database {

// 实现结构体
struct FaissIndex::Impl {
    faiss::Index* index = nullptr;
    faiss::IndexIVFFlat* index_ivf = nullptr;
    faiss::IndexIDMap* index_idmap = nullptr;  // 支持add_with_ids
    bool is_flat_index = false;  // 是否为Flat索引
    
    Impl(size_t dim, size_t nlist, MetricType metric) {
        // 检查是否使用Flat索引（nlist <= 32时）
        if (nlist <= 32) {
            is_flat_index = true;
            faiss::IndexFlat* flat_index = nullptr;
            if (metric == MetricType::IP) {
                flat_index = new faiss::IndexFlatIP(dim);
            } else {
                flat_index = new faiss::IndexFlatL2(dim);
            }
            // 使用IDMap包装，支持add_with_ids
            index_idmap = new faiss::IndexIDMap(flat_index);
            index = index_idmap;
        } else {
            // 创建量化器（用于IVF索引）
            faiss::IndexFlat* quantizer = nullptr;
            if (metric == MetricType::IP) {
                quantizer = new faiss::IndexFlatIP(dim);
            } else {
                quantizer = new faiss::IndexFlatL2(dim);
            }
            
            // 创建IVF索引
            index_ivf = new faiss::IndexIVFFlat(quantizer, dim, nlist);
            index = index_ivf;
        }
    }
    
    ~Impl() {
        delete index;
    }
};

FaissIndex::FaissIndex(size_t dimension, size_t nlist, MetricType metric)
    : pimpl_(std::make_unique<Impl>(dimension, nlist, metric))
    , dimension_(dimension)
    , nlist_(nlist)
    , metric_(metric) {
}

FaissIndex::~FaissIndex() = default;

FaissIndex::FaissIndex(FaissIndex&&) noexcept = default;
FaissIndex& FaissIndex::operator=(FaissIndex&&) noexcept = default;

bool FaissIndex::train(const float* train_data, size_t count) {
    if (!pimpl_->index) return false;
    
    try {
        pimpl_->index->train(count, train_data);
        is_trained_ = true;
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool FaissIndex::add(const float* data, int64_t id) {
    if (!pimpl_->index) {
        std::cerr << "[FaissIndex::add] index is null" << std::endl;
        return false;
    }
    
    // 检查是否需要训练（IVF索引需要，Flat索引不需要）
    if (!pimpl_->is_flat_index && !is_trained_) {
        std::cerr << "[FaissIndex::add] index not trained" << std::endl;
        return false;
    }
    
    try {
        std::vector<int64_t> ids(1, id);
        pimpl_->index->add_with_ids(1, data, ids.data());
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[FaissIndex::add] exception: " << e.what() << std::endl;
        return false;
    }
}

bool FaissIndex::addBatch(const float* data, const int64_t* ids, size_t count) {
    if (!pimpl_->index) return false;
    
    // 检查是否需要训练（IVF索引需要，Flat索引不需要）
    if (!pimpl_->is_flat_index && !is_trained_) return false;
    
    try {
        pimpl_->index->add_with_ids(count, data, ids);
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

std::vector<SearchResult> FaissIndex::search(const float* query, size_t k) {
    std::vector<SearchResult> results;
    
    if (!pimpl_->index || pimpl_->index->ntotal == 0) {
        return results;
    }
    
    try {
        // 分配结果内存
        std::vector<float> distances(k);
        std::vector<int64_t> labels(k);
        
        // 执行搜索
        pimpl_->index->search(1, query, k, distances.data(), labels.data());
        
        // 转换结果
        for (size_t i = 0; i < k; i++) {
            if (labels[i] >= 0) {
                results.push_back({labels[i], distances[i]});
            }
        }
    } catch (const std::exception& e) {
        // 返回空结果
    }
    
    return results;
}

bool FaissIndex::save(const std::string& path) {
    if (!pimpl_->index) return false;
    
    try {
        faiss::write_index(pimpl_->index, path.c_str());
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

bool FaissIndex::load(const std::string& path) {
    try {
        delete pimpl_->index;
        pimpl_->index = faiss::read_index(path.c_str());
        
        // 检查索引类型
        pimpl_->index_ivf = dynamic_cast<faiss::IndexIVFFlat*>(pimpl_->index);
        pimpl_->index_idmap = dynamic_cast<faiss::IndexIDMap*>(pimpl_->index);
        
        // 如果是IDMap，尝试获取内部索引
        if (pimpl_->index_idmap) {
            pimpl_->index_ivf = dynamic_cast<faiss::IndexIVFFlat*>(
                pimpl_->index_idmap->index);
        }
        
        is_trained_ = true;
        return true;
    } catch (const std::exception& e) {
        return false;
    }
}

size_t FaissIndex::size() const {
    if (!pimpl_->index) return 0;
    return pimpl_->index->ntotal;
}

void FaissIndex::setNprobe(size_t nprobe) {
    if (pimpl_->index_ivf) {
        pimpl_->index_ivf->nprobe = nprobe;
    }
}

} // namespace database
} // namespace esf_database
