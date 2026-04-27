/**
 * @file cvfh_estimator.cpp
 * @brief OUR-CVFH特征提取器实现
 */

#include "cvfh_estimator.h"
#include <stdexcept>
#include <pcl/common/transforms.h>

namespace feature_extraction {

CVFHEstimator::CVFHEstimator() {
    config_ = CVFHConfig();

    // 配置OUR-CVFH估计器
    cvfh_estimator_.setRadiusNormals(config_.normal_radius);  // 法线搜索半径
    cvfh_estimator_.setEPSAngleThreshold(0.13f);  // 约7.5度
    cvfh_estimator_.setCurvatureThreshold(1.0f);
    cvfh_estimator_.setNormalizeBins(false);
    cvfh_estimator_.setAxisRatio(config_.axis_ratio);
    cvfh_estimator_.setMinPoints(config_.min_points);
}

CVFHEstimator::~CVFHEstimator() {
}

void CVFHEstimator::setConfig(const CVFHConfig& config) {
    config_ = config;

    // 更新CVFH估计器参数
    cvfh_estimator_.setRadiusNormals(config_.normal_radius);
    cvfh_estimator_.setAxisRatio(config_.axis_ratio);
    cvfh_estimator_.setMinPoints(config_.min_points);
}

const CVFHConfig& CVFHEstimator::getConfig() const {
    return config_;
}

NormalCloudPtr CVFHEstimator::estimateNormals(const PointCloudXYZPtr& input_cloud) {
    if (!input_cloud) {
        throw std::runtime_error("输入点云为空");
    }

    if (input_cloud->empty()) {
        throw std::runtime_error("输入点云点数为0");
    }

    NormalCloudPtr normals(new NormalCloud);
    normal_estimator_.setInputCloud(input_cloud);
    normal_estimator_.setRadiusSearch(config_.normal_radius);
    normal_estimator_.compute(*normals);

    if (normals->empty()) {
        throw std::runtime_error("法线估计失败，返回空结果");
    }

    return normals;
}

FeatureVector CVFHEstimator::extract(const PointCloudXYZPtr& input_cloud) {
    // 自动计算法线
    auto normals = estimateNormals(input_cloud);
    return extract(input_cloud, normals);
}

FeatureVector CVFHEstimator::extract(const PointCloudXYZPtr& input_cloud,
                                     const NormalCloudPtr& normals) {
    // 输入验证
    if (!input_cloud) {
        throw std::runtime_error("输入点云为空");
    }

    if (input_cloud->empty()) {
        throw std::runtime_error("输入点云点数为0");
    }

    if (!normals) {
        throw std::runtime_error("法线点云为空");
    }

    if (normals->empty()) {
        throw std::runtime_error("法线点数为0");
    }

    if (normals->size() != input_cloud->size()) {
        throw std::runtime_error("法线点数与输入点云不匹配");
    }

    // 创建特征点云
    CVFHFeatureCloudPtr features(new CVFHFeatureCloud);

    // 执行OUR-CVFH特征提取
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        const_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr&>(input_cloud));

    cvfh_estimator_.setInputCloud(cloud);
    cvfh_estimator_.setInputNormals(normals);
    cvfh_estimator_.compute(*features);

    // 检查结果
    if (features->empty()) {
        throw std::runtime_error("OUR-CVFH特征提取失败，返回空结果");
    }

    // OUR-CVFH可能返回多个特征（聚类结果），取第一个主特征
    // 或者对多个特征进行平均
    FeatureVector result(308, 0.0f);

    if (features->size() == 1) {
        // 单个特征，直接使用
        const auto& signature = features->points[0];
        for (int i = 0; i < 308; ++i) {
            result[i] = signature.histogram[i];
        }
    } else {
        // 多个特征，取第一个或平均
        // 这里选择第一个主特征（能量最大的）
        float max_energy = 0.0f;
        int best_idx = 0;

        for (size_t i = 0; i < features->size(); ++i) {
            float energy = 0.0f;
            for (int j = 0; j < 308; ++j) {
                energy += features->points[i].histogram[j] * features->points[i].histogram[j];
            }
            if (energy > max_energy) {
                max_energy = energy;
                best_idx = i;
            }
        }

        const auto& signature = features->points[best_idx];
        for (int i = 0; i < 308; ++i) {
            result[i] = signature.histogram[i];
        }
    }

    return result;
}

void CVFHEstimator::extract(const PointCloudXYZPtr& input_cloud,
                           const NormalCloudPtr& normals,
                           CVFHFeatureCloudPtr& feature_cloud) {
    // 输入验证
    if (!input_cloud || input_cloud->empty()) {
        throw std::runtime_error("输入点云为空");
    }

    if (!normals || normals->empty()) {
        throw std::runtime_error("法线点云为空");
    }

    if (normals->size() != input_cloud->size()) {
        throw std::runtime_error("法线点数与输入点云不匹配");
    }

    // 确保输出指针有效
    if (!feature_cloud) {
        feature_cloud.reset(new CVFHFeatureCloud);
    }

    // 设置输入并计算
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        const_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr&>(input_cloud));

    cvfh_estimator_.setInputCloud(cloud);
    cvfh_estimator_.setInputNormals(normals);
    cvfh_estimator_.compute(*feature_cloud);
}

int CVFHEstimator::dimension() {
    return 308;
}

std::string CVFHEstimator::getDescription() {
    return "OUR-CVFH (Oriented, Unique and Repeatable Viewpoint Feature Histogram) - "
           "308维特征向量, 对视点变化鲁棒, "
           "能够处理部分遮挡的点云, 适用于复杂场景识别";
}

}  // namespace feature_extraction
