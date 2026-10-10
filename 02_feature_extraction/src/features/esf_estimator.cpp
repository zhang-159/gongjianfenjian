/**
 * @file esf_estimator.cpp
 * @brief ESF特征提取器实现
 */

#include "esf_estimator.h"
#include <stdexcept>
#include <pcl/common/transforms.h>
#include <pcl/filters/random_sample.h>
#include <chrono>

namespace feature_extraction {

ESFEstimator::ESFEstimator() {
    // 默认配置
    config_ = ESFConfig();
}

ESFEstimator::~ESFEstimator() {
    // 默认析构
}

void ESFEstimator::setConfig(const ESFConfig& config) {
    config_ = config;
}

const ESFConfig& ESFEstimator::getConfig() const {
    return config_;
}

FeatureVector ESFEstimator::extract(const PointCloudXYZPtr& input_cloud) {
    // 输入验证
    if (!input_cloud) {
        throw std::runtime_error("输入点云为空");
    }

    if (input_cloud->empty()) {
        throw std::runtime_error("输入点云点数为0");
    }

    // 创建特征点云
    ESFFeatureCloudPtr features(new ESFFeatureCloud);

    // 按配置采样点数随机降采样，避免 ESF 对大点云过拟合
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        const_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr&>(input_cloud));
    if (cloud->size() > static_cast<size_t>(config_.sample_points)) {
        pcl::RandomSample<pcl::PointXYZ> sample;
        sample.setInputCloud(cloud);
        sample.setSample(static_cast<unsigned int>(config_.sample_points));
        sample.setSeed(static_cast<unsigned int>(
            std::chrono::steady_clock::now().time_since_epoch().count()));
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        sample.filter(*cloud);
    }

    // 执行ESF特征提取
    esf_estimator_.setInputCloud(cloud);
    esf_estimator_.compute(*features);

    // 检查结果
    if (features->empty()) {
        throw std::runtime_error("ESF特征提取失败，返回空结果");
    }

    // 转换为FeatureVector格式
    FeatureVector result;
    const auto& signature = features->points[0];

    // ESFSignature640 有640个描述子
    // 分别在 histogram[0-9], histogram[10-127], histogram[128-635] 三个区域
    for (int i = 0; i < 640; ++i) {
        result.push_back(signature.histogram[i]);
    }

    return result;
}

void ESFEstimator::extract(const PointCloudXYZPtr& input_cloud,
                          ESFFeatureCloudPtr& feature_cloud) {
    // 输入验证
    if (!input_cloud) {
        throw std::runtime_error("输入点云为空");
    }

    if (input_cloud->empty()) {
        throw std::runtime_error("输入点云点数为0");
    }

    // 确保输出指针有效
    if (!feature_cloud) {
        feature_cloud.reset(new ESFFeatureCloud);
    }

    // 设置输入并计算
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        const_cast<pcl::PointCloud<pcl::PointXYZ>::Ptr&>(input_cloud));
    if (cloud->size() > static_cast<size_t>(config_.sample_points)) {
        pcl::RandomSample<pcl::PointXYZ> sample;
        sample.setInputCloud(cloud);
        sample.setSample(static_cast<unsigned int>(config_.sample_points));
        sample.setSeed(static_cast<unsigned int>(
            std::chrono::steady_clock::now().time_since_epoch().count()));
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        sample.filter(*cloud);
    }

    esf_estimator_.setInputCloud(cloud);
    esf_estimator_.compute(*feature_cloud);
}

int ESFEstimator::dimension() {
    return 640;
}

std::string ESFEstimator::getDescription() {
    return "ESF (Ensemble of Shape Functions) - 640维特征向量, "
           "包含距离、角度、面积三个形状函数集合, "
           "适用于完整点云的快速识别";
}

}  // namespace feature_extraction
