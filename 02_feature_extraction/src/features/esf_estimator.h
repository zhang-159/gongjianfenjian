#pragma once

/**
 * @file esf_estimator.h
 * @brief ESF (Ensemble of Shape Functions) 特征提取器
 * @note ESF特征是640维向量，用于快速点云识别
 */

#include "../utils/types.h"
#include <string>

namespace feature_extraction {

/**
 * @class ESFEstimator
 * @brief ESF特征提取器
 * 
 * ESF特征描述了点云的形状函数集合，包括：
 * - 距离函数 (Distance)
 * - 角度函数 (Angle)  
 * - 面积函数 (Area)
 * 
 * 共640维特征向量，适用于完整点云的快速识别。
 */
class ESFEstimator {
public:
    /**
     * @brief 构造函数
     */
    ESFEstimator();

    /**
     * @brief 析构函数
     */
    ~ESFEstimator();

    /**
     * @brief 设置配置参数
     * @param config ESF配置
     */
    void setConfig(const ESFConfig& config);

    /**
     * @brief 获取配置参数
     * @return ESF配置
     */
    const ESFConfig& getConfig() const;

    /**
     * @brief 提取ESF特征
     * @param input_cloud 输入点云（点数建议1024-8192）
     * @return 640维ESF特征向量
     * @throws std::runtime_error 如果点云为空或点数不足
     */
    FeatureVector extract(const PointCloudXYZPtr& input_cloud);

    /**
     * @brief 提取ESF特征（批量版本）
     * @param input_cloud 输入点云
     * @param feature_cloud 输出特征点云
     * @throws std::runtime_error 如果点云为空
     */
    void extract(const PointCloudXYZPtr& input_cloud, 
                 ESFFeatureCloudPtr& feature_cloud);

    /**
     * @brief 获取ESF特征维度
     * @return 640
     */
    static int dimension();

    /**
     * @brief 获取特征描述
     * @return 特征描述字符串
     */
    static std::string getDescription();

private:
    ESFConfig config_;
    pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf_estimator_;
};

}  // namespace feature_extraction
