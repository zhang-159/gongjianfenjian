#pragma once

/**
 * @file cvfh_estimator.h
 * @brief OUR-CVFH (Oriented, Unique and Repeatable Viewpoint Feature Histogram) 特征提取器
 * @note OUR-CVFH特征是308维向量，对部分遮挡和不同视角具有更好的鲁棒性
 */

#include "../utils/types.h"
#include <pcl/features/normal_3d.h>
#include <string>

namespace feature_extraction {

/**
 * @class CVFHEstimator
 * @brief OUR-CVFH特征提取器
 *
 * OUR-CVFH特征是VFH的改进版本：
 * - 对视点变化更加鲁棒
 * - 能够处理部分遮挡的点云
 * - 使用聚类方法分割多个视图
 *
 * 共308维特征向量，适用于部分点云和复杂场景识别。
 */
class CVFHEstimator {
public:
    /**
     * @brief 构造函数
     */
    CVFHEstimator();

    /**
     * @brief 析构函数
     */
    ~CVFHEstimator();

    /**
     * @brief 设置配置参数
     * @param config CVFH配置
     */
    void setConfig(const CVFHConfig& config);

    /**
     * @brief 获取配置参数
     * @return CVFH配置
     */
    const CVFHConfig& getConfig() const;

    /**
     * @brief 提取OUR-CVFH特征
     * @param input_cloud 输入点云
     * @param normals 法线估计（必须先计算）
     * @return 308维OUR-CVFH特征向量
     * @throws std::runtime_error 如果点云为空或法线无效
     */
    FeatureVector extract(const PointCloudXYZPtr& input_cloud,
                          const NormalCloudPtr& normals);

    /**
     * @brief 提取OUR-CVFH特征（自动计算法线）
     * @param input_cloud 输入点云
     * @return 308维OUR-CVFH特征向量
     * @throws std::runtime_error 如果点云为空
     */
    FeatureVector extract(const PointCloudXYZPtr& input_cloud);

    /**
     * @brief 提取OUR-CVFH特征（批量版本）
     * @param input_cloud 输入点云
     * @param normals 法线估计
     * @param feature_cloud 输出特征点云
     * @throws std::runtime_error 如果点云为空或法线无效
     */
    void extract(const PointCloudXYZPtr& input_cloud,
                 const NormalCloudPtr& normals,
                 CVFHFeatureCloudPtr& feature_cloud);

    /**
     * @brief 估计法线
     * @param input_cloud 输入点云
     * @return 法线点云
     * @throws std::runtime_error 如果点云为空
     */
    NormalCloudPtr estimateNormals(const PointCloudXYZPtr& input_cloud);

    /**
     * @brief 获取OUR-CVFH特征维度
     * @return 308
     */
    static int dimension();

    /**
     * @brief 获取特征描述
     * @return 特征描述字符串
     */
    static std::string getDescription();

private:
    CVFHConfig config_;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator_;
    pcl::OURCVFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> cvfh_estimator_;
};

}  // namespace feature_extraction
