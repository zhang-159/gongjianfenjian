#pragma once

/**
 * @file common_types.h
 * @brief 公共类型定义 - 所有模块共享的基础类型
 *
 * 本文件定义了点云处理中最常用的基础类型，
 * 所有模块的 types.h 都应包含此文件并在其基础上扩展。
 */

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/impl/point_types.hpp>

namespace common_types {

// =============================================
// 基础点云类型
// =============================================
using PointXYZ = pcl::PointXYZ;
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using PointCloudXYZConstPtr = pcl::PointCloud<pcl::PointXYZ>::ConstPtr;

// =============================================
// 法线类型
// =============================================
using Normal = pcl::Normal;
using NormalCloud = pcl::PointCloud<pcl::Normal>;
using NormalCloudPtr = pcl::PointCloud<pcl::Normal>::Ptr;
using NormalCloudConstPtr = pcl::PointCloud<pcl::Normal>::ConstPtr;

}  // namespace common_types
