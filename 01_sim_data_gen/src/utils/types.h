#pragma once

/**
 * @file types.h
 * @brief 仿真数据生成模块的类型定义
 */

#include "../../../libs/common_types.h"

namespace sim_data_gen {

// 导入公共类型到本模块命名空间
using common_types::PointXYZ;
using common_types::PointCloudXYZ;
using common_types::PointCloudXYZPtr;
using common_types::PointCloudXYZConstPtr;
using common_types::Normal;
using common_types::NormalCloud;
using common_types::NormalCloudPtr;
using common_types::NormalCloudConstPtr;

}  // namespace sim_data_gen
