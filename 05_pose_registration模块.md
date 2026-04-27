# 05_pose_registration 配准定位模块

## 1. 模块概述

### 1.1 功能定位

`05_pose_registration` 是工件分拣系统的**配准定位模块**，负责在完成类别识别（04模块）后，计算工件在相机坐标系下的精确6DoF位姿。

### 1.2 输入输出

| 类型 | 内容 | 来源 |
|------|------|------|
| **输入** | 场景点云、模板点云 | 相机采集 / 04识别结果 |
| **输出** | 6DoF位姿(XYZ+RPY)、变换矩阵、配准质量分数 | 下发机器人抓取 |

### 1.3 技术路线

```
场景点云 ──▶ 预处理 ──▶ FPFH特征 ──▶ SAC-IA粗配准 ──▶ ICP精配准 ──▶ 6DoF位姿
模板点云 ──▶ (降采样/去噪) ─────────▶ (33维) ───▶ (全局对齐) ──▶ (精修) ──▶ (XYZ+RPY)
```

---

## 2. 目录结构

```
05_pose_registration/
├── include/                              # 头文件目录
│   ├── pose_registration.h               # 公共类型定义
│   ├── fpfh_estimator.h                  # FPFH特征提取器
│   ├── sac_ia_registration.h             # SAC-IA粗配准
│   ├── icp_registration.h                # ICP精配准
│   ├── pose_estimator.h                  # 位姿估计器(整合)
│   └── cloud_preprocessor.h              # 预处理工具
├── src/                                  # 源文件目录
│   ├── fpfh_estimator.cpp
│   ├── sac_ia_registration.cpp
│   ├── icp_registration.cpp
│   ├── pose_estimator.cpp
│   ├── cloud_preprocessor.cpp
│   └── main.cpp                          # 测试程序
├── config/                               # 配置文件目录
│   └── registration_config.yaml          # 配准参数配置
├── data/                                 # 测试数据目录
│   ├── templates/                        # 模板点云
│   └── scenes/                           # 场景点云
└── README.md                             # 模块说明
```

---

## 3. 核心类设计

### 3.1 数据结构

#### Pose6DOF - 6自由度位姿

```cpp
// include/pose_registration.h

#pragma once

#include <Eigen/Dense>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace esf_database {
namespace registration {

// 6DoF位姿结构
struct Pose6DOF {
    Eigen::Vector3f position;    // x, y, z (米)
    Eigen::Vector3f rpy;          // roll, pitch, yaw (弧度)
    
    Eigen::Vector3f position_mm() const {
        return position * 1000.0f;  // 转换为毫米
    }
    
    Eigen::Vector3f toDegrees() const {
        return rpy * 180.0f / M_PI;
    }
    
    std::string toString() const {
        return "pos(" + std::to_string(position.x()) + ", " 
             + std::to_string(position.y()) + ", " 
             + std::to_string(position.z()) + ") "
             + "rpy(" + std::to_string(toDegrees().x()) + "°, "
             + std::to_string(toDegrees().y()) + "°, "
             + std::to_string(toDegrees().z()) + "°)";
    }
};

// 配准结果
struct RegistrationResult {
    Eigen::Matrix4f transformation;     // 4x4 变换矩阵 T_template_to_scene
    Pose6DOF pose_6dof;                 // 6DoF 位姿
    float fitness_score;                 // 适配度分数 [0,1]，越高越好
    float position_error;                // 位置误差估计 (米)
    float rotation_error;                // 旋转误差估计 (度)
    bool success;                       // 是否成功
    std::string message;                // 状态信息
    int sacia_iterations;               // SAC-IA迭代次数
    int icp_iterations;                // ICP迭代次数
    
    bool isValid(float max_pos_err = 0.01f, float max_rot_err = 15.0f) const {
        return success && position_error < max_pos_err && rotation_error < max_rot_err;
    }
};

}}
```

#### Config - 配置参数

```cpp
// 配准配置参数
struct RegistrationConfig {
    // ========== 预处理参数 ==========
    float voxel_leaf_size = 0.005f;    // 体素降采样大小 (米)
    int outlier_mean_k = 50;            // 离群点移除邻居数
    float outlier_std_dev = 1.0f;       // 标准差倍数
    
    // ========== FPFH参数 ==========
    int normal_k_search = 20;           // 法线估计K近邻
    int fpfh_k_search = 20;             // FPFH K近邻
    float fpfh_radius = 0.05f;          // FPFH搜索半径
    
    // ========== SAC-IA参数 ==========
    float sac_min_sample_distance = 0.05f;       // 最小采样距离
    float sac_max_correspondence_distance = 0.2f; // 最大对应点距离
    int sac_nr_iterations = 500;                  // RANSAC迭代次数
    int sac_nr_samples = 200;                     // 采样点数
    
    // ========== ICP参数 ==========
    float icp_max_correspondence_distance = 0.05f; // 最大对应点距离
    int icp_max_iterations = 100;                  // 最大迭代次数
    float icp_transformation_epsilon = 1e-8f;      // 变换收敛阈值
    float icp_euclidean_fitness_epsilon = 0.01f;   // 欧氏距离收敛阈值
    
    // ========== 质量阈值 ==========
    float min_fitness_score = 0.5f;      // 最小适配度分数
    float max_position_error = 0.01f;     // 最大位置误差 (10mm)
    float max_rotation_error = 15.0f;     // 最大旋转误差 (15度)
};
```

---

### 3.2 CloudPreprocessor - 点云预处理

```cpp
// include/cloud_preprocessor.h

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "pose_registration.h"

namespace esf_database {
namespace registration {

class CloudPreprocessor {
public:
    CloudPreprocessor() = default;
    explicit CloudPreprocessor(const RegistrationConfig& config);
    
    void setConfig(const RegistrationConfig& config);
    const RegistrationConfig& getConfig() const;
    
    // 预处理点云 (降采样 + 离群点移除)
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // 仅体素降采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsample(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // 仅离群点移除
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeOutliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // 中心化 (将点云移至原点)
    pcl::PointCloud<pcl::PointXYZ>::Ptr center(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        Eigen::Vector4f& centroid);

private:
    RegistrationConfig config_;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter_;
};

}}
```

---

### 3.3 FPFHEstimator - FPFH特征提取

```cpp
// include/fpfh_estimator.h

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include "pose_registration.h"

namespace esf_database {
namespace registration {

class FPFHEstimator {
public:
    FPFHEstimator() = default;
    explicit FPFHEstimator(const RegistrationConfig& config);
    
    void setConfig(const RegistrationConfig& config);
    
    // 提取FPFH特征
    // @param cloud 点云
    // @param normals 法线点云 (必须先计算)
    // @return FPFH特征点云 (33维)
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr extract(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const pcl::PointCloud<pcl::PointNormal>::Ptr& normals);
    
    // 提取FPFH特征 (自动计算法线)
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr extractAutoNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    
    // 静态方法：计算法线
    static pcl::PointCloud<pcl::PointNormal>::Ptr computeNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        int k_search = 20);
    
    // 获取FPFH维度
    static constexpr size_t kDimension = 33;

private:
    RegistrationConfig config_;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> normal_estimator_;
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::FPFHSignature33> fpfh_estimator_;
};

}}
```

---

### 3.4 SACIARegistration - SAC-IA粗配准

```cpp
// include/sac_ia_registration.h

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include "pose_registration.h"

namespace esf_database {
namespace registration {

class SACIARegistration {
public:
    SACIARegistration() = default;
    explicit SACIARegistration(const RegistrationConfig& config);
    
    void setConfig(const RegistrationConfig& config);
    
    // 执行SAC-IA粗配准
    // @param source 源点云 (模板)
    // @param target 目标点云 (场景)
    // @param source_fpfh 源FPFH特征
    // @param target_fpfh 目标FPFH特征
    // @return 4x4变换矩阵 (source -> target)
    Eigen::Matrix4f register(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& source_fpfh,
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& target_fpfh);
    
    // 获取适配度分数
    float getFitnessScore() const { return fitness_score_; }
    
    // 获取迭代次数
    int getIterations() const { return iterations_; }

private:
    RegistrationConfig config_;
    float fitness_score_ = 0.0f;
    int iterations_ = 0;
};

}}
```

---

### 3.5 ICPRegistration - ICP精配准

```cpp
// include/icp_registration.h

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "pose_registration.h"

namespace esf_database {
namespace registration {

class ICPRegistration {
public:
    ICPRegistration() = default;
    explicit ICPRegistration(const RegistrationConfig& config);
    
    void setConfig(const RegistrationConfig& config);
    
    // 执行ICP精配准
    // @param source 源点云 (模板)
    // @param target 目标点云 (场景)
    // @param initial_transform 初始变换 (来自SAC-IA)
    // @return 4x4变换矩阵 (source -> target)
    Eigen::Matrix4f register(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
        const Eigen::Matrix4f& initial_transform);
    
    // 获取适配度分数
    float getFitnessScore() const { return fitness_score_; }
    
    // 获取迭代次数
    int getIterations() const { return iterations_; }
    
    // 获取对齐后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr getAlignedCloud() const { return aligned_cloud_; }

private:
    RegistrationConfig config_;
    float fitness_score_ = 0.0f;
    int iterations_ = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud_;
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
};

}}
```

---

### 3.6 PoseEstimator - 位姿估计器 (整合类)

```cpp
// include/pose_estimator.h

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include "pose_registration.h"
#include "cloud_preprocessor.h"
#include "fpfh_estimator.h"
#include "sac_ia_registration.h"
#include "icp_registration.h"

namespace esf_database {
namespace registration {

class PoseEstimator {
public:
    PoseEstimator() = default;
    explicit PoseEstimator(const RegistrationConfig& config);
    
    void setConfig(const RegistrationConfig& config);
    const RegistrationConfig& getConfig() const;
    
    // 加载配置文件
    bool loadConfigFromFile(const std::string& yaml_path);
    
    // 执行完整配准流程
    // @param template_cloud 模板点云 (参考物体)
    // @param scene_cloud 场景点云 (实际采集)
    // @param initial_transform 初始变换 (可选，来自04识别)
    RegistrationResult estimate(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& template_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene_cloud,
        const Eigen::Matrix4f& initial_transform = Eigen::Matrix4f::Identity());
    
    // 仅执行ICP精配准 (已知粗略对齐)
    RegistrationResult refineICP(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& template_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene_cloud,
        const Eigen::Matrix4f& initial_transform);
    
    // 从变换矩阵提取6DoF位姿
    static Pose6DOF extractPose6DOF(const Eigen::Matrix4f& transform);
    
    // 从旋转矩阵提取RPY (处理万向锁)
    static Eigen::Vector3f extractRPY(const Eigen::Matrix3f& rotation);
    
    // 估计配准误差
    static float estimatePositionError(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target);
    
    static float estimateRotationError(float fitness_score);

private:
    RegistrationConfig config_;
    CloudPreprocessor preprocessor_;
    FPFHEstimator fpfh_estimator_;
    SACIARegistration sacia_;
    ICPRegistration icp_;
    
    // 内部处理
    RegistrationResult runPipeline(
        pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud,
        const Eigen::Matrix4f& initial_transform,
        bool run_sacia);
};

}}
```

---

## 4. 配置文件格式

### registration_config.yaml

```yaml
# 05_pose_registration 配置文件
# 版本: v1.0
# 描述: 点云配准参数配置

# ========== 预处理参数 ==========
preprocessing:
  voxel_leaf_size: 0.005    # 体素降采样大小 (米)
  mean_k: 50                 # 离群点移除邻居数
  std_dev_mul: 1.0           # 标准差倍数

# ========== FPFH特征参数 ==========
fpfh:
  normal_k_search: 20         # 法线估计K近邻
  fpfh_k_search: 20          # FPFH K近邻
  radius: 0.05               # 搜索半径 (米)

# ========== SAC-IA粗配准参数 ==========
sac_ia:
  min_sample_distance: 0.05         # 最小采样距离
  max_correspondence_distance: 0.2  # 最大对应点距离
  nr_iterations: 500                # RANSAC迭代次数
  nr_samples: 200                   # 采样点数

# ========== ICP精配准参数 ==========
icp:
  max_correspondence_distance: 0.05 # 最大对应点距离
  max_iterations: 100              # 最大迭代次数
  transformation_epsilon: 1e-8       # 变换收敛阈值
  euclidean_fitness_epsilon: 0.01   # 欧氏距离收敛阈值

# ========== 质量阈值 ==========
quality:
  min_fitness_score: 0.5      # 最小适配度分数
  max_position_error: 0.01   # 最大位置误差 (米)
  max_rotation_error: 15.0   # 最大旋转误差 (度)
```

---

## 5. 使用示例

### 5.1 基本用法

```cpp
#include "pose_estimator.h"
#include <pcl/io/pcd_io.h>

int main() {
    using namespace esf_database::registration;
    
    // 1. 创建配准器并加载配置
    PoseEstimator estimator;
    estimator.loadConfigFromFile("./config/registration_config.yaml");
    
    // 2. 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::io::loadPCDFile<pcl::PointXYZ>("./data/templates/one.pcd", *template_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZ>("./data/scenes/scene.pcd", *scene_cloud);
    
    // 3. 执行配准
    RegistrationResult result = estimator.estimate(template_cloud, scene_cloud);
    
    // 4. 检查结果
    if (result.isValid()) {
        std::cout << "配准成功!\n";
        std::cout << "位置: " << result.pose_6dof.position.transpose() << "\n";
        std::cout << "RPY: " << result.pose_6dof.toDegrees().transpose() << " (度)\n";
        std::cout << "适配度: " << result.fitness_score << "\n";
        
        // 发送给机器人
        sendToRobot(result.transformation);
    } else {
        std::cerr << "配准失败: " << result.message << "\n";
    }
    
    return 0;
}
```

### 5.2 与04模块整合 (06_realtime_app)

```cpp
#include "04_category_recognition/include/esf_recognizer.h"
#include "05_pose_registration/include/pose_estimator.h"

class RecognitionPipeline {
public:
    // 完整识别+配准结果
    struct FullResult {
        RecognitionResult recognition;    // 来自04
        RegistrationResult registration;  // 来自05
        bool isValid() const {
            return recognition.isValid() && registration.isValid();
        }
    };
    
    FullResult process(const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene_cloud) {
        FullResult result;
        
        // Step 1: 类别识别 (04)
        result.recognition = recognizer_.recognize(scene_cloud);
        if (!result.recognition.isValid()) {
            return result;
        }
        
        // Step 2: 获取对应模板点云
        auto template_cloud = loadTemplate(result.recognition.category_id);
        
        // Step 3: 配准定位 (05)
        result.registration = pose_estimator_.estimate(template_cloud, scene_cloud);
        
        return result;
    }

private:
    ESFRecognizer recognizer_;
    PoseEstimator pose_estimator_;
};
```

---

## 6. 关键算法说明

### 6.1 RPY角度提取 (处理万向锁)

```cpp
Eigen::Vector3f PoseEstimator::extractRPY(const Eigen::Matrix3f& R) {
    Eigen::Vector3f rpy;
    
    // 计算 Pitch (Y轴旋转)
    float sp = -R(2, 0);
    sp = std::clamp(sp, -1.0f, 1.0f);
    rpy(1) = std::asin(sp);  // Pitch
    
    // 检查万向锁 (pitch = ±90度)
    if (std::abs(sp) < 0.9999f) {
        rpy(2) = std::atan2(R(1, 0), R(0, 0));  // Yaw
        rpy(0) = std::atan2(R(2, 1), R(2, 2));  // Roll
    } else {
        // 万向锁情况
        rpy(2) = 0.0f;
        rpy(0) = std::atan2(-R(0, 1), R(1, 1));
    }
    
    return rpy;  // [Roll, Pitch, Yaw] 弧度
}
```

### 6.2 配准误差估计

```cpp
// 位置误差估计 (基于对应点平均距离)
float PoseEstimator::estimatePositionError(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& aligned,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& target) {
    
    float total_error = 0.0f;
    size_t valid_count = 0;
    
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(target);
    
    std::vector<int> k_indices(1);
    std::vector<float> k_distances(1);
    
    for (const auto& point : aligned->points) {
        if (kdtree.nearestKSearch(point, 1, k_indices, k_distances) > 0) {
            total_error += std::sqrt(k_distances[0]);
            valid_count++;
        }
    }
    
    return valid_count > 0 ? total_error / valid_count : 1.0f;
}

// 旋转误差估计 (基于适配度分数)
float PoseEstimator::estimateRotationError(float fitness_score) {
    // 适配度分数高 -> 旋转误差小
    // 适配度分数低 -> 旋转误差大
    return (1.0f - fitness_score) * 45.0f;  // 最大误差45度
}
```

---

## 7. VS项目配置

### 7.1 项目属性设置

由于未选择CMake项目，需要手动配置：

1. **平台**: x64
2. **配置类型**: 应用程序(.exe)
3. **C++标准**: C++17

### 7.2 附加包含目录

```
$(ProjectDir)include
$(ProjectDir)..\libs
$(PCL_ROOT)include\pcl-1.14
$(EIGEN_ROOT)include\eigen3
```

### 7.3 附加库目录

```
$(ProjectDir)..\lib
$(PCL_ROOT)lib
$(PCL_ROOT)lib\cmake\pcl\PCLConfig.cmake
```

### 7.4 附加依赖项

```
pcl_common.lib
pcl_features.lib
pcl_filters.lib
pcl_io.lib
pcl_registration.lib
pcl_search.lib
pcl_kdtree.lib
```

---

## 8. 测试数据

### 8.1 测试点云

| 文件 | 描述 | 点数 |
|------|------|------|
| `data/templates/one.pcd` | 模板点云 | ~2000 |
| `data/scenes/scene_aligned.pcd` | 已对齐场景 | ~2000 |
| `data/scenes/scene_rotated.pcd` | 旋转场景 | ~2000 |

### 8.2 测试用例

1. **基本配准**: 模板 vs 已对齐场景
2. **旋转配准**: 模板 vs 旋转场景
3. **噪声测试**: 模板 vs 含噪声场景
4. **部分遮挡**: 模板 vs 部分场景

---

## 9. 迁移自Project1的代码

### 9.1 直接复用

| 函数 | 源文件 | 说明 |
|------|--------|------|
| `voxelGridDownsampling` | main.cpp | 体素降采样 |
| `statisticalOutlierRemoval` | main.cpp | 离群点移除 |
| `normalEstimation` | main.cpp | 法线估计 |
| `computeFPFH` | main.cpp | FPFH特征 |
| `sacIaAlignment` | main.cpp | SAC-IA配准 |
| `icpAlignment` | main.cpp | ICP配准 |
| `extractRPY` | main.cpp | RPY提取 |

### 9.2 需适配修改

| 函数 | 适配说明 |
|------|----------|
| `loadConfig` | 改为YAML格式 |
| `printTransformationMatrix` | 移除(改为日志) |

---

## 10. 版本历史

| 版本 | 日期 | 修改内容 |
|------|------|----------|
| v1.0 | 2026-04-27 | 初始版本 |

---

## 附录A: 变换矩阵含义

```
T_scene_to_template =

  [ R11  R12  R13  Tx ]      旋转矩阵 (3×3) + 平移向量 (3×1)
  [ R21  R22  R23  Ty ]      T 表示: 场景中某点在模板坐标系下的位置
  [ R31  R32  R33  Tz ]      应用: template_point = T * scene_point
  [  0    0    0    1  ]

6DoF位姿:
  position = [Tx, Ty, Tz]     位置 (米)
  rpy = [roll, pitch, yaw]    姿态 (弧度)
```

## 附录B: 坐标系说明

```
相机坐标系 ──────▶ 手眼标定矩阵 ──────▶ 机器人基座坐标系
(camera)         (eye_to_base)         (robot_base)

模板坐标系 ──────▶ 配准矩阵 ──────▶ 场景坐标系
(template)       (T)                    (scene)

最终结果:
  在机器人基座下的抓取位姿 = eye_to_base × T_scene_to_template × 抓取点
```
