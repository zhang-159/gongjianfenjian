# 05配准模块库封装指南

## 封装目标

将 `D:\vsxiangmu\Project1` 项目封装为静态库/动态库，供 06 实时应用调用。

---

## 一、推荐：封装为静态库 (.lib)

静态库优点：
- 编译时链接，无需分发运行时依赖
- 调试方便，代码直接嵌入
- 适合 Windows + PCL 项目

---

## 二、需要导出的接口

### 2.1 核心类导出

| 类名 | 功能 | 用途 |
|-----|------|-----|
| `CloudPreprocessor` | 点云预处理（降采样、去噪、中心化） | 配准前预处理 |
| `FPFHEstimator` | FPFH特征提取（33维） | 粗配准特征 |
| `SACIARegistration` | SAC-IA粗配准 | 获取初始变换 |
| `ICPRegistration` | ICP精配准 | 精确位姿 |
| `PoseEstimator` | 完整配准流程 | 核心调用入口 |

### 2.2 数据结构导出

```cpp
// 位姿结构
struct Pose6DOF {
    Eigen::Vector3f position;    // 位置 (米)
    Eigen::Vector3f rpy;       // 翻滚、俯仰、偏航 (弧度)
};

// 配准配置
struct RegistrationConfig {
    float voxel_leaf_size = 0.005f;
    int normal_k_search = 20;
    float fpfh_radius = 0.05f;
    int sac_nr_iterations = 500;
    int icp_max_iterations = 100;
    // ... 其他参数
};

// 配准结果
struct RegistrationResult {
    Eigen::Matrix4f transformation;  // 4x4变换矩阵
    Pose6DOF pose_6dof;              // 6DoF位姿
    float fitness_score;             // 适配度分数
    bool success;                    // 是否成功
    std::string message;             // 状态信息
};
```

---

## 三、API 接口设计

### 3.1 预处理接口

```cpp
class CloudPreprocessor {
public:
    void setConfig(const RegistrationConfig& config);
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
};
```

### 3.2 FPFH特征提取

```cpp
class FPFHEstimator {
public:
    void setConfig(const RegistrationConfig& config);

    // 自动计算法线并提取FPFH
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr extractAutoNormals(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    // 使用预计算的法线
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr extract(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
        const pcl::PointCloud<pcl::PointNormal>::Ptr& normals);

    static constexpr size_t kDimension = 33;  // FPFH维度
};
```

### 3.3 SAC-IA粗配准

```cpp
class SACIARegistration {
public:
    void setConfig(const RegistrationConfig& config);

    Eigen::Matrix4f registerPointClouds(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,      // 场景点云
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,      // 模板点云
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& source_fpfh,
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr& target_fpfh);

    float getFitnessScore() const;
    int getIterations() const;
};
```

### 3.4 ICP精配准

```cpp
class ICPRegistration {
public:
    void setConfig(const RegistrationConfig& config);

    Eigen::Matrix4f registerPointClouds(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& source,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& target,
        const Eigen::Matrix4f& initial_transform);   // SAC-IA结果

    float getFitnessScore() const;
    int getIterations() const;
    pcl::PointCloud<pcl::PointXYZ>::Ptr getAlignedCloud() const;
};
```

### 3.5 完整配准流程（推荐入口）

```cpp
class PoseEstimator {
public:
    PoseEstimator() = default;
    explicit PoseEstimator(const RegistrationConfig& config);
    void setConfig(const RegistrationConfig& config);

    // 一键配准
    RegistrationResult estimatePose(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene_cloud,   // 场景点云
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& template_cloud // 模板点云
    );

    // 手动分步配准
    RegistrationResult estimatePoseWithSteps(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& scene_cloud,
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& template_cloud,
        bool& sacia_success,
        bool& icp_success);

    // 获取中间结果
    Eigen::Matrix4f getSACIaTransform() const;
    Eigen::Matrix4f getICPFTransform() const;
};
```

---

## 四、调用示例

### 4.1 简单调用（一键配准）

```cpp
#include "pose_estimator.h"

// 加载配置
RegistrationConfig config;
config.voxel_leaf_size = 0.005f;
config.fpfh_radius = 0.05f;

// 创建配准器
PoseEstimator estimator(config);

// 加载模板点云（根据04识别的类别ID）
auto template_cloud = loadTemplatePointCloud(category_id);

// 场景点云
auto scene_cloud = captureScenePointCloud();

// 配准
auto result = estimator.estimatePose(scene_cloud, template_cloud);

if (result.success) {
    std::cout << "位姿: " << result.pose_6dof.toString() << std::endl;
    std::cout << "位置误差: " << result.fitness_score << std::endl;
}
```

### 4.2 分步调用（精细控制）

```cpp
bool sacia_ok, icp_ok;
auto result = estimator.estimatePoseWithSteps(
    scene_cloud, template_cloud, sacia_ok, icp_ok);

if (!sacia_ok) {
    std::cout << "SAC-IA失败" << std::endl;
}
if (!icp_ok) {
    std::cout << "ICP失败" << std::endl;
}
```

---

## 五、依赖项

### 5.1 必须的外部库

| 库 | 版本 | 用途 |
|----|-----|-----|
| PCL | 1.14.0 | 点云处理 |
| Eigen | 3.4+ | 线性代数 |

### 5.2 头文件

```cpp
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>
#include <Eigen/Dense>
```

---

## 六、导出宏定义

```cpp
// pose_registration_export.h
#ifdef POSE_REGISTRATION_EXPORTS
    #define POSE_REGISTRATION_API __declspec(dllexport)
#else
    #define POSE_REGISTRATION_API __declspec(dllimport)
#endif
```

类声明：

```cpp
class POSE_REGISTRATION_API PoseEstimator {
    // ...
};
```

---

## 七、文件结构建议

```
pose_registration_lib/
├── include/
│   ├── pose_registration_export.h    // 导出宏
│   ├── pose_registration_types.h     // 数据结构
│   ├── cloud_preprocessor.h
│   ├── fpfh_estimator.h
│   ├── sac_ia_registration.h
│   ├── icp_registration.h
│   └── pose_estimator.h              // 主入口
├── src/
│   ├── cloud_preprocessor.cpp
│   ├── fpfh_estimator.cpp
│   ├── sac_ia_registration.cpp
│   ├── icp_registration.cpp
│   └── pose_estimator.cpp
├── CMakeLists.txt
└── README.md
```

---

## 八、配置参数说明

| 参数 | 默认值 | 说明 |
|-----|-------|-----|
| `voxel_leaf_size` | 0.005m | 体素降采样大小 |
| `normal_k_search` | 20 | 法线估计邻居数 |
| `fpfh_radius` | 0.05m | FPFH搜索半径 |
| `sac_nr_iterations` | 500 | SAC-IA迭代次数 |
| `icp_max_iterations` | 100 | ICP最大迭代 |
| `icp_max_correspondence_distance` | 0.05m | ICP对应点距离阈值 |
| `min_fitness_score` | 0.5 | 成功配准的最小分数 |

---

## 九、封装步骤清单

- [ ] 1. 在 `D:\vsxiangmu\Project1` 中创建 `pose_registration_export.h`
- [ ] 2. 将数据结构提取到 `pose_registration_types.h`
- [ ] 3. 在每个导出类前添加 `POSE_REGISTRATION_API` 宏
- [ ] 4. 配置 CMake/VS 项目为静态库 (.lib)
- [ ] 5. 添加 PCL 和 Eigen 依赖
- [ ] 6. 编译生成 `pose_registration.lib`
- [ ] 7. 在 06 实时应用中链接该库
- [ ] 8. 测试与 04 模块的联合调用

---

## 十、与04模块的集成

```
04_category_recognition (FAISS检索)
         │
         ▼ 输出: category_id, template_id
         │
         ▼
05_pose_registration_lib (配准)
         │
         ▼ 输入: category_id → 加载模板点云
         │
         ▼ 输入: scene_cloud (相机采集)
         │
         ▼ 输出: Pose6DOF (位姿)
```
