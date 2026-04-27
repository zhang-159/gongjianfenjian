# 方案一：06实时应用项目设计

## 一、项目概述

**项目名称**: `06_realtime_app`

**项目定位**: 整合04（类别判定）和05（配准定位），实现完整的实时工件识别与位姿估计流程。

**开发环境**: Windows + VS2022 + PCL 1.14.0

---

## 二、项目结构

```
06_realtime_app/
│
├── src/
│   ├── main.cpp                    # 主程序入口
│   │
│   ├── pipeline/                   # 识别流程管道
│   │   ├── recognition_pipeline.h/cpp    # 完整识别流程
│   │   └── result_handler.h/cpp          # 结果处理
│   │
│   ├── recognition/                # 【来自04】类别判定模块
│   │   ├── adaptive_selector.h/cpp       # 自适应特征库选择
│   │   ├── dual_fusion.h/cpp            # 双库融合
│   │   ├── difficulty_estimator.h/cpp   # 难度评估
│   │   └── feature_comparator.h/cpp     # 特征比对
│   │
│   ├── registration/               # 【来自05】配准定位模块
│   │   ├── fpfh_estimator.h/cpp        # FPFH特征提取
│   │   ├── sac_ia_registration.h/cpp     # SAC-IA粗配准
│   │   ├── icp_registration.h/cpp        # ICP精配准
│   │   └── pose_estimator.h/cpp         # 位姿估计器
│   │
│   ├── preprocessing/              # 点云预处理
│   │   ├── cloud_preprocessor.h/cpp     # 预处理主类
│   │   ├── downsampler.h/cpp           # 降采样
│   │   ├── outlier_filter.h/cpp         # 离群点过滤
│   │   └── crop_box.h/cpp              # 裁剪
│   │
│   ├── database/                   # 数据库接口
│   │   ├── db_connector.h/cpp           # 数据库连接器
│   │   ├── esf_index.h/cpp              # ESF索引
│   │   └── cvfh_index.h/cpp            # CVFH索引
│   │
│   ├── camera/                    # 相机接口
│   │   ├── camera_interface.h/cpp        # 相机抽象接口
│   │   └── mekamon_camera.h/cpp         # 梅卡曼德相机实现
│   │
│   └── common/                     # 公共模块
│       ├── types.h                     # 类型定义
│       ├── config.h/cpp                # 配置管理
│       └── logger.h/cpp                # 日志系统
│
├── config/
│   ├── recognition_config.yaml          # 识别参数配置
│   ├── registration_config.yaml         # 配准参数配置
│   └── camera_config.yaml              # 相机配置
│
├── include/
│   └── realtime_app.h                  # 对外头文件
│
├── data/                          # 运行时数据
│   ├── templates/                  # 工件模板点云
│   └── logs/                       # 运行日志
│
└── CMakeLists.txt                 # 构建配置
```

---

## 三、核心模块设计

### 3.1 识别流程管道（RecognitionPipeline）

```cpp
// recognition_pipeline.h
class RecognitionPipeline {
public:
    // 初始化
    bool initialize(const std::string& config_path);
    
    // 完整识别流程
    RecognitionResult recognize(const PointCloud& input_cloud);
    
    // 单步操作（用于调试）
    CategoryResult classifyCategory(const PointCloud& cloud);
    PoseResult estimatePose(const PointCloud& cloud, int category_id);

private:
    // 04的模块
    std::unique_ptr<AdaptiveSelector> adaptive_selector_;
    
    // 05的模块
    std::unique_ptr<FPFHEstimator> fpfh_estimator_;
    std::unique_ptr<SACIARegistration> sac_ia_;
    std::unique_ptr<ICPRegistration> icp_;
    
    // 数据库
    std::unique_ptr<ESFIndex> esf_index_;
    std::unique_ptr<CVFHIndex> cvfh_index_;
};
```

### 3.2 整合流程示意

```
┌──────────────────────────────────────────────────────────────────────┐
│                    RecognitionPipeline 完整流程                         │
├──────────────────────────────────────────────────────────────────────┤
│                                                                      │
│  输入: 现场相机点云                                                    │
│       │                                                             │
│       ▼                                                             │
│  ┌─────────────┐                                                    │
│  │  点云预处理   │  ← 来自05的预处理模块                                │
│  │  降采样/滤波  │                                                    │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │  聚类分割   │  ← 从现场点云分割出单个工件                          │
│  └──────┬──────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐     ┌─────────────┐     ┌─────────────┐            │
│  │  类别判定   │ ──> │  类别判定   │ ──> │  类别判定   │            │
│  │  (04模块)  │     │  自适应选择  │     │  输出类别ID │            │
│  └──────┬─────┘     └─────────────┘     └─────────────┘            │
│         │                                   │                        │
│         │  类别ID + 模板点云                 │                        │
│         ▼                                   ▼                        │
│  ┌─────────────┐                      ┌─────────────┐                │
│  │  FPFH特征   │                      │ 模板点云加载 │                │
│  └──────┬─────┘                      └──────┬──────┘                │
│         │                                   │                        │
│         ▼                                   ▼                        │
│  ┌─────────────┐                      ┌─────────────┐                │
│  │  SAC-IA    │                      │  FPFH特征   │                │
│  │  粗配准    │ ─────────────────────>│  (模板)    │                │
│  └──────┬─────┘                      └─────────────┘                │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │  ICP精配准  │  ← 来自05的配准模块                                 │
│  └──────┬─────┘                                                    │
│         │                                                           │
│         ▼                                                           │
│  ┌─────────────┐                                                    │
│  │  6DoF位姿   │                                                    │
│  │  输出       │                                                    │
│  └─────────────┘                                                    │
│                                                                      │
└──────────────────────────────────────────────────────────────────────┘
```

---

## 四、04与05的整合方式

### 4.1 为什么必须在同一进程

```
场景: 实时工件抓取系统

问题: 如果04和05分成两个独立进程
┌─────────────┐      IPC通信       ┌─────────────┐
│  04进程     │  ─────────────>   │  05进程     │
│  类别判定   │   (序列化/反序列化) │  配准定位   │
└─────────────┘                    └─────────────┘

问题:
1. 点云数据量大 (~1MB/帧)，IPC开销大
2. 延迟增加 (进程切换 + 序列化)
3. 实时性无法保证 (<100ms要求)
```

### 4.2 整合后的数据结构

```cpp
// types.h

// 识别结果
struct RecognitionResult {
    int category_id;              // 类别ID
    float confidence;             // 置信度
    Eigen::Matrix4f pose;          // 6DoF位姿
    float position_error;          // 位置误差(mm)
    float rotation_error;          // 旋转误差(度)
    std::string timestamp;         // 时间戳
};

// 类别判定结果 (04的输出)
struct CategoryResult {
    int category_id;
    float confidence;
    std::string feature_library;   // "ESF" / "CVFH" / "FUSED"
};

// 配准结果 (05的输出)
struct PoseResult {
    Eigen::Matrix4f transform;
    float fitness_score;           // 配准适配度
    int iterations;               // 迭代次数
};
```

### 4.3 代码整合示例

```cpp
// recognition_pipeline.cpp

RecognitionResult RecognitionPipeline::recognize(
    const PointCloud& input_cloud) {
    
    // ========== Step 1: 类别判定 (来自04) ==========
    auto category_result = classifyCategory(input_cloud);
    
    // ========== Step 2: 获取模板点云 ==========
    PointCloud template_cloud = loadTemplate(category_result.category_id);
    
    // ========== Step 3: 配准定位 (来自05) ==========
    auto pose_result = estimatePose(input_cloud, template_cloud);
    
    // ========== Step 4: 组合结果 ==========
    return RecognitionResult {
        .category_id = category_result.category_id,
        .confidence = category_result.confidence,
        .pose = pose_result.transform,
        .position_error = pose_result.fitness_score * 10.0f,  // 估算
        .rotation_error = calculateRotationError(pose_result.transform),
        .timestamp = getCurrentTimestamp()
    };
}
```

---

## 五、参数配置

### 5.1 识别参数 (recognition_config.yaml)

```yaml
recognition:
  # 自适应选择策略
  adaptive:
    difficulty_threshold: 3.0      # 难度阈值
    score_threshold_high: 1.5      # 高置信度阈值
    score_threshold_low: 1.1      # 低置信度阈值
    top_k: 5                       # Top-K检索数
    
  # 双库融合
  fusion:
    esf_weight: 0.4
    cvfh_weight: 0.6
    
  # 检索参数
  retrieval:
    esf_max_results: 5
    cvfh_max_results: 5
```

### 5.2 配准参数 (registration_config.yaml)

```yaml
registration:
  # FPFH特征
  fpfh:
    radius: 0.05                  # 搜索半径(m)
    
  # SAC-IA粗配准
  sac_ia:
    max_iterations: 500
    sample_size: 4
    max_correspondence_distance: 0.05
    transformation_epsilon: 1e-6
    
  # ICP精配准
  icp:
    max_iterations: 50
    max_correspondence_distance: 0.02
    transformation_epsilon: 1e-8
    euclidean_fitness_epsilon: 1e-6
```

---

## 六、开发阶段划分

```
Phase 6.1: 项目框架搭建
├── 创建VS项目结构
├── 配置PCL 1.14.0
├── 集成公共库
└── 创建配置框架

Phase 6.2: 04模块整合
├── 导入AdaptiveSelector
├── 导入ESF/CVFH索引
├── 配置数据库连接
└── 测试类别判定流程

Phase 6.3: 05模块整合
├── 导入FPFH Estimator
├── 导入SAC-IA Registration
├── 导入ICP Registration
└── 测试配准流程

Phase 6.4: 端到端集成
├── 整合04+05流程
├── 参数调优
├── 性能优化
└── 完整测试

Phase 6.5: 部署验证
├── 相机接口集成
├── 实时性能测试
└── 抓取任务验证
```

---

## 七、依赖关系

```
06_realtime_app
│
├── [外部依赖]
│   ├── PCL 1.14.0          # 点云处理
│   ├── Eigen              # 矩阵运算
│   └── FAISS              # 向量检索
│
├── [04项目模块]           # 通过源码引入
│   ├── adaptive_selector
│   ├── esf_index
│   ├── cvfh_index
│   └── dual_fusion
│
├── [05项目模块]           # 通过源码引入
│   ├── fpfh_estimator
│   ├── sac_ia_registration
│   └── icp_registration
│
└── [公共库]               # 通过库文件引入
    ├── common_types
    ├── logger
    └── config_parser
```

---

## 八、验证计划

### 验证项目

| 验证项 | 方法 | 验收标准 |
|--------|------|----------|
| 类别判定准确率 | 50-100个测试样本 | Top1 > 85% |
| 配准精度 | 位置误差测量 | < 5mm |
| 实时性 | 端到端延迟统计 | < 100ms |
| 稳定性 | 连续运行测试 | 24小时无崩溃 |

### 测试用例设计

```cpp
// test_recognition.cpp

void testCategoryRecognition() {
    // 加载测试数据集
    // 运行类别判定
    // 统计准确率
}

void testPoseEstimation() {
    // 准备测试点云对
    // 运行配准
    // 测量误差
}

void testEndToEnd() {
    // 模拟相机输入
    // 完整pipeline运行
    // 统计延迟
}
```

---

*文档版本: v1.0*
*创建日期: 2026-04-09*
*关联项目: 方案一流程进度*