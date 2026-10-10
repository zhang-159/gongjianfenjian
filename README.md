# 工件分拣 — 点云特征检索项目

基于 PCL 1.14 + FAISS 的工件点云识别 / 检索工程。当前实现 **ESF 单特征库** 流水线。
本项目面向大规模平板类工件的分类问题，不走2D图像的识别（四千个类别训练极难收敛，以及数据集采集问题），也不采用常规3D点云匹配（类别太多，无法预知来料配方），提取平板工件的特征描述子作为分类标签，FAISS索引，以余弦相似度或者L2距离作为分类判别依据。
> 文档说明：本仓库当前**只有一个**说明文档（本文件），仅描述已实现的内容。
> 之前那些 03/04/05/06 各模块的"开发计划" / "技术方案" / "双特征库构建方案"等 MD 已全部删除，因为其中描述的多数功能（CVFH 识别器、自适应选择、双库融合、05/06 模块）目前**尚未实现**，留着会造成误导。

---

## 1. 已实现的模块

```
gongjianfenjian/
├── 01_sim_data_gen/        # 仿真数据生成（STL 读取 + 外参变换）
├── 02_feature_extraction/  # 特征提取（ESF + CVFH 估计器，PCL 实现）
├── 03_database_build/      # FAISS 索引构建工具
├── 04_category_recognition/# 类别判定（ESF + FAISS Top-K 检索）
├── libs/                   # 公共头
├── data/                   # 共享数据
└── README.md               # 本文档（唯一说明）
```

> 仓库根还有 `gongjianfenjian.sln` / `.vcxproj` —— 整个工程是 **VS2022 + MSVC + C++17** 单一解决方案。

---

## 2. 各模块当前状态

| 模块 | 当前实现 | 状态 |
|---|---|---|
| **01** `sim_data_gen` | `STLReader` + `ExtrinsicTransform`（4×4 旋转+平移矩阵生成） | ✅ 已实现 |
| **02** `feature_extraction` | `ESFEstimator`(640 维) + `CVFHEstimator`(308 维) | ✅ 已实现 |
| **03** `database_build` | `FaissIndex` 封装类（train / add / search / save / load / nprobe），以及一个**被整体注释掉的** `build_faiss_index.cpp` 入口 | ⚠️ 部分实现（无运行入口） |
| **04** `category_recognition` | `ESFRecognizer`：加载 ESF 索引 + ID 映射 + Top-K 检索，置信度 = `Top-1/Top-2` 距离比 | ✅ 已实现 |
| **05** `pose_registration` | **未创建** | ❌ |
| **06** `realtime_app` | **未创建** | ❌ |
| **双特征库融合** | `adaptive_selector` / `dual_fusion` / `difficulty_estimator` / `feature_comparator` 全部未实现 | ❌ |

> **重要：** `04_category_recognition` 中**只有 ESF 单库**识别器，没有 CVFH 识别器、没有自适应选择、没有融合。如果以后真的要做双库融合，需要补 `cvfh_recognizer.{h,cpp}` 和自适应选择相关文件。

---

## 3. 编译与运行

### 3.1 依赖

| 组件 | 版本 | 说明 |
|---|---|---|
| MSVC | VS2022 | C++17 |
| PCL | 1.14.0 | 点云 |
| Eigen | 3.4+ | 线性代数 |
| FAISS | latest | 索引 / 检索 |
| CMake | 3.20+ | 备用（当前仓库用 .sln/.vcxproj） |

### 3.2 打开解决方案

```
gongjianfenjian.sln
```

直接 VS2022 打开、生成、运行 `04_category_recognition` 项目即可做一次端到端 ESF 识别测试。

### 3.3 04 模块的运行路径（硬编码于 `04_category_recognition/src/main.cpp`）

```
索引:    D:/vsxiangmu/gongjianfenjian/data/fingerprints/esf_index.bin
ID 映射: D:/vsxiangmu/gongjianfenjian/data/fingerprints/esf_ids.txt
测试点云: D:/vsxiangmu/gongjianfenjian/data/pointclouds/raw/one_rx0_ry0_rz0.pcd
```

索引不存在时，main 会报错并提示先跑 03 工具构建索引。**注意** `03_database_build/src/build_faiss_index.cpp` 里的 `main` 是被注释掉的，需要先取消注释再编译才能用。

---

## 4. 命名空间

```cpp
namespace esf_database {
    namespace point_cloud { /* 01: STLReader / ExtrinsicTransform */ }
    namespace features     { /* 02: ESFEstimator / CVFHEstimator */ }
    namespace database     { /* 03: FaissIndex / SearchResult */ }
    namespace recognition  { /* 04: ESFRecognizer / RecognitionResult */ }
}
```

`02_feature_extraction` 例外，它使用 `namespace feature_extraction`。

---

## 5. 数据目录

```
data/
├── models/                 # STL 原始模型（git ignore）
├── pointclouds/            # 仿真点云 PCD（git ignore）
├── esf_features/           # .esf 二进制特征向量（git ignore）
├── cvfh_features/          # .cvfh 二进制特征向量（git ignore）
├── fingerprints/           # FAISS 索引 .bin + ID 映射 .txt（git ignore）
└── logs/                   # 运行日志（git ignore）
```

每个子目录留有 `.gitkeep` 占位。运行时数据全部由 01/02/03 程序重新生成，不入版本库。

---

## 6. 当前未实现但代码里出现的接口

- `04_category_recognition/include/category_recognition.h` 的 `RecognitionConfig` 里**预留了** `cvfh_index_path` / `cvfh_ids_path` 字段，**但没有任何代码使用它们**。
- `03_database_build/src/faiss_index.h` 同时支持 640 维和 308 维，**但当前只跑了 ESF 640 维**。
- `01_sim_data_gen/include/sim_data_gen.h` / `02_feature_extraction/src/utils/types.h` 提供基础类型别名（`PointCloudXYZPtr` 等），是 01/02/03/04 共用的基础层。

后续如果要扩到双库融合 / 配准 / 实时应用，建议在 04 现有结构上补：
```
04_category_recognition/src/
├── cvfh_recognizer.{h,cpp}        # CVFH 检索
├── adaptive_selector.{h,cpp}      # 自适应选择
├── dual_fusion.{h,cpp}            # 双库融合
├── difficulty_estimator.{h,cpp}   # 难度评估
└── main.cpp                       # 已有
```
