# data/ 目录说明

本目录用于存放工件点云数据、特征文件、指纹库和日志等运行时数据。

**注意：本目录的文件不在 Git 仓库中管理**（见 `.gitignore`），请在本地或专用存储中保留。

## 子目录结构

| 子目录 | 说明 |
|---|---|
| `models/` | 工件原始 STL/STEP 3D 模型 |
| `pointclouds/` | 仿真或采集的点云（PCD 格式） |
| `esf_features/` | 提取的 ESF 特征向量（640 维） |
| `cvfh_features/` | 提取的 OUR-CVFH 特征向量（345 维） |
| `fingerprints/` | FAISS 指纹库（`*.bin` + 类别映射 `*_ids.txt`） |
| `logs/` | 运行日志 |
| `11/` | 类别 11 的样本点云 |

## 数据生成流程

1. 仿真数据生成：运行 `01_sim_data_gen` 产出 `pointclouds/`
2. 特征提取：运行 `02_feature_extraction` 产出 `esf_features/` 和 `cvfh_features/`
3. 数据库构建：运行 `03_database_build` 产出 `fingerprints/`
4. 类别判定：运行 `04_category_recognition` 进行查询
5. 配准定位：运行 `05_pose_registration` 进行位姿估计
6. 实时应用：运行 `06_realtime_app` 进行端到端识别与定位

## 示例数据

仓库不包含实际点云/特征文件。如需测试：

1. 在 `data/pointclouds/raw/` 下放置若干 PCD 文件
2. 在 `data/models/` 下放置对应的 STL 模型
3. 运行 `02_feature_extraction` 提取特征
4. 运行 `03_database_build` 构建指纹库
