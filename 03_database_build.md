# 03_database_build 数据库构建项目

## 概述

将特征提取阶段生成的 ESF 和 CVFH 特征向量，构建为 FAISS 指纹库，实现快速检索。

---

## 阶段一：创建项目基础结构

### 目标
创建空的项目框架，验证项目能编译通过。

### 目录结构

```
03_database_build/
├── src/
│   ├── build_faiss_index.cpp    # 主程序入口
│   ├── faiss_index.h           # FAISS索引封装
│   └── faiss_index.cpp
├── config/
│   └── build_config.yaml       # 构建参数配置
└── CMakeLists.txt (可选)
```

### 任务
- [ ] 创建目录结构
- [ ] 创建空的主程序文件（只含 main 函数，无逻辑）
- [ ] 创建空的 FAISS 封装类头文件
- [ ] 创建空的 FAISS 封装类实现文件
- [ ] 将项目添加到解决方案

---

## 阶段二：封装 FAISS 索引类

### 目标
实现 FAISS 索引的封装，支持 ESF(640维) 和 CVFH(345维) 两种特征库。

### 核心功能

| 方法 | 说明 |
|-----|-----|
| `buildIVF(dim, nlist)` | 构建 IndexIVFFlat 索引 |
| `train(data, count)` | 训练索引 |
| `add(data, ids)` | 添加向量到索引 |
| `search(query, k)` | Top-K 检索 |
| `save(path)` | 保存索引到文件 |
| `load(path)` | 从文件加载索引 |

### 索引规格

| 特征库 | 维度 | 推荐 nlist |
|-----|-----|----------|
| ESF | 640 | 4096 |
| OUR-CVFH | 345 | 4096 |

### 任务
- [ ] 实现 FaissIndex 类的完整功能
- [ ] 实现向量读取辅助函数（读取 .esf 和 .cvfh 文件）
- [ ] 添加错误处理

---

## 阶段三：实现主程序

### 目标
从特征文件批量读取向量，构建索引并保存。

### 数据流程

```
特征文件 (*.esf, *.cvfh)
        ↓
批量读取向量
        ↓
构建 FAISS 索引 (IndexIVFFlat)
        ↓
保存索引文件 (*.bin)
```

### 输出文件

| 文件 | 说明 |
|-----|-----|
| `data/fingerprints/esf_index.bin` | ESF 指纹库 |
| `data/fingerprints/cvfh_index.bin` | CVFH 指纹库 |
| `data/fingerprints/esf_ids.txt` | ESF 向量ID映射 |
| `data/fingerprints/cvfh_ids.txt` | CVFH 向量ID映射 |

### 命令行参数

| 参数 | 说明 |
|-----|-----|
| 无参数 | 构建所有索引 |
| `--esf-only` | 仅构建 ESF 索引 |
| `--cvfh-only` | 仅构建 CVFH 索引 |
| `--rebuild` | 强制重建（忽略已存在的索引） |
| `--help` | 显示帮助 |

### 任务
- [ ] 实现特征文件读取函数
- [ ] 实现索引构建逻辑
- [ ] 实现主程序入口
- [ ] 添加进度显示和统计信息

---

## 阶段四：添加配置管理

### 目标
将关键参数抽离到配置文件，支持灵活调整。

### build_config.yaml 示例

```yaml
database_build:
  # 特征维度
  esf_dimension: 640
  cvfh_dimension: 345

  # 索引参数
  faiss:
    nlist: 4096        # IVF 列表数
    nprobe: 64         # 搜索时探查的列表数
    metric: "L2"       # 距离度量: L2 或 IP(内积)

  # 数据路径
  paths:
    esf_input: "data/esf_features/train"
    cvfh_input: "data/cvfh_features/train"
    esf_output: "data/fingerprints/esf_index.bin"
    cvfh_output: "data/fingerprints/cvfh_index.bin"
    esf_ids_output: "data/fingerprints/esf_ids.txt"
    cvfh_ids_output: "data/fingerprints/cvfh_ids.txt"
```

### 任务
- [ ] 创建 build_config.yaml
- [ ] 实现配置加载类
- [ ] 将主程序中的硬编码路径替换为配置项

---

## 完成标准

1. 阶段一：项目能编译通过，无链接错误
2. 阶段二：FaissIndex 类功能完整，可独立测试
3. 阶段三：能成功构建索引，生成 .bin 文件
4. 阶段四：参数可通过配置文件调整

---

## 依赖关系

- PCL 1.14.0（特征文件读取）
- FAISS（索引构建）
- yaml-cpp（配置解析）
