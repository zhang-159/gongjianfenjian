# 04类别判定模块开发计划

## 1. 目标概述

实现基于ESF特征的类别判定模块，用于从点云中识别工件类别。

## 2. 当前状态

| 项目 | 状态 |
|-----|------|
| ESF特征库 | ✅ 已构建 (1个工件，2个姿态) |
| FAISS索引 | ✅ 已实现 (`FaissIndex`类) |
| 04模块 | ❌ 未创建 |

## 3. 目录结构

```
04_category_recognition/
├── config/
│   └── recognition_config.yaml    # 识别配置
├── include/
│   ├── category_recognition.h     # 模块总头文件
│   ├── esf_recognizer.h          # ESF识别器声明
│   └── cvfh_recognizer.h         # CVFH识别器声明（预留）
├── src/
│   ├── esf_recognizer.cpp         # ESF识别器实现
│   ├── cvfh_recognizer.cpp        # CVFH识别器实现（预留）
│   ├── main.cpp                   # 测试程序
│   └── CMakeLists.txt             # 构建文件
└── 04_category_recognition.md     # 本文档
```

> **备注：** 当前版本实现ESF单库识别，后续将扩展为 **双特征库（ESF + CVFH）融合**，包括：
> - CVFH特征识别器
> - 自适应特征库选择
> - 双库融合算法
> - 难度评估机制

## 4. 开发步骤

---

### 步骤1：创建目录结构和配置文件

**目标：** 创建模块骨架和配置文件

**文件清单：**
- `config/recognition_config.yaml` - 识别配置

**配置内容：**
```yaml
recognition:
  # 置信度阈值
  confidence_threshold: 1.5    # Top-1/Top-2 > 此值则信任ESF结果

  # 特征库路径
  esf_index_path: "../data/fingerprints/esf_index.bin"
  esf_ids_path: "../data/fingerprints/esf_ids.txt"

  # 检索参数
  top_k: 5                     # Top-K检索数量
```

**交付物：** 配置文件

---

### 步骤2：定义识别结果结构

**目标：** 定义识别结果的通用数据结构

**文件：** `include/category_recognition.h`

**代码：**
```cpp
namespace esf_database {
namespace recognition {

// 识别结果结构
struct RecognitionResult {
    int category_id;              // 类别ID，-1表示不确定
    std::string category_name;    // 类别名称
    float confidence;             // 置信度
    std::string feature_type;     // 特征类型："ESF"
    
    // 调试信息
    std::vector<int> topk_ids;     // Top-K 类别ID
    std::vector<float> topk_scores; // Top-K 置信度
};

} // namespace recognition
} // namespace esf_database
```

**交付物：** `category_recognition.h`

---

### 步骤3：实现ESF识别器类

**目标：** 实现 `ESFRecognizer` 类，封装FAISS检索逻辑

**文件：** 
- `include/esf_recognizer.h`
- `src/esf_recognizer.cpp`

**核心功能：**

```cpp
class ESFRecognizer {
public:
    // 加载FAISS索引和ID映射
    bool loadIndex(const std::string& index_path, 
                   const std::string& ids_path);
    
    // 提取ESF特征（调用PCL）
    std::vector<float> extractESF(const PointCloudXYZPtr& cloud);
    
    // 执行识别
    RecognitionResult recognize(const PointCloudXYZPtr& cloud);
    
private:
    // 解析ID映射文件
    bool loadIdMapping(const std::string& path);
    
    // 计算置信度
    float computeConfidence(const std::vector<float>& distances);
    
    // FAISS索引
    std::unique_ptr<database::FaissIndex> faiss_index_;
    
    // ID映射: vector_id -> {object_id, pose_id}
    std::vector<IdMapping> id_mapping_;
};
```

**交付物：** `esf_recognizer.h` 和 `esf_recognizer.cpp`

---

### 步骤4：创建CMakeLists.txt

**目标：** 配置项目构建

**文件：** `src/CMakeLists.txt`

**内容：**
- 链接PCL库
- 链接FAISS库
- 设置输出目录

---

### 步骤5：编写测试程序

**目标：** 验证识别功能

**文件：** `src/main.cpp`

**测试流程：**
```
1. 加载测试点云
2. 调用 ESFRecognizer
3. 输出识别结果
```

---

## 5. 验收标准

| 步骤 | 验收条件 |
|-----|---------|
| 步骤1 | 配置文件可正常读取 |
| 步骤2 | 结构体定义编译通过 |
| 步骤3 | 加载索引成功，检索返回正确结果 |
| 步骤4 | cmake编译无错误 |
| 步骤5 | 测试程序输出正确的类别ID |

## 6. 后续扩展（双库融合）

当前仅实现ESF单库识别器。后续将扩展为双特征库架构：

| 步骤 | 内容 | 说明 |
|-----|------|-----|
| 6 | CVFH识别器 | `cvfh_recognizer.h/cpp` |
| 7 | 自适应选择策略 | 难度评估 + 动态阈值 |
| 8 | 双库融合 | 加权融合 ESF(0.4) + CVFH(0.6) |

> **双库融合架构：**
> ```
> 场景点云 → ESF检索 ─┬─→ 简单样本 → 返回ESF结果
>                     │
>                     └─→ 困难样本 → CVFH检索 ─┬─→ 更困难 → 双库融合
>                                              └─→ 返回CVFH结果
> ```

---

## 7. 开发进度

| 步骤 | 状态 | 完成日期 |
|-----|------|---------|
| 步骤1 | ✅ 完成 | 2026-04-27 |
| 步骤2 | ✅ 完成 | 2026-04-27 |
| 步骤3 | ⏳ 待开发 | - |
| 步骤4 | ⏳ 待开发 | - |
| 步骤5 | ⏳ 待开发 | - |
| 后续6-8 | ⏳ 待开发 | - |
| ⚠️ 备注 | 03的build_faiss_index.cpp的main函数已注释，避免冲突 | 2026-04-27 |
