# examples/

本目录用于存放**少量示例点云文件（PCD）**，方便新协作者快速了解数据格式和功能用法。

> ⚠️ `examples/` 下的 PCD 文件**会被 Git 追踪**。不要把所有运行时数据都堆到这里——大批量数据请放在仓库外（本地磁盘 / 数据库 / 对象存储），用 `data/pointclouds/`、`data/fingerprints/` 等目录生成。

## 推荐内容

- 1～2 个代表性点云（一个简单几何体，一个真实工件）
- 每个不超过 10000 点
- 单个 PCD 文件控制在 100KB 以内

## 命名规范

| 类型 | 命名 | 示例 |
|---|---|---|
| 简单几何体 | `<shape>_example.pcd` | `cube_example.pcd`, `sphere_example.pcd` |
| 工件 | `<object>_<id>.pcd` | `bracket_001.pcd`, `gear_002.pcd` |

## 添加步骤

```powershell
# 1. 把你的 PCD 放到本目录下
Copy-Item D:\my_pcds\bracket_001.pcd D:\vsxiangmu\gongjianfenjian\examples\

# 2. 由于根目录 .gitignore 忽略了 *.pcd，用强制添加
cd D:\vsxiangmu\gongjianfenjian
git add -f examples/bracket_001.pcd

# 3. 提交并推送
git commit -m "docs: 添加 bracket_001 示例点云"
git push
```

## 读取示例（C++ / PCL）

```cpp
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
if (pcl::io::loadPCDFile<pcl::PointXYZ>("examples/cube_example.pcd", *cloud) == -1) {
    PCL_ERROR("读取示例点云失败\n");
    return -1;
}
std::cout << "载入 " << cloud->size() << " 个点" << std::endl;
```
