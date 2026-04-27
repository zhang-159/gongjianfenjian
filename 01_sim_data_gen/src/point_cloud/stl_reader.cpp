#include "stl_reader.h"
#include "../utils/types.h"

#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <fstream>
#include <set>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include "extrinsic_transform.h"
#include "../../../libs/common_types.h"

namespace sim_data_gen {
namespace point_cloud {

// 内部辅助结构体和函数
namespace {

// 点结构体，用于去重比较
struct PointXYZComparator {
    float x, y, z;

    bool operator<(const PointXYZComparator& other) const {
        const float epsilon = 1e-6f;
        if (std::abs(x - other.x) > epsilon) return x < other.x;
        if (std::abs(y - other.y) > epsilon) return y < other.y;
        return z < other.z;
    }
};

pcl::PointXYZ toPCLPoint(const PointXYZComparator& p) {
    return pcl::PointXYZ(p.x, p.y, p.z);
}

// 计算三角面面积
float computeTriangleArea(const pcl::PointXYZ& v0, const pcl::PointXYZ& v1, const pcl::PointXYZ& v2) {
    float v0v1_x = v1.x - v0.x, v0v1_y = v1.y - v0.y, v0v1_z = v1.z - v0.z;
    float v0v2_x = v2.x - v0.x, v0v2_y = v2.y - v0.y, v0v2_z = v2.z - v0.z;

    float cross_x = v0v1_y * v0v2_z - v0v1_z * v0v2_y;
    float cross_y = v0v1_z * v0v2_x - v0v1_x * v0v2_z;
    float cross_z = v0v1_x * v0v2_y - v0v1_y * v0v2_x;

    return 0.5f * std::sqrt(cross_x * cross_x + cross_y * cross_y + cross_z * cross_z);
}

// 边缘采样
void sampleTriangleEdges(const pcl::PointXYZ& v0, const pcl::PointXYZ& v1, const pcl::PointXYZ& v2,
                        int samples_per_edge, std::vector<pcl::PointXYZ>& sampled_points) {
    for (int i = 1; i < samples_per_edge; ++i) {
        float t = static_cast<float>(i) / samples_per_edge;

        pcl::PointXYZ p1, p2, p3;
        p1.x = v0.x + t * (v1.x - v0.x); p1.y = v0.y + t * (v1.y - v0.y); p1.z = v0.z + t * (v1.z - v0.z);
        p2.x = v1.x + t * (v2.x - v1.x); p2.y = v1.y + t * (v2.y - v1.y); p2.z = v1.z + t * (v2.z - v1.z);
        p3.x = v2.x + t * (v0.x - v2.x); p3.y = v2.y + t * (v0.y - v2.y); p3.z = v2.z + t * (v0.z - v2.z);

        sampled_points.push_back(p1);
        sampled_points.push_back(p2);
        sampled_points.push_back(p3);
    }
}

// 内部采样（重心坐标随机采样）
void sampleTriangleInterior(const pcl::PointXYZ& v0, const pcl::PointXYZ& v1, const pcl::PointXYZ& v2,
                           int samples_for_triangle, std::vector<pcl::PointXYZ>& sampled_points) {
    for (int i = 0; i < samples_for_triangle; ++i) {
        float u = static_cast<float>(rand()) / RAND_MAX;
        float v = static_cast<float>(rand()) / RAND_MAX;

        if (u + v > 1.0f) {
            u = 1.0f - u;
            v = 1.0f - v;
        }

        float w = 1.0f - u - v;
        pcl::PointXYZ point;
        point.x = u * v0.x + v * v1.x + w * v2.x;
        point.y = u * v0.y + v * v1.y + w * v2.y;
        point.z = u * v0.z + v * v1.z + w * v2.z;
        sampled_points.push_back(point);
    }
}

// 基于面积的混合采样
pcl::PointCloud<pcl::PointXYZ>::Ptr sampleMeshHybridAreaBased(
    const pcl::PolygonMesh& mesh,
    int samples_per_edge,
    float points_per_mm2) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(mesh.cloud, *vertices);

    std::set<PointXYZComparator> all_sampled_points;

    // 1. 添加所有原始顶点
    for (const auto& vertex : vertices->points) {
        PointXYZComparator p{vertex.x, vertex.y, vertex.z};
        all_sampled_points.insert(p);
    }

    // 2. 对每个三角面进行混合采样
    for (const auto& polygon : mesh.polygons) {
        if (polygon.vertices.size() < 3) continue;

        pcl::PointXYZ v0 = vertices->points[polygon.vertices[0]];
        pcl::PointXYZ v1 = vertices->points[polygon.vertices[1]];
        pcl::PointXYZ v2 = vertices->points[polygon.vertices[2]];

        // 边缘采样
        std::vector<pcl::PointXYZ> edge_points;
        sampleTriangleEdges(v0, v1, v2, samples_per_edge, edge_points);
        for (const auto& p : edge_points) {
            all_sampled_points.insert(PointXYZComparator{p.x, p.y, p.z});
        }

        // 内部采样（基于面积）
        float area = computeTriangleArea(v0, v1, v2);
        int samples_for_triangle = static_cast<int>(area * points_per_mm2);
        if (samples_for_triangle < 1) samples_for_triangle = 1;

        std::vector<pcl::PointXYZ> interior_points;
        sampleTriangleInterior(v0, v1, v2, samples_for_triangle, interior_points);
        for (const auto& p : interior_points) {
            all_sampled_points.insert(PointXYZComparator{p.x, p.y, p.z});
        }
    }

    cloud->width = all_sampled_points.size();
    cloud->height = 1;
    cloud->is_dense = false;
    cloud->points.resize(all_sampled_points.size());

    size_t idx = 0;
    for (const auto& p : all_sampled_points) {
        cloud->points[idx++] = toPCLPoint(p);
    }

    return cloud;
}

// 计算点云质心
pcl::PointXYZ computeCentroid(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointXYZ centroid(0.0f, 0.0f, 0.0f);
    if (cloud->empty()) return centroid;

    for (const auto& point : *cloud) {
        centroid.x += point.x;
        centroid.y += point.y;
        centroid.z += point.z;
    }

    size_t count = cloud->size();
    centroid.x /= count;
    centroid.y /= count;
    centroid.z /= count;
    return centroid;
}

// 单位标准化：检测并转换
void standardizeUnit(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty()) return;

    float max_abs = 0.0f;
    for (const auto& point : *cloud) {
        max_abs = std::max(max_abs, std::abs(point.x));
        max_abs = std::max(max_abs, std::abs(point.y));
        max_abs = std::max(max_abs, std::abs(point.z));
    }

    // 如果坐标普遍小于1，认为是米，转换为毫米
    if (max_abs < 1.0f) {
        for (auto& point : *cloud) {
            point.x *= 1000.0f;
            point.y *= 1000.0f;
            point.z *= 1000.0f;
        }
    }
}

// 中心化：移到原点
void centerPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    pcl::PointXYZ centroid = computeCentroid(cloud);
    for (auto& point : *cloud) {
        point.x -= centroid.x;
        point.y -= centroid.y;
        point.z -= centroid.z;
    }
}

// PCA对齐：旋转点云使主轴与XYZ轴对齐
void alignWithPCA(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) {
    if (cloud->empty() || cloud->size() < 3) return;

    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();

    // 构建变换矩阵：旋转使得特征向量与坐标轴对齐
    // 特征向量按列存储，需要转置才能将点云旋转到标准坐标系
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    transform.block<3, 3>(0, 0) = eigenvectors.transpose();

    pcl::transformPointCloud(*cloud, *cloud, transform);
}

}  // namespace

STLReader::STLReader() {
    // 初始化随机数种子
    srand(static_cast<unsigned int>(time(nullptr)));
}

PointCloudXYZPtr STLReader::read(const std::string& file_path, const SamplingConfig& config) {
    SamplingConfig cfg = config;
    cfg.target_points = 0;  // 完整采样
    return readWithSamplingImpl(file_path, cfg);
}

PointCloudXYZPtr STLReader::readWithTargetPoints(
    const std::string& file_path,
    int target_points,
    const SamplingConfig& config) {

    SamplingConfig cfg = config;
    cfg.target_points = target_points;
    return readWithSamplingImpl(file_path, cfg);
}

PointCloudXYZPtr STLReader::readWithSamplingImpl(const std::string& file_path, const SamplingConfig& config) {
    if (!fileExists(file_path)) {
        last_error_ = "文件不存在: " + file_path;
        return nullptr;
    }

    std::string ext = getExtension(file_path);

    // 读取STL/PLY/OBJ文件
    pcl::PolygonMesh mesh;
    int result = 0;

    if (ext == "stl") {
        result = pcl::io::loadPolygonFileSTL(file_path, mesh);
    } else if (ext == "ply") {
        result = pcl::io::loadPLYFile(file_path, mesh);
    } else if (ext == "obj") {
        result = pcl::io::loadPolygonFileOBJ(file_path, mesh);
    } else {
        last_error_ = "不支持的文件格式: " + ext;
        return nullptr;
    }

    if (result == 0) {
        last_error_ = "读取文件失败: " + file_path;
        return nullptr;
    }

    // 基于面积的混合采样
    auto cloud = sampleMeshHybridAreaBased(mesh, config.samples_per_edge, config.points_per_mm2);

    if (cloud->empty()) {
        last_error_ = "采样点云失败: " + file_path;
        return nullptr;
    }

    // 单位标准化（可选）
    if (config.enable_unit_standardization) {
        standardizeUnit(cloud);
    }

    // 中心化（可选）
    if (config.enable_centering) {
        centerPointCloud(cloud);
    }

    // PCA对齐（可选）
    if (config.enable_pca_alignment) {
        alignWithPCA(cloud);
    }

    // 如果设置了目标点数，进行降采样
    if (config.target_points > 0 && static_cast<int>(cloud->size()) > config.target_points) {
        auto downsampled_cloud = std::make_shared<PointCloudXYZ>();
        float step = static_cast<float>(cloud->size()) / config.target_points;

        for (float i = 0; i < cloud->size(); i += step) {
            int index = static_cast<int>(i);
            downsampled_cloud->points.push_back(cloud->points[index]);
        }

        downsampled_cloud->width = static_cast<uint32_t>(downsampled_cloud->points.size());
        downsampled_cloud->height = 1;
        downsampled_cloud->is_dense = true;

        last_error_.clear();
        return downsampled_cloud;
    }

    last_error_.clear();
    return cloud;
}

bool STLReader::fileExists(const std::string& file_path) {
    std::ifstream file(file_path);
    return file.good();
}

std::string STLReader::getExtension(const std::string& file_path) {
    size_t pos = file_path.find_last_of('.');
    if (pos == std::string::npos || pos == file_path.length() - 1) {
        return "";
    }

    std::string ext = file_path.substr(pos + 1);

    // 转换为小写
    for (auto& c : ext) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c + 32);
        }
    }

    return ext;
}

const std::string& STLReader::getLastError() const {
    return last_error_;
}

}  // namespace point_cloud
}  // namespace sim_data_gen
