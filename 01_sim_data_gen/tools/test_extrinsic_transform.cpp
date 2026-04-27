/**
 * @file test_extrinsic_transform.cpp
 * @brief Extrinsic Transform Module Test
 *
 * Test Content:
 * 1. Pose parameter generation
 * 2. Euler angle to rotation matrix
 * 3. Extrinsic matrix calculation
 * 4. Point cloud transformation
 * 5. Batch transformation
 */

#include "../include/sim_data_gen.h"
#include <iostream>
#include <iomanip>
#include <filesystem>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace fs = std::filesystem;

void printPose(const sim_data_gen::transform::Pose& pose) {
    std::cout << "Pose[" << pose.pose_id << "]: "
              << pose.name
              << " | rx=" << pose.rx << " rad"
              << " ry=" << pose.ry << " rad"
              << " rz=" << pose.rz << " rad"
              << " | t=(" << pose.tx << ", " << pose.ty << ", " << pose.tz << ")"
              << std::endl;
}

void visualizePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                         const std::string& title) {
    pcl::visualization::PCLVisualizer viewer(title);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer.setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer.addCoordinateSystem(0.1);
    viewer.initCameraParameters();

    std::cout << "Press 'q' to close visualization window..." << std::endl;
    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Extrinsic Transform Module Test" << std::endl;
    std::cout << "========================================" << std::endl;

    // Test 1: Read STL file
    std::string stl_path = "d:\\vsxiangmu\\gongjianfenjian\\01_sim_data_gen\\data\\models\\raw\\one.stl";
    std::string pcd_output_dir = "d:\\vsxiangmu\\gongjianfenjian\\01_sim_data_gen\\data\\pointclouds";

    if (!fs::exists(stl_path)) {
        std::cout << "STL file not found: " << stl_path << std::endl;
        return -1;
    }

    std::cout << "\n[Test 1] Read STL file..." << std::endl;
    sim_data_gen::point_cloud::STLReader stl_reader;
    sim_data_gen::point_cloud::SamplingConfig sampling_config;
    sampling_config.enable_centering = true;
    sampling_config.enable_pca_alignment = true;

    auto cloud = stl_reader.read(stl_path, sampling_config);
    if (!cloud) {
        std::cout << "Read failed: " << stl_reader.getLastError() << std::endl;
        return -1;
    }
    std::cout << "Read success, point count: " << cloud->size() << std::endl;

    // Test 2: Create extrinsic transformer
    std::cout << "\n[Test 2] Create extrinsic transformer..." << std::endl;
    sim_data_gen::transform::ExtrinsicConfig config;
    // Pose sampling: X/Y 0-80 step 20, Z 0-360 step 20
    // 5 * 5 * 18 = 450 poses
    config.rx_min = 0.0f;
    config.rx_max = 80.0f;
    config.rx_step = 20.0f;
    config.ry_min = 0.0f;
    config.ry_max = 80.0f;
    config.ry_step = 20.0f;
    config.rz_min = 0.0f;
    config.rz_max = 360.0f;
    config.rz_step = 20.0f;

    sim_data_gen::transform::ExtrinsicTransform transformer(config);
    std::cout << "Pose params: X[0-80/20] Y[0-80/20] Z[0-360/20]" << std::endl;
    std::cout << "Expected pose count: " << transformer.getTotalPoses() << " (should be 450)" << std::endl;

    // Test 3: Generate pose parameters
    std::cout << "\n[Test 3] Generate pose parameters..." << std::endl;
    auto poses = transformer.generatePoses();
    std::cout << "Generated pose count: " << poses.size() << std::endl;

    // Show first 3 and last 3 poses
    std::cout << "\nFirst 3 poses:" << std::endl;
    for (int i = 0; i < 3 && i < poses.size(); ++i) {
        printPose(poses[i]);
    }
    std::cout << "\nLast 3 poses:" << std::endl;
    for (int i = std::max(0, static_cast<int>(poses.size()) - 3);
         i < poses.size(); ++i) {
        printPose(poses[i]);
    }

    // Test 4: Compute extrinsic matrix
    std::cout << "\n[Test 4] Compute extrinsic matrix..." << std::endl;
    if (!poses.empty()) {
        auto transform = transformer.computeExtrinsicMatrix(poses[0]);
        std::cout << "First pose extrinsic matrix:\n" << transform << std::endl;
    }

    // Test 5: Single point cloud transformation
    std::cout << "\n[Test 5] Single point cloud transformation..." << std::endl;
    if (!poses.empty()) {
        auto transformed_cloud = transformer.applyTransform(cloud,
            transformer.computeExtrinsicMatrix(poses[0]));
        std::cout << "Transformed point count: " << transformed_cloud->size() << std::endl;
    }

    // Test 6: Batch transformation (save all 450 poses)
    std::cout << "\n[Test 6] Batch transformation (save all " << poses.size() << " poses)..." << std::endl;
    std::string model_name = "one";
    int saved_count = 0;
    int total_count = static_cast<int>(poses.size());

    for (int i = 0; i < total_count; ++i) {
        auto transformed_cloud = transformer.applyTransform(cloud,
            transformer.computeExtrinsicMatrix(poses[i]));

        // Generate filename: {model_name}_rx{}_ry{}_rz{}.pcd
        std::string filename = model_name + "_" + poses[i].name + ".pcd";
        std::string output_path = pcd_output_dir + "/raw/" + filename;

        // Save point cloud
        pcl::io::savePCDFileASCII(output_path, *transformed_cloud);
        ++saved_count;

        // Show progress every 50 poses
        if ((i + 1) % 50 == 0 || i == total_count - 1) {
            std::cout << "Progress: " << (i + 1) << "/" << total_count
                      << " (" << std::fixed << std::setprecision(1)
                      << (100.0f * (i + 1) / total_count) << "%)"
                      << " - " << filename << std::endl;
        }
    }
    std::cout << "Save completed: " << saved_count << " point cloud files" << std::endl;

    // Test 7: Visualization (optional, uncomment to enable)
    // std::cout << "\n[Test 7] Visualization..." << std::endl;
    // if (!poses.empty()) {
    //     auto transformed_cloud = transformer.applyTransform(cloud,
    //         transformer.computeExtrinsicMatrix(poses[0]));
    //     visualizePointCloud(transformed_cloud, "Transformed Point Cloud");
    // }

    std::cout << "\n========================================" << std::endl;
    std::cout << "Test completed!" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}
