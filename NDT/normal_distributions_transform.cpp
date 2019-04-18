#include <iostream>
#include <pcl/io/pcd_io.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>

// 读点云文件函数
pcl::PointCloud<pcl::PointXYZ>::Ptr read_cloud_point(std::string const &file_path){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ> (file_path, *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read the pcd file\n");
        return nullptr;
    }
    return cloud;
}

// 可视化点云函数
void visualizer(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud){
    // 初始化可视化对象
    boost::shared_ptr<pcl::visualization::PCLVisualizer> 
            viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // 设置背景颜色为黑色
    viewer_final->setBackgroundColor(0, 0, 0);
    
    // 上色和可视化目标点云（红色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            target_color (target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (target_cloud, target_color, "target cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

    // 上色和可视化配准后的输入点云（绿色）
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
            output_color (output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ> (output_cloud, output_color, "output cloud");
    viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output cloud");

    // 启动可视化窗口
    viewer_final->addCoordinateSystem (1.0, "global");
    viewer_final->initCameraParameters();

    // 等待直到可视化窗口关闭
    while (!viewer_final->wasStopped ())
    {
        viewer_final->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

int main(int argc, char **argv) {
    // 读取输入点云文件和目标点云文件
    auto target_cloud = read_cloud_point(argv[1]);
    std::cout << "Loaded " << target_cloud->size () << " data points from cloud1.pcd" << std::endl;

    auto input_cloud = read_cloud_point(argv[2]);
    std::cout << "Loaded " << input_cloud->size () << " data points from cloud2.pcd" << std::endl;

    // 运用近似体元网格来对输入点云进行滤波处理，将数据降低到原数据量的10%，以加快生成NDT配准模型的速度
    // 创建cloud对象存储过滤后的点云数据
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // 创建网格过滤器
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    // 配置体元的叶片尺寸
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    // 设置输入点云
    approximate_voxel_filter.setInputCloud(input_cloud);
    // 进行滤波处理并存入cloud对象
    approximate_voxel_filter.filter(*filtered_cloud);

    // 输出过滤后的点云数据量
    std::cout << "Filtered cloud contains " << filtered_cloud->size()
    << " data points from cloud2.pcd" << std::endl;

    // NDT配准的准备工作
    // 创建配准对象
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    // 设置epsilon参数（判断是否收敛的阈值）
    ndt.setTransformationEpsilon(0.01);
    // 设置牛顿法优化的最大步长
    ndt.setStepSize(0.1);
    // 设置网格化时立方体的边长
    ndt.setResolution(1.0);

    // 设置最大迭代次数35次
    ndt.setMaximumIterations(35);
    // 设置ndt配准的输入源
    ndt.setInputSource(filtered_cloud);
    // 设置配准的目标
    ndt.setInputTarget(target_cloud);

    // 初始化变换参数
    // 初始化旋转参数角轴
    Eigen::AngleAxisf init_rotation(0.6931, Eigen::Vector3f::UnitZ());
    // 初始化平移参数
    Eigen::Translation3f init_translation(1.79387, 0.720047, 0);
    // 组合成猜想的变化模式
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

    // 创建输出点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // 生成ndt配准对象的模型
    ndt.align(*output_cloud, init_guess);
    // 输出收敛信息和算法本次的评分
    std::cout << "Normal Distribution Transform has converged:" << ndt.hasConverged()
            << " score: " << ndt.getFitnessScore() << std::endl;

    // 对原始输入点云进行变换，并将结果保存在cloud3.pcd
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
    pcl::io::savePCDFileASCII("cloud3.pcd", *output_cloud);

    // 将目标点云和配准点云可视化
    visualizer(target_cloud, output_cloud);
    

    return 0;
}
