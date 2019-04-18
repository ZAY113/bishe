#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

// 读点云文件函数
pcl::PointCloud<pcl::PointXYZ>::Ptr read_cloud_point(std::string const &file_path){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud) == -1)
    {
        PCL_ERROR("Couldn't read the pcd file\n");
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

    //  启动可视化窗口
    viewer_final->addCoordinateSystem(1.0, "global");
    viewer_final->initCameraParameters();

    // 等待直到可视化窗口关闭
    while (!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    }
}

int main(int argc, char** argv)
{
    // 创建两个共享指针，分别指向输入点云和输出点云。每个点的类型在pcl命名空间中设置为PointXYZ。
    auto target_cloud = read_cloud_point(argv[1]);
    std::cout << "Loaded " << target_cloud->size() << " data points from cloud1.pcd" << std::endl;

    auto input_cloud = read_cloud_point(argv[2]);
    std::cout << "Loaded " << input_cloud->size() << " data points from cloud2.pcd" << std::endl;

    // 创建一个ICP实例，并为其提供输入点云与目标点云
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(input_cloud);
    icp.setInputTarget(target_cloud);
    // 创建输出点云对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    // 生成icp配准对象的模型
    icp.align(*output_cloud);
    // 输出收敛信息和算法本次的评分
    std::cout << "Iterative_closest_point has converged:" << icp.hasConverged() << 
            " score: " << icp.getFitnessScore() << std::endl;
    
    // 对输入点云进行变换，并将结果保存在cloud3.pcd
    pcl::transformPointCloud(*input_cloud, *output_cloud, icp.getFinalTransformation());
    pcl::io::savePCDFileASCII("cloud3.pcd", *output_cloud);

    // 将目标点云和配准点云可视化
    visualizer(target_cloud, output_cloud);

    return (0);

}