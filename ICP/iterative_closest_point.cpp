#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

int main(int argc, char** argv)
{
    // 创建两个共享指针，分别指向输入点云和输出点云。每个点的类型在pcl命名空间中设置为PointXYZ。
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

    // 填入输入点云数据。随机数生成的数据，设置合适的参数。并输出点的数量和值。
    cloud_in->width = 5;
    cloud_in->height = 1;
    cloud_in->is_dense = false;
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);        
    }
    std::cout << "Saved " << cloud_in->points.size() << " data points to input:"
        << std::endl;
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        std::cout << "      " << cloud_in->points[i].x << " " << cloud_in->points[i].y
            << " " << cloud_in->points[i].z << std::endl;
    }
    *cloud_out = *cloud_in;
    std::cout << "size: " << cloud_out->points.size() << std::endl;
    // 在点云上执行简单的刚性变换，并再次输出数据值
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
    }
    std::cout << "Transformed " << cloud_in->points.size() << " data points:" << std::endl;
    for (size_t i = 0; i < cloud_out->points.size(); ++i)
    {
        std::cout << "      " << cloud_out->points[i].x << " " << cloud_out->points[i].y
            << " " << cloud_out->points[i].z << std::endl;
    }
    // 创建一个ICP实例，并为其提供输入点云与目标点云
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud_in);
    icp.setInputTarget(cloud_out);
    // 创建Final保存配准后的结果，并且如果对齐好，则可以收敛，最后输出适应度分数。
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    std::cout << "has converged:" << icp.hasConverged() << " score: " << 
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    return (0);

}