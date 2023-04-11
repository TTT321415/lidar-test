#include <iostream>
#include <fstream>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include "projectPointCloud.h"
#include "histogram.h"
#include "hist_filter.h"
#include "ransac.h"
#include "split3.h"

#include <chrono>

#define SHOW

pcl::PointCloud<pcl::PointXYZI>::Ptr readBinPointCloud(const std::string &filename)
{
    std::ifstream ifs(filename, std::ios::in | std::ios::binary); // 以二进制模式打开文件流，从文件中读取数据
    // if (!ifs.good())
    // {
    //     // error
    // }
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>); // 创建点云对象指针
    while (!ifs.eof()) // 循环读取文件中的点云数据
    {
        float data[4]; // 存储点云数据的数组
        pcl::PointXYZI point; // 存储单个点的对象
        ifs.read((char *)&data, sizeof(float) * 4); // 从文件流中读取点云数据
        point.x = data[0]; // 将读取的数据赋值给点的x坐标
        point.y = data[1]; // 将读取的数据赋值给点的y坐标
        point.z = data[2]; // 将读取的数据赋值给点的z坐标
        result->push_back(point); // 将点添加到点云对象中
    }
    ifs.close(); // 关闭文件流
    return result; // 返回读取到的点云对象指针
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorfulPointCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr laserCloudIn, const LidarArgs &lidarArgs, const cv::Mat3b &colorMat)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result(new pcl::PointCloud<pcl::PointXYZRGB>);
    float verticalAngle, horizonAngle, range; // 声明垂直角度、水平角度和距离变量
    size_t rowIdn, columnIdn, index, cloudSize; // 声明行索引、列索引、索引和点云大小变量
    pcl::PointXYZRGB thisPoint; // 创建一个PointXYZRGB类型的点

    cloudSize = laserCloudIn->points.size(); // 获取输入点云的大小

    for (size_t i = 0; i < cloudSize; ++i) // 遍历输入点云中的每个点
    {
        thisPoint.x = laserCloudIn->points[i].x; // 获取点的x坐标
        thisPoint.y = laserCloudIn->points[i].y; // 获取点的y坐标
        thisPoint.z = laserCloudIn->points[i].z; // 获取点的z坐标

        // 计算点的垂直角度
        verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
        rowIdn = (verticalAngle + lidarArgs.angBottom) / lidarArgs.angResY; // 计算点所在的行索引

        if (rowIdn < 0 || rowIdn >= lidarArgs.nScan) // 如果行索引越界，则跳过该点
            continue;

        // 计算点的水平角度
        horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
        columnIdn = -round((horizonAngle - 90.0) / lidarArgs.angResX) + lidarArgs.horizonScan / 2; // 计算点所在的列索引
        if (columnIdn >= lidarArgs.horizonScan) // 如果列索引越界，则进行修正
            columnIdn -= lidarArgs.horizonScan;

        if (columnIdn < 0 || columnIdn >= lidarArgs.horizonScan) // 如果列索引越界，则跳过该点
            continue;

        range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z); // 计算点的距离
        if (range < sensorMinimumRange) // 如果点的距离小于传感器的最小测量范围，则跳过该点
            continue;

        auto color = colorMat.at<cv::Vec3b>(lidarArgs.nScan - 1 - rowIdn, columnIdn); // 获取颜色矩阵中对应位置的颜色
        thisPoint.r = color[2]; // 设置点的红色通道值
        thisPoint.g = color[1]; // 设置点的绿色通道值
        thisPoint.b = color[0]; // 设置点的蓝色通道值
        
        result->push_back(thisPoint);
    }
    return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readKitti(const std::string &filename)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc(new pcl::PointCloud<pcl::PointXYZI>); // 创建一个空的PointXYZI点云对象

    auto pc_ori = readBinPointCloud(filename); // 从二进制文件读取原始点云数据
    Eigen::Matrix4f Tr; // 创建一个4x4的浮点型变换矩阵Tr
    Tr << 4.276802385584e-04, -9.999672484946e-01, -8.084491683471e-03, 0, // 设置Tr的值
        -7.210626507497e-03, 8.081198471645e-03, -9.999413164504e-01, 0,
        9.999738645903e-01, 4.859485810390e-04, -7.206933692422e-03, 0,
        0, 0, 0, 1;
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity(); // 创建一个单位矩阵
    transform_2.rotate(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitY())); // 在Y轴上旋转90度
    Tr *= transform_2.matrix(); // 将旋转矩阵与Tr相乘，更新Tr的值
    pcl::transformPointCloud(*pc_ori, *pc, Tr); // 将原始点云数据按照Tr进行刚体变换，得到变换后的点云数据
    return pc; // 返回变换后的点云数据
}


int main(int, char **)
{
    // 读取KITTI数据点云
    auto pc = readKitti("../004000.bin");
    
    // 获取Lidar参数
    auto lidarArg = lidarArgsMap.at("HDL-64E");
    
    // 投影点云到图像平面
    cv::Mat disp = projectPointCloud(pc, lidarArg, [](double q) { return q < M_PI_4 && q > -M_PI_4; });
    
    // 计算投影点云在图像平面的UV直方图
    cv::Mat1s uHist, vHist;
    uv_histogram(disp, uHist, vHist);
    
    // 进行点云过滤
    std::vector<cv::Point3i> pList;
    auto filter = lidarArg.nScan - lidarArg.angBottom / lidarArg.angResY;
    blank_filter(vHist, pList, 5, filter);
    
    // 使用RANSAC算法拟合
    double a, b, c; // 曲线参数
    ransac(pList, a, b, c, 1000,2.0); // 使用RANSAC算法拟合曲线，得到曲线参数a, b, c
    
    // 在图像上绘制RANSAC拟合
    cv::Mat3b result = split3(disp, a, b, c,2.0); // 调用split3函数，传入图像和曲线参数，得到分割结果图像
    
#ifdef SHOW
    // 可视化展示结果
    cv::imshow("lidar", disp);
    
    // 绘制UV直方图
    cv::Mat vShow = vHist.clone();
    vShow.col(0).setTo(0);
    cv::normalize(vShow, vShow, 255, 0, cv::NORM_MINMAX);
    vShow.convertTo(vShow, CV_8U);
    
    cv::Mat3b lineShow;  // 用于显示的图像
    cv::resize(vShow, lineShow, cv::Size(), 4, 4, cv::INTER_NEAREST);  // 调整图像大小

    std::vector<cv::Point> curvePoints;  // 用于存储曲线上的点坐标

    // 计算曲线上的点坐标
    for (int x = 0; x < lineShow.cols; x++)
    {
     int y = static_cast<int>(a * x * x + b * x + c);  // 计算二次曲线上的 y 坐标
     curvePoints.emplace_back(x * 4, lineShow.rows - y * 4);  // 添加点坐标到 vector 中
    }

    // 绘制曲线
    std::vector<std::vector<cv::Point>> curves = {curvePoints};  // 将点坐标放入 vector<vector<Point>> 中
    cv::polylines(lineShow, curves, false, cv::Scalar(255), 2);  // 绘制曲线

    cv::imshow("ransac-result", lineShow);  // 显示绘制好的曲线图像

    
    // 展示最终结果
    cv::imshow("result", result);
    pcl::visualization::CloudViewer viewer("pointcloud of result");
    viewer.showCloud(colorfulPointCloud(pc, lidarArg, result));
    
    cv::waitKey();
    while (!viewer.wasStopped())
        ;
#endif
}
