#include "split3.h"  // 自定义头文件，包含函数的声明

const cv::Vec3b roadColor = {0, 255, 0};  // 定义道路区域颜色为绿色
const cv::Vec3b posColor = {0, 0, 255};   // 定义正斜率区域颜色为红色
const cv::Vec3b negColor = {255, 0, 0};   // 定义负斜率区域颜色为蓝色

// 函数实现，将输入图像按照给定的直线参数 k、b 进行分割，并返回分割结果图像
cv::Mat3b split3(cv::Mat input, double a, double b, double c, double delta)
{
    cv::Mat3b result = cv::Mat3b::zeros(input.size());  // 创建一个和输入图像大小相同的全黑图像作为结果图像
    for (int i = 0; i < input.rows; i++)  // 遍历输入图像的每一行
    {
        for (int j = 0; j < input.cols; j++)  // 遍历输入图像的每一列
        {
            double d_top = (a * i + b * j + c + delta) / std::sqrt(a * a + b * b);      // 计算点(i, j)到直线的上边界距离
            double d_bottom = (a * i + b * j + c - delta) / std::sqrt(a * a + b * b);   // 计算点(i, j)到直线的下边界距离
            uint8_t color = input.at<uint8_t>(i, j);  // 获取输入图像点(i, j)的像素值
            if (color == 0)  // 如果像素值为0，表示该点为背景，则在结果图像中将该点颜色设为黑色
            {
                result.at<cv::Vec3b>(i, j) = {color, color, color};
                continue;
            }
            else if (d_top >= color && d_bottom < color)  // 如果像素值在上下边界距离之间，则认为该点在道路区域，将其颜色设为绿色
            {
                result.at<cv::Vec3b>(i, j) = roadColor;
            }
            else if (d_bottom < color)  // 如果像素值在下边界距离之内，则认为该点在正斜率区域，将其颜色设为红色
            {
                result.at<cv::Vec3b>(i, j) = posColor;
            }
            else if (d_top > color)  // 如果像素值在上边界距离之内，则认为该点在负斜率区域，将其颜色设为蓝色
            {
                result.at<cv::Vec3b>(i, j) = negColor;
            }
        }
    }
    return result;  // 返回分割结果图像
}
