#ifndef SPLIT3_H
#define SPLIT3_H

#include <opencv2/opencv.hpp>


// 函数声明，将输入图像按照给定的直线参数 k、b 进行分割，并返回分割结果图像
cv::Mat3b split3(cv::Mat input, double a, double b, double c, double delta);

#endif // SPLIT3_H
