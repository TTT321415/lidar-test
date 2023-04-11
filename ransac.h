#ifndef RANSAC_H
#define RANSAC_H

#include <vector>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <opencv2/opencv.hpp>

// RANSAC 算法实现函数声明
void ransac(const std::vector<cv::Point3i> &points, double &a, double &b, double &c, size_t times, double delta);

#endif // RANSAC_H
