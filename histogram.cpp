#include "histogram.h" // 包含头文件 "histogram.h"，用于函数声明
using namespace cv; // 使用OpenCV命名空间

void uv_histogram(const Mat &img, Mat1s &u, Mat1s &v)
{
    u = Mat1s::zeros(256, img.cols); // 创建一个256行、img的列数列的Mat1s类型矩阵u，并初始化为零
    v = Mat1s::zeros(img.rows, 256); // 创建一个img的行数行、256列的Mat1s类型矩阵v，并初始化为零
    const size_t size = img.rows * img.cols; // 计算img的像素总数
    for (int i = 0; i < img.rows; i++) // 遍历img的行
    {
        for (int j = 0; j < img.cols; j++) // 遍历img的列
        {
            int p = img.at<uint8_t>(i, j); // 获取img的当前行i和列j处的像素值
            u.at<short>(p, j)++; // 在矩阵u的第p行、第j列处的像素值加一
            v.at<short>(i, p)++; // 在矩阵v的第i行、第p列处的像素值加一
        }
    }
    return; // 返回空
}
