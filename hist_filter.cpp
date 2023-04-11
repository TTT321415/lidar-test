#include "hist_filter.h" // 包含头文件 "hist_filter.h"，用于函数声明
using namespace std; // 使用标准命名空间
using namespace cv; // 使用OpenCV命名空间

void blank_filter(const Mat1s &hist_v, vector<Point3i> &list, int threshold, int horizon_filter)
{
    for (int i = horizon_filter; i < hist_v.rows; i++) // 遍历输入矩阵hist_v的行，从horizon_filter开始
    {
        for (int j = 1; j < hist_v.cols; j++) // 遍历输入矩阵hist_v的列，从1开始
        {
            if (hist_v.at<short>(i, j) > threshold) // 检查hist_v的当前行i和列j处的值是否大于阈值
            {
                list.push_back(Point3i(j, i, hist_v.at<int16_t>(i, j))); // 如果条件成立，将(j, i)和hist_v(i, j)处的值作为Point3i对象添加到向量list中
            }
        }
    }
    return; // 返回空
}
