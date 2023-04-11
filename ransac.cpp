#include <cstdlib>  // 标准库，包含随机数生成函数
#include <cstdint>  // 标准库，包含整数类型定义
#include "ransac.h"  // 自定义头文件，包含 RANSAC 算法的声明
using namespace std;  // 使用 std 命名空间
using namespace cv;   // 使用 OpenCV 命名空间

// RANSAC 算法实现函数
void ransac(const vector<Point3i> &points, double &a, double &b, double &c, size_t times, double delta)
{
    double min_error = UINT32_MAX;  // 最小误差初始化为最大无符号整数值
    for (size_t i = 0; i < times; i++)  // 迭代指定的次数
    {
        int m = rand() % points.size();  // 随机选择三个点的索引
        int n = rand() % points.size();
        int p = rand() % points.size();
        if (m == n || m == p || n == p)  // 三个点不应该重复
            continue;
        
        // 通过选取的三个点，计算二次曲线的系数 a, b, c
        double x1 = points[m].x;
        double x2 = points[n].x;
        double x3 = points[p].x;
        double y1 = points[m].y;
        double y2 = points[n].y;
        double y3 = points[p].y;
        
        double D = x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2);
        double A = ((x1 * x1 + y1 * y1) * (y2 - y3) + (x2 * x2 + y2 * y2) * (y3 - y1) + (x3 * x3 + y3 * y3) * (y1 - y2)) / (2 * D);
        double B = ((x1 * x1 + y1 * y1) * (x3 - x2) + (x2 * x2 + y2 * y2) * (x1 - x3) + (x3 * x3 + y3 * y3) * (x2 - x1)) / (2 * D);
        double C = ((x1 * x1 + y1 * y1) * (x2 * y3 - x3 * y2) + (x2 * x2 + y2 * y2) * (x3 * y1 - x1 * y3) + (x3 * x3 + y3 * y3) * (x1 * y2 - x2 * y1)) / (2 * D);
        
        if (!isnormal(A) || !isnormal(B) || !isnormal(C))  // 排除系数为非正常数（例如 NaN、Inf）的情况
            continue;
        double error = 0;  // 初始化误差为0
        for (auto &p : points)  // 遍历所有点
        {
            double x = p.x;
            double y = p.y;
            // 计算当前点到拟合曲线的距离
            double distance = abs(A * x * x + B * x + C - y);
            // 累加误差
            error += distance;
        }

        if (error < min_error)  // 如果当前误差更小，则更新最小误差和拟合曲线的系数
        {
            min_error = error;
            a = A;
            b = B;
            c = C;
        }
    }
}

