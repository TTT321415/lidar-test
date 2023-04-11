#include <vector> // 包含vector头文件，用于定义向量容器
#include <opencv2/opencv.hpp> // 包含OpenCV头文件，用于图像处理

void blank_filter(const cv::Mat1s &hist_v, std::vector<cv::Point3i> &list, int threshold, int horizon_filter = 0);
// 定义函数blank_filter，接受一个cv::Mat1s类型的输入参数hist_v，一个存储cv::Point3i类型对象的向量list的引用参数，一个int类型的阈值参数threshold，和一个默认值为0的int类型的horizon_filter参数
