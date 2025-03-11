#include <string>
#include <opencv2/opencv.hpp>

void testUnwarpPipeline(const cv::Mat& image, const std::string& baseDebugPath);

bool cvUnwarpPreprocess(cv::Mat& outImage);