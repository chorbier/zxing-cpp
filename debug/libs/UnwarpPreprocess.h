#include <string>
#include <opencv2/opencv.hpp>

#define DEBUG_DRAW

#ifdef DEBUG_DRAW
void testUnwarpPipeline(const cv::Mat& image, const std::string& baseDebugPath);
#endif

bool cvUnwarpPreprocess(cv::Mat& outImage);