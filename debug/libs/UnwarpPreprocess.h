#include <string>
#include <opencv2/opencv.hpp>

#define DEBUG_DRAW

struct UnwarpParams
{
	int outputSize = 160;
	int offset = 25;
	int outputPointsDist = 4;
	float approxPolyEpsilon = 0.01;
	float pointFilterDistanceThreshold = 6.0;
};

#ifdef DEBUG_DRAW
void testUnwarpPreprocess(const cv::Mat& image, const std::string& baseDebugPath, const UnwarpParams& params);
bool testUnwarpPreprocessPredefined(cv::Mat& outResult, const cv::Mat& imageIn, const std::vector<std::pair<cv::Mat, cv::Mat>>& warps, std::function<bool(const cv::Mat&)> processResult, const std::string& baseDebugPath, const UnwarpParams& params);
#endif

void cvUnwarpPreprocess(cv::Mat& outResult, const cv::Mat& image, const UnwarpParams& params);
bool cvUnwarpPreprocessPredefined(cv::Mat& outResult, const cv::Mat& imageIn, const std::vector<std::pair<cv::Mat, cv::Mat>>& warps, std::function<bool(const cv::Mat&)> processResult, const UnwarpParams& params);
void resizeWarp(const cv::Mat& warpIn, cv::Mat& warpOut, const UnwarpParams& params);
