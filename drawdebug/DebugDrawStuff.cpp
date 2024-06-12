#include "opencv2/opencv.hpp"
#include "DebugDrawStuff.h"
#include "ZXing/BitMatrix.h"
#include "ZXing/Point.h"

#include <filesystem>
namespace fs = std::filesystem;

namespace ZXing {
static std::vector<uint8_t> imageData;

cv::Mat BitMatrixToMat(const BitMatrix& image) {
	
	imageData.resize(image.width() * image.height() * 3);

	for(int y = image.height(); y--;) {
		for(int x = image.width(); x--;) {
			if(image.get(x, y)) {
				std::memset(&imageData[(y * image.width() + x) * 3], 255, sizeof(uint8_t) * 3);
			} else {
				std::memset(&imageData[(y * image.width() + x) * 3], 0, sizeof(uint8_t) * 3);
			}
		}
	}

	// cv::Mat img(image.width(), image.height(), CV_8UC3, imageData.data(), true);

	cv::Mat img(image.width(), image.height(), CV_8UC3, imageData.data());
	return img;
}

void drawDebugImage(const BitMatrix& image, const std::string& filename) {

	cv::Mat img = BitMatrixToMat(image);

// cv::Mat A(100, 100, CV_64F, x);

	// fs::path newFilename = debugOutputFilepath.stem();
	// newFilename += fs::path(*postfix);
	// newFilename += debugOutputFilepath.extension();
	// newFilename = debugOutputFolder / newFilename;
	// // std::string FILENAME = newFilename;
	cv::imwrite(debugOutputFolder / filename, img);
}

void drawDebugImageWithLines(const BitMatrix& image, const std::string& filename, const std::vector<double>& corners) {

	cv::Mat img = BitMatrixToMat(image);
// cv::Mat A(100, 100, CV_64F, x);

	// fs::path newFilename = debugOutputFilepath.stem();
	// newFilename += fs::path(*postfix);
	// newFilename += debugOutputFilepath.extension();
	// newFilename = debugOutputFolder / newFilename;
	// // std::string FILENAME = newFilename;

	std::vector<std::vector<cv::Point>> contours = {{}};
	contours[0].resize(4);
	for(int i =0; i < 4; i++) {
		contours[0][i] = {corners[i * 2], corners[i * 2 + 1]};
	}

	cv::drawContours(img, contours, 0, cv::Scalar(0,0,255), 1);
	// cv::imshow("test", img);

	cv::imwrite(debugOutputFolder / filename, img);

}

}

