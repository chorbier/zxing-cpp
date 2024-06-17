#include "opencv2/opencv.hpp"
#include "DebugDrawStuff.h"
#include "ZXing/BitMatrix.h"
#include "ZXing/Point.h"

#include <filesystem>
namespace fs = std::filesystem;

namespace ZXing {
// static std::vector<uint8_t> imageData;

void BitMatrixToMat(const BitMatrix& image, cv::Mat& OutMat) {


	// imageData.resize(image.width() * image.height() * 3);

	// imageData.resize(potW * potH * 3);
	// std::memset(imageData.data(), 255, sizeof(uint8_t) * imageData.size());

	for (int y = 0; y<image.height(); y++) {
		for (int x = 0; x < image.width(); x++) {
			uchar col = image.get(x, y) ? 0 : 255;
			for(int k = 0; k < 3; k++) {
				OutMat.at<cv::Vec3b>(cv::Point(x,y))[k] = col;
			}
		}
	}

	// cv::Mat img(image.width(), image.height(), CV_8UC3, imageData.data(), true);

	// OutMat = cv::Mat(image.width(), image.height(), CV_8UC3, imageData.data());
	// OutMat = cv::Mat(potW, potH, CV_8UC3, imageData.data()).clone();
}

int DebugFileIndex = 0;

void drawDebugImage(const BitMatrix& image, const std::string& postfix) {

	cv::Mat img(image.width(), image.height(), CV_8UC3);
	BitMatrixToMat(image, img);

	// cv::Mat A(100, 100, CV_64F, x);

		// fs::path newFilename = debugOutputFilepath.stem();
		// newFilename += fs::path(*postfix);
		// newFilename += debugOutputFilepath.extension();
		// newFilename = debugOutputFolder / newFilename;
		// // std::string FILENAME = newFilename;
		// cv::imwrite(debugOutputFolder / filename, img);

	auto name = std::to_string(DebugFileIndex++);
	name = std::string(4 - name.length(), '0') + name;
	cv::imwrite(debugOutputFolder / (name + ".jpg"), img);
	// cv::imwrite(debugOutputFolder / filename, img);
}


void drawDebugImageWithLines(const BitMatrix& image, const std::string& postfix, const std::vector<double>& corners) {

	cv::Mat img(image.width(), image.height(), CV_8UC3);
	BitMatrixToMat(image, img);
	// cv::Mat A(100, 100, CV_64F, x);

		// fs::path newFilename = debugOutputFilepath.stem();
		// newFilename += fs::path(*postfix);
		// newFilename += debugOutputFilepath.extension();
		// newFilename = debugOutputFolder / newFilename;
		// // std::string FILENAME = newFilename;

	std::vector<std::vector<cv::Point>> contours = { {} };
	contours[0].resize(4);
	for (int i = 0; i < 4; i++) {
		contours[0][i] = { corners[i * 2], corners[i * 2 + 1] };
	}

	cv::drawContours(img, contours, 0, cv::Scalar(0, 0, 255), 1);
	// cv::imshow("test", img);

	auto name = std::to_string(DebugFileIndex++);
	name = std::string(4 - name.length(), '0') + name;
	if(postfix.length() > 0) {
		name+="_"+postfix;
	}

	cv::imwrite(debugOutputFolder / (name + ".jpg"), img);

}

}

