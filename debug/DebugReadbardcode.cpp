#include "ReadBarcode.h"

#include <iostream>
#include "ImageView.h"
#include "Result.h"
#include "opencv2/opencv.hpp"

#include "BinaryBitmap.h"
#include "DecodeHints.h"
#include "GlobalHistogramBinarizer.h"
#include "HybridBinarizer.h"
#include "MultiFormatReader.h"
#include "Pattern.h"
#include "ThresholdBinarizer.h"


#include "datamatrix/DMReader.h"
#include "Point.h"

#include <climits>
#include <memory>
#include <stdexcept>


namespace ZXing {



struct PointComparator
{
	cv::Point2f centroid;

	PointComparator(cv::Point2f c) : centroid(c) {}

	bool operator()(cv::Point2f a, cv::Point2f b)
	{
		if (a.x < centroid.x && b.x < centroid.x) {
			return a.y < b.y;
		} else if (a.x >= centroid.x && b.x >= centroid.x) {
			return a.y > b.y;
		} else {
			return a.x < centroid.x;
		}
	}
};

std::vector<cv::Point2f> sortPoints(std::vector<cv::Point2f> points)
{
	cv::Point2f centroid(0, 0);
	for (const auto& point : points) {
		centroid += point;
	}
	centroid *= (1.0 / points.size());

	std::sort(points.begin(), points.end(), PointComparator(centroid));
	return points;
}

cv::Mat createCircularMask(int h, int w, cv::Point center, int radius)
{
	cv::Mat mask = cv::Mat::zeros(h, w, CV_8U);
	cv::circle(mask, center, radius, cv::Scalar(255), -1);
	return mask;
}

std::vector<cv::Point2f> findEdgesOfSquare(cv::Mat img)
{
	cv::Mat imgGr;
	cv::cvtColor(img, imgGr, cv::COLOR_BGR2GRAY);

	int blockSize = std::ceil(imgGr.rows * 0.07);
	if (blockSize % 2 == 0)
		blockSize += 1;

	int blockCenter = (blockSize - 1) / 2;
	cv::Mat thresh;
	cv::adaptiveThreshold(imgGr, thresh, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, blockSize, 5);

	blockSize = std::ceil(imgGr.rows * 0.04);
	if (blockSize % 2 == 0)
		blockSize += 1;

	blockCenter = (blockSize - 1) / 2;
	cv::Mat kernel = createCircularMask(blockSize, blockSize, cv::Point(blockCenter, blockCenter), blockCenter);
	cv::Mat closedImg;
	cv::morphologyEx(thresh, closedImg, cv::MORPH_CLOSE, kernel);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	cv::findContours(closedImg, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

	auto cnt =
		std::max_element(contours.begin(), contours.end(), [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
			return cv::contourArea(a, false) < cv::contourArea(b, false);
		});

	double epsilon = 0.1 * cv::arcLength(*cnt, true);
	std::vector<cv::Point> approx;
	cv::approxPolyDP(*cnt, approx, epsilon, true);

	std::vector<cv::Point2f> approx2f(approx.begin(), approx.end());
	approx2f = sortPoints(approx2f);

	// # проверить заглушку
	//    if (approx2f.size() != 4) {
	//        return {
	//                cv::Point2f(5, 5),
	//                cv::Point2f(5, img.rows - 5),
	//                cv::Point2f(img.cols - 5, img.rows - 5),
	//                cv::Point2f(img.cols - 5, 5)
	//        };
	//    }

	float width = static_cast<float>(img.cols);  // Subtract the margin from the width
	float height = static_cast<float>(img.rows); // Subtract the margin from the height
	for (auto& approx : approx2f) {
		approx.x = (approx.x) / width;  // Subtract the margin and normalize
		approx.y = (approx.y) / height; // Subtract the margin and normalize
	}

	return approx2f;
}

inline ImageView ImageViewFromMat(const cv::Mat& image)
{
	auto fmt = ImageFormat::None; // Use the fully qualified name
	switch (image.channels()) {
	case 1: fmt = ImageFormat::Lum; break; // Use the fully qualified name
	case 3: fmt = ImageFormat::BGR; break; // Use the fully qualified name
	case 4: fmt = ImageFormat::BGRX; break; // Use the fully qualified name
	}

	if (image.depth() != CV_8U || fmt == ImageFormat::None)
		return {nullptr, 0, 0, ImageFormat::None}; // Use the fully qualified name

	return {image.data, image.cols, image.rows, fmt};
}


std::unique_ptr<Results> try_decode_image_crpt(cv::Mat image_cv, cv::Mat image, const DecodeHints& hints)
{
	std::unique_ptr<Results> zxing_results = nullptr;

	std::vector<cv::Point2f> edges = findEdgesOfSquare(image_cv);
	std::vector<PointF> pointFs;
	pointFs.reserve(edges.size());

	for (const auto& cvPoint : edges) {
		pointFs.emplace_back(static_cast<double>(cvPoint.x), static_cast<double>(cvPoint.y));
	}
		for (const auto& point : pointFs) {
	    std::cerr << "Points: (" << point.x << ", " << point.y << ") ";
		}

	try {
		zxing_results = std::make_unique<Results>(
			ReadBarcodesCRPT(ImageViewFromMat(image), pointFs[0], pointFs[1], pointFs[2], pointFs[3], hints));
	} catch (...) {
		zxing_results = nullptr;
	}
	return zxing_results;
}
}
int main(int argc, char *argv[])
{
	const auto hints = ZXing::DecodeHints()
						   .setFormats(ZXing::BarcodeFormat::EAN13 | ZXing::BarcodeFormat::EAN8 | ZXing::BarcodeFormat::DataMatrix
									   | ZXing::BarcodeFormat::QRCode | ZXing::BarcodeFormat::PDF417)
						   .setTryRotate(true)
						   .setTryDownscale(true)
						   .setDownscaleFactor(4)
						   .setBinarizer(ZXing::Binarizer::LocalAverage)
						   .setIsPure(false)
						   .setMaxNumberOfSymbols(0xff)
						   .setEanAddOnSymbol(ZXing::EanAddOnSymbol::Ignore);
	// Create an instance of ImageView with the image you want to process
	cv::Mat image_cv = cv::imread("test3.png", cv::IMREAD_COLOR);

	std::unique_ptr<ZXing::Results> zxing_results_ptr = ZXing::try_decode_image_crpt(image_cv, image_cv, hints);
	std::cout << "Zxing results size: " << zxing_results_ptr->size()  << std::endl;
	std::cout << "Processed "  << std::endl;
    for (const auto& result : *zxing_results_ptr) {
        std::cout << "Barcode text: " << result.text() << std::endl;
    }

	return 0;
}


// препроцессинги и детектор уголков
