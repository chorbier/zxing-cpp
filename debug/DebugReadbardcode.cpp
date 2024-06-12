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
#include "DebugDrawStuff.h"

#include <climits>
#include <memory>
#include <stdexcept>
#include <chrono>

#include <filesystem>
namespace fs = std::filesystem;

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

double findEdgesDuration = 0.0;

std::vector<cv::Point2f> findEdgesOfSquare(cv::Mat img)
{
    auto start = std::chrono::high_resolution_clock::now();
    
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


	// DEBUG DRAW
	{
		auto drawImageCopy = img.clone();

		std::vector<std::vector<cv::Point>> contours = {approx};

		cv::drawContours(drawImageCopy, contours, 0, cv::Scalar(0,0,255), 1);

		cv::imwrite(debugOutputFilepath, drawImageCopy);

	}

	//END DEBUG DRAW

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

	findEdgesDuration += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();

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

cv::Mat get_next_possible_image(cv::Mat image, int candidate) {
	cv::Mat image_candidate;
	cv::Mat image_mixed;
	cv::Mat blurred;
	cv::Mat image_scaled;
	cv::Mat image_resized;
	cv::Mat image_gr;
	cv::Mat image_gr2;

	cv::cvtColor(image, image_gr, cv::COLOR_BGR2GRAY);
	switch (candidate)
	{
	case 0:
		image_candidate = image.clone();

	case 1:
		cv::adaptiveThreshold(image_gr, image_candidate, 255, 0, 0, 37, 2);
		image_gr.release();
		break;

	case 2:
		cv::GaussianBlur(image, blurred, cv::Size(23, 23), 0);
		cv::addWeighted(image, 1.9, blurred, -0.9, 0, image_candidate);
		blurred.release();
		break;

	case 3:
		cv::adaptiveThreshold(image_gr, image_candidate, 255, cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY, 17, 4);
		image_gr.release();
		break;

	case 4:
		cv::GaussianBlur(image, blurred, cv::Size(15,15), 0);
		cv::addWeighted(image, 1.8485371046459401, blurred, -0.8485371046459401, 0, image_mixed);
		blurred.release();
		cv::cvtColor(image_mixed, image_gr2, cv::COLOR_BGR2GRAY);
		image_mixed.release();
		cv::threshold(image_gr2, image_candidate, 100, 255, 2);
		image_gr2.release();
		break;

	case 5:
		cv::GaussianBlur(image, blurred, cv::Size(45,45), 0);
		cv::addWeighted(image, 1.5037081148746935, blurred, -0.5037081148746935, 0, image_mixed);
		blurred.release();
		cv::cvtColor(image_mixed, image_gr2, cv::COLOR_BGR2GRAY);
		image_mixed.release();
		cv::threshold(image_gr2, image_candidate, 132, 255, 1);
		image_gr2.release();
		break;


	case 6:
		cv::convertScaleAbs(image,image_scaled, 0.7448063260333111, 30);
		//int width = ;
		//int height = ;
		cv::resize(image_scaled, image_resized, cv::Size(int(image.cols * 2.1), int(image.rows * 2.1)), cv::INTER_LINEAR);
		image_scaled.release();
		cv::GaussianBlur(image_resized, blurred, cv::Size(49,49), 0);
		cv::addWeighted(image_resized, 1.9705662110228968, blurred, -0.9705662110228968, 0, image_mixed);
		image_resized.release();
		blurred.release();
		cv::cvtColor(image_mixed, image_candidate, cv::COLOR_BGR2GRAY);
		image_mixed.release();
		break;


	default:
		break;
	}
	return image_candidate;
}

double tryToDecodeTime = 0.0;
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
	    // std::cerr << "Points: (" << point.x << ", " << point.y << ") ";
		}

	auto start = std::chrono::high_resolution_clock::now();
	try {
		zxing_results = std::make_unique<Results>(
			ReadBarcodesCRPT(ImageViewFromMat(image), pointFs[0], pointFs[1], pointFs[2], pointFs[3], hints));
	} catch (...) {
		zxing_results = nullptr;
	}
	tryToDecodeTime += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();


	return zxing_results;
}
}
int main(int argc, char *argv[])
{
	const auto hints = ZXing::DecodeHints()
						   .setFormats(ZXing::BarcodeFormat::EAN13 | ZXing::BarcodeFormat::EAN8 | ZXing::BarcodeFormat::DataMatrix
									   | ZXing::BarcodeFormat::QRCode | ZXing::BarcodeFormat::PDF417)
							.setTryInvert(false)
						   .setTryRotate(true)
						   .setTryDownscale(true)
						   .setDownscaleFactor(4)
						   .setBinarizer(ZXing::Binarizer::LocalAverage)
						   .setIsPure(false)
						   .setMaxNumberOfSymbols(0xff)
						   .setEanAddOnSymbol(ZXing::EanAddOnSymbol::Ignore);
	// Create an instance of ImageView with the image you want to process


    // std::string folder("/home/chorbier/dm-tests/real");
    // std::vector<cv::String> filenames;
    // cv::glob(folder, filenames, false);

	// int cnt = 0;

	// double totalTime = 0;
	// double imreadTime = 0;
	// auto start = std::chrono::high_resolution_clock::now();

	// for(auto& fileName : filenames) {

	// 	auto path = fs::path(fileName);

	// 	auto start = std::chrono::high_resolution_clock::now();
	// 	cv::Mat image_cv = cv::imread(fileName, cv::IMREAD_COLOR);
	// 	imreadTime += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();

		
	// 	ZXing::debugOutputFilepath = ZXing::debugOutputFolder / path.filename();

	// 	std::unique_ptr<ZXing::Results> zxing_results_ptr = ZXing::try_decode_image_crpt(image_cv, image_cv, hints);
	// 	bool undetected = true;
	// 	for (const auto& result : *zxing_results_ptr) {
	// 		// std::cout << fileName << std::endl << "Barcode text: " << result.text() << std::endl;
	// 		undetected = false;

	// 		cv::imwrite(ZXing::debugOutputFolder/ "detected" / path.filename(), image_cv);
	// 		cnt++;
	// 		break;
	// 	}
	// 	if(undetected) {
	// 		cv::imwrite(ZXing::debugOutputFolder/ "undetected" / path.filename(), image_cv);
	// 	}

	// }

		// cv::imwrite("/home/chorbier/dm_debug/test.png", image_cv);


	// totalTime += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();

	// std::cout << float(cnt) / float(filenames.size()) << " " << cnt << " of " << filenames.size() << std::endl;
	// std::cout << "findEdges: " << ZXing::findEdgesDuration / totalTime << std::endl << "try to decode: " << ZXing::tryToDecodeTime / totalTime << std::endl << "imread: " << imreadTime / totalTime;
	// std::cout << std::endl;

		// std::cout << "Zxing results size: " << zxing_results_ptr->size()  << std::endl;
		// std::cout << "Processed "  << std::endl;
		// for (const auto& result : *zxing_results_ptr) {
		// 	std::cout << "Barcode text: " << result.text() << std::endl;
		// }

	// cv::Mat image_cv = cv::imread("/home/chorbier/dm-tests/dm1.png", cv::IMREAD_COLOR);

	// std::unique_ptr<ZXing::Results> zxing_results_ptr = ZXing::try_decode_image_crpt(image_cv, image_cv, hints);
	// std::cout << "Zxing results size: " << zxing_results_ptr->size()  << std::endl;
	// std::cout << "Processed "  << std::endl;
    // for (const auto& result : *zxing_results_ptr) {
    //     std::cout << "Barcode text: " << result.text() << std::endl;
    // }

    std::string folder("/home/chorbier/dm-tests/real");
    std::vector<cv::String> filenames;
    cv::glob(folder, filenames, false);

	int cnt = 0;

	double totalTime = 0;
	double imreadTime = 0;
	auto start = std::chrono::high_resolution_clock::now();

	for(auto& fileName : filenames) {
		auto start = std::chrono::high_resolution_clock::now();
		cv::Mat image_cv = cv::imread(fileName, cv::IMREAD_COLOR);
		imreadTime += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();
		
		ZXing::debugOutputFilepath = ZXing::debugOutputFolder / path.filename();

		std::unique_ptr<ZXing::Results> zxing_results_ptr;
		int width = image_cv.cols;
		int height = image_cv.rows;
		if (width * height >= 100) {
			for (int candidate = 0; candidate <= 6; candidate++) {
				cv::Mat image_candidate = ZXing::get_next_possible_image(image_cv, candidate);
				std::unique_ptr<ZXing::Results> zxing_results_ptr = ZXing::try_decode_image_crpt(image_cv, image_candidate, hints);
				image_candidate.release();
				if (zxing_results_ptr != nullptr && zxing_results_ptr->size() >= 1) {

					bool undetected = true;
					for (const auto& result : *zxing_results_ptr) {
						// std::cout << fileName << std::endl << "Barcode text: " << result.text() << std::endl;
						undetected = false;
						cv::imwrite(ZXing::debugOutputFolder/ "detected" / path.filename(), image_cv);
						cnt++;
						break;
					}
					if(undetected) {
						cv::imwrite(ZXing::debugOutputFolder/ "undetected" / path.filename(), image_cv);
					}
					// std::cout << "Zxing results size: " << zxing_results_ptr->size() << std::endl;
					// std::cout << "Processed " << std::endl;
					// for (const auto& result : *zxing_results_ptr) {

					// 		std::cout << "Barcode text: " << result.text() << std::endl;
					// 	break;
					// }
				}
			}
		}
	}
	
	totalTime += std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - start).count();

	std::cout << float(cnt) / float(filenames.size()) << " " << cnt << " of " << filenames.size() << std::endl;
	std::cout << "findEdges: " << ZXing::findEdgesDuration / totalTime << std::endl << "try to decode: " << ZXing::tryToDecodeTime / totalTime << std::endl << "imread: " << imreadTime / totalTime;
	std::cout << std::endl;



	return 0;
}


// препроцессинги и детектор уголков
