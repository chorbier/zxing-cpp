#include "UnwarpPreprocess.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include "delaunator.hpp"

#ifdef DEBUG_DRAW
#include <filesystem>
#include <fstream>
#include <chrono>
#endif

using namespace cv;
using namespace std;

Point2f mean(const vector<Point2f>& points) {
	Point2f centroid(points[0]);
	for (int i = 1; i < points.size(); i++) {
		centroid += points[i];
	}
	centroid /= static_cast<float>(points.size());
	return centroid;
}

inline bool barycentricCoords(const Point2f& p, const Point2f& v1, const Point2f& v2, const Point2f& v3, Point3d& outCoords) {
	// Вычисляем барицентрические координаты для точки p относительно треугольника v1, v2, v3
	double denom = (v2.y - v3.y) * (v1.x - v3.x) + (v3.x - v2.x) * (v1.y - v3.y);
	if (denom == 0) {
		return false;  // Точка вне треугольника или треугольник вырожден
	}
	double a = ((v2.y - v3.y) * (p.x - v3.x) + (v3.x - v2.x) * (p.y - v3.y)) / denom;
	double b = ((v3.y - v1.y) * (p.x - v3.x) + (v1.x - v3.x) * (p.y - v3.y)) / denom;
	double c = 1 - a - b;
	outCoords = { a, b, c };
	return true;
	// return {a, b, c};
}

template <typename Func>
void RasterizeTriangle(const Point2f& v1, const Point2f& v2, const Point2f& v3, Func processBary) {
	Point2f vertices[3] = { v1, v2, v3 };
	std::sort(vertices, vertices + 3, [](const Point2f& a, const Point2f& b) {
		return a.y < b.y;
	});

	Point2f A = vertices[0];
	Point2f B = vertices[1];
	Point2f C = vertices[2];

	float dx1 = B.y != A.y ? (B.x - A.x) / (B.y - A.y) : 0.0;
	float dx2 = C.y != A.y ? (C.x - A.x) / (C.y - A.y) : 0.0;
	float dx3 = C.y != B.y ? (C.x - B.x) / (C.y - B.y) : 0.0;

	double mul = std::round(A.y) + 0.5 - A.y;
	float x1incr = dx1;
	float x2incr = dx2;
	if (dx1 >= dx2) {
		std::swap(x1incr, x2incr);
	}
	float x1 = A.x + x1incr * mul;
	float x2 = A.x + x2incr * mul;

	for (int y = std::round(A.y); y <= std::floor(B.y - 0.5); y++) {
		int x = x1 + 0.5;
		int endX = x2 + 0.5;
		for (; x < endX; x++) {
			Point3d bary;
			barycentricCoords({ x + 0.5, y + 0.5 }, v1, v2, v3, bary);
			processBary(x, y, bary);
		}
		x1 += x1incr;
		x2 += x2incr;
	}

	mul = std::round(B.y) + 0.5 - B.y;
	if (dx1 > dx2) {
		x2 = B.x + mul * dx3;
		x1incr = dx2;
		x2incr = dx3;
	}
	else {
		x1 = B.x + mul * dx3;
		x1incr = dx3;
		x2incr = dx2;
	}
	if (x2 < x1) {
		std::swap(x1, x2);
		std::swap(x1incr, x2incr);
	}

	for (int y = std::round(B.y); y <= std::floor(C.y - 0.5); ++y) {
		int x = x1 + 0.5;
		int endX = x2 + 0.5;
		for (; x < endX; x++) {
			Point3d bary;
			barycentricCoords(Point2f(x, y) + Point2f(0.5, 0.5), v1, v2, v3, bary);
			processBary(x, y, bary);
		}
		x1 += x1incr;
		x2 += x2incr;
	}
}

// Функция для линейной интерполяции на основе триангуляции Делоне
void griddata(const vector<Point2f>& points, const vector<Point2f>& values, Mat& gridX, Mat& gridY) {
	if (points.size() != values.size()) {
		throw invalid_argument("Points and values must have the same size.");
	}

	// Создаем объект Subdiv2D для триангуляции Делоне
	Rect rect(0, 0, gridX.cols, gridX.rows);
	Subdiv2D subdiv(rect);

	// Добавляем точек в триангуляцию
	subdiv.insert(points);

	// Получаем список треугольников
	vector<Vec6f> triangleList;
	subdiv.getTriangleList(triangleList);

	// Интерполируем значения для каждого треугольника
	for (const auto& t : triangleList) {
		Point2f v1(t[0], t[1]);
		Point2f v2(t[2], t[3]);
		Point2f v3(t[4], t[5]);

		// Находим индексы вершин треугольника
		int idx1 = -1, idx2 = -1, idx3 = -1;
		for (size_t i = 0; i < points.size(); ++i) {
			if (points[i] == v1) idx1 = i;
			if (points[i] == v2) idx2 = i;
			if (points[i] == v3) idx3 = i;
		}

		if (idx1 != -1 && idx2 != -1 && idx3 != -1) {
			RasterizeTriangle(v1, v2, v3, [&](const int& x, const int& y, const Point3d& bary) {
				auto interpolatedValue = bary.x * values[idx1] + bary.y * values[idx2] + bary.z * values[idx3];
				gridX.at<float>(x, y) = interpolatedValue.x;
				gridY.at<float>(x, y) = interpolatedValue.y;
			});
		}
	}
}

// Функция для линейной интерполяции на основе триангуляции Делоне
void griddataFaster(const vector<Point2f>& points, const vector<Point2f>& values, Mat& gridX, Mat& gridY) {
	if (points.size() != values.size()) {
		throw invalid_argument("Points and values must have the same size.");
	}

	vector<double> coords;
	coords.reserve(points.size() * 2);
	for (const auto& p : points) {
		coords.push_back(p.x);
		coords.push_back(p.y);
	}

	delaunator::Delaunator d(coords);

	// Интерполируем значения для каждого треугольника
	for (size_t i = 0; i < d.triangles.size(); i += 3) {

		int idx1 = d.triangles[i], idx2 = d.triangles[i + 1], idx3 = d.triangles[i + 2];
		Point2f v1 = points[idx1];
		Point2f v2 = points[idx2];
		Point2f v3 = points[idx3];

		if (idx1 != -1 && idx2 != -1 && idx3 != -1) {
			RasterizeTriangle(v1, v2, v3, [&](const int& x, const int& y, const Point3d& bary) {
				auto interpolatedValue = bary.x * values[idx1] + bary.y * values[idx2] + bary.z * values[idx3];
				gridX.at<float>(x, y) = interpolatedValue.x;
				gridY.at<float>(x, y) = interpolatedValue.y;
			});

		}
	}
	// return result;
}

vector<Point2f> orderPoints(vector<Point2f> pts) {
	vector<Point2f> rect(4);
	vector<float> sum(pts.size()), diff(pts.size());

	for (size_t i = 0; i < pts.size(); ++i) {
		sum[i] = pts[i].x + pts[i].y;
		diff[i] = pts[i].x - pts[i].y;
	}

	rect[0] = pts[min_element(sum.begin(), sum.end()) - sum.begin()]; // Top-left
	rect[2] = pts[max_element(sum.begin(), sum.end()) - sum.begin()]; // Bottom-right
	rect[1] = pts[min_element(diff.begin(), diff.end()) - diff.begin()]; // Top-right
	rect[3] = pts[max_element(diff.begin(), diff.end()) - diff.begin()]; // Bottom-left

	Point2f centroid = (rect[0] + rect[1] + rect[2] + rect[3]) / 4.0f;
	vector<float> angles(4);
	for (int i = 0; i < 4; ++i) {
		angles[i] = atan2(rect[i].y - centroid.y, rect[i].x - centroid.x);
	}

	vector<size_t> indices = { 0, 1, 2, 3 };
	sort(indices.begin(), indices.end(), [&angles](size_t i1, size_t i2) { return angles[i1] > angles[i2]; });

	vector<Point2f> sortedRect(4);
	for (int i = 0; i < 4; ++i) {
		sortedRect[i] = rect[indices[i]];
	}

	return sortedRect;
}

pair<vector<Point>, vector<Point2f>> findMainContour(Mat thresh, float epsilon) {
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(thresh, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	for (const auto& cnt : contours) {
		if (contourArea(cnt) < 100) continue;
		vector<Point> approx;
		approxPolyDP(cnt, approx, epsilon * arcLength(cnt, true), true);
		if (approx.size() == 4) {
			return { cnt, vector<Point2f>(approx.begin(), approx.end()) };
		}
	}

	auto cnt = *max_element(contours.begin(), contours.end(), [](const vector<Point>& a, const vector<Point>& b) {
		return contourArea(a) < contourArea(b);
	});

	RotatedRect rect = minAreaRect(cnt);
	vector<Point2f> box(4);
	rect.points(box.data());
	return { cnt, box };
}

float distanceToSegment(Point2f pt, Point2f start, Point2f end) {
	Point2f ab = end - start;
	Point2f ap = pt - start;

	float segmentLengthSq = ab.dot(ab);
	if (segmentLengthSq == 0) return norm(ap);

	float t = ap.dot(ab) / segmentLengthSq;
	t = max(0.0f, min(1.0f, t));

	Point2f closestPoint = start + t * ab;
	return norm(pt - closestPoint);
}

void filterContourPoints(vector<Point>& contour, const vector<Point2f>& corners, float threshold) {

	int j = 0;

	for (const auto& pt : contour) {
		for (int i = 0; i < 4; ++i) {
			Point2f start = corners[i];
			Point2f end = corners[(i + 1) % 4];
			float dist = distanceToSegment(pt, start, end);
			if (dist < threshold) {
				contour[j++] = pt;
				break;
			}
		}
	}
	contour.resize(j);

	// return filtered;
}

vector<vector<Point2f>> splitContourIntoSides(const vector<Point>& contour, const vector<Point2f>& corners) {
	vector<Point2f> contourPts(contour.begin(), contour.end());
	vector<int> indices(4);
	for (int i = 0; i < 4; ++i) {
		double minDist = numeric_limits<double>::max();
		for (size_t j = 0; j < contourPts.size(); ++j) {
			double dist = norm(contourPts[j] - corners[i]);
			if (dist < minDist) {
				minDist = dist;
				indices[i] = j;
			}
		}
	}

	vector<vector<Point2f>> sides(4);
	for (int i = 0; i < 4; ++i) {
		int startIdx = indices[i];
		int endIdx = indices[(i + 1) % 4];
		if (startIdx < endIdx) {
			sides[i] = vector<Point2f>(contourPts.begin() + startIdx, contourPts.begin() + endIdx + 1);
		}
		else {
			sides[i] = vector<Point2f>(contourPts.begin() + startIdx, contourPts.end());
			sides[i].insert(sides[i].end(), contourPts.begin(), contourPts.begin() + endIdx + 1);
		}
		vector<Point2f> approxSide;
		approxPolyDP(sides[i], approxSide, 0.0001 * arcLength(sides[i], false), false);
		sides[i] = approxSide;
	}

	return sides;
}

vector<Point2f> adjustCornersToContour(const vector<Point>& contour, const vector<Point2f>& corners, float alpha = 0.01f) {
	auto fitLine = [](const vector<Point2f>& points) -> Vec3f {
		if (points.size() < 2) return { 0, 0, 0 };
		Point2f centroid = mean(points);
		Mat A(points.size(), 2, CV_32F);
		for (size_t i = 0; i < points.size(); ++i) {
			A.at<float>(i, 0) = points[i].x - centroid.x;
			A.at<float>(i, 1) = points[i].y - centroid.y;
		}
		Mat w, u, vt;
		SVDecomp(A, w, u, vt);
		Vec3f line(vt.at<float>(1, 0), vt.at<float>(1, 1), -(vt.at<float>(1, 0) * centroid.x + vt.at<float>(1, 1) * centroid.y));
		float norm = sqrt(line[0] * line[0] + line[1] * line[1]);
		if (norm == 0) return { 0, 0, 0 };
		line /= norm;
		return line;
	};

	auto lineIntersection = [](const Vec3f& line1, const Vec3f& line2) -> Point2f {
		Matx22f matrix(line1[0], line1[1], line2[0], line2[1]);
		Vec2f rhs(-line1[2], -line2[2]);
		Matx21f intersection;
		if (solve(matrix, rhs, intersection)) {
			return Point2f(intersection(0), intersection(1));
		}
		return Point2f();
	};

	vector<vector<Point2f>> sides = splitContourIntoSides(contour, corners);
	vector<Vec3f> lines(4);
	for (int i = 0; i < 4; ++i) {
		lines[i] = fitLine(sides[i]);
	}

	vector<Point2f> newCorners(4);
	for (int i = 0; i < 4; ++i) {
		int j = (i + 1) % 4;
		Point2f intersection = lineIntersection(lines[i], lines[j]);
		if (intersection != Point2f()) {
			newCorners[i] = intersection;
		}
		else {
			newCorners[i] = corners[i];
		}
	}

	vector<Point2f> contourPts(contour.begin(), contour.end());
	vector<Point2f> adjusted(4);
	for (int i = 0; i < 4; ++i) {
		double minDist = numeric_limits<double>::max();
		Point2f closestPt;
		for (const auto& pt : contourPts) {
			double dist = norm(pt - newCorners[i]);
			if (dist < minDist) {
				minDist = dist;
				closestPt = pt;
			}
		}
		adjusted[i] = alpha * newCorners[i] + (1 - alpha) * closestPt;
	}

	return adjusted;
}

void createGridDataCurved(Mat& outRemapX, Mat& outRemapY, const vector<Point2f>& corners, const Mat& xOfs, const Mat& yOfs, int outputSize = 160, int offset = 25, int warpPointsCout = 40) {
	vector<Point2f> dstCorners = {
		Point2f(offset, offset),
		Point2f(outputSize - offset, offset),
		Point2f(outputSize - offset, outputSize - offset),
		Point2f(offset, outputSize - offset)
	};

	//     # 1---0
	//     # |   |
	//     # 2---3

	vector<Point2f> validPoints;
	vector<Point2f> validValues;

	for(int y = 0; y < warpPointsCout; y++) {
		float yAlpha = float(y) / float(warpPointsCout - 1);
		float validY = offset + float(outputSize - 2 * offset) * yAlpha; 
		for(int x = 0; x < warpPointsCout; x++) {
			float xAlpha = float(x) / float(warpPointsCout - 1);
			float xSrc = xAlpha + xOfs.at<float>(0, y);
			float ySrc = yAlpha + yOfs.at<float>(0, x);

			validPoints.push_back({offset + float(outputSize - 2 * offset) * xAlpha, validY});
			validValues.push_back((corners[0] * (1.0 - ySrc) + corners[3] * ySrc) * (1.0 - xSrc) + (corners[1] * (1.0 - ySrc) + corners[2] * ySrc) * xSrc);
		}
	}
	// griddata(validPoints, validValues, outRemapX, outRemapY);
	griddataFaster(validPoints, validValues, outRemapX, outRemapY);
}

void createRemapGrid(Mat& outRemapX, Mat& outRemapY, const vector<Point2f>& corners, const vector<vector<Point2f>>& sides, int outputSize = 160, int offset = 25, int outputPointsDist = 4) {
	vector<Point2f> dstCorners = {
		Point2f(offset, offset),
		Point2f(outputSize - offset, offset),
		Point2f(outputSize - offset, outputSize - offset),
		Point2f(offset, outputSize - offset)
	};

	// // Инициализация карт смещений
	// Mat mapX(outputSize, outputSize, CV_32F, Scalar(0));
	// Mat mapY(outputSize, outputSize, CV_32F, Scalar(0));

	vector<Point2f> validPoints;
	vector<Point2f> validValues;

	// Для каждой стороны
	for (int i = 0; i < 4; ++i) {
		const vector<Point2f>& srcSide = sides[i]; // Исходная сторона
		Point2f dstStart = dstCorners[i];         // Начальная точка целевой стороны
		Point2f dstEnd = dstCorners[(i + 1) % 4]; // Конечная точка целевой стороны

		// Параметризация целевой стороны
		vector<float> t(outputSize / outputPointsDist);
		for (int j = 0; j < t.size(); ++j) {
			t[j] = static_cast<float>(j) / (t.size() - 1);
		}

		// Точки на целевой стороне
		vector<Point2f> dstPoints(t.size());
		for (size_t j = 0; j < t.size(); ++j) {
			dstPoints[j] = dstStart + t[j] * (dstEnd - dstStart);
		}

		// Параметризация исходной стороны
		vector<float> srcLengths(srcSide.size() - 1);
		for (size_t j = 1; j < srcSide.size(); ++j) {
			srcLengths[j - 1] = norm(srcSide[j] - srcSide[j - 1]);
		}

		vector<float> srcCumLength(srcLengths.size() + 1, 0);
		for (size_t j = 1; j < srcCumLength.size(); ++j) {
			srcCumLength[j] = srcCumLength[j - 1] + srcLengths[j - 1];
		}

		float totalLength = srcCumLength.back();
		if (totalLength == 0) continue;

		vector<float> srcT(srcCumLength.size());
		for (size_t j = 0; j < srcT.size(); ++j) {
			srcT[j] = srcCumLength[j] / totalLength;
		}

		// Заполнение карт смещений
		for (size_t j = 0; j < dstPoints.size(); ++j) {
			Point2f dstPt = dstPoints[j];
			int dx = static_cast<int>(round(dstPt.x));
			int dy = static_cast<int>(round(dstPt.y));
			if (dx >= 0 && dx < outputSize && dy >= 0 && dy < outputSize) {
				float tVal = t[j];
				size_t idx = lower_bound(srcT.begin(), srcT.end(), tVal) - srcT.begin();
				Point2f srcPt;
				if (idx == 0) {
					srcPt = srcSide[0];
				}
				else if (idx >= srcSide.size()) {
					srcPt = srcSide.back();
				}
				else {
					float w = (tVal - srcT[idx - 1]) / (srcT[idx] - srcT[idx - 1]);
					srcPt = (1 - w) * srcSide[idx - 1] + w * srcSide[idx];
				}
				validPoints.push_back(dstPt);
				validValues.push_back(srcPt);
				// mapX.at<float>(dy, dx) = srcPt.x;
				// mapY.at<float>(dy, dx) = srcPt.y;
			}
		}
	}

	// griddata(validPoints, validValues, outRemapX, outRemapY);
	griddataFaster(validPoints, validValues, outRemapX, outRemapY);
}

void warpWithRemap(Mat& outWarped, const Mat& image, const vector<Point2f>& corners, const vector<vector<Point2f>>& sides, int outputSize, int offset, int outputPointsDist, cv::InterpolationFlags interpolationMethod = INTER_LINEAR) {
	Mat remapX(outputSize, outputSize, CV_32F, Scalar(0));
	Mat remapY(outputSize, outputSize, CV_32F, Scalar(0));
	createRemapGrid(remapX, remapY, corners, sides, outputSize, offset, outputPointsDist);
	remap(image, outWarped, remapX, remapY, interpolationMethod, BORDER_CONSTANT, Scalar(255, 255, 255));
}

Mat adaptiveBinarization(const Mat& image) {
	Mat gray, blurred;
	cvtColor(image, gray, COLOR_BGR2GRAY);
	bilateralFilter(gray, blurred, 6, 75, 75);

	Mat gradX, gradY;
	Sobel(blurred, gradX, CV_32F, 1, 0, 3);
	Sobel(blurred, gradY, CV_32F, 0, 1, 3);

	Mat gradMag;
	magnitude(gradX, gradY, gradMag);

	Mat thresh;
	adaptiveThreshold(blurred, thresh, 70, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 21, 12);

	return thresh;
}

#ifdef DEBUG_DRAW

struct TimeStamp {
	string name;
	chrono::_V2::system_clock::time_point start;
	chrono::_V2::system_clock::time_point end;
	uint32_t cnt = 0;
	double elapsed=0;
	TimeStamp(const string& inName) :name(inName) {
		start = chrono::high_resolution_clock::now();
	};
	void Start() {
		start = chrono::high_resolution_clock::now();
	}
	void Stop() {
		end = chrono::high_resolution_clock::now();
		elapsed+=chrono::duration<double, std::milli>(end - start).count();
		cnt++;
	};
	double GetElapsed() const {
		return elapsed;
		// return chrono::duration<double, std::milli>(end - start).count();
	}

};

void testUnwarpPreprocess(const Mat& image, const string& baseDebugPath, const UnwarpParams& params) {
	vector<TimeStamp> timers;

	Mat orig = image.clone();
	// Бинаризация
	timers.push_back(TimeStamp("Бинаризация"));
	Mat thresh = adaptiveBinarization(image);
	(timers.end() - 1)->Stop();

	// Морфологические операции
	timers.push_back(TimeStamp("Морфологические операции"));
	Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
	Mat morph;
	morphologyEx(thresh, morph, MORPH_CLOSE, kernel);
	(timers.end() - 1)->Stop();

	// Поиск контура и углов
	timers.push_back(TimeStamp("Поиск контура и углов"));
	auto [contour, corners] = findMainContour(morph, params.approxPolyEpsilon);
	corners = orderPoints(corners);
	(timers.end() - 1)->Stop();

	// Корректировка углов до ближайших точек контура
	timers.push_back(TimeStamp("Корректировка углов до ближайших точек контура"));
	corners = adjustCornersToContour(contour, corners);
	corners = adjustCornersToContour(contour, corners);  // Второй проход для уточнения
	(timers.end() - 1)->Stop();

	// Фильтрация точек контура
	timers.push_back(TimeStamp("Фильтрация точек контура"));
	filterContourPoints(contour, corners, params.pointFilterDistanceThreshold);
	(timers.end() - 1)->Stop();

	// Разделение контура на корректные стороны
	vector<vector<Point2f>> sides = splitContourIntoSides(contour, corners);

	// Визуализация с номерами углов
	Mat debugImg = image.clone();
	for (size_t i = 0; i < corners.size(); ++i) {
		circle(debugImg, corners[i], 4, Scalar(0, 0, 255), -1);
		putText(debugImg, to_string(i), Point(corners[i].x + 10, corners[i].y + 10),
				FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
	}
	for (const auto& side : sides) {
		for (const auto& p : side) {
			circle(debugImg, p, 2, Scalar(0, 255, 0), -1);
		}
	}

	// Выпрямление с учетом кривизны
	timers.push_back(TimeStamp("Выпрямление с учетом кривизны"));
	Mat warped;
	warpWithRemap(warped, image, corners, sides, params.outputSize, params.offset, params.outputPointsDist, INTER_LINEAR);
	(timers.end() - 1)->Stop();

	// Сохранение результатов на диск
	std::filesystem::path basePath(baseDebugPath);
	std::filesystem::create_directories(basePath);

	//DEBUG
	Mat remapX(160, 160, CV_32F, Scalar(0));
	Mat remapY(160, 160, CV_32F, Scalar(0));
	createRemapGrid(remapX, remapY, corners, sides, params.outputSize, params.offset);

	imwrite((basePath / "originalImage.png").string(), orig);
	imwrite((basePath / "morphImage.png").string(), morph);
	imwrite((basePath / "binaryImage.png").string(), thresh);
	imwrite((basePath / "debugImage.png").string(), debugImg);
	imwrite((basePath / "warpedImage.png").string(), warped);
	double minVal, maxVal;
	minMaxLoc(remapX, &minVal, &maxVal);
	imwrite((basePath / "mapX.png").string(), (remapX - minVal) / (maxVal - minVal) * 255);
	minMaxLoc(remapY, &minVal, &maxVal);
	imwrite((basePath / "mapY.png").string(), (remapY - minVal) / (maxVal - minVal) * 255);

	std::ofstream timersFile;
	timersFile.open((basePath / "timers.txt").string());
	for (const auto& t : timers) {
		timersFile << t.name << t.GetElapsed() << endl;
	}
	timersFile.close();
}

bool testUnwarpPreprocessPredefined(cv::Mat& outResult, const cv::Mat& image, const std::vector<std::pair<cv::Mat, cv::Mat>>& warps, std::function<bool(const cv::Mat&)> processResult, const std::string& baseDebugPath, const UnwarpParams& params, int warpPointsCout) {
	// Бинаризация
	vector<TimeStamp> timers;
	std::filesystem::path basePath(baseDebugPath);
	std::filesystem::create_directories(basePath);

	auto printTimers = [&timers, basePath]() {
		std::ofstream timersFile;
		timersFile.open((basePath / "timers.txt").string());
		float summ = 0;
		for (const auto& t : timers) {
			summ+=t.GetElapsed();
			if(t.cnt == 1) {
				timersFile << t.name << ": " << t.GetElapsed() << endl;
			} else {
				timersFile << t.name << ": " << " cnt: "<< t.cnt << " timeTotal: " << t.GetElapsed() << " average: " << t.GetElapsed() / t.cnt << endl;
			}
		}
		timersFile << "total: " << summ << endl;
		timersFile.close();
	};

	int maxSize = 320;

	timers.push_back(TimeStamp("Бинаризация"));

	Mat thresh;
	float scale = 1;
	if(std::max(image.cols, image.rows) > maxSize) {
		Mat resized;
		scale = static_cast<float>(maxSize) / static_cast<float>(std::max(image.cols, image.rows));
		cv::resize(image, resized, {static_cast<int>(static_cast<float>(image.rows) * scale), static_cast<int>(static_cast<float>(image.cols) * scale)}, 0,0, cv::INTER_LINEAR);
		thresh = adaptiveBinarization(resized);
	}
	else {
		thresh = adaptiveBinarization(image);
	}
	(timers.end() - 1)->Stop();

	// Морфологические операции
	timers.push_back(TimeStamp("Морфологические операции"));
	Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(15, 15));
	Mat morph;
	morphologyEx(thresh, morph, MORPH_CLOSE, kernel);
	(timers.end() - 1)->Stop();

	// Поиск контура и углов
	timers.push_back(TimeStamp("Поиск контура и углов"));
	auto [contour, corners] = findMainContour(morph, params.approxPolyEpsilon);
	corners = orderPoints(corners);
	(timers.end() - 1)->Stop();

	// Корректировка углов до ближайших точек контура
	timers.push_back(TimeStamp("Корректировка углов до ближайших точек контура"));
	corners = adjustCornersToContour(contour, corners);
	corners = adjustCornersToContour(contour, corners);  // Второй проход для уточнения
	(timers.end() - 1)->Stop();

	if(scale != 1) {
		for(auto& c : corners) {
			c /= scale;
		}
	}

	Mat debugImg = image.clone();
	for (size_t i = 0; i < corners.size(); ++i) {
		circle(debugImg, corners[i], 4, Scalar(0, 0, 255), -1);
		putText(debugImg, to_string(i), Point(corners[i].x + 10, corners[i].y + 10),
				FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
	}

	imwrite((basePath / "originalImage.png").string(), image);
	imwrite((basePath / "morphImage.png").string(), morph);
	imwrite((basePath / "binaryImage.png").string(), thresh);
	imwrite((basePath / "debugImage.png").string(), debugImg);


	int unwarpCnt = 0;
	auto& unwarpTimer = timers.emplace_back("Выпрямление с учетом кривизны");
	auto& testTimer = timers.emplace_back("Проверка выпрямления");
	Mat remapX(params.outputSize, params.outputSize, CV_32F, Scalar(0));
	Mat remapY(params.outputSize, params.outputSize, CV_32F, Scalar(0));

	for(const auto& [xOfs, yOfs] : warps) {
		if(xOfs.rows != warpPointsCout || yOfs.rows != warpPointsCout) {
			throw invalid_argument("Offset.rows and warpPointsCout must be equal");
		}
		unwarpTimer.Start();
		createGridDataCurved(remapX, remapY, corners, xOfs, yOfs, params.outputSize, params.offset, warpPointsCout);
		remap(image, outResult, remapX, remapY, INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 255, 255));
		unwarpTimer.Stop();
		imwrite((basePath / ("warpedImage_"+std::to_string(unwarpCnt)+".png")).string(), outResult);
		unwarpCnt++;
		testTimer.Start();
		bool success = processResult(outResult);
		testTimer.Stop();
		if(success) {
			printTimers();
			return true;
		}
	}

	printTimers();

	// double minVal, maxVal;
	// minMaxLoc(remapX, &minVal, &maxVal);
	// imwrite((basePath / "mapX.png").string(), (remapX - minVal) / (maxVal - minVal) * 255);
	// minMaxLoc(remapY, &minVal, &maxVal);
	// imwrite((basePath / "mapY.png").string(), (remapY - minVal) / (maxVal - minVal) * 255);

	return false;

}

#endif

void cvUnwarpPreprocess(cv::Mat& outResult, const cv::Mat& image, const UnwarpParams& params) {
	// Бинаризация
	Mat thresh = adaptiveBinarization(image);

	// Морфологические операции
	Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
	Mat morph;
	morphologyEx(thresh, morph, MORPH_CLOSE, kernel);

	// Поиск контура и углов
	auto [contour, corners] = findMainContour(morph, params.approxPolyEpsilon);
	corners = orderPoints(corners);

	// Корректировка углов до ближайших точек контура
	corners = adjustCornersToContour(contour, corners);
	corners = adjustCornersToContour(contour, corners);  // Второй проход для уточнения

	// Фильтрация точек контура
	filterContourPoints(contour, corners, params.pointFilterDistanceThreshold);

	// Разделение контура на корректные стороны
	vector<vector<Point2f>> sides = splitContourIntoSides(contour, corners);
	//INTER_LANCZOS4 TOOOO SLOOOW
	warpWithRemap(outResult, image, corners, sides, params.outputSize, params.offset, params.outputPointsDist, INTER_LINEAR);
}


bool cvUnwarpPreprocessPredefined(cv::Mat& outResult, const cv::Mat& image, const std::vector<std::pair<cv::Mat, cv::Mat>>& warps, std::function<bool(const cv::Mat&)> processResult,const UnwarpParams& params, int warpPointsCout) {
	// Бинаризация
	int maxSize = 320;

	Mat thresh;
	float scale = 1;
	if(std::max(image.cols, image.rows) > maxSize) {
		Mat resized;
		scale = static_cast<float>(maxSize) / static_cast<float>(std::max(image.cols, image.rows));
		cv::resize(image, resized, {static_cast<int>(static_cast<float>(image.rows) * scale), static_cast<int>(static_cast<float>(image.cols) * scale)}, 0,0, cv::INTER_LINEAR);
		thresh = adaptiveBinarization(resized);
	}
	else {
		thresh = adaptiveBinarization(image);
	}

	// Морфологические операции
	Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
	Mat morph;
	morphologyEx(thresh, morph, MORPH_CLOSE, kernel);

	// Поиск контура и углов
	auto [contour, corners] = findMainContour(morph, params.approxPolyEpsilon);
	corners = orderPoints(corners);

	// Корректировка углов до ближайших точек контура
	corners = adjustCornersToContour(contour, corners);
	corners = adjustCornersToContour(contour, corners);  // Второй проход для уточнения

	if(scale != 1.0) {
		for(auto& c : corners) {
			c /= scale;
		}
	}

	Mat remapX(params.outputSize, params.outputSize, CV_32F, Scalar(0));
	Mat remapY(params.outputSize, params.outputSize, CV_32F, Scalar(0));

	for(const auto& [xOfs, yOfs] : warps) {
		if(xOfs.rows != warpPointsCout || yOfs.rows != warpPointsCout) {
			throw invalid_argument("Offset.rows and warpPointsCout must be equal");
		}
		createGridDataCurved(remapX, remapY, corners, xOfs, yOfs, params.outputSize, params.offset, warpPointsCout);
		remap(image, outResult, remapX, remapY, INTER_LINEAR, BORDER_CONSTANT, Scalar(255, 255, 255));
		if(processResult(outResult)) {
			return true;
		}
	}
	return false;
}
