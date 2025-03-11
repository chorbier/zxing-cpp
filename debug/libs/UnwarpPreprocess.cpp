#include <UnwarpPreprocess.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <filesystem>

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

void meshgrid(const Range& xRange, const Range& yRange, Mat& gridX, Mat& gridY) {
    vector<float> x, y;
    for (int i = xRange.start; i <= xRange.end; ++i) x.push_back(i);
    for (int j = yRange.start; j <= yRange.end; ++j) y.push_back(j);

    Mat xMat = Mat(x).reshape(1, 1);
    Mat yMat = Mat(y).reshape(1, 1);

    repeat(xMat, yMat.total(), 1, gridX);
    repeat(yMat.t(), 1, xMat.total(), gridY);
}

Mat griddata(const vector<Point2f>& points, const vector<float>& values, const Mat& gridX, const Mat& gridY, int method = INTER_LINEAR) {
    Mat gridZ = Mat::zeros(gridX.size(), CV_32F);
    Mat pointsMat(points);
    Mat valuesMat(values);

    Mat mapX, mapY;
    meshgrid(Range(0, gridX.cols - 1), Range(0, gridX.rows - 1), mapX, mapY);

    remap(valuesMat, gridZ, mapX, mapY, method);
    return gridZ;
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

    vector<size_t> indices = {0, 1, 2, 3};
    sort(indices.begin(), indices.end(), [&angles](size_t i1, size_t i2) { return angles[i1] > angles[i2]; });

    vector<Point2f> sortedRect(4);
    for (int i = 0; i < 4; ++i) {
        sortedRect[i] = rect[indices[i]];
    }

    return sortedRect;
}

pair<vector<Point>, vector<Point2f>> findMainContour(Mat thresh) {
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    for (const auto& cnt : contours) {
        if (contourArea(cnt) < 100) continue;
        vector<Point> approx;
        approxPolyDP(cnt, approx, 0.01 * arcLength(cnt, true), true);
        if (approx.size() == 4) {
            return {cnt, vector<Point2f>(approx.begin(), approx.end())};
        }
    }

    auto cnt = *max_element(contours.begin(), contours.end(), [](const vector<Point>& a, const vector<Point>& b) {
        return contourArea(a) < contourArea(b);
    });

    RotatedRect rect = minAreaRect(cnt);
    vector<Point2f> box(4);
    rect.points(box.data());
    return {cnt, box};
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

vector<Point> filterContourPoints(const vector<Point>& contour, const vector<Point2f>& corners, float threshold = 5.0f) {
    vector<Point> filtered;

    for (const auto& pt : contour) {
        for (int i = 0; i < 4; ++i) {
            Point2f start = corners[i];
            Point2f end = corners[(i + 1) % 4];
            float dist = distanceToSegment(pt, start, end);
            if (dist < threshold) {
                filtered.push_back(pt);
                break;
            }
        }
    }

    return filtered;
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
        } else {
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
        if (points.size() < 2) return {0, 0, 0};
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
        if (norm == 0) return {0, 0, 0};
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
        } else {
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

pair<Mat, Mat> createRemapGrid(const vector<Point2f>& corners, const vector<vector<Point2f>>& sides, int outputSize = 160) {
    int offset = 25;
    vector<Point2f> dstCorners = {
        Point2f(offset, offset),
        Point2f(outputSize - offset, offset),
        Point2f(outputSize - offset, outputSize - offset),
        Point2f(offset, outputSize - offset)
    };

    // Инициализация карт смещений
    Mat mapX(outputSize, outputSize, CV_32F, Scalar(0));
    Mat mapY(outputSize, outputSize, CV_32F, Scalar(0));

    // Для каждой стороны
    for (int i = 0; i < 4; ++i) {
        const vector<Point2f>& srcSide = sides[i]; // Исходная сторона
        Point2f dstStart = dstCorners[i];         // Начальная точка целевой стороны
        Point2f dstEnd = dstCorners[(i + 1) % 4]; // Конечная точка целевой стороны

        // Параметризация целевой стороны
        vector<float> t(outputSize / 4);
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
                } else if (idx >= srcSide.size()) {
                    srcPt = srcSide.back();
                } else {
                    float w = (tVal - srcT[idx - 1]) / (srcT[idx] - srcT[idx - 1]);
                    srcPt = (1 - w) * srcSide[idx - 1] + w * srcSide[idx];
                }
                mapX.at<float>(dy, dx) = srcPt.x;
                mapY.at<float>(dy, dx) = srcPt.y;
            }
        }
    }

    // Собираем точки, где значения в mapX и mapY не равны нулю
    vector<Point2f> validPoints;
    vector<float> validValuesX, validValuesY;
    for (int y = 0; y < outputSize; ++y) {
        for (int x = 0; x < outputSize; ++x) {
            if (mapX.at<float>(y, x) != 0 || mapY.at<float>(y, x) != 0) {
                validPoints.push_back(Point2f(x, y));
                validValuesX.push_back(mapX.at<float>(y, x));
                validValuesY.push_back(mapY.at<float>(y, x));
            }
        }
    }

    // Создаём сетку для интерполяции
    Mat gridX, gridY;
    meshgrid(Range(0, outputSize), Range(0, outputSize), gridX, gridY);

    // Интерполяция mapX и mapY
    Mat remapX = griddata(validPoints, validValuesX, gridX, gridY, INTER_LINEAR);
    Mat remapY = griddata(validPoints, validValuesY, gridX, gridY, INTER_LINEAR);

    return {remapX, remapY};
}

Mat warpWithRemap(const Mat& image, const vector<Point2f>& corners, const vector<vector<Point2f>>& sides, int outputSize = 160) {
    auto maps = createRemapGrid(corners, sides, outputSize);
    Mat warped;
    remap(image, warped, maps.first, maps.second, INTER_LANCZOS4, BORDER_CONSTANT, Scalar(255, 255, 255));
    return warped;
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

void testUnwarpPipeline(const Mat& image, const string& baseDebugPath) {
    Mat orig = image.clone();
    // Бинаризация
    Mat thresh = adaptiveBinarization(image);

    // Морфологические операции
    Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    Mat morph;
    morphologyEx(thresh, morph, MORPH_CLOSE, kernel);

    // Поиск контура и углов
    auto [contour, corners] = findMainContour(morph);
    corners = orderPoints(corners);

    // Корректировка углов до ближайших точек контура
    corners = adjustCornersToContour(contour, corners);
    corners = adjustCornersToContour(contour, corners);  // Второй проход для уточнения

    // Фильтрация точек контура
    contour = filterContourPoints(contour, corners, 6.0f);

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
    Mat warped = warpWithRemap(image, corners, sides, 160);

    // Сохранение результатов на диск
	std::filesystem::path basePath(baseDebugPath);
	std::filesystem::create_directories(basePath);

	//DEBUG
	auto maps = createRemapGrid(corners, sides, 160);

    imwrite( (basePath / "originalImage.png").string(), orig);
    imwrite( (basePath / "binaryImage.png").string(), thresh);
    imwrite( (basePath / "debugImage.png").string(), debugImg);
    imwrite( (basePath / "warpedImage.png").string(), warped);
    imwrite( (basePath / "mapX.png").string(), maps.first);
    imwrite( (basePath / "mapY.png").string(), maps.second);
	
    // imwrite("originalImage.png", orig);
    // imwrite("binaryImage.png", thresh);
    // imwrite("debugImage.png", debugImg);
    // imwrite("warpedImage.png", warped);
}

bool cvUnwarpPreprocess(Mat& outImage) {
    Mat& image = outImage;
    // Бинаризация
    Mat thresh = adaptiveBinarization(image);

    // Морфологические операции
    Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    Mat morph;
    morphologyEx(thresh, morph, MORPH_CLOSE, kernel);

    // Поиск контура и углов
    auto [contour, corners] = findMainContour(morph);
    corners = orderPoints(corners);

    // Корректировка углов до ближайших точек контура
    corners = adjustCornersToContour(contour, corners);
    corners = adjustCornersToContour(contour, corners);  // Второй проход для уточнения

    // Фильтрация точек контура
    contour = filterContourPoints(contour, corners, 6.0f);

    // Разделение контура на корректные стороны
    vector<vector<Point2f>> sides = splitContourIntoSides(contour, corners);

    image = warpWithRemap(image, corners, sides, 160);
}