#include <UnwarpPreprocess.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

using namespace cv;
using namespace std;

Point2f mean(const vector<Point2f>& points) {
    Point2f centroid( points[0]);
    for (int i = 1; i < points.size(); i++) {
        centroid += points[i];
    }
    centroid /= static_cast<float>(points.size());
    return centroid;
}

void meshgrid(const Range& x_range, const Range& y_range, Mat& grid_x, Mat& grid_y) {
    vector<float> x, y;
    for (int i = x_range.start; i <= x_range.end; ++i) x.push_back(i);
    for (int j = y_range.start; j <= y_range.end; ++j) y.push_back(j);

    Mat x_mat = Mat(x).reshape(1, 1);
    Mat y_mat = Mat(y).reshape(1, 1);

    repeat(x_mat, y_mat.total(), 1, grid_x);
    repeat(y_mat.t(), 1, x_mat.total(), grid_y);
}

Mat griddata(const vector<Point2f>& points, const vector<float>& values, const Mat& grid_x, const Mat& grid_y, int method = INTER_LINEAR) {
    Mat grid_z = Mat::zeros(grid_x.size(), CV_32F);
    Mat points_mat(points);
    Mat values_mat(values);

    Mat map_x, map_y;
    meshgrid(Range(0, grid_x.cols - 1), Range(0, grid_x.rows - 1), map_x, map_y);

    remap(values_mat, grid_z, map_x, map_y, method);
    return grid_z;
}

vector<Point2f> order_points(vector<Point2f> pts) {
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

    vector<Point2f> sorted_rect(4);
    for (int i = 0; i < 4; ++i) {
        sorted_rect[i] = rect[indices[i]];
    }

    return sorted_rect;
}

pair<vector<Point>, vector<Point2f>> find_main_contour(Mat thresh) {
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

float distance_to_segment(Point2f pt, Point2f start, Point2f end) {
    Point2f ab = end - start;
    Point2f ap = pt - start;

    float segment_length_sq = ab.dot(ab);
    if (segment_length_sq == 0) return norm(ap);

    float t = ap.dot(ab) / segment_length_sq;
    t = max(0.0f, min(1.0f, t));

    Point2f closest_point = start + t * ab;
    return norm(pt - closest_point);
}

vector<Point> filter_contour_points(const vector<Point>& contour, const vector<Point2f>& corners, float threshold = 5.0f) {
    vector<Point> filtered;

    for (const auto& pt : contour) {
        for (int i = 0; i < 4; ++i) {
            Point2f start = corners[i];
            Point2f end = corners[(i + 1) % 4];
            float dist = distance_to_segment(pt, start, end);
            if (dist < threshold) {
                filtered.push_back(pt);
                break;
            }
        }
    }

    return filtered;
}

vector<vector<Point2f>> split_contour_into_sides(const vector<Point>& contour, const vector<Point2f>& corners) {
	vector<Point2f> contour_pts(contour.begin(), contour.end());
	vector<int> indices(4);
	for (int i = 0; i < 4; ++i) {
		double min_dist = numeric_limits<double>::max();
		for (size_t j = 0; j < contour_pts.size(); ++j) {
			double dist = norm(contour_pts[j] - corners[i]);
			if (dist < min_dist) {
				min_dist = dist;
				indices[i] = j;
			}
		}
	}

	vector<vector<Point2f>> sides(4);
	for (int i = 0; i < 4; ++i) {
		int start_idx = indices[i];
		int end_idx = indices[(i + 1) % 4];
		if (start_idx < end_idx) {
			sides[i] = vector<Point2f>(contour_pts.begin() + start_idx, contour_pts.begin() + end_idx + 1);
		} else {
			sides[i] = vector<Point2f>(contour_pts.begin() + start_idx, contour_pts.end());
			sides[i].insert(sides[i].end(), contour_pts.begin(), contour_pts.begin() + end_idx + 1);
		}
		vector<Point2f> approx_side;
		approxPolyDP(sides[i], approx_side, 0.0001 * arcLength(sides[i], false), false);
		sides[i] = approx_side;
	}

	return sides;
}

vector<Point2f> adjust_corners_to_contour(const vector<Point>& contour, const vector<Point2f>& corners, float alpha = 0.01f) {
    auto fit_line = [](const vector<Point2f>& points) -> Vec3f {
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

    auto line_intersection = [](const Vec3f& line1, const Vec3f& line2) -> Point2f {
        Matx22f matrix(line1[0], line1[1], line2[0], line2[1]);
        Vec2f rhs(-line1[2], -line2[2]);
        Matx21f intersection;
        if (solve(matrix, rhs, intersection)) {
            return Point2f(intersection(0), intersection(1));
        }
        return Point2f();
    };

    vector<vector<Point2f>> sides = split_contour_into_sides(contour, corners);
    vector<Vec3f> lines(4);
    for (int i = 0; i < 4; ++i) {
        lines[i] = fit_line(sides[i]);
    }

    vector<Point2f> new_corners(4);
    for (int i = 0; i < 4; ++i) {
        int j = (i + 1) % 4;
        Point2f intersection = line_intersection(lines[i], lines[j]);
        if (intersection != Point2f()) {
            new_corners[i] = intersection;
        } else {
            new_corners[i] = corners[i];
        }
    }

    vector<Point2f> contour_pts(contour.begin(), contour.end());
    vector<Point2f> adjusted(4);
    for (int i = 0; i < 4; ++i) {
        double min_dist = numeric_limits<double>::max();
        Point2f closest_pt;
        for (const auto& pt : contour_pts) {
            double dist = norm(pt - new_corners[i]);
            if (dist < min_dist) {
                min_dist = dist;
                closest_pt = pt;
            }
        }
        adjusted[i] = alpha * new_corners[i] + (1 - alpha) * closest_pt;
    }

    return adjusted;
}

pair<Mat, Mat> create_remap_grid(const vector<Point2f>& corners, const vector<vector<Point2f>>& sides, int output_size = 160) {
    int offset = 25;
    vector<Point2f> dst_corners = {
        Point2f(offset, offset),
        Point2f(output_size - offset, offset),
        Point2f(output_size - offset, output_size - offset),
        Point2f(offset, output_size - offset)
    };

    Mat map_x(output_size, output_size, CV_32F, Scalar(0));
    Mat map_y(output_size, output_size, CV_32F, Scalar(0));

    for (int i = 0; i < 4; ++i) {
        const vector<Point2f>& src_side = sides[i];
        Point2f dst_start = dst_corners[i];
        Point2f dst_end = dst_corners[(i + 1) % 4];

        vector<float> t(output_size / 4);
        for (int j = 0; j < t.size(); ++j) {
            t[j] = static_cast<float>(j) / (t.size() - 1);
        }

        vector<Point2f> dst_points(t.size());
        for (size_t j = 0; j < t.size(); ++j) {
            dst_points[j] = dst_start + t[j] * (dst_end - dst_start);
        }

        vector<float> src_lengths(src_side.size() - 1);
        for (size_t j = 1; j < src_side.size(); ++j) {
            src_lengths[j - 1] = norm(src_side[j] - src_side[j - 1]);
        }

        vector<float> src_cum_length(src_lengths.size() + 1, 0);
        for (size_t j = 1; j < src_cum_length.size(); ++j) {
            src_cum_length[j] = src_cum_length[j - 1] + src_lengths[j - 1];
        }

        float total_length = src_cum_length.back();
        if (total_length == 0) continue;

        vector<float> src_t(src_cum_length.size());
        for (size_t j = 0; j < src_t.size(); ++j) {
            src_t[j] = src_cum_length[j] / total_length;
        }

        for (size_t j = 0; j < dst_points.size(); ++j) {
            Point2f dst_pt = dst_points[j];
            int dx = static_cast<int>(round(dst_pt.x));
            int dy = static_cast<int>(round(dst_pt.y));
            if (dx >= 0 && dx < output_size && dy >= 0 && dy < output_size) {
                float t_val = t[j];
                size_t idx = lower_bound(src_t.begin(), src_t.end(), t_val) - src_t.begin();
                Point2f src_pt;
                if (idx == 0) {
                    src_pt = src_side[0];
                } else if (idx >= src_side.size()) {
                    src_pt = src_side.back();
                } else {
                    float w = (t_val - src_t[idx - 1]) / (src_t[idx] - src_t[idx - 1]);
                    src_pt = (1 - w) * src_side[idx - 1] + w * src_side[idx];
                }
                map_x.at<float>(dy, dx) = src_pt.x;
                map_y.at<float>(dy, dx) = src_pt.y;
            }
        }
    }

    // Создаём сетку для интерполяции
    Mat grid_x, grid_y;
    meshgrid(Range(0, output_size - 1), Range(0, output_size - 1), grid_x, grid_y);

    // Интерполируем map_x и map_y
    Mat valid_x = map_x.clone();
    Mat valid_y = map_y.clone();
    Mat valid_grid_x, valid_grid_y;
    meshgrid(Range(0, output_size - 1), Range(0, output_size - 1), valid_grid_x, valid_grid_y);

    Mat remap_x = griddata(vector<Point2f>(), vector<float>(), valid_grid_x, valid_grid_y, INTER_LINEAR);
    Mat remap_y = griddata(vector<Point2f>(), vector<float>(), valid_grid_x, valid_grid_y, INTER_LINEAR);

    return {remap_x, remap_y};
}

Mat warp_with_remap(const Mat& image, const vector<Point2f>& corners, const vector<vector<Point2f>>& sides, int output_size = 160) {
    auto maps = create_remap_grid(corners, sides, output_size);
    Mat warped;
    remap(image, warped, maps.first, maps.second, INTER_LANCZOS4, BORDER_CONSTANT, Scalar(255, 255, 255));
    return warped;
}

Mat adaptive_binarization(const Mat& image) {
    Mat gray, blurred;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    bilateralFilter(gray, blurred, 6, 75, 75);

    Mat grad_x, grad_y;
    Sobel(blurred, grad_x, CV_32F, 1, 0, 3);
    Sobel(blurred, grad_y, CV_32F, 0, 1, 3);

    Mat grad_mag;
    magnitude(grad_x, grad_y, grad_mag);

    Mat thresh;
    adaptiveThreshold(blurred, thresh, 70, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 21, 12);

    return thresh;
}

void TestPipeline(const Mat& image) {
    Mat orig = image.clone();
    // Бинаризация
    Mat thresh = adaptive_binarization(image);

    // Морфологические операции
    Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    Mat morph;
    morphologyEx(thresh, morph, MORPH_CLOSE, kernel);

    // Поиск контура и углов
    auto [contour, corners] = find_main_contour(morph);
    corners = order_points(corners);

    // Корректировка углов до ближайших точек контура
    corners = adjust_corners_to_contour(contour, corners);
    corners = adjust_corners_to_contour(contour, corners);  // Второй проход для уточнения

    // Фильтрация точек контура
    contour = filter_contour_points(contour, corners, 6.0f);

    // Разделение контура на корректные стороны
    vector<vector<Point2f>> sides = split_contour_into_sides(contour, corners);

    // Визуализация с номерами углов
    Mat debug_img = image.clone();
    for (size_t i = 0; i < corners.size(); ++i) {
        circle(debug_img, corners[i], 4, Scalar(0, 0, 255), -1);
        putText(debug_img, to_string(i), Point(corners[i].x + 10, corners[i].y + 10),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
    }
    for (const auto& side : sides) {
        for (const auto& p : side) {
            circle(debug_img, p, 2, Scalar(0, 255, 0), -1);
        }
    }

    // Выпрямление с учетом кривизны
    Mat warped = warp_with_remap(image, corners, sides, 160);

    // Сохранение результатов на диск
    imwrite("original_image.png", orig);
    imwrite("binary_image.png", thresh);
    imwrite("debug_image.png", debug_img);
    imwrite("warped_image.png", warped);
};

bool CVUnwarpPreprocess(Mat& out_image) {
	Mat& image = out_image;
    // Бинаризация
    Mat thresh = adaptive_binarization(image);

    // Морфологические операции
    Mat kernel = getStructuringElement(MORPH_RECT, Size(11, 11));
    Mat morph;
    morphologyEx(thresh, morph, MORPH_CLOSE, kernel);

    // Поиск контура и углов
    auto [contour, corners] = find_main_contour(morph);
    corners = order_points(corners);

    // Корректировка углов до ближайших точек контура
    corners = adjust_corners_to_contour(contour, corners);
    corners = adjust_corners_to_contour(contour, corners);  // Второй проход для уточнения

    // Фильтрация точек контура
    contour = filter_contour_points(contour, corners, 6.0f);

    // Разделение контура на корректные стороны
    vector<vector<Point2f>> sides = split_contour_into_sides(contour, corners);

    // Визуализация с номерами углов
    Mat debug_img = image.clone();
    for (size_t i = 0; i < corners.size(); ++i) {
        circle(debug_img, corners[i], 4, Scalar(0, 0, 255), -1);
        putText(debug_img, to_string(i), Point(corners[i].x + 10, corners[i].y + 10),
                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 1);
    }
    for (const auto& side : sides) {
        for (const auto& p : side) {
            circle(debug_img, p, 2, Scalar(0, 255, 0), -1);
        }
    }

    image = warp_with_remap(image, corners, sides, 160);
}