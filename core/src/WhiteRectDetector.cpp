/*
* Copyright 2016 Nu-book Inc.
* Copyright 2016 ZXing authors
*/
// SPDX-License-Identifier: Apache-2.0

#include "WhiteRectDetector.h"
#include "DebugDrawStuff.h"

#include "BitMatrix.h"
#include "BitMatrixCursor.h"
#include "ResultPoint.h"

namespace ZXing {

static const int INIT_SIZE = 10;
static const int CORR = 1;

bool DetectWhiteRect(const BitMatrix& image, ResultPoint& p0, ResultPoint& p1, ResultPoint& p2, ResultPoint& p3)
{
	// return DetectWhiteRect(image, INIT_SIZE, image.width() / 2, image.height() / 2, p0, p1, p2, p3);
	return DetectWhiteRect(image, INIT_SIZE, image.width() / 2, image.height() / 2, p0, p1, p2, p3, 2);
}

/**
* Determines whether a segment contains a black point
*
* @param a          min value of the scanned coordinate
* @param b          max value of the scanned coordinate
* @param fixed      value of fixed coordinate
* @param horizontal set to true if scan must be horizontal, false if vertical
* @return true if a black point has been found, else false.
*/
static bool ContainsBlackPoint(const BitMatrix& image, int a, int b, int fixed, bool horizontal) {

	a = std::max(a, 0);
	if (horizontal) {
		if (fixed < 0 || fixed >= image.height())
			return false;
		b = std::min(b, image.width() - 1);
		for (int x = a; x <= b; x++) {
			if (image.get(x, fixed)) {
				return true;
			}
		}
	}
	else {
		if (fixed < 0 || fixed >= image.width())
			return false;
		b = std::min(b, image.height() - 1);
		for (int y = a; y <= b; y++) {
			if (image.get(fixed, y)) {
				return true;
			}
		}
	}

	return false;
}

bool lineIntersection(const PointF& startA, const PointF& endA, const PointF& startB, const PointF& endB, PointF& outIntersection)
{
	const PointF vA = endA - startA;
	const PointF vB = endB - startB;

	const float S = (-vA.y * (startA.x - startB.x) + vA.x * (startA.y - startB.y)) / (-vB.x * vA.y + vA.x * vB.y);
	const float T = (vB.x * (startA.y - startB.y) - vB.y * (startA.x - startB.x)) / (-vB.x * vA.y + vA.x * vB.y);

	outIntersection.x = startA.x + (T * vA.x);
	outIntersection.y = startA.y + (T * vA.y);
	if (S >= 0 && S <= 1 && T >= 0 && T <= 1) {
		return true;
	}
	return false;
}

static bool ContainsBlackPoint(const BitMatrix& image, int a, int b, int fixed, bool horizontal, int tolerance) {

	a = std::max(a, 0);
	if (horizontal) {
		if (fixed < 0 || fixed >= image.height())
			return false;
		b = std::min(b, image.width() - 1);
		int t = tolerance;
		for (int x = a; x <= b; x++) {
			if (image.get(x, fixed)) {
				if(t-- == 0) {
					return true;
				}
			} else {
				t = tolerance;
			}

		}
	}
	else {
		if (fixed < 0 || fixed >= image.width())
			return false;
		b = std::min(b, image.height() - 1);
		int t = tolerance;
		for (int y = a; y <= b; y++) {
			if (image.get(fixed, y)) {
				if(t-- == 0) {
					return true;
				}
			} else {
				t = tolerance;
			}
		}
	}

	return false;
}

static bool GetBlackPointOnSegment(const BitMatrix& image, int aX, int aY, int bX, int bY, ResultPoint& result)
{
	PointF a(aX, aY), b(bX, bY);
	BitMatrixCursorF cur(image, a, b - a);

	auto dist = std::lround(distance(a, b) / length(cur.d));

	for (int i = 0; i < dist; i++) {
		if (cur.isBlack()) {
			result = cur.p;
			return true;
		}
		cur.step();
	}
	return false;
}

static bool GetMultiBlackPointsOnSegment(const BitMatrix& image, int aX, int aY, int bX, int bY, int tolerance, ResultPoint& resultFirst, ResultPoint& resultLast)
{
	PointF a(aX, aY), b(bX, bY);
	BitMatrixCursorF cur(image, a, b - a);

	auto dist = std::lround(distance(a, b) / length(cur.d));
	int t = tolerance;
	ResultPoint first;
	for (int i = 0; i < dist; i++) {
		if (cur.isBlack()) {
			first = cur.p;
			t--;
		} else {
			if(t <= 0) {
				cur.step(-1.0);
				resultFirst = first;
				resultLast = cur.p;
				return true;
			} else {
				t = tolerance;
			}
		}
		cur.step();
	}
	return false;
}


/**
* recenters the points of a constant distance towards the center
*
* @param y bottom most point
* @param z left most point
* @param x right most point
* @param t top most point
* @return {@link ResultPoint}[] describing the corners of the rectangular
*         region. The first and last points are opposed on the diagonal, as
*         are the second and third. The first point will be the topmost
*         point and the last, the bottommost. The second point will be
*         leftmost and the third, the rightmost
*/
static void CenterEdges(const ResultPoint& y, const ResultPoint& z, const ResultPoint& x, const ResultPoint& t, int width, ResultPoint& p0, ResultPoint& p1, ResultPoint& p2, ResultPoint& p3)
{
	//
	//       t            t
	//  z                      x
	//        x    OR    z
	//   y                    y
	//

	float yi = y.x();
	float yj = y.y();
	float zi = z.x();
	float zj = z.y();
	float xi = x.x();
	float xj = x.y();
	float ti = t.x();
	float tj = t.y();

	if (yi < width / 2.0f) {
		p0 = ResultPoint(ti - CORR, tj + CORR);
		p1 = ResultPoint(zi + CORR, zj + CORR);
		p2 = ResultPoint(xi - CORR, xj - CORR);
		p3 = ResultPoint(yi + CORR, yj - CORR);
	}
	else {
		p0 = ResultPoint(ti + CORR, tj + CORR);
		p1 = ResultPoint(zi + CORR, zj - CORR);
		p2 = ResultPoint(xi - CORR, xj + CORR);
		p3 = ResultPoint(yi - CORR, yj - CORR);
	}
}

bool DetectWhiteRect(const BitMatrix& image, int initSize, int x, int y, ResultPoint& p0, ResultPoint& p1,
					 ResultPoint& p2, ResultPoint& p3)
{
	int height = image.height();
	int width = image.width();
	int halfsize = initSize / 2;
	int left = x - halfsize;
	int right = x + halfsize;
	int up = y - halfsize;
	int down = y + halfsize;
	if (up < 0 || left < 0 || down >= height || right >= width) {
		return false;
	}

	bool aBlackPointFoundOnBorder = true;
	bool atLeastOneBlackPointFoundOnBorder = false;

	bool atLeastOneBlackPointFoundOnRight = false;
	bool atLeastOneBlackPointFoundOnBottom = false;
	bool atLeastOneBlackPointFoundOnLeft = false;
	bool atLeastOneBlackPointFoundOnTop = false;

	while (aBlackPointFoundOnBorder) {

		aBlackPointFoundOnBorder = false;

		// .....
		// .   |
		// .....
		bool rightBorderNotWhite = true;
		while ((rightBorderNotWhite || !atLeastOneBlackPointFoundOnRight) && right < width) {
			rightBorderNotWhite = ContainsBlackPoint(image, up, down, right, false);
			if (rightBorderNotWhite) {
				right++;
				aBlackPointFoundOnBorder = true;
				atLeastOneBlackPointFoundOnRight = true;
			}
			else if (!atLeastOneBlackPointFoundOnRight) {
				right++;
			}
		}

		// .....
		// .   .
		// .___.
		bool bottomBorderNotWhite = true;
		while ((bottomBorderNotWhite || !atLeastOneBlackPointFoundOnBottom) && down < height) {
			bottomBorderNotWhite = ContainsBlackPoint(image, left, right, down, true);
			if (bottomBorderNotWhite) {
				down++;
				aBlackPointFoundOnBorder = true;
				atLeastOneBlackPointFoundOnBottom = true;
			}
			else if (!atLeastOneBlackPointFoundOnBottom) {
				down++;
			}
		}

		// .....
		// |   .
		// .....
		bool leftBorderNotWhite = true;
		while ((leftBorderNotWhite || !atLeastOneBlackPointFoundOnLeft) && left >= 0) {
			leftBorderNotWhite = ContainsBlackPoint(image, up, down, left, false);
			if (leftBorderNotWhite) {
				left--;
				aBlackPointFoundOnBorder = true;
				atLeastOneBlackPointFoundOnLeft = true;
			}
			else if (!atLeastOneBlackPointFoundOnLeft) {
				left--;
			}
		}

		// .___.
		// .   .
		// .....
		bool topBorderNotWhite = true;
		while ((topBorderNotWhite || !atLeastOneBlackPointFoundOnTop) && up >= 0) {
			topBorderNotWhite = ContainsBlackPoint(image, left, right, up, true);
			if (topBorderNotWhite) {
				up--;
				aBlackPointFoundOnBorder = true;
				atLeastOneBlackPointFoundOnTop = true;
			}
			else if (!atLeastOneBlackPointFoundOnTop) {
				up--;
			}
		}

		if (aBlackPointFoundOnBorder) {
			atLeastOneBlackPointFoundOnBorder = true;
		}

	}

	if (up < 0 || left < 0 || down >= height || right >= width)
		return false;

	if (atLeastOneBlackPointFoundOnBorder) {

		int maxSize = right - left;

		ResultPoint z;
		bool found = false;
		for (int i = 1; !found && i < maxSize; i++) {
			found = GetBlackPointOnSegment(image, left, down - i, left + i, down, z);
		}

		if (!found) {
			return false;
		}

		ResultPoint t;
		found = false;
		//go down right
		for (int i = 1; !found && i < maxSize; i++) {
			found = GetBlackPointOnSegment(image, left, up + i, left + i, up, t);
		}

		if (!found) {
			return false;
		}

		ResultPoint x;
		found = false;
		//go down left
		for (int i = 1; !found && i < maxSize; i++) {
			found = GetBlackPointOnSegment(image, right, up + i, right - i, up, x);
		}

		if (!found) {
			return false;
		}

		ResultPoint y;
		found = false;
		//go up left
		for (int i = 1; !found && i < maxSize; i++) {
			found = GetBlackPointOnSegment(image, right, down - i, right - i, down, y);
		}

		if (!found) {
			return false;
		}

		CenterEdges(y, z, x, t, width, p0, p1, p2, p3);
		return true;
	}
	else {
		return false;
	}
}

bool DetectWhiteRect(const BitMatrix& image, int initSize, int x, int y, ResultPoint& p0, ResultPoint& p1,
					 ResultPoint& p2, ResultPoint& p3, int tolerance)
{
	int height = image.height();
	int width = image.width();
	int halfsize = initSize / 2;
	int left = x - halfsize;
	int right = x + halfsize;
	int up = y - halfsize;
	int down = y + halfsize;
	if (up < 0 || left < 0 || down >= height || right >= width) {
		return false;
	}

	bool aBlackPointFoundOnBorder = true;
	bool atLeastOneBlackPointFoundOnBorder = false;

	bool atLeastOneBlackPointFoundOnRight = false;
	bool atLeastOneBlackPointFoundOnBottom = false;
	bool atLeastOneBlackPointFoundOnLeft = false;
	bool atLeastOneBlackPointFoundOnTop = false;

	while (aBlackPointFoundOnBorder) {

		aBlackPointFoundOnBorder = false;

		// .....
		// .   |
		// .....
		bool rightBorderNotWhite = true;
		while ((rightBorderNotWhite || !atLeastOneBlackPointFoundOnRight) && right < width) {
			rightBorderNotWhite = ContainsBlackPoint(image, up, down, right, false, tolerance);
			if (rightBorderNotWhite) {
				right++;
				aBlackPointFoundOnBorder = true;
				atLeastOneBlackPointFoundOnRight = true;
			}
			else if (!atLeastOneBlackPointFoundOnRight) {
				right++;
			}
		}

		// .....
		// .   .
		// .___.
		bool bottomBorderNotWhite = true;
		while ((bottomBorderNotWhite || !atLeastOneBlackPointFoundOnBottom) && down < height) {
			bottomBorderNotWhite = ContainsBlackPoint(image, left, right, down, true, tolerance);
			if (bottomBorderNotWhite) {
				down++;
				aBlackPointFoundOnBorder = true;
				atLeastOneBlackPointFoundOnBottom = true;
			}
			else if (!atLeastOneBlackPointFoundOnBottom) {
				down++;
			}
		}

		// .....
		// |   .
		// .....
		bool leftBorderNotWhite = true;
		while ((leftBorderNotWhite || !atLeastOneBlackPointFoundOnLeft) && left >= 0) {
			leftBorderNotWhite = ContainsBlackPoint(image, up, down, left, false, tolerance);
			if (leftBorderNotWhite) {
				left--;
				aBlackPointFoundOnBorder = true;
				atLeastOneBlackPointFoundOnLeft = true;
			}
			else if (!atLeastOneBlackPointFoundOnLeft) {
				left--;
			}
		}

		// .___.
		// .   .
		// .....
		bool topBorderNotWhite = true;
		while ((topBorderNotWhite || !atLeastOneBlackPointFoundOnTop) && up >= 0) {
			topBorderNotWhite = ContainsBlackPoint(image, left, right, up, true, tolerance);
			if (topBorderNotWhite) {
				up--;
				aBlackPointFoundOnBorder = true;
				atLeastOneBlackPointFoundOnTop = true;
			}
			else if (!atLeastOneBlackPointFoundOnTop) {
				up--;
			}
		}

		if (aBlackPointFoundOnBorder) {
			atLeastOneBlackPointFoundOnBorder = true;
		}

	}

	// drawDebugImageWithLines(image, "test2", {float(left), float(up), float(right), float(up), float(right), float(down), float(left), float(down)});

	if (up < 0 || left < 0 || down >= height || right >= width)
		return false;

	if (atLeastOneBlackPointFoundOnBorder) {

		int maxSize = right - left;

		ResultPoint z;
		// ResultPoint z1, z2;
		bool found = false;
		for (int i = 1; !found && i < maxSize; i++) {
			found = GetBlackPointOnSegment(image, left, down - i, left + i, down, z);
			// found = GetMultiBlackPointsOnSegment(image, left, down - i, left + i, down, tolerance, z1, z2);
		}

		if (!found) {
			return false;
		}

		ResultPoint t;
		// ResultPoint t1, t2;
		found = false;
		//go down right
		for (int i = 1; !found && i < maxSize; i++) {
			found = GetBlackPointOnSegment(image, left, up + i, left + i, up, t);
			// found = GetMultiBlackPointsOnSegment(image, left, up + i, left + i, up, tolerance, t1, t2);
		}

		if (!found) {
			return false;
		}

		ResultPoint x;
		// ResultPoint x1, x2;
		found = false;
		//go down left
		for (int i = 1; !found && i < maxSize; i++) {
			found = GetBlackPointOnSegment(image, right, up + i, right - i, up, x);
			// found = GetMultiBlackPointsOnSegment(image, right, up + i, right - i, up, tolerance, x1, x2);
		}

		if (!found) {
			return false;
		}

		ResultPoint y;
		// ResultPoint y1, y2;
		found = false;
		//go up left
		for (int i = 1; !found && i < maxSize; i++) {
			found = GetBlackPointOnSegment(image, right, down - i, right - i, down, y);
			// found = GetMultiBlackPointsOnSegment(image, right, down - i, right - i, down, tolerance, y1, y2);
		}

		if (!found) {
			return false;
		}

		// CenterEdges(y1, z1, x1, t1, width, p0, p1, p2, p3);

		// drawDebugImageWithLines(image, "test3", {p0.x(), p0.y(), p1.x(), p1.y(), p2.x(), p2.y(), p3.x(), p3.y()});
		CenterEdges(y, z, x, t, width, p0, p1, p2, p3);
		return true;
	}
	else {
		return false;
	}
}

} // ZXing
