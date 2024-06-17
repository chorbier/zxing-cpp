/*
* Copyright 2016 Nu-book Inc.
* Copyright 2016 ZXing authors
* Copyright 2020 Axel Waggershauser
*/
// SPDX-License-Identifier: Apache-2.0

#include "GridSampler.h"
#include <cfloat>
#ifdef PRINT_DEBUG
#include "LogMatrix.h"
#include "BitMatrixIO.h"
#endif

namespace ZXing {

#ifdef PRINT_DEBUG
LogMatrix log;
#endif

template <typename U>
PointT<U> Min(const PointT<U>& A, U B)
{
	return PointT(std::min(A.x, B), std::min(A.y, B));
}

template <typename U>
PointT<U> Max(const PointT<U>& A, U B)
{
	return PointT<U>(std::max(A.x, B), std::max(A.y, B));
}

template <typename U>
PointT<U> Abs(const PointT<U>& A)
{
	return PointT<U>(std::abs(A.x), std::abs(A.y));
}

bool pixelTraversal(const BitMatrix& img, const PointF& p0, const PointF& p1, bool searchFor, PointF& outResultPoint) {

	PointF rd = p1 - p0;
	PointI p(p0);
	PointF rdinv(rd.x == 0 ? FLT_MAX : 1.0 / rd.x, rd.y == 0 ? FLT_MAX : 1.0 / rd.y);
	PointI stp(rd.x > 0 ? 1 : (rd.x < 0 ? -1 : 0), rd.y > 0 ? 1 : (rd.y < 0 ? -1 : 0));
	PointF delta = Min(rdinv * stp, 1.0);
	PointF t_max = Abs((p + PointF(Max(stp, 0)) - p0) * rdinv);

	float prevNext_t = 0;
	bool startOutside = !img.isIn(p);
	for (int i = 512; i--;) {

		float next_t = std::min(t_max.x, t_max.y);
		if (next_t > 1.0) {
			i = 0;
		}
		PointI cmp(t_max.y >= t_max.x ? 1 : 0, t_max.x >= t_max.y ? 1 : 0);

		if (img.isIn(p)) {
			if (img.get(p) == searchFor) {
				outResultPoint = (prevNext_t * rd) + p0;
				return true;
			}
			startOutside = false;
		} else {
			if(!startOutside) {
				outResultPoint = (prevNext_t * rd) + p0;
				return false;
			}
		}
		t_max += PointF(cmp) * delta;
		p += cmp * stp;
		prevNext_t = next_t;
	}

	outResultPoint = outResultPoint = (prevNext_t * rd) + p0;
	return false;
}

bool segmentIntersection(const PointF& startA, const PointF& endA, const PointF& startB, const PointF& endB, PointF& outIntersection)
{
	const PointF vA = endA - startA;
	const PointF vB = endB - startB;

	const float S = (-vA.y * (startA.x - startB.x) + vA.x * (startA.y - startB.y)) / (-vB.x * vA.y + vA.x * vB.y);
	const float T = (vB.x * (startA.y - startB.y) - vB.y * (startA.x - startB.x)) / (-vB.x * vA.y + vA.x * vB.y);

	if (S >= 0 && S <= 1 && T >= 0 && T <= 1) {
		outIntersection.x = startA.x + (T * vA.x);
		outIntersection.y = startA.y + (T * vA.y);
		return true;
	}
	return false;
}


void CorrectCorners(const BitMatrix& image, PointF& topLeft, PointF& bottomLeft, PointF& bottomRight, PointF& topRight, int gridSize, float subpixelOffset)
{
	//auto&& [x0, x1, y0, y1, mod2Pix] = roi;

	const float gridStepsMargin = 3.0;

	float marginWidth = (gridStepsMargin / float(gridSize)) * length(topLeft - bottomLeft);

	float tex = 1.0 / gridSize;

	auto CheckAlongLine = [&](const PointF& start, const PointF& end, const PointF& tan) {

		//AddDebugPoint(end, 4, DrawColor::blue);

		PointF bestResultPos;
		float bestResultLen = INFINITY;

		float step = 1.0;
		float startProbePix = 1.5;

		for (int i = 3; i--;) {
			float probePix = (float(i) * step + startProbePix) * tex;
			auto probePos = probePix * start + (1.0 - probePix) * end;

			auto startPOffseted = probePos - marginWidth * tan;

			auto endP = probePos + (marginWidth * 2.0) * tan;

			PointF traceResult;

			bool success = pixelTraversal(image, startPOffseted, endP, true, traceResult);

			if (success) {

				float len = length(traceResult - startPOffseted);
				if (len < bestResultLen) {
					bestResultLen = len;
					bestResultPos = traceResult;
				}
			}
		}

		if (!std::isfinite(bestResultLen))
			return start;

		return bestResultPos;
	};

	PointF corners[] = { topLeft, bottomLeft, bottomRight, topRight };


	auto CorrectCorner = [&](int pointIndex, PointF& OutResult) {
		auto& CurPoint = corners[pointIndex];
		auto& PrevPoint = corners[(pointIndex + 3) % 4];
		auto& NextPoint = corners[(pointIndex + 1) % 4];

		//auto PrevRes = CheckAlongLine(CurPoint, PrevPoint, normalized(NextPoint - CurPoint));
		//auto NextRes = CheckAlongLine(CurPoint, NextPoint, normalized(PrevPoint - CurPoint));
		auto PrevRes = CheckAlongLine(PrevPoint, CurPoint, normalized(NextPoint - CurPoint));
		auto NextRes = CheckAlongLine(NextPoint, CurPoint, normalized(PrevPoint - CurPoint));

		auto DirPrev = normalized(PrevRes - PrevPoint);
		auto DirNext = normalized(NextRes - NextPoint);

		return segmentIntersection(
			PrevPoint, PrevPoint + (1.1 * length(PrevPoint - CurPoint)) * DirPrev,
			NextPoint, NextPoint + (1.1 * length(NextPoint - CurPoint)) * DirNext,
			OutResult
		);

	};

	bool editedCorners[] = {
		CorrectCorner(0, topLeft),
		CorrectCorner(1, bottomLeft),
		CorrectCorner(2, bottomRight),
		CorrectCorner(3, topRight),
	};

	PointF correctedCorners[] = { topLeft, bottomLeft, bottomRight, topRight };

	PointF* outRes[] = { &topLeft, &bottomLeft, &bottomRight, &topRight };

	PointF offsets[4];

	float halfPixelStep = subpixelOffset / gridSize;

	for (int i = 0; i < 4; i++) {

		auto& cur = correctedCorners[i];
		auto& prev = correctedCorners[(i + 3) % 4];
		auto& next = correctedCorners[(i + 1) % 4];

		if (editedCorners[i]) {
			*outRes[i] += halfPixelStep * (next - cur) + halfPixelStep * (prev - cur);
		}
	}
}

Warp ComputeWarp(const BitMatrix& image, const ZXing::ROI& roi, float subpixelOffset = 0.5)
{
	float marginWidth = image.width() * 0.05;
	PointF traceResult;

	auto&& [x0, x1, y0, y1, mod2Pix] = roi;

	Warp warp(x1 - x0, y1 - y0);

	float gridSize = x1 - x0;

	float offsetMul = subpixelOffset / gridSize;

	auto CalcFor = [&](const PointF& startP, const PointF& endP, std::vector<PointF>& outAr, int id, float addLen = 0) {

		bool success = false;

		float dist = distance(endP, startP);

		if (dist > 0.000001) {
			auto dir = (endP - startP) / dist;
			//auto dir = normalized(endP - startP);

			//float SubPixelOffset = 0.0;

			auto startPOffseted = startP - marginWidth * dir;
			success = pixelTraversal(image, startPOffseted, endP, true, traceResult);
			//calculatedPoints.push_back(PointF(traceResult));
			if (success) {
				//auto Result = startPOffseted + (distance(startPOffseted, traceResult) + SubPixelOffset) * dir - startP;
				//auto Result = traceResult - startP + SubPixelOffset * dir;

				auto Result = traceResult - startP + offsetMul * (endP - startP);
				//auto Result = (distance(startPOffseted, PointF(traceResult)) + SubPixelOffset) * dir;
				//calculatedPoints.push_back(startP + Result);

				outAr[id] = Result;
				return;
			}
		}
	};



	for (int x = x0; x < x1; ++x) {
		auto startP = mod2Pix(centered(PointI{ x, y1 - 1 }));
		auto endP = mod2Pix(centered(PointI{ x, y0 }));

		CalcFor(startP, endP, warp.xOffsets, x - x0, y1 - y0);
	}

	for (int y = y0; y < y1; ++y) {
		auto startP = mod2Pix(centered(PointI{ x0, y }));
		auto endP = mod2Pix(centered(PointI{ x1 - 1, y }));

		CalcFor(startP, endP, warp.yOffsets, y - y0, x1 - x0);
	}

	auto Filter = [&](std::vector<PointF>& outAr) {

		struct MedianWindowData {
			float len;
			int id;
		};

		std::vector<MedianWindowData> lens(outAr.size());

		for (int i = outAr.size(); i--;) {
			lens[i] = { float(length(outAr[i])), i };
		}
		int windowSize = outAr.size() / 3;

		if (windowSize < 5) {
			return;
		}

		windowSize += 1 - (windowSize & 0b1);

		int halfWindowSize = windowSize / 2;

		float avDist = 0;


		std::vector<MedianWindowData> medianWindow(windowSize);

		auto arCpy = outAr;

		for (int i = windowSize; i <= outAr.size(); i++) {
			int j = i - halfWindowSize - 1;
			std::memcpy(medianWindow.data(), &lens[i - windowSize], windowSize * sizeof(MedianWindowData));
			std::sort(medianWindow.data(), &medianWindow[windowSize - 1], [](const MedianWindowData& a, const MedianWindowData& b) {return a.len < b.len; });
			outAr[j] = arCpy[medianWindow[halfWindowSize].id];
		}
	};
	return std::move(warp);
}

//Returns nessesary clockwise rotations TODO Merge this trace clusterfuck with warp calculation
int FindRotation(const BitMatrix& image, PointF& topLeft, PointF& bottomLeft, PointF& bottomRight, PointF& topRight, int gridSize)
{
	PerspectiveTransform mod2Pix = {Rectangle(gridSize, gridSize, 0.5), {topLeft, topRight, bottomRight, bottomLeft}};
	
	float marginWidth = image.width() * 0.05;
	PointF traceResult;

	int x0 = 0, x1 = gridSize, y0 = 0, y1 = gridSize;

	auto CalcFor = [&](const PointF& startP, const PointF& endP, float& outLen) -> bool {

		float dist = distance(endP, startP);

		if (dist > 0.000001) {
			auto dir = (endP - startP) / dist;
			//auto dir = normalized(endP - startP);

			//float SubPixelOffset = 0.0;

			auto startPOffseted = startP - marginWidth * dir;
			bool success = pixelTraversal(image, startPOffseted, endP, true, traceResult);
			//calculatedPoints.push_back(PointF(traceResult));
			if (success) {
				outLen = distance(traceResult, startP);
				return true;
			}
		}
		return false;
	};

	struct TraceStatistic
	{
		bool prevResultSuccess = false;
		float prevLen = 0;
		int cntr = 0;
		float summDelta = 0;
		float getAvDelta(){return summDelta / float(cntr);}
		TraceStatistic() = default;
	};

	TraceStatistic forwardTrace;
	TraceStatistic backwardTrace;

	auto TestPath = [&](const PointI& startPI, const PointI& endPI) {
		auto startPf = mod2Pix(centered(startPI));
		auto endPf = mod2Pix(centered(endPI));
		float curLen = 0;
		//Do forwardTrace
		if(bool success = CalcFor(startPf, endPf, curLen)) {
			if(forwardTrace.prevResultSuccess) {
				forwardTrace.summDelta += std::abs(curLen - forwardTrace.prevLen);
				forwardTrace.cntr++;
			}
			forwardTrace.prevResultSuccess = true;
			forwardTrace.prevLen = curLen;
		}
		//Do backwardTrace
		if(bool success = CalcFor(endPf, startPf, curLen)) {
			if(backwardTrace.prevResultSuccess) {
				backwardTrace.summDelta += std::abs(curLen - backwardTrace.prevLen);
				backwardTrace.cntr++;
			}
			backwardTrace.prevResultSuccess = true;
			backwardTrace.prevLen = curLen;
		}
	};

	for (int x = x0; x < x1; ++x) {
		TestPath({ x, y1 - 1 },{ x, y0 });
	}
	float DeltaX0 = forwardTrace.getAvDelta();
	float DeltaX1 = backwardTrace.getAvDelta();

	forwardTrace = TraceStatistic();
	backwardTrace = TraceStatistic();

	for (int y = y0; y < y1; ++y) {
		TestPath({x0, y}, {x1-1,y});
	}
	float DeltaY0 = forwardTrace.getAvDelta();
	float DeltaY1 = backwardTrace.getAvDelta();

	if(DeltaX0 < DeltaX1) {
		return DeltaY0 < DeltaY1 ? 0 : 1;
	} else {
		return DeltaY0 < DeltaY1 ? 3 : 2;
	}
}

PointF Interp(const std::vector<PointF>& ar, float alpha)
{
	if (alpha <= 0.0) return *ar.begin();
	if (alpha >= 1.0) return *(ar.end() - 1);
	float flIndex = float(ar.size() - 1) * alpha;
	int i = int(flIndex);
	float frac = (flIndex - float(i));
	return frac * ar.at(i) + (1.0 - frac) * ar.at(i + 1);
}

DetectorResult SampleGrid(const BitMatrix& image, int width, int height, const PerspectiveTransform& mod2Pix)
{
	return SampleGrid(image, width, height, {ROI{0, width, 0, height, mod2Pix}});
}

DetectorResult SampleGridWarped(const BitMatrix& image, int width, int height, const PerspectiveTransform& mod2Pix)
{
	return SampleGridWarped(image, width, height, {ROI{0, width, 0, height, mod2Pix}});
}

DetectorResult SampleGrid(const BitMatrix& image, int width, int height, const ROIs& rois)
{
#ifdef PRINT_DEBUG
	LogMatrix log;
	static int i = 0;
	LogMatrixWriter lmw(log, image, 5, "grid" + std::to_string(i++) + ".pnm");
#endif
	if (width <= 0 || height <= 0)
		return {};

	for (auto&& [x0, x1, y0, y1, mod2Pix] : rois) {
		// To deal with remaining examples (see #251 and #267) of "numercial instabilities" that have not been
		// prevented with the Quadrilateral.h:IsConvex() check, we check for all boundary points of the grid to
		// be inside.
		auto isInside = [&mod2Pix = mod2Pix, &image](PointI p) { return image.isIn(mod2Pix(centered(p))); };
		for (int y = y0; y < y1; ++y)
			if (!isInside({x0, y}) || !isInside({x1 - 1, y}))
				return {};
		for (int x = x0; x < x1; ++x)
			if (!isInside({x, y0}) || !isInside({x, y1 - 1}))
				return {};
	}

	BitMatrix res(width, height);
	for (auto&& [x0, x1, y0, y1, mod2Pix] : rois) {
		for (int y = y0; y < y1; ++y)
			for (int x = x0; x < x1; ++x) {
				auto p = mod2Pix(centered(PointI{x, y}));
#ifdef PRINT_DEBUG
				log(p, 3);
#endif
				if (image.get(p))
					res.set(x, y);
			}
	}

#ifdef PRINT_DEBUG
	printf("width: %d, height: %d\n", width, height);
//	printf("%s", ToString(res).c_str());
#endif

	auto projectCorner = [&](PointI p) {
		for (auto&& [x0, x1, y0, y1, mod2Pix] : rois)
			if (x0 <= p.x && p.x <= x1 && y0 <= p.y && p.y <= y1)
				return PointI(mod2Pix(PointF(p)) + PointF(0.5, 0.5));

		return PointI();
	};

	return {std::move(res),
			{projectCorner({0, 0}), projectCorner({width, 0}), projectCorner({width, height}), projectCorner({0, height})}};
}


DetectorResult SampleGridWarped(const BitMatrix& image, int width, int height, const ROIs& rois)
{
#ifdef PRINT_DEBUG
	LogMatrix log;
	static int i = 0;
	LogMatrixWriter lmw(log, image, 5, "grid" + std::to_string(i++) + ".pnm");
#endif
	if (width <= 0 || height <= 0)
		return {};



	for (auto&& [x0, x1, y0, y1, mod2Pix] : rois) {
		// To deal with remaining examples (see #251 and #267) of "numercial instabilities" that have not been
		// prevented with the Quadrilateral.h:IsConvex() check, we check for all boundary points of the grid to
		// be inside.
		auto isInside = [&mod2Pix = mod2Pix, &image](PointI p) { return image.isIn(mod2Pix(centered(p))); };
		for (int y = y0; y < y1; ++y)
			if (!isInside({x0, y}) || !isInside({x1 - 1, y}))
				return {};
		for (int x = x0; x < x1; ++x)
			if (!isInside({x, y0}) || !isInside({x, y1 - 1}))
				return {};
	}

	BitMatrix res(width, height);
	for (auto& roi : rois) {
		auto&& [x0, x1, y0, y1, mod2Pix] = roi;

		Warp warp = ComputeWarp(image, roi);

		for (int y = y0; y < y1; ++y) {
			auto offsetY = Interp(warp.yOffsets, float(y) / float(y1 - 1));
			for (int x = x0; x < x1; ++x) {
				auto offsetX = Interp(warp.xOffsets, float(x) / float(x1 - 1));
				auto p = mod2Pix(centered(PointI{x, y}));

				p += offsetX;
				p += offsetY;

				// Due to a "numerical instability" in the PerspectiveTransform generation/application it has been observed
				// that even though all boundary grid points get projected inside the image, it can still happen that an
				// inner grid points is not. See #563. A true perspective transformation cannot have this property.
				// The following check takes 100% care of the issue and turned out to be less of a performance impact than feared.
				// TODO: Check some mathematical/numercial property of mod2Pix to determine if it is a perspective transforation.

				if (!image.isIn(p))
					return {};

#ifdef PRINT_DEBUG
				log(p, 3);
#endif
				if (image.get(p))
					res.set(x, y);
			}
		}
	}

#ifdef PRINT_DEBUG
	printf("width: %d, height: %d\n", width, height);
//	printf("%s", ToString(res).c_str());
#endif

	auto projectCorner = [&](PointI p) {
		for (auto&& [x0, x1, y0, y1, mod2Pix] : rois)
			if (x0 <= p.x && p.x <= x1 && y0 <= p.y && p.y <= y1)
				return PointI(mod2Pix(PointF(p)) + PointF(0.5, 0.5));

		return PointI();
	};

	return {std::move(res),
			{projectCorner({0, 0}), projectCorner({width, 0}), projectCorner({width, height}), projectCorner({0, height})}};
}
} // ZXing
