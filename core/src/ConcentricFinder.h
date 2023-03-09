/*
* Copyright 2020 Axel Waggershauser
*/
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "BitMatrixCursor.h"
#include "Pattern.h"
#include "Quadrilateral.h"
#include "ZXAlgorithms.h"

#include <optional>

namespace ZXing {

template <typename T, size_t N>
static float CenterFromEnd(const std::array<T, N>& pattern, float end)
{
	if (N == 5) {
		float a = pattern[4] + pattern[3] + pattern[2] / 2.f;
		float b = pattern[4] + (pattern[3] + pattern[2] + pattern[1]) / 2.f;
		float c = (pattern[4] + pattern[3] + pattern[2] + pattern[1] + pattern[0]) / 2.f;
		return end - (2 * a + b + c) / 4;
	} else if (N == 3) {
		float a = pattern[2] + pattern[1] / 2.f;
		float b = (pattern[2] + pattern[1] + pattern[0]) / 2.f;
		return end - (2 * a + b) / 3;
	} else { // aztec
		auto a = std::accumulate(pattern.begin() + (N/2 + 1), pattern.end(), pattern[N/2] / 2.f);
		return end - a;
	}
}

template<int N, typename Cursor>
std::optional<Pattern<N>> ReadSymmetricPattern(Cursor& cur, int range)
{
	static_assert(N % 2 == 1);
	assert(range > 0);
	Pattern<N> res = {};
	auto constexpr s_2 = Size(res)/2;
	auto cuo = cur.turnedBack();

	auto next = [&](auto& cur, int i) {
		auto v = cur.stepToEdge(1, range);
		res[s_2 + i] += v;
		if (range)
			range -= v;
		return v;
	};

	for (int i = 0; i <= s_2; ++i) {
		if (!next(cur, i) || !next(cuo, -i))
			return {};
	}
	res[s_2]--; // the starting pixel has been counted twice, fix this

	return res;
}

template<bool RELAXED_THRESHOLD = false, typename PATTERN>
int CheckSymmetricPattern(BitMatrixCursorI& cur, PATTERN pattern, int range, bool updatePosition)
{
	auto pOri = cur.p;
	auto view = ReadSymmetricPattern<pattern.size()>(cur, range);
	if (!view || !IsPattern<RELAXED_THRESHOLD>(*view, pattern))
		return cur.p = pOri, 0;

	if (updatePosition)
		cur.step(CenterFromEnd(*view, 0.5) - 1);
	else
		cur.p = pOri;

	return Reduce(*view);
}

std::optional<PointF> CenterOfRing(const BitMatrix& image, PointI center, int range, int nth, bool requireCircle = true);

std::optional<PointF> FinetuneConcentricPatternCenter(const BitMatrix& image, PointF center, int range, int finderPatternSize);

std::optional<QuadrilateralF> FindConcentricPatternCorners(const BitMatrix& image, PointF center, int range, int ringIndex);

struct ConcentricPattern : public PointF
{
	int size = 0;
};

template <bool RELAXED_THRESHOLD = false, typename PATTERN>
std::optional<ConcentricPattern> LocateConcentricPattern(const BitMatrix& image, PATTERN pattern, PointF center, int range)
{
	auto cur = BitMatrixCursor(image, PointI(center), {});
	int minSpread = image.width(), maxSpread = 0;
	for (auto d : {PointI{0, 1}, {1, 0}}) {
		int spread = CheckSymmetricPattern<RELAXED_THRESHOLD>(cur.setDirection(d), pattern, range, true);
		if (!spread)
			return {};
		UpdateMinMax(minSpread, maxSpread, spread);
	}

#if 1
	for (auto d : {PointI{1, 1}, {1, -1}}) {
		int spread = CheckSymmetricPattern<true>(cur.setDirection(d), pattern, range * 2, false);
		if (!spread)
			return {};
		UpdateMinMax(minSpread, maxSpread, spread);
	}
#endif

	if (maxSpread > 5 * minSpread)
		return {};

	auto newCenter = FinetuneConcentricPatternCenter(image, PointF(cur.p), range, pattern.size());
	if (!newCenter)
		return {};

	return ConcentricPattern{*newCenter, (maxSpread + minSpread) / 2};
}

} // ZXing

