/*
* Copyright 2016 Nu-book Inc.
* Copyright 2016 ZXing authors
*/
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "Point.h"
#include <DecoderResult.h>
#include "GridSampler.h"

#ifdef __cpp_impl_coroutine
#include <Generator.h>
#include <DetectorResult.h>
#endif

namespace ZXing {

class BitMatrix;
class DetectorResult;

namespace DataMatrix {

#ifdef __cpp_impl_coroutine
using DetectorResults = Generator<DetectorResult>;
#else
using DetectorResults = DetectorResult;
#endif

DetectorResults Detect(const BitMatrix& image, bool tryHarder, bool tryRotate, bool isPure);

DetectorResults DetectSamplegridV1(const BitMatrix& image, bool tryHarder, bool tryRotate, bool isPure, DecoderResult& outDecoderResult);

DetectorResults DetectDefined(const BitMatrix& image, const PointF& P0, const PointF& P1, const PointF& P2, const PointF& P3, bool tryHarder, bool tryRotate, bool isPure, DecoderResult& outDecoderResult);

} // DataMatrix
} // ZXing
