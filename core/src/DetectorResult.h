/*
* Copyright 2016 Nu-book Inc.
* Copyright 2016 ZXing authors
*/
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "BitMatrix.h"
#include "Quadrilateral.h"

#include <utility>

namespace ZXing {

/**
* Encapsulates the result of detecting a barcode in an image. This includes the raw
* matrix of black/white pixels corresponding to the barcode and the position of the code
* in the input image.
*/

enum class ResultedDefect : unsigned short {
	Default,
	MissingSync,
	LMarker,
	PrintShift
};

class DetectorResult
{
	BitMatrix _bits;
	QuadrilateralI _position;
	ResultedDefect _resultedDefect = ResultedDefect::Default;

	DetectorResult(const DetectorResult&) = delete;
	DetectorResult& operator=(const DetectorResult&) = delete;

public:
	DetectorResult() = default;
	DetectorResult(DetectorResult&&) noexcept = default;
	DetectorResult& operator=(DetectorResult&&) noexcept = default;

	DetectorResult(BitMatrix&& bits, QuadrilateralI&& position) : _bits(std::move(bits)), _position(std::move(position)) {}

	const BitMatrix& bits() const & { return _bits; }
	BitMatrix&& bits() && { return std::move(_bits); }
	const QuadrilateralI& position() const & { return _position; }
	QuadrilateralI&& position() && { return std::move(_position); }
	void setResultedDefect(const ResultedDefect& resultedDefect) {_resultedDefect = resultedDefect; }
	const ResultedDefect& resultedDefect() const & { return _resultedDefect; }

	bool isValid() const { return !_bits.empty(); }
};

} // ZXing
