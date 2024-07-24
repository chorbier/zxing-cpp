/*
* Copyright 2016 Nu-book Inc.
* Copyright 2016 ZXing authors
*/
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "Reader.h"

namespace ZXing::DataMatrix {

class Reader : public ZXing::Reader
{
public:
	using ZXing::Reader::Reader;

	Result decode(const BinaryBitmap& image) const override;

	Result decode(const BinaryBitmap& image, const PointF& P0, const PointF& P1, const PointF& P2, const PointF& P3);

#ifdef __cpp_impl_coroutine
	Results decode(const BinaryBitmap& image, int maxSymbols) const override;
#endif

};

class DMCRPTReader : public ZXing::Reader
{
public:
	using ZXing::Reader::Reader;

	Result decode(const BinaryBitmap& image) const override;
#ifdef __cpp_impl_coroutine
	Results decode(const BinaryBitmap& image, int maxSymbols) const override;
#endif
};



} // namespace ZXing::DataMatrix
