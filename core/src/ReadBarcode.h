/*
* Copyright 2019 Axel Waggershauser
*/
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "DecodeHints.h"
#include "ImageView.h"
#include "Result.h"

namespace ZXing {

/**
 * Read barcode from an ImageView
 *
 * @param buffer  view of the image data including layout and format
 * @param hints  optional DecodeHints to parameterize / speed up decoding
 * @return #Result structure
 */
Result ReadBarcode(const ImageView& buffer, const DecodeHints& hints = {});

/**
 * Read barcodes from an ImageView
 *
 * @param buffer  view of the image data including layout and format
 * @param hints  optional DecodeHints to parameterize / speed up decoding
 * @return #Results list of results found, may be empty
 */
Results ReadBarcodes(const ImageView& buffer, const DecodeHints& hints = {});
Results readbarcodescrpt_detector_v1_samplegridv1(const ImageView& buffer, const PointF& P0, const PointF& P1, const PointF& P2, const PointF& P3, const DecodeHints& hints = {});
Results readbarcodescrpt_samplegridv1(const ImageView& buffer, const DecodeHints& hints = {}, bool returnEdges = false);

} // ZXing

