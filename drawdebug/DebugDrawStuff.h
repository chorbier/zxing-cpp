#pragma once
#include <filesystem>
namespace fs = std::filesystem;

namespace ZXing {

static fs::path debugOutputFolder = "/home/chorbier/dm_debug/";
static fs::path debugOutputFilepath;

void drawDebugImage(const class BitMatrix& image, const std::string& filename);
void drawDebugImageWithLines(const class BitMatrix& image, const std::string& filename, const std::vector<double>& corners);

}

