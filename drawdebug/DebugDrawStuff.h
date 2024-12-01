#pragma once
#include <filesystem>
#include <vector>
namespace fs = std::filesystem;

namespace ZXing {

static fs::path debugOutputFolder = "/home/chorbier/dm_debug/";
static fs::path debugOutputFilepath;

void drawDebugImage(const class BitMatrix& image, const std::string& postfix);
void drawDebugImageWithLines(const class BitMatrix& image, const std::string& postfix, const std::vector<double>& corners);
void drawDebugImageWithPoints(const BitMatrix& image, const std::string& postfix, const std::vector<double>& inPoints, int radius = 0, int col = 0);
void drawDebugImageWithColoredPoints(const BitMatrix& image, const std::string& postfix, const std::vector<double>& inPoints, int radius = 0 );
}

