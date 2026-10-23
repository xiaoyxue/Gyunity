#pragma once

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION

#include "common/Core.h"
#include "math/Linalg.h"
#include "container/Array.h"


GYT_NAMESPACE_BEGIN

class ImageIO {
public:
	struct BmpHeader
	{
		uint32   mFileSize;        // Size of file in bytes
		uint32   mReserved01;      // 2x 2 reserved bytes
		uint32   mDataOffset;      // Offset in bytes where data can be found (54)

		uint32   mHeaderSize;      // 40B
		uint32   mWidth;           // Width in pixels
		uint32   mHeight;          // Height in pixels

		int16    mColorPlates;     // Must be 1
		int16    mBitsPerPixel;    // We use 24bpp
		uint32   mCompression;     // We use BI_RGB ~ 0, uncompressed
		uint32   mImageSize;       // mWidth x mHeight x 3B
		uint32   mHorizRes;        // Pixels per meter (75dpi ~ 2953ppm)
		uint32   mVertRes;         // Pixels per meter (75dpi ~ 2953ppm)
		uint32   mPaletteColors;   // Not using palette - 0
		uint32   mImportantColors; // 0 - all are important
	};

	static void WritePngFile(const std::string& filename, const std::vector<Vec3>& image, uint32 resX, uint32 resY, real gamma = 2.2);

	static void WriteBmpFile(const std::string& filename, const std::vector<Vec3>& image, uint32 resX, uint32 resY, real gamma = 2.2);

	static int ToInt(real x, real gamma = 2.2f);

	static std::vector<Vec3> LoadImage(const std::string& filename, int resX, int resY);

	static Array2D<Vec3> LoadImage(const std::string& filename);

	static Array2D<Vec3> LoadTexture(std::string filename);

	static void WriteImage(const std::string& filename, const std::vector<Vec3>& image, uint32 resX, uint32 resY, real gamma = 2.2);

};

GYT_NAMESPACE_END