#include "ImageIO.h"
#include "svpng.inc"
#include "math/MathUtils.h"
#include "container/Array.h"
#include <stb/stb_image.h>
#include <stb/stb_image_write.h>


GYT_NAMESPACE_BEGIN

void ImageIO::WritePngFile(const std::string& filename, const std::vector<Vec3>& image, uint32 resX, uint32 resY, real gamma) {
	FILE* f = fopen(filename.c_str(), "wb");
	typedef unsigned char byte;
	byte* pngImage = new byte[resX * resY * 3];
	for (size_t j = 0; j < resY; ++j) {
		for (size_t i = 0; i < resX; ++i) {
			size_t index = 3 * j * resX + 3 * i;
			size_t imageBufferIndex = j * resX + i;
			pngImage[index] = (byte)(ToInt(image[imageBufferIndex].x, gamma));
			pngImage[index + 1] = (byte)(ToInt(image[imageBufferIndex].y, gamma));
			pngImage[index + 2] = (byte)(ToInt(image[imageBufferIndex].z, gamma));
		}
	}
	svpng(f, resX, resY, pngImage, 0);
	delete[] pngImage;
	fclose(f);
}

void ImageIO::WriteBmpFile(const std::string& filename, const std::vector<Vec3>& image, uint32 resX, uint32 resY, real gamma)
{
	std::ofstream bmp(filename.c_str(), std::ios::binary);
	BmpHeader header;
	bmp.write("BM", 2);
	header.mFileSize = (uint32)(sizeof(BmpHeader) + 2) + resX * resY * 3;
	header.mReserved01 = 0;
	header.mDataOffset = (uint32)(sizeof(BmpHeader) + 2);
	header.mHeaderSize = 40;
	header.mWidth = resX;
	header.mHeight = resY;
	header.mColorPlates = 1;
	header.mBitsPerPixel = 24;
	header.mCompression = 0;
	header.mImageSize = resX * resY * 3;
	header.mHorizRes = 2953;
	header.mVertRes = 2953;
	header.mPaletteColors = 0;
	header.mImportantColors = 0;
	bmp.write((char*)&header, sizeof(header));

	for (size_t y = 0; y < resY; y++)
	{
		for (size_t x = 0; x < resX; x++)
		{
			const Vec3& rgbF = image[x + (resY - y - 1) * resX];
			typedef unsigned char byte;
			byte bgrB[3];
			bgrB[0] = byte(ToInt(rgbF.z, gamma));
			bgrB[1] = byte(ToInt(rgbF.y, gamma));
			bgrB[2] = byte(ToInt(rgbF.x, gamma));

			bmp.write((char*)&bgrB, sizeof(bgrB));
		}
	}
}

int ImageIO::ToInt(real x, real gamma) {
	return int(pow(Clamp(x, 0.f, 1.f), 1 / gamma) * 255 + .5f);
}

std::vector<Vec3> ImageIO::LoadImage(const std::string& filename, int resX, int resY) {
	int channels;
	float* data = stbi_loadf(filename.c_str(), &resX, &resY, &channels, 0);
	std::vector<Vec3> image(resX * resY);
	for (int y = 0; y < resY; ++y) {
		for (int x = 0; x < resX; ++x) {
			int idx = y * resX + x;
			for (int k = 0; k < channels; ++k) {
				image[idx][k] = data[idx * channels + k];
			}
		}
	}
	return image;
}

Array2D<Vec3> ImageIO::LoadImage(const std::string& filename) {
	int resX, resY, channels;
	float* data = stbi_loadf(filename.c_str(), &resX, &resY, &channels, 0);
	std::vector<Vec3> texture(resX * resY);
	for (int y = 0; y < resY; ++y) {
		for (int x = 0; x < resX; ++x) {
			int idx = y * resX + x;
			for (int k = 0; k < channels; ++k) {
				texture[idx][k] = data[idx * channels + k];
			}
		}
	}
	return Array2D<Vec3>(texture, resY, resX);
}

Array2D<Vec3> ImageIO::LoadTexture(std::string filename) {
	int resX, resY, channels;
	float* data = stbi_loadf(filename.c_str(), &resX, &resY, &channels, 0);
	std::vector<Vec3> texture(resX * resY);
	for (int y = 0; y < resY; ++y) {
		for (int x = 0; x < resX; ++x) {
			int idxTexture = y * resX + x;
			int idx = (resY - 1 - y) * resX + x;
			for (int k = 0; k < channels; ++k) {
				texture[idxTexture][k] = data[idx * channels + k];
			}
		}
	}
	return Array2D<Vec3>(texture, resY, resX);
}


bool ImageIO::WriteImage(const std::string& filename, const std::vector<Vec3>& image, uint32 resX, uint32 resY, real gamma) {
	static const int channels = 3;
	if (filename.size() < 4) return false;
	std::string suffix = filename.substr(filename.length() - 4);
	if (suffix == ".hdr") {
		std::vector<float> dataToWrite(resX * resY * channels);
		for (int y = 0; y < resY; ++y) {
			for (int x = 0; x < resX; ++x) {
				for (int k = 0; k < channels; ++k) {
					int idx = y * resX + x;
					dataToWrite[idx * channels + k] = image[idx][k];
				}
			}
		}
		return stbi_write_hdr(filename.c_str(), resX, resY, channels, &dataToWrite[0]) != 0;
	}
	else {
		std::vector<unsigned char> dataToWrite(resX * resY * channels);
		for (int y = 0; y < resY; ++y) {
			for (int x = 0; x < resX; ++x) {
				for (int k = 0; k < channels; ++k) {
					int idx = y * resX + x;
					dataToWrite[idx * channels + k] = unsigned char(ToInt(image[idx][k]));
				}
			}
		}
		if (suffix == ".bmp") {
			return stbi_write_bmp(filename.c_str(), resX, resY, channels, &dataToWrite[0]) != 0;
		}
		else if (suffix == ".png") {
			return stbi_write_png(filename.c_str(), resX, resY, channels, &dataToWrite[0], channels * resX) != 0;
		}
		return false;
	}
}


GYT_NAMESPACE_END