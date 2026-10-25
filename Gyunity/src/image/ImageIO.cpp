#include "ImageIO.h"
#include "svpng.inc"
#include "math/MathUtils.h"
#include "container/Array.h"
#include <stb/stb_image.h>
#include <stb/stb_image_write.h>
#include <openexr.h>
#include <climits>
#include <cctype>


GYT_NAMESPACE_BEGIN

namespace {
struct RgbImage {
	int width = 0, height = 0;
	std::vector<Vec3> pixels;
};

bool ImageError(const std::string& message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool ExrCheck(exr_result_t code, const std::string& operation) {
	return code == EXR_ERR_SUCCESS || ImageError(operation + " failed (OpenEXR code " + std::to_string(code) + ").");
}

struct ExrContext {
	exr_context_t value = nullptr;
	~ExrContext() { if (value) ExrCheck(exr_finish(&value), "Close OpenEXR context"); }
	bool Finish() { return ExrCheck(exr_finish(&value), "Finish OpenEXR file"); }
};

struct ExrDecode {
	exr_const_context_t context;
	exr_decode_pipeline_t value = EXR_DECODE_PIPELINE_INITIALIZER;
	~ExrDecode() { ExrCheck(exr_decoding_destroy(context, &value), "Destroy OpenEXR decoder"); }
};

struct ExrEncode {
	exr_const_context_t context;
	exr_encode_pipeline_t value = EXR_ENCODE_PIPELINE_INITIALIZER;
	~ExrEncode() { ExrCheck(exr_encoding_destroy(context, &value), "Destroy OpenEXR encoder"); }
};

bool IsExr(const std::string& filename) {
	if (filename.size() < 4) return false;
	std::string suffix = filename.substr(filename.size() - 4);
	for (char& c : suffix) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return suffix == ".exr";
}

bool CheckImageSize(int64 width, int64 height) {
	if (width <= 0 || height <= 0 || width > INT_MAX || height > INT_MAX || width * height > INT_MAX / 3)
		return ImageError("Image dimensions exceed the supported pixel range.");
	return true;
}

bool ReadRgb(const std::string& filename, RgbImage& image) {
	int width = 0, height = 0;
	std::vector<float> raw;
	if (IsExr(filename)) {
		ExrContext file;
		if (!ExrCheck(exr_start_read(&file.value, filename.c_str(), nullptr), "Open EXR '" + filename + "'")) return false;
		int parts = 0;
		exr_storage_t storage;
		exr_attr_box2i_t window;
		if (!ExrCheck(exr_get_count(file.value, &parts), "Read EXR part count") ||
			!ExrCheck(exr_get_storage(file.value, 0, &storage), "Read EXR storage") ||
			!ExrCheck(exr_get_data_window(file.value, 0, &window), "Read EXR data window")) return false;
		if (parts != 1 || storage != EXR_STORAGE_SCANLINE)
			return ImageError("EXR loading currently requires a single, flat scanline part: " + filename);
		const int64 w = int64(window.max.x) - window.min.x + 1;
		const int64 h = int64(window.max.y) - window.min.y + 1;
		if (!CheckImageSize(w, h)) return false;
		if (w > INT_MAX / (3 * int(sizeof(float)))) return ImageError("EXR row stride exceeds the supported range.");
		width = static_cast<int>(w);
		height = static_cast<int>(h);
		raw.resize(size_t(width) * height * 3);
		for (int64 y = window.min.y; y <= window.max.y;) {
			exr_chunk_info_t chunk;
			if (!ExrCheck(exr_read_scanline_chunk_info(file.value, 0, static_cast<int>(y), &chunk), "Read EXR chunk")) return false;
			if (chunk.start_x != window.min.x || chunk.width != width || chunk.height <= 0 ||
				chunk.start_y < window.min.y || int64(chunk.start_y) + chunk.height > int64(window.max.y) + 1)
				return ImageError("Invalid EXR scanline chunk bounds: " + filename);
			ExrDecode decode{ file.value };
			if (!ExrCheck(exr_decoding_initialize(file.value, 0, &chunk, &decode.value), "Initialize EXR decoder")) return false;
			bool found[3] = {};
			for (int c = 0; c < decode.value.channel_count; ++c) {
				auto& channel = decode.value.channels[c];
				int index = -1;
				if (std::strcmp(channel.channel_name, "R") == 0) index = 0;
				if (std::strcmp(channel.channel_name, "G") == 0) index = 1;
				if (std::strcmp(channel.channel_name, "B") == 0) index = 2;
				channel.decode_to_ptr = nullptr;
				if (index < 0) continue;
				if (channel.x_samples != 1 || channel.y_samples != 1 ||
					(channel.data_type != EXR_PIXEL_HALF && channel.data_type != EXR_PIXEL_FLOAT))
					return ImageError("EXR requires nonsubsampled HALF/FLOAT RGB channels: " + filename);
				found[index] = true;
				channel.user_data_type = EXR_PIXEL_FLOAT;
				channel.user_bytes_per_element = sizeof(float);
				channel.user_pixel_stride = 3 * sizeof(float);
				channel.user_line_stride = width * 3 * sizeof(float);
				channel.decode_to_ptr = reinterpret_cast<uint8_t*>(raw.data() +
					size_t(chunk.start_y - window.min.y) * width * 3 + index);
			}
			if (!found[0] || !found[1] || !found[2]) return ImageError("Missing EXR RGB channels: " + filename);
			if (!ExrCheck(exr_decoding_choose_default_routines(file.value, 0, &decode.value), "Configure EXR decoding") ||
				!ExrCheck(exr_decoding_run(file.value, 0, &decode.value), "Decode EXR chunk")) return false;
			y = int64(chunk.start_y) + chunk.height;
		}
		if (!file.Finish()) return false;
	}
	else {
		int channels = 0;
		std::unique_ptr<float, decltype(&stbi_image_free)> data(
			stbi_loadf(filename.c_str(), &width, &height, &channels, 3), stbi_image_free);
		if (!data) {
			const char* reason = stbi_failure_reason();
			return ImageError("Cannot read image '" + filename + "': " + (reason ? reason : "unknown decoding error"));
		}
		if (!CheckImageSize(width, height)) return false;
		raw.assign(data.get(), data.get() + size_t(width) * height * 3);
	}
	RgbImage decoded{ width, height, std::vector<Vec3>(size_t(width) * height) };
	for (size_t i = 0; i < decoded.pixels.size(); ++i) {
		for (int c = 0; c < 3; ++c) {
			if (!std::isfinite(raw[3 * i + c])) return ImageError("Nonfinite image channel: " + filename);
			decoded.pixels[i][c] = raw[3 * i + c];
		}
	}
	image = std::move(decoded);
	return true;
}

bool WriteExr(const std::string& filename, const std::vector<Vec3>& pixels, uint32 width, uint32 height) {
	if (width == 0 || height == 0 || width > INT_MAX || height > INT_MAX ||
		uint64(width) * height > INT_MAX / 3 || width > INT_MAX / (3 * sizeof(float)) ||
		pixels.size() != size_t(width) * height) {
		GetDefaultLogger()->Error("Cannot write EXR: invalid image dimensions or pixel count.", false);
		return false;
	}
	std::vector<float> raw(size_t(width) * height * 3);
	for (size_t i = 0; i < pixels.size(); ++i) {
		for (int c = 0; c < 3; ++c) {
			if (!std::isfinite(pixels[i][c]) || std::abs(double(pixels[i][c])) > std::numeric_limits<float>::max()) {
				GetDefaultLogger()->Error("Cannot write EXR: nonfinite or out-of-range image data.", false);
				return false;
			}
			raw[3 * i + c] = static_cast<float>(pixels[i][c]);
		}
	}
	ExrContext file;
	if (!ExrCheck(exr_start_write(&file.value, filename.c_str(), EXR_WRITE_FILE_DIRECTLY, nullptr), "Open EXR output")) return false;
	int part;
	if (!ExrCheck(exr_add_part(file.value, nullptr, EXR_STORAGE_SCANLINE, &part), "Add EXR part") ||
		!ExrCheck(exr_initialize_required_attr_simple(file.value, part, width, height, EXR_COMPRESSION_ZIP), "Initialize EXR header")) return false;
	for (const char* name : { "R", "G", "B" })
		if (!ExrCheck(exr_add_channel(file.value, part, name, EXR_PIXEL_FLOAT, EXR_PERCEPTUALLY_LOGARITHMIC, 1, 1), "Add EXR channel")) return false;
	if (!ExrCheck(exr_write_header(file.value), "Write EXR header")) return false;
	for (uint32 y = 0; y < height;) {
		exr_chunk_info_t chunk;
		if (!ExrCheck(exr_write_scanline_chunk_info(file.value, part, y, &chunk), "Prepare EXR chunk")) return false;
		ExrEncode encode{ file.value };
		if (!ExrCheck(exr_encoding_initialize(file.value, part, &chunk, &encode.value), "Initialize EXR encoder")) return false;
		for (int c = 0; c < encode.value.channel_count; ++c) {
			auto& channel = encode.value.channels[c];
			const int index = std::strcmp(channel.channel_name, "R") == 0 ? 0 :
				(std::strcmp(channel.channel_name, "G") == 0 ? 1 : 2);
			channel.user_data_type = EXR_PIXEL_FLOAT;
			channel.user_bytes_per_element = sizeof(float);
			channel.user_pixel_stride = 3 * sizeof(float);
			channel.user_line_stride = width * 3 * sizeof(float);
			channel.encode_from_ptr = reinterpret_cast<const uint8_t*>(raw.data() + size_t(y) * width * 3 + index);
		}
		if (!ExrCheck(exr_encoding_choose_default_routines(file.value, part, &encode.value), "Configure EXR encoding") ||
			!ExrCheck(exr_encoding_run(file.value, part, &encode.value), "Encode EXR chunk")) return false;
		y += chunk.height;
	}
	return file.Finish();
}
}

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

bool ImageIO::LoadImage(const std::string& filename, std::vector<Vec3>& pixels, int& width, int& height) {
	RgbImage image;
	if (!ReadRgb(filename, image)) return false;
	pixels = std::move(image.pixels);
	width = image.width;
	height = image.height;
	return true;
}

bool ImageIO::LoadImage(const std::string& filename, Array2D<Vec3>& output) {
	RgbImage image;
	if (!ReadRgb(filename, image)) return false;
	output = Array2D<Vec3>(image.pixels, image.height, image.width);
	return true;
}

bool ImageIO::LoadTexture(const std::string& filename, Array2D<Vec3>& output) {
	RgbImage image;
	if (!ReadRgb(filename, image)) return false;
	for (int y = 0; y < image.height / 2; ++y)
		for (int x = 0; x < image.width; ++x)
			std::swap(image.pixels[size_t(y) * image.width + x],
				image.pixels[size_t(image.height - y - 1) * image.width + x]);
	output = Array2D<Vec3>(image.pixels, image.height, image.width);
	return true;
}


bool ImageIO::WriteImage(const std::string& filename, const std::vector<Vec3>& image, uint32 resX, uint32 resY, real gamma) {
	if (IsExr(filename)) return WriteExr(filename, image, resX, resY);
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