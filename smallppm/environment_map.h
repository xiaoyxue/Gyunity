#pragma once

#include "image_texture.h"

NAMESPACE_BEGIN

class EnvironmentMap {
public:
	EnvironmentMap(const std::string &filename, real rotate, float scale = 1.f) {
		this->scale = scale;
		LoadImage(filename, rotate, scale);
	}

	Vec3 Sample(Vec3* dir, real* pdfW, const Vec2& u) const {
		real uv[2];
		real pdf;
		distribution->SampleContinuous(u[0], u[1], uv, &pdf);

		*pdfW = pdf / (2 * PI * PI * sinTheta(uv[1], image->Height()));
		*dir = LatLongToDir(uv[0], uv[1]);

		return LookupRadiance(uv[0], uv[1]);
	}

	Vec3 Lookup(const Vec3& dir, real* pdfW = nullptr) const {
		Vec3 normDir = dir.Norm();
		Vec2 uv = DirToLatLong(normDir);
		Vec3 radiance = LookupRadiance(uv[0], uv[1]);

		if (pdfW) {
			Vec2 uv = DirToLatLong(normDir);
			*pdfW = distribution->Pdf(uv[0], uv[1]) / (2 * PI * PI * sinTheta(uv[1], image->Height()));
		}
		return radiance;
	}

	real PdfDir(const Vec3& dir) const {
		Vec3 normDir = dir.Norm();
		Vec2 uv = DirToLatLong(normDir);
		return distribution->Pdf(uv[0], uv[1]) / (2 * PI * PI * sinTheta(uv[1], image->Height()));
	}

	Vec3 LookupRadiance(real u, real v) const {
		return image->Sample(Vec2(u, 1 - v)) * scale;
	}
private:

	void LoadImage(const std::string& filename, real rotate, real scale) {

		ImageTexture<Vec3>* imageTexture = new ImageTexture<Vec3>(filename);
		image.reset(imageTexture);
		
		distribution.reset(ConvertImageToPdf(image.get()));
	}

	Distribution2D* ConvertImageToPdf(const ImageTexture<Vec3>* pImage) const {
		int height = image->Height();
		int width = image->Width();
		real* data = new real[width * height];
		for (int r = 0; r < height; ++r) {
			real v = (real)(r + 0.5f) / (real)(height);
			real sinTheta = std::sin(PI * v);
			int colOffset = r * width;

			for (int c = 0; c < width; ++c) {
				real u = (real)(c + 0.5f) / (real)(width);
				data[c + colOffset] = sinTheta * pImage->ElementAt(c, r).Y();
			}
		}
		return new Distribution2D(data, width, height);
	}

	Vec3 LatLongToDir(real u, real v) const {
		real phi = u * 2 * PI;
		real theta = v * PI;
		
		real sinTheta = std::sin(theta);

		//return Vec3(sinTheta * std::cos(phi), sinTheta * std::sin(phi), std::cos(theta));
		return Vec3(sinTheta * std::cos(phi), std::cos(theta), sinTheta * std::sin(phi));
	}

	Vec2 DirToLatLong(const Vec3& dir) const {
		//real phi = (dir.x != 0 || dir.y != 0) ? std::atan2f(dir.y, dir.x) : 0;
		//real theta = std::acos(dir.z);
		real phi = (dir.x != 0 || dir.z != 0) ? std::atan2f(dir.z, dir.x) : 0;
		real theta = std::acos(dir.y);

		real u = Clamp(0.5 - phi * 0.5f * INV_PI, 0, 1);
		real v = Clamp(theta * INV_PI, 0, 1);
		
		return Vec2(u, v);
	}


	real sinTheta(const real v, const real height) const {
		real result;
		if (v < 1)
			result = std::sin(PI * (int)((v * height) + 0.5f) / (real)(height));
		else
			result = std::sin(PI * (real)((height - 1) + 0.5f) / (real)(height));

		return result;
	}

public:
	std::unique_ptr<ImageTexture<Vec3>> image;
	std::unique_ptr<Distribution2D> distribution;
	real scale;
};

NAMESPACE_END