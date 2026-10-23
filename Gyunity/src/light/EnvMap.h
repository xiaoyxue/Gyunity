#pragma once

#include "texture/ImageTexture.h"

GYT_NAMESPACE_BEGIN

class EnvironmentMap 
{
public:
	EnvironmentMap(const std::string &filename, real rotate, float scale = 1.f) {
		this->mScale = scale;
		LoadImage(filename, rotate, scale);
		mDistribution.reset(ConvertImageToPdf(mImage.get()));
	}

	Vec3 Sample(Vec3* dir, real* pdfW, const Vec2& u) const {
		real uv[2];
		real pdf;
		mDistribution->SampleContinuous(u[0], u[1], uv, &pdf);
		real sinTheta = SinTheta(uv[1], mImage->Height());
		if (sinTheta == 0)
			*pdfW = 0;
		else
			*pdfW = pdf / (2 * PI * PI * sinTheta);
		*dir = LatLongToDir(uv[0], uv[1]);

		return LookupRadiance(uv[0], uv[1]);
	}

	Vec3 Lookup(const Vec3& dir, real* pdfW = nullptr) const {
		Vec3 normDir = dir.Norm();
		Vec2 uv = DirToLatLong(normDir);
		Vec3 radiance = LookupRadiance(uv[0], uv[1]);

		if (pdfW) {
			Vec2 uv = DirToLatLong(normDir);
			real sinTheta = SinTheta(uv[1], mImage->Height());

			if (sinTheta == 0) 
				*pdfW = 0;
			else
				*pdfW = mDistribution->Pdf(uv[0], uv[1]) / (2 * PI * PI * sinTheta);

		}
		GYT_DEBUG_PIXEL_IF(ThreadIndex()) {
			if (std::isnan(radiance.x)) {
				std::cout << "Radiance: " << radiance << " dir: " << dir << " uv: " << uv << std::endl;
			}
		}
		return radiance;
	}

	real PdfDir(const Vec3& dir) const {
		Vec3 normDir = dir.Norm();
		Vec2 uv = DirToLatLong(normDir);
		real sinTheta = SinTheta(uv[1], mImage->Height());
		if (sinTheta == 0)
			return 0;
		else 
			return mDistribution->Pdf(uv[0], uv[1]) / (2 * PI * PI * sinTheta);
	}

	Vec3 LookupRadiance(real u, real v) const {
		return mImage->Sample(Vec2(u, 1 - v)) * mScale;
	}
private:

	void LoadImage(const std::string& filename, real rotate, real scale) {

		ImageTexture<Vec3>* imageTexture = new ImageTexture<Vec3>(filename);
		mImage.reset(imageTexture);
		
	}

	Distribution2D* ConvertImageToPdf(const ImageTexture<Vec3>* pImage) const {
		int height = mImage->Height();
		int width = mImage->Width();
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

		phi = (phi < 0) ? (phi + 2 * PI) : phi;

		//real u = Clamp(0.5 - phi * 0.5f * INV_PI, 0, 1);
		//real u = ((phi < 0.f) ? phi + 2 * PI : phi) / (2 * PI);
		real u = phi * INV_PI * 0.5;
		real v = Clamp(theta * INV_PI, 0, 1);
		
		return Vec2(u, v);
	}


	real SinTheta(const real v, const real height) const {
		real result;
		if (v < 1)
			result = std::sin(PI * (int)((v * height) + 0.5f) / (real)(height));
		else
			result = std::sin(PI * (real)((height - 1) + 0.5f) / (real)(height));

		return result;
	}

public:
	std::unique_ptr<ImageTexture<Vec3>> mImage;
	std::unique_ptr<Distribution2D> mDistribution;
	real mScale;
};

GYT_NAMESPACE_END