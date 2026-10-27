#pragma once
#include "common/DebugUtils.h"

#include "texture/ImageTexture.h"
#include "visual/Sampling.h"
#include "system/Threading.h"
#include "math/Transform.h"

GYT_NAMESPACE_BEGIN

class EnvironmentMap 
{
public:
	EnvironmentMap(const std::string &filename, real rotate, float scale = 1.f,
		const Transform& directionToWorld = Transform(), bool wrapLongitude = false)
		: mDirectionToWorld(directionToWorld), mWorldToDirection(Inverse(directionToWorld)), mScale(scale) {
		for (int row = 0; row < 3; ++row)
			for (int col = 0; col < 3; ++col)
				mTransformDirections = mTransformDirections || directionToWorld.GetMatrix()(row, col) != (row == col ? 1 : 0);
		if (!LoadImage(filename, rotate, scale, wrapLongitude)) return;
		mDistribution.reset(ConvertImageToPdf(mImage.get()));
		ComputeRadianceIntegral();
	}

	bool IsValid() const { return mImage && mImage->IsValid() && mDistribution; }

	Vec3 Sample(Vec3* dir, real* pdfDir, const Vec2& u) const {
		real uv[2];
		real pdf;
		mDistribution->SampleContinuous(u[0], u[1], uv, &pdf);
		real sinTheta = SinTheta(uv[1]);
		if (sinTheta == 0)
			*pdfDir = 0;
		else
			*pdfDir = pdf / (2 * PI * PI * sinTheta);
		const Vec3 local = LatLongToDir(uv[0], uv[1]);
		*dir = mTransformDirections ? mDirectionToWorld.TransformVector(local).Norm() : local;

		return LookupRadiance(uv[0], uv[1]);
	}

	Vec3 Lookup(const Vec3& dir, real* pdfDir = nullptr) const {
		Vec3 normDir = mTransformDirections ? mWorldToDirection.TransformVector(dir).Norm() : dir.Norm();
		Vec2 uv = DirToLatLong(normDir);
		Vec3 radiance = LookupRadiance(uv[0], uv[1]);

		if (pdfDir) {
			Vec2 uv = DirToLatLong(normDir);
			real sinTheta = SinTheta(uv[1]);

			if (sinTheta == 0) 
				*pdfDir = 0;
			else
				*pdfDir = mDistribution->Pdf(uv[0], uv[1]) / (2 * PI * PI * sinTheta);

		}
		if (IsPixelDebugEnabled()) {
			if (std::isnan(radiance.x)) {
				std::cout << "Radiance: " << radiance << " dir: " << dir << " uv: " << uv << std::endl;
			}
		}
		return radiance;
	}

	real PdfDir(const Vec3& dir) const {
		Vec3 normDir = mTransformDirections ? mWorldToDirection.TransformVector(dir).Norm() : dir.Norm();
		Vec2 uv = DirToLatLong(normDir);
		real sinTheta = SinTheta(uv[1]);
		if (sinTheta == 0)
			return 0;
		else 
			return mDistribution->Pdf(uv[0], uv[1]) / (2 * PI * PI * sinTheta);
	}

	Vec3 LookupRadiance(real u, real v) const {
		return mImage->Sample(Vec2(u, 1 - v)) * mScale;
	}

	Vec3 RadianceIntegral() const { return mRadianceIntegral * mScale; }
private:
	Transform mDirectionToWorld, mWorldToDirection;
	bool mTransformDirections = false;
	Vec3 mRadianceIntegral;

	bool LoadImage(const std::string& filename, real rotate, real scale, bool wrapLongitude) {

		ImageTexture<Vec3>* imageTexture = new ImageTexture<Vec3>(filename,
			wrapLongitude ? WrapMode::REPEAT_U : WrapMode::CLAMP);
		mImage.reset(imageTexture);
		if (!mImage->IsValid()) return false;
		for (int y = 0; y < mImage->Height(); ++y) {
			for (int x = 0; x < mImage->Width(); ++x) {
				const Vec3 value = mImage->ElementAt(x, y);
				if (value.x < 0 || value.y < 0 || value.z < 0) {
					GetDefaultLogger()->Error("Environment maps require nonnegative RGB radiance: " + filename, false);
					return false;
				}
			}
		}
		return true;
	}

	Distribution2D* ConvertImageToPdf(const ImageTexture<Vec3>* pImage) const {
		const int height = pImage->Height();
		const int width = pImage->Width();
		std::vector<real> data(width * height);
		bool nonblack = false;
		for (int r = 0; r < height; ++r) {
			real v = (real)(r + 0.5f) / (real)(height);
			real sinTheta = std::sin(PI * v);
			int colOffset = r * width;

			for (int c = 0; c < width; ++c) {
				// Exact UV-cell average of the bilinear reconstruction. Its 3x3
				// footprint covers every contributing texel, including wrapped seams.
				real average = 0;
				for (int dy = -1; dy <= 1; ++dy)
					for (int dx = -1; dx <= 1; ++dx)
						average += (dx == 0 ? 0.75f : 0.125f) *
							(dy == 0 ? 0.75f : 0.125f) *
							pImage->Texel(c + dx, height - 1 - r + dy).Y();
				data[c + colOffset] = sinTheta * average;
				nonblack = nonblack || average > 0;
			}
		}
		// A black environment has no importance distribution. Choose a normalized
		// latitude proposal locally rather than sampling a zero-integral Distribution2D.
		if (!nonblack)
			for (int r = 0; r < height; ++r)
				for (int c = 0; c < width; ++c)
					data[r * width + c] = std::sin(PI * (r + 0.5f) / height);
		return new Distribution2D(data.data(), width, height);
	}

	void ComputeRadianceIntegral() {
		const int height = mImage->Height(), width = mImage->Width();
		const double pi = 3.14159265358979323846;
		double integral[3] = {};
		double previous[3] = {};
		// Longitude integrates to the mean of each row for both clamp and repeat.
		// Between texel centers that mean is linear in latitude; integrate it
		// analytically against sin(theta), including the clamped polar half-cells.
		for (int r = 0; r < height; ++r) {
			double current[3] = {};
			for (int c = 0; c < width; ++c) {
				const Vec3 value = mImage->ElementAt(c, r);
				for (int channel = 0; channel < 3; ++channel)
					current[channel] += double(value[channel]) / width;
			}
			const double b = pi * (r + 0.5) / height;
			if (r == 0) {
				for (int channel = 0; channel < 3; ++channel)
					integral[channel] = current[channel] * (1 - std::cos(b));
			} else {
				const double a = pi * (r - 0.5) / height;
				const double total = std::cos(a) - std::cos(b);
				const double right = (std::sin(b) - std::sin(a)) / (b - a) - std::cos(b);
				for (int channel = 0; channel < 3; ++channel)
					integral[channel] += previous[channel] * (total - right) + current[channel] * right;
			}
			for (int channel = 0; channel < 3; ++channel) {
				previous[channel] = current[channel];
				if (r == height - 1)
					integral[channel] += current[channel] * (1 + std::cos(b));
			}
		}
		for (int channel = 0; channel < 3; ++channel)
			mRadianceIntegral[channel] = real(2 * pi * integral[channel]);
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
		// acos(y) collapses near-pole directions to the pole when y rounds to +/-1.
		real theta = std::atan2(std::sqrt(dir.x * dir.x + dir.z * dir.z), dir.y);

		phi = (phi < 0) ? (phi + 2 * PI) : phi;

		//real u = Clamp(0.5 - phi * 0.5f * INV_PI, 0, 1);
		//real u = ((phi < 0.f) ? phi + 2 * PI : phi) / (2 * PI);
		real u = phi * INV_PI * 0.5;
		real v = Clamp(theta * INV_PI, 0, 1);
		
		return Vec2(u, v);
	}


	real SinTheta(real v) const {
		if (v <= 0 || v >= 1) return 0;
		return std::sin(PI * v);
	}

public:
	std::unique_ptr<ImageTexture<Vec3>> mImage;
	std::unique_ptr<Distribution2D> mDistribution;
	real mScale;
};

GYT_NAMESPACE_END