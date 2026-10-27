#pragma once
#include "Texture.h"
#include "math/MathUtils.h"
#include "image/ImageIO.h"
#include "math/Linalg.h"

GYT_NAMESPACE_BEGIN

enum class WrapMode {
	REPEAT, BLACK, CLAMP, REPEAT_U
};

enum class FilterMode {
	NEAREST,
	BILINEAR
};

template<typename T>
class ImageTexture : public Texture<T> 
{
public:
	Array2D<T> mTexture;
	const WrapMode mWrapMode;
	const FilterMode mFilterMode;
	bool mValid = false;
public:
	ImageTexture(const Array2D<T>& image, WrapMode wrapMode = WrapMode::CLAMP, FilterMode filterMode = FilterMode::BILINEAR) :
		mTexture(image), mWrapMode(wrapMode), mFilterMode(filterMode) {
		mValid = mTexture.mRes[0] > 0 && mTexture.mRes[1] > 0;
		if (!mValid) GetDefaultLogger()->Error("Cannot create an image texture from an empty image.", false);
	}

	ImageTexture(const std::string& filename, WrapMode wrapMode = WrapMode::CLAMP, FilterMode filterMode = FilterMode::BILINEAR) :
		mWrapMode(wrapMode), mFilterMode(filterMode) {
		mValid = ImageIO::LoadTexture(filename, mTexture);
	}

	bool IsValid() const { return mValid; }

	T Sample(const Vec2& coord) const override {
		if (!mValid) {
			GetDefaultLogger()->Error("Attempted to sample an invalid image texture; check IsValid after loading.", false);
			std::abort();
		}
		int resX = mTexture.mRes[1];
		int resY = mTexture.mRes[0];
		if (mFilterMode == FilterMode::BILINEAR) {
			return Triangle(coord[0], coord[1]);
		}
		else {
			return Nearest(coord[0], coord[1]);
		}
	}

	T Sample(const Intersection& isect) const override {
		return Sample(isect.mUv);
	}

	T Texel(int u, int v) const {
		if (mWrapMode == WrapMode::CLAMP) {
			u = Clamp(u, 0, mTexture.mRes[1] - 1);
			v = Clamp(v, 0, mTexture.mRes[0] - 1);
		}
		else if (mWrapMode == WrapMode::REPEAT_U) {
			u = (u % mTexture.mRes[1] + mTexture.mRes[1]) % mTexture.mRes[1];
			v = Clamp(v, 0, mTexture.mRes[0] - 1);
		}
		return mTexture[v][u];
	}

	T Triangle(float u, float v) const {
		int resX = mTexture.mRes[1];
		int resY = mTexture.mRes[0];
		float s = resX * u - 0.5f;
		float t = resY * v - 0.5f;
		int s0 = std::floor(s), t0 = std::floor(t);
		float ds = s - s0, dt = t - t0;
		T texel = Lerp(dt,
			Lerp(ds, Texel(s0, t0), Texel(s0 + 1, t0)),
			Lerp(ds, Texel(s0, t0 + 1), Texel(s0 + 1, t0 + 1)));
		return texel;
	}

	T Nearest(float u, float v) const {
		int resX = mTexture.mRes[1];
		int resY = mTexture.mRes[0];
		T texel = Texel(int(u * resX), int(v * resY));
		return texel;
	}

	int Height() const {
		return mValid ? mTexture.mRes[0] : 0;
	}

	int Width() const {
		return mValid ? mTexture.mRes[1] : 0;
	}

	Vec3 ElementAt(int i, int j) const {
		return mTexture[mTexture.mRes[0] - j - 1][i];
		//return texture[j][i];

	}
};

GYT_NAMESPACE_END