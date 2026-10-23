#pragma once
#include "SamplerEnum.h"
#include "common/Core.h"
#include "common/Core.h"

GYT_NAMESPACE_BEGIN

// Determine the index of the i-th sample falling into a pixel, based on the
// elementary interval property of the Halton sequence.
// This is an implementation of the two-dimensional case of the more general
// construction in L. Gruenschloss, M. Raab, and A. Keller: "Enumerating Quasi-Monte
// Carlo Point Sequences in Elementary Intervals".
// This assumes that identity digit permutations are used for the first two components,
// i.e. basis 2 and 3.
class HaltonEnum : public SamplerEnum
{
public:
	// Initialize the enumeration for the given resolution.
	HaltonEnum(uint32 width, uint32 height);

	// Return how many samples per pixel can be queried before sample index overflow occurs.
	uint64 GetMaxSamplesPerPixel() const { return ~0ull / mIncrement; }

	// Return the index of the i-th sample falling into the given pixel (x, y) within the
	// previously given resolution bounds. i must be smaller than the value returned by
	// GetMaxSamplesPerPixel.
	uint64 ComputeIndex(uint64 i, uint32 x, uint32 y) const;

	// Scale the x-component of a sample in [0,1) to [0,width).
	real ScaleX(real x) const;

	// Scale the y-component of a sample in [0,1) to [0,height).
	real ScaleY(real y) const;

	uint64 GetIndex(uint32 sampleNum, uint32 x, uint32 y) const override {
		return ComputeIndex(sampleNum, x, y);
	}

	real SampleX(uint32 x, real u) const override {
		return ScaleX(u) - (real)x;
	}

	real SampleY(uint32 y, real v) const override {
		return ScaleY(v) - (real)y;
	}

private:
	static std::pair<int, int> ExtendedEuclid(int a, int b);
	static uint64 Halton2Inverse(uint64 i, uint32 digits);
	static uint64 Halton3Inverse(uint64 i, uint32 digits);

	uint32 mP2; // Smallest integer with 2^mP2 >= width.
	uint32 mP3; // Smallest integer with 3^mP3 >= height.
	uint32 mX; // 3^mP3 * ((2^mP2)^(-1) mod 3^mP3).
	uint32 mY; // 2^mP2 * ((3^mP3)^(-1) mod 2^mP2).
	real mScaleX; // 2^mP2.
	real mScaleY; // 3^mP3.
	uint64 mIncrement; // Product of prime powers, i.e. mRes2 * mRes3.
};

inline HaltonEnum::HaltonEnum(uint32 width, uint32 height) : SamplerEnum(width, height)
{
	assert(width && height);

	mP2 = 0;
	uint32 w = 1;
	while (w < width) // Find 2^mP2 >= width.
	{
		++mP2;
		w *= 2;
	}
	mScaleX = real(w);

	mP3 = 0;
	uint32 h = 1;
	while (h < height) // Find 3^mP3 >= height.
	{
		++mP3;
		h *= 3;
	}
	mScaleY = real(h);

	mIncrement = w * h; // There's exactly one sample per pixel.

	// Determine the multiplicative inverses.
	const std::pair<int, int> inv = ExtendedEuclid(static_cast<int>(h), static_cast<int>(w));
	const uint32 inv2 = (inv.first < 0) ? (inv.first + w) : (inv.first % w);
	const uint32 inv3 = (inv.second < 0) ? (inv.second + h) : (inv.second % h);
	assert((!inv2 && w == 1) || (inv2 > 0 && (h * inv2) % w == 1));
	assert((!inv3 && h == 1) || (inv3 > 0 && (w * inv3) % h == 1));
	mX = h * inv2;
	mY = w * inv3;
}

inline uint64 HaltonEnum::ComputeIndex(const uint64 i, const uint32 x, const uint32 y) const
{
	// Promote to 64 bits to avoid overflow.
	const uint64 hx = Halton2Inverse(x, mP2);
	const uint64 hy = Halton3Inverse(y, mP3);
	// Apply Chinese remainder theorem.
	const uint64  offset = (hx * mX + hy * mY) % mIncrement;
	return offset + i * mIncrement;
}

inline real HaltonEnum::ScaleX(const real x) const
{
	return x * mScaleX;
}

inline real HaltonEnum::ScaleY(const real y) const
{
	return y * mScaleY;
}

inline std::pair<int, int> HaltonEnum::ExtendedEuclid(const int a, const int b)
{
	if (!b)
		return std::make_pair(1u, 0u);
	const int q = a / b;
	const int r = a % b;
	const std::pair<int, int> st = ExtendedEuclid(b, r);
	return std::make_pair(st.second, st.first - q * st.second);
}

inline uint64 HaltonEnum::Halton2Inverse(uint64 index, const uint32 digits)
{
	index = ReverseBit64(index);
	return index >> (64 - digits);
}

inline uint64 HaltonEnum::Halton3Inverse(uint64 index, const uint32 digits)
{
	uint32 result = 0;
	for (uint32 d = 0; d < digits; ++d)
	{
		result = result * 3 + index % 3;
		index /= 3;
	}
	return result;
}



GYT_NAMESPACE_END