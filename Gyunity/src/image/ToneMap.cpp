#include "ToneMap.h"
#include "common/Logger.h"
#include <algorithm>
#include <cmath>

GYT_NAMESPACE_BEGIN

namespace {

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool Finite(const Vec3& value) {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

}

bool ParseToneMapOperator(const std::string& name, ToneMapOperator* op) {
	if (name == "none") *op = ToneMapOperator::None;
	else if (name == "reinhard") *op = ToneMapOperator::Reinhard;
	else if (name == "aces") *op = ToneMapOperator::AcesFilmic;
	else return false;
	return true;
}

const char* ToneMapOperatorName(ToneMapOperator op) {
	switch (op) {
	case ToneMapOperator::Reinhard: return "reinhard";
	case ToneMapOperator::AcesFilmic: return "aces";
	default: return "none";
	}
}

Vec3 ReinhardToneMap(const Vec3& rgb, real maxLuminance) {
	// pbrt-v3 RGBSpectrum::y() weights.
	const real y = real(0.212671) * rgb.x + real(0.715160) * rgb.y + real(0.072169) * rgb.z;
	const real scale = (1 + y / (maxLuminance * maxLuminance)) / (1 + y);
	return rgb * scale;
}

real AcesFilmic(real x) {
	if (x <= 0) return 0;
	const real a = real(2.51), b = real(0.03), c = real(2.43), d = real(0.59), e = real(0.14);
	return std::min(real(1), std::max(real(0), (x * (a * x + b)) / (x * (c * x + d) + e)));
}

bool ApplyToneMap(std::vector<Vec3>& image, const ToneMapSettings& settings) {
	if (!std::isfinite(settings.scale) || settings.scale <= 0)
		return Fail("Tone-map scale must be positive and finite.");
	if (!std::isfinite(settings.maxLuminance) || settings.maxLuminance <= 0)
		return Fail("Tone-map maximum luminance must be positive and finite.");
	for (Vec3& pixel : image) {
		if (!Finite(pixel)) return Fail("Cannot tone map a nonfinite pixel.");
		pixel = pixel * settings.scale;
		if (settings.op == ToneMapOperator::Reinhard) {
			pixel = ReinhardToneMap(pixel, settings.maxLuminance);
		}
		else if (settings.op == ToneMapOperator::AcesFilmic) {
			pixel = Vec3(AcesFilmic(pixel.x), AcesFilmic(pixel.y), AcesFilmic(pixel.z));
		}
	}
	return true;
}

unsigned char EncodeSrgb8(real value) {
	const real encoded = value <= real(0.0031308) ? real(12.92) * value
		: real(1.055) * std::pow(value, real(1) / real(2.4)) - real(0.055);
	return static_cast<unsigned char>(std::min(real(255), std::max(real(0), real(255) * encoded + real(0.5))));
}

GYT_NAMESPACE_END
