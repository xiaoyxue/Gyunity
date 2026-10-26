#pragma once
#include "common/Core.h"
#include "math/Linalg.h"
#include <string>
#include <vector>

GYT_NAMESPACE_BEGIN

// Display operators for low-dynamic-range outputs, following PBRT's imgtool "convert".
enum class ToneMapOperator { None, Reinhard, AcesFilmic };

struct ToneMapSettings {
	ToneMapOperator op = ToneMapOperator::None;
	// Luminance mapped to white by Reinhard (pbrt --maxluminance, default 1).
	real maxLuminance = 1;
	// Linear scale applied before tone mapping (pbrt --scale, default 1).
	real scale = 1;
};

bool ParseToneMapOperator(const std::string& name, ToneMapOperator* op);
const char* ToneMapOperatorName(ToneMapOperator op);

// pbrt-v3 imgtool --tonemap: y is the pbrt-v3 RGB luminance and every channel is
// multiplied by (1 + y / maxY^2) / (1 + y).
Vec3 ReinhardToneMap(const Vec3& rgb, real maxLuminance);

// pbrt-v4 imgtool --acesfilmic: Narkowicz's ACES fit applied per channel, with
// x <= 0 mapped to 0 and the result clamped to [0, 1]. No implicit exposure.
real AcesFilmic(real x);

// Applies scale, then the selected operator, in place. Fails on invalid settings or
// nonfinite pixels.
bool ApplyToneMap(std::vector<Vec3>& image, const ToneMapSettings& settings);

// pbrt GammaCorrect followed by its 8-bit quantization: Clamp(255 * sRGB(v) + 0.5, 0, 255).
unsigned char EncodeSrgb8(real value);

GYT_NAMESPACE_END
