#include "gtest/gtest.h"
#include "image/ToneMap.h"
#include <limits>

using namespace Gyunity;

TEST(ToneMap, ReinhardMatchesPbrtV3Imgtool) {
	// pbrt-v3: y = 0.212671 R + 0.715160 G + 0.072169 B; scale = (1 + y / maxY^2) / (1 + y).
	const Vec3 white = ReinhardToneMap(Vec3(1), 1);
	EXPECT_NEAR(white.x, 1, 1e-6f);
	const Vec3 gray = ReinhardToneMap(Vec3(1), 2);
	EXPECT_NEAR(gray.y, 0.625f, 1e-6f);
	const Vec3 color(4, 0.5f, 0.25f);
	const double y = 0.212671 * 4 + 0.715160 * 0.5 + 0.072169 * 0.25;
	const double scale = (1 + y / 9) / (1 + y);
	const Vec3 mapped = ReinhardToneMap(color, 3);
	EXPECT_NEAR(mapped.x, 4 * scale, 1e-5);
	EXPECT_NEAR(mapped.y, 0.5 * scale, 1e-6);
	EXPECT_NEAR(mapped.z, 0.25 * scale, 1e-6);
}

TEST(ToneMap, AcesFilmicMatchesPbrtV4Imgtool) {
	EXPECT_EQ(AcesFilmic(-1), 0);
	EXPECT_EQ(AcesFilmic(0), 0);
	EXPECT_NEAR(AcesFilmic(1), 2.54 / 3.16, 1e-6);
	const double x = 0.18;
	EXPECT_NEAR(AcesFilmic(real(x)), x * (2.51 * x + 0.03) / (x * (2.43 * x + 0.59) + 0.14), 1e-6);
	EXPECT_EQ(AcesFilmic(100), 1);
}

TEST(ToneMap, SrgbEncodingMatchesPbrtByteQuantization) {
	EXPECT_EQ(EncodeSrgb8(-1), 0);
	EXPECT_EQ(EncodeSrgb8(0), 0);
	EXPECT_EQ(EncodeSrgb8(1), 255);
	EXPECT_EQ(EncodeSrgb8(2), 255);
	EXPECT_EQ(EncodeSrgb8(0.5f), 188);
	EXPECT_EQ(EncodeSrgb8(0.002f), 7);
}

TEST(ToneMap, ApplyScalesBeforeTheOperatorAndRejectsInvalidInput) {
	std::vector<Vec3> image = { Vec3(0.5f), Vec3(2, 1, 0) };
	ToneMapSettings settings;
	settings.op = ToneMapOperator::AcesFilmic;
	settings.scale = 2;
	ASSERT_TRUE(ApplyToneMap(image, settings));
	EXPECT_NEAR(image[0].x, AcesFilmic(1), 1e-6f);
	EXPECT_NEAR(image[1].x, AcesFilmic(4), 1e-6f);
	EXPECT_EQ(image[1].z, 0);
	std::vector<Vec3> invalid = { Vec3(std::numeric_limits<real>::infinity()) };
	EXPECT_FALSE(ApplyToneMap(invalid, settings));
	settings.maxLuminance = 0;
	EXPECT_FALSE(ApplyToneMap(image, settings));
	ToneMapOperator op;
	EXPECT_TRUE(ParseToneMapOperator("reinhard", &op));
	EXPECT_EQ(op, ToneMapOperator::Reinhard);
	EXPECT_FALSE(ParseToneMapOperator("filmic", &op));
}
