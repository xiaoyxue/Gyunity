#include "gtest/gtest.h"
#include "light/EnvMap.h"
#include <cmath>
#include <fstream>

using namespace Gyunity;

namespace {

class EnvironmentMapPdf : public testing::Test {
protected:
	std::unique_ptr<EnvironmentMap> environment;

	void SetUp() override {
		const char* path = "..\\resources\\media\\pictures\\uffizi-large.hdr";
		ASSERT_TRUE(std::ifstream(path, std::ios::binary).good())
			<< "Run GyunityTest from GyunityApp so scene resources resolve.";
		environment = std::make_unique<EnvironmentMap>(path, 0);
		// A small constant map makes the polar band and its analytical density explicit.
		environment->mImage = std::make_unique<ImageTexture<Vec3>>(
			Array2D<Vec3>(std::vector<Vec3>(16, Vec3(2)), 4, 4));
		real weights[16];
		for (int row = 0; row < 4; ++row) {
			for (int col = 0; col < 4; ++col) {
				weights[row * 4 + col] = std::sin(PI * (row + 0.5f) / 4);
			}
		}
		environment->mDistribution = std::make_unique<Distribution2D>(weights, 4, 4);
	}
};

}

TEST_F(EnvironmentMapPdf, SampleUsesContinuousLatitudeJacobian) {
	for (real latitudeSample : { 0.02f, 0.2f, 0.5f, 0.8f, 0.98f }) {
		const Vec2 sample(0.3f, latitudeSample);
		real uv[2], uvPdf;
		environment->mDistribution->SampleContinuous(sample.x, sample.y, uv, &uvPdf);
		Vec3 direction;
		real actual;
		const Vec3 radiance = environment->Sample(&direction, &actual, sample);
		const double expected = uvPdf / (2 * double(PI) * PI * std::sin(double(PI) * uv[1]));
		EXPECT_GT(actual, 0);
		EXPECT_NEAR(actual, expected, expected * 1e-5);
		EXPECT_EQ(radiance, Vec3(2));
		EXPECT_NEAR(environment->PdfDir(direction), expected, expected * 1e-4);
	}
}

TEST_F(EnvironmentMapPdf, LookupUsesSameContinuousJacobian) {
	for (real v : { 0.01f, 0.125f, 0.375f, 0.625f, 0.875f, 0.99f }) {
		// Use the known forward latitude, not an acos(float y) round trip
		// that loses transverse information near the poles.
		const double theta = double(PI) * v;
		const Vec3 direction(real(std::sin(theta)), real(std::cos(theta)), 0);
		const double expected = environment->mDistribution->Pdf(0, v) /
			(2 * double(PI) * PI * std::sin(theta));
		real pdf;
		environment->Lookup(direction, &pdf);
		EXPECT_GT(pdf, 0);
		EXPECT_NEAR(pdf, expected, expected * 1e-5);
		EXPECT_FLOAT_EQ(pdf, environment->PdfDir(direction));
	}
}

TEST_F(EnvironmentMapPdf, PolesHaveZeroPdf) {
	for (const Vec3& direction : { Vec3(0, 1, 0), Vec3(0, -1, 0) }) {
		real pdf = -1;
		environment->Lookup(direction, &pdf);
		EXPECT_EQ(pdf, 0);
		EXPECT_EQ(environment->PdfDir(direction), 0);
	}
}
