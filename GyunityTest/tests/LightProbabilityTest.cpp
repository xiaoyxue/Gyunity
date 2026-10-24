#include "gtest/gtest.h"
#include "visual/Scene.h"
#include "accelerator/BruteForceAccel.h"
#include "light/AreaLight.h"
#include "light/PointLight.h"
#include "light/EnvLight.h"
#include "shape/Sphere.h"
#include <limits>

using namespace Gyunity;

namespace {

std::shared_ptr<Triangle> EmittingTriangle() {
	auto shape = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(0, 0, 0), Vec3(2, 0, 0), Vec3(0, 2, 0), Vec3(0, 0, 1));
	shape->mShapeId = 0;
	return shape;
}

void Initialize(const std::shared_ptr<Scene>& scene) {
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
}

class LightProbabilityEnvironment : public testing::Test {
protected:
	std::shared_ptr<EnvironmentLight> environment;
	std::shared_ptr<Scene> scene;

	void SetUp() override {
		const char* path = "..\\resources\\media\\pictures\\uffizi-large.hdr";
		ASSERT_TRUE(std::ifstream(path, std::ios::binary).good())
			<< "Run GyunityTest from GyunityApp so resource paths resolve.";
		environment = std::make_shared<EnvironmentLight>(path, 0);
		scene = std::make_shared<Scene>();
		scene->AddPrimitive(std::make_shared<Primitive>(
			std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3(0))));
		scene->AddLight(environment);
	}
};

class UnsupportedEmissionLight : public Light {
public:
	Vec3 Power() const override { return Vec3(1); }
	Vec3 SampleLi(const Intersection&, Vec3*, real*, Intersection*, const Vec2&) const override { return Vec3(0); }
	real PdfLi(const Intersection&, const Vec3&) const override { return 0; }
};

}

TEST(LightProbability, DiscreteQueriesMatchSamplingAndKnownPmf) {
	const real weights[] = { 1, 3, 0 };
	Distribution1D distribution(weights, 3);
	const real expected[] = { 0.25f, 0.75f, 0 };
	int counts[3] = {};
	for (int i = 0; i < 10000; ++i) {
		real sampledPdf, queriedPdf;
		const int index = distribution.SampleDiscrete((i + 0.5f) / 10000, &sampledPdf);
		ASSERT_GE(index, 0);
		ASSERT_LT(index, 3);
		++counts[index];
		ASSERT_TRUE(distribution.DiscretePdf(index, &queriedPdf));
		EXPECT_FLOAT_EQ(sampledPdf, queriedPdf);
		EXPECT_NEAR(queriedPdf, expected[index], 1e-7f);
	}
	EXPECT_EQ(counts[0], 2500);
	EXPECT_EQ(counts[1], 7500);
	EXPECT_EQ(counts[2], 0);
	real sum = 0;
	for (int index = 0; index < 3; ++index) {
		real pdf;
		ASSERT_TRUE(distribution.DiscretePdf(index, &pdf));
		sum += pdf;
	}
	EXPECT_FLOAT_EQ(sum, 1);
}

TEST(LightProbability, ZeroWeightsUseTheExistingUniformCdf) {
	const real weights[] = { 0, 0 };
	Distribution1D distribution(weights, 2);
	for (real u : { 0.f, 0.25f, 0.75f, 1.f }) {
		real sampledPdf, queriedPdf;
		const int index = distribution.SampleDiscrete(u, &sampledPdf);
		ASSERT_TRUE(distribution.DiscretePdf(index, &queriedPdf));
		EXPECT_FLOAT_EQ(sampledPdf, 0.5f);
		EXPECT_FLOAT_EQ(queriedPdf, 0.5f);
	}
}

TEST(LightProbability, ClosedUpperEndpointDoesNotSelectTrailingZeroWeights) {
	const real weights[] = { 1, 0, 0 };
	Distribution1D distribution(weights, 3);
	real pdf;
	EXPECT_EQ(distribution.SampleDiscrete(1, &pdf), 0);
	EXPECT_FLOAT_EQ(pdf, 1);
}

TEST(LightProbability, InvalidQueriesReportFailureWithoutExceptions) {
	const real weights[] = { 1, 3 };
	Distribution1D distribution(weights, 2);
	real pdf = -1;
	EXPECT_FALSE(distribution.DiscretePdf(-1, &pdf));
	EXPECT_EQ(pdf, 0);
	EXPECT_FALSE(distribution.DiscretePdf(2, &pdf));
	EXPECT_EQ(pdf, 0);
	for (real u : { -0.1f, 1.1f, std::numeric_limits<real>::quiet_NaN() }) {
		EXPECT_EQ(distribution.SampleDiscrete(u, &pdf), -1);
		EXPECT_EQ(pdf, 0);
	}
	auto scene = std::make_shared<Scene>();
	EXPECT_FALSE(scene->LightSelectionPdf(0, &pdf));
	EXPECT_EQ(pdf, 0);
	EXPECT_EQ(scene->SampleOneLight(&pdf, 0.5f), nullptr);
	EXPECT_EQ(pdf, 0);
}

TEST(LightProbability, ScenePmfUsesTheSameDistributionBeforeAndAfterInitialization) {
	auto scene = std::make_shared<Scene>();
	scene->AddLight(std::make_shared<PointLight>(Vec3(0), Vec3(1)));
	scene->AddLight(std::make_shared<PointLight>(Vec3(1, 0, 0), Vec3(3)));
	real pdf;
	ASSERT_TRUE(scene->LightSelectionPdf(0, &pdf));
	EXPECT_FLOAT_EQ(pdf, 0.5f);
	Initialize(scene);
	ASSERT_TRUE(scene->LightSelectionPdf(0, &pdf));
	EXPECT_NEAR(pdf, 0.25f, 1e-6f);
	ASSERT_TRUE(scene->LightSelectionPdf(1, &pdf));
	EXPECT_NEAR(pdf, 0.75f, 1e-6f);
	for (int i = 0; i < 100; ++i) {
		real sampledPdf;
		Light* light = scene->SampleOneLight(&sampledPdf, (i + 0.5f) / 100);
		const size_t index = light == scene->GetLights()[0].get() ? 0 : 1;
		ASSERT_EQ(light, scene->GetLights()[index].get());
		ASSERT_TRUE(scene->LightSelectionPdf(index, &pdf));
		EXPECT_FLOAT_EQ(pdf, sampledPdf);
	}
}

TEST(LightProbability, AreaEmissionSampleAndPdfAgree) {
	AreaLight light(EmittingTriangle(), Vec3(2));
	for (const Vec2& sample : { Vec2(0.2f, 0.3f), Vec2(0.5f, 0.5f), Vec2(0.8f, 0.7f) }) {
		Intersection point;
		Vec3 dir;
		real sampledPos, sampledDir, queriedPos, queriedDir;
		light.SampleLight(&point, &dir, &sampledPos, &sampledDir, Vec2(0.3f, 0.4f), sample);
		ASSERT_TRUE(light.PdfLight(point, dir, &queriedPos, &queriedDir));
		EXPECT_FLOAT_EQ(queriedPos, 0.5f);
		EXPECT_FLOAT_EQ(queriedPos, sampledPos);
		EXPECT_NEAR(queriedDir, sampledDir, 1e-6f);
		const Vec3 originalPos = point.mPos;
		const Vec3 originalDir = dir;
		real repeatedPos, repeatedDir;
		ASSERT_TRUE(light.PdfLight(point, dir, &repeatedPos, &repeatedDir));
		EXPECT_EQ(point.mPos, originalPos);
		EXPECT_EQ(dir, originalDir);
		EXPECT_FLOAT_EQ(repeatedPos, queriedPos);
		EXPECT_FLOAT_EQ(repeatedDir, queriedDir);
	}
}

TEST(LightProbability, AreaDirectionalPdfIntegratesToOneAndRejectsBackside) {
	AreaLight light(EmittingTriangle(), Vec3(1));
	Intersection point;
	point.mNormal = Vec3(0, 0, 1);
	double integral = 0;
	for (int i = 0; i < 1024; ++i) {
		const real z = (i + 0.5f) / 1024;
		real pdfPos, pdfDir;
		ASSERT_TRUE(light.PdfLight(point, Vec3(std::sqrt(1 - z * z), 0, z), &pdfPos, &pdfDir));
		integral += pdfDir * 2 * double(PI) / 1024;
	}
	EXPECT_NEAR(integral, 1, 1e-6);
	real pdfPos, pdfDir;
	ASSERT_TRUE(light.PdfLight(point, Vec3(0, 0, -1), &pdfPos, &pdfDir));
	EXPECT_EQ(pdfDir, 0);
}

TEST(LightProbability, PointEmissionUsesDeltaPositionMassAndUniformDirection) {
	PointLight light(Vec3(1, 2, 3), Vec3(4));
	Intersection point;
	Vec3 dir;
	real sampledPos, sampledDir, pdfPos, pdfDir;
	light.SampleLight(&point, &dir, &sampledPos, &sampledDir, Vec2(0.2f, 0.3f), Vec2(0.4f, 0.7f));
	ASSERT_TRUE(light.IsDeltaLight());
	ASSERT_TRUE(light.PdfLight(point, dir, &pdfPos, &pdfDir));
	EXPECT_FLOAT_EQ(pdfPos, 1);
	EXPECT_FLOAT_EQ(pdfPos, sampledPos);
	EXPECT_FLOAT_EQ(pdfDir, sampledDir);
	EXPECT_NEAR(pdfDir * 4 * PI, 1, 1e-6f);
	point.mPos = Vec3(10, 0, 0);
	ASSERT_TRUE(light.PdfLight(point, dir, &pdfPos, &pdfDir));
	EXPECT_EQ(pdfPos, 0);
}

TEST(LightProbability, OriginPmfIsSeparateFromConditionalEmissionPdf) {
	auto scene = std::make_shared<Scene>();
	auto shape = EmittingTriangle();
	auto light = std::make_shared<AreaLight>(shape, Vec3(1));
	scene->AddLight(light);
	scene->AddLight(std::make_shared<AreaLight>(shape, Vec3(3)));
	Initialize(scene);
	Intersection point;
	point.mNormal = Vec3(0, 0, 1);
	real selectionPdf, positionPdf, directionPdf;
	ASSERT_TRUE(scene->LightSelectionPdf(0, &selectionPdf));
	ASSERT_TRUE(light->PdfLight(point, Vec3(0, 0, 1), &positionPdf, &directionPdf));
	EXPECT_NEAR(selectionPdf, 0.25f, 1e-6f);
	EXPECT_FLOAT_EQ(positionPdf, 0.5f);
	EXPECT_NEAR(selectionPdf * positionPdf * directionPdf, 0.125f * INV_PI, 1e-6f);
}

TEST(LightProbability, UnsupportedEmissionReturnsAnExplicitFailure) {
	UnsupportedEmissionLight light;
	Intersection point;
	real pdfPos = 1, pdfDir = 1;
	EXPECT_FALSE(light.PdfLight(point, Vec3(0, 0, 1), &pdfPos, &pdfDir));
	EXPECT_EQ(pdfPos, 0);
	EXPECT_EQ(pdfDir, 0);
}

TEST_F(LightProbabilityEnvironment, RequiresInitializedBounds) {
	real pdfPos = 1, pdfDir = 1;
	Intersection point;
	EXPECT_FALSE(environment->PdfLight(point, Vec3(0, 0, 1), &pdfPos, &pdfDir));
	EXPECT_EQ(pdfPos, 0);
	EXPECT_EQ(pdfDir, 0);
}

TEST_F(LightProbabilityEnvironment, EmissionPdfMatchesSamplingAndUsesOppositeMapDirection) {
	Initialize(scene);
	for (const Vec2& sample : { Vec2(0.2f, 0.3f), Vec2(0.5f, 0.6f), Vec2(0.8f, 0.4f) }) {
		Intersection point;
		Vec3 dir;
		real sampledPos, sampledDir, pdfPos, pdfDir;
		environment->SampleLight(&point, &dir, &sampledPos, &sampledDir, sample, Vec2(0.3f, 0.4f));
		ASSERT_TRUE(environment->PdfLight(point, dir, &pdfPos, &pdfDir));
		EXPECT_NEAR(pdfPos, 1 / (3 * PI), 1e-6f);
		EXPECT_FLOAT_EQ(pdfPos, sampledPos);
		EXPECT_NEAR(pdfDir, sampledDir, std::max(real(1e-6), sampledDir * 2e-3f));
		EXPECT_FLOAT_EQ(pdfDir, environment->PdfLi(point, -dir));
		point.mPos += dir * 1000;
		real shiftedPos, shiftedDir;
		ASSERT_TRUE(environment->PdfLight(point, dir, &shiftedPos, &shiftedDir));
		EXPECT_FLOAT_EQ(shiftedPos, pdfPos);
		EXPECT_FLOAT_EQ(shiftedDir, pdfDir);
	}
}

TEST_F(LightProbabilityEnvironment, MixedFiniteAndEnvironmentLightPmfsMatchSelection) {
	scene->AddLight(std::make_shared<PointLight>(Vec3(0, 2, 0), Vec3(100)));
	Initialize(scene);
	double sum = 0;
	for (size_t i = 0; i < scene->GetLights().size(); ++i) {
		real pdf;
		ASSERT_TRUE(scene->LightSelectionPdf(i, &pdf));
		sum += pdf;
	}
	EXPECT_NEAR(sum, 1, 1e-6);
	for (int i = 0; i < 100; ++i) {
		real sampledPdf, queriedPdf;
		const Light* light = scene->SampleOneLight(&sampledPdf, (i + 0.5f) / 100);
		const size_t index = light == environment.get() ? 0 : 1;
		ASSERT_TRUE(scene->LightSelectionPdf(index, &queriedPdf));
		EXPECT_FLOAT_EQ(sampledPdf, queriedPdf);
	}
}
