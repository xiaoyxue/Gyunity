#include "gtest/gtest.h"
#include "visual/Scene.h"
#include "accelerator/BruteForceAccel.h"
#include "integrator/LT.h"
#include "camera/PinholeCamera.h"
#include "light/EnvLight.h"
#include "light/PointLight.h"
#include "light/AreaLight.h"
#include "shape/Sphere.h"

using namespace Gyunity;

namespace {

class FixedEmissionSamples : public StateSequence {
public:
	real Sample() override { return 0.3f; }
};

// Unlike a constant parameterless emission, the sampled direction determines radiance.
class SampledEnvironment : public Light {
public:
	bool IsEnvironmentLight() const override { return true; }
	Vec3 Power() const override { return Vec3(2); }
	Vec3 SampleLight(Intersection* point, Vec3* direction, real* positionPdf, real* directionPdf,
		const Vec2&, const Vec2&) const override {
		point->mPos = Vec3(0, 0, 1);
		point->mNormal = Vec3(0, 0, 1);
		*direction = Vec3(0, 0, 1);
		*positionPdf = 1;
		*directionPdf = 1;
		return Vec3(2, 3, 4);
	}
	Vec3 SampleLi(const Intersection&, Vec3*, real*, Intersection*, const Vec2&) const override {
		return Vec3(0);
	}
	real PdfLi(const Intersection&, const Vec3&) const override { return 0; }
};

Vec3 InitialLightWeight(const std::shared_ptr<Light>& light) {
	auto scene = std::make_shared<Scene>();
	scene->AddLight(light);
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
	auto film = std::make_shared<Film>(8, 8);
	PinHoleCamera camera(film, Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
	LightTracing tracer(std::make_shared<RandomSampler>(123), 2, 1);
	FixedEmissionSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path(3);
	const int count = tracer.GenerateLightPath(*scene, camera, samples, arena, path);
	EXPECT_EQ(count, 1);
	return path[0].mThroughput;
}

}

TEST(LightTracingEmission, UsesSampledEnvironmentRadiance) {
	auto environment = std::make_shared<SampledEnvironment>();
	ASSERT_EQ(environment->Emission(), Vec3(0));
	EXPECT_EQ(InitialLightWeight(environment), Vec3(2, 3, 4));
}

TEST(LightTracingEmission, PointAndAreaLightRadianceIsUnchanged) {
	EXPECT_EQ(InitialLightWeight(std::make_shared<PointLight>(Vec3(0, 0, 1), Vec3(3))), Vec3(3));
	auto sphere = std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3(0));
	sphere->mShapeId = 0;
	EXPECT_EQ(InitialLightWeight(std::make_shared<AreaLight>(sphere, Vec3(4))), Vec3(4));
}
