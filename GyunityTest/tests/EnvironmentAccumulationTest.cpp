#include "gtest/gtest.h"
#include "visual/Scene.h"
#include "accelerator/BruteForceAccel.h"
#include "material/Mirror.h"
#include "texture/ConstantTexture.h"
#include "light/AreaLight.h"
#include "integrator/PT.h"
#include "integrator/VPT.h"
#include "sampler/SamplerEnum.h"

using namespace Gyunity;

namespace {

// A constant environment removes texture sampling from this accumulation test.
class ConstantEnvironment : public Light {
public:
	bool IsEnvironmentLight() const override { return true; }
	Vec3 Emission(const Ray&) const override { return Vec3(2); }
	Vec3 Power() const override { return Vec3(2); }
	Vec3 SampleLi(const Intersection&, Vec3*, real*, Intersection*, const Vec2&) const override {
		ADD_FAILURE() << "The emissive mirror path must not invoke direct-light sampling.";
		return Vec3(0);
	}
	real PdfLi(const Intersection&, const Vec3&) const override { return 0; }
};

class FixedEnvironmentSamples : public StateSequence {
public:
	real Sample() override { return 0.3f; }
};

std::shared_ptr<Scene> MakeScene(bool withEnvironment) {
	auto scene = std::make_shared<Scene>();
	auto shape = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-2, -2, 0), Vec3(2, -2, 0), Vec3(0, 2, 0), Vec3(0, 0, 1));
	auto mirror = std::make_shared<MirrorMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
	auto emitter = std::make_shared<AreaLight>(shape, Vec3(3));
	scene->AddPrimitive(std::make_shared<Primitive>(shape, mirror, emitter));
	scene->AddLight(emitter);
	if (withEnvironment) scene->AddLight(std::make_shared<ConstantEnvironment>());
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
	return scene;
}

}

TEST(EnvironmentAccumulation, SurfaceAndEnvironmentContributionsAreBothRetained) {
	auto sampler = std::make_shared<RandomSampler>(123);
	auto enumeration = std::make_shared<SamplerEnum>();
	PathTracing pt(1, 3, sampler, enumeration);
	VolPathTracing vpt(1, 3, sampler, enumeration, false);
	for (bool withEnvironment : { false, true }) {
		auto scene = MakeScene(withEnvironment);
		FixedEnvironmentSamples samples;
		MemoryPool arena;
		const Ray ray(Vec3(0, 0, 1), Vec3(0, 0, -1));
		const Vec3 surfaceResult = pt.Li(ray, *scene, samples, arena);
		arena.Reset();
		const Vec3 volumeResult = vpt.Li(ray, *scene, samples, arena);
		const real expected = withEnvironment ? 5.f : 3.f;
		for (int channel = 0; channel < 3; ++channel) {
			EXPECT_NEAR(surfaceResult[channel], expected, 1e-5f);
			EXPECT_NEAR(volumeResult[channel], expected, 1e-5f);
		}
	}
}

TEST(EnvironmentAccumulation, PrimaryEnvironmentEscapeIsNotDoubleCounted) {
	auto scene = MakeScene(true);
	auto sampler = std::make_shared<RandomSampler>(123);
	auto enumeration = std::make_shared<SamplerEnum>();
	PathTracing pt(1, 3, sampler, enumeration);
	VolPathTracing vpt(1, 3, sampler, enumeration, false);
	FixedEnvironmentSamples samples;
	MemoryPool arena;
	const Ray ray(Vec3(0, 0, 1), Vec3(0, 0, 1));
	EXPECT_EQ(pt.Li(ray, *scene, samples, arena), Vec3(2));
	EXPECT_EQ(vpt.Li(ray, *scene, samples, arena), Vec3(2));
}
