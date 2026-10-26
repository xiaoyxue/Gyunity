#include "gtest/gtest.h"
#include "integrator/PT.h"
#include "accelerator/BruteForceAccel.h"
#include "light/AreaLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Mirror.h"
#include "texture/ConstantTexture.h"

using namespace Gyunity;

namespace {

class ConstantSequence : public StateSequence {
public:
	explicit ConstantSequence(real value) : mValue(value) {}
	real Sample() override {
		++mCursor;
		return mValue;
	}
private:
	real mValue;
};

enum class EmitterView { Front, Back, Mirror };

void ExpectEmitterRadiance(EmitterView view, bool emissive, real reflectance) {
	Scene scene{};
	auto triangle = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-2, -2, 0), Vec3(2, -2, 0), Vec3(0, 2, 0), Vec3(0, 0, 1));
	auto material = std::make_shared<DiffuseMaterial>(
		std::make_shared<ConstantTexture<Vec3>>(Vec3(reflectance)));
	auto emitter = std::make_shared<AreaLight>(triangle, Vec3(2));
	scene.AddPrimitive(std::make_shared<Primitive>(triangle, material, emissive ? emitter : nullptr));
	// Keep selection probabilities unchanged in the nonemissive control.
	scene.AddLight(emitter);
	const real lightZ = view == EmitterView::Mirror ? 0.5f : view == EmitterView::Back ? -2.f : 2.f;
	scene.AddLight(std::make_shared<PointLight>(Vec3(0, 0, lightZ), Vec3(lightZ * lightZ)));
	if (view == EmitterView::Mirror) {
		auto mirror = std::make_shared<Triangle>(Transform(), Transform(),
			Vec3(-2, -2, 1), Vec3(2, -2, 1), Vec3(0, 2, 1), Vec3(0, 0, -1));
		scene.AddPrimitive(std::make_shared<Primitive>(mirror,
			std::make_shared<MirrorMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)))));
	}
	scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
	scene.Initialize();

	const Ray ray = view == EmitterView::Mirror
		? Ray(Vec3(0, 0, 0.5f), Vec3(0, 0, 1))
		: view == EmitterView::Back ? Ray(Vec3(0, 0, -1), Vec3(0, 0, 1))
		: Ray(Vec3(0, 0, 1), Vec3(0, 0, -1));
	PathTracing integrator(1, view == EmitterView::Mirror ? 2 : 1,
		std::make_shared<RandomSampler>(1), nullptr);
	Vec3 mean(0);
	real cumulative = 0;
	const real emission = emissive && view != EmitterView::Back ? 2.f : 0.f;
	for (size_t selected = 0; selected < scene.GetLights().size(); ++selected) {
		real pmf;
		ASSERT_TRUE(scene.LightSelectionPdf(selected, &pmf));
		ASSERT_GT(pmf, 0);
		ConstantSequence sequence(cumulative + 0.5f * pmf);
		MemoryPool arena;
		const Vec3 result = integrator.Li(ray, scene, sequence, arena);
		const real direct = selected == 1 ? reflectance * INV_PI / pmf : 0;
		for (int channel = 0; channel < 3; ++channel) {
			EXPECT_NEAR(result[channel], emission + direct, 2e-6f);
		}
		mean += result * pmf;
		cumulative += pmf;
	}
	for (int channel = 0; channel < 3; ++channel) {
		EXPECT_NEAR(mean[channel], emission + reflectance * INV_PI, 2e-6f);
	}
}

}

TEST(PathTracingEmission, PrimaryHitIncludesReflectedDirectLighting) {
	ExpectEmitterRadiance(EmitterView::Front, true, 0.5f);
}

TEST(PathTracingEmission, SpecularHitIncludesReflectedDirectLighting) {
	ExpectEmitterRadiance(EmitterView::Mirror, true, 0.5f);
}

TEST(PathTracingEmission, NonemittingBackSideStillReflectsDirectLighting) {
	ExpectEmitterRadiance(EmitterView::Back, true, 0.5f);
}

TEST(PathTracingEmission, NonemissiveSurfacePreservesDirectLighting) {
	ExpectEmitterRadiance(EmitterView::Front, false, 0.5f);
}

TEST(PathTracingEmission, AbsorbingEmitterPreservesEmission) {
	ExpectEmitterRadiance(EmitterView::Front, true, 0);
}
