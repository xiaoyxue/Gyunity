#include "gtest/gtest.h"
#include "integrator/PT.h"
#include "integrator/VPT.h"
#include "integrator/SPPM.h"
#include "accelerator/BruteForceAccel.h"
#include "camera/PinholeCamera.h"
#include "image/ImageIO.h"
#include "light/AreaLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Mirror.h"
#include "material/Roughness.h"
#include "texture/ConstantTexture.h"
#include <cmath>
#include <cstdlib>

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
enum class EmitterIntegrator { Path, Volume, EquiAngularVolume, Sppm, SppmGlossy };

class EyePhotonSampler : public Sampler {
public:
	explicit EyePhotonSampler(real eyeValue) : mEyeValue(eyeValue) {}
	real Sample(unsigned int, unsigned long long instance) override {
		return instance == 0 ? 0.97f : mEyeValue;
	}
	std::shared_ptr<Sampler> Clone(uint32) override {
		return std::make_shared<EyePhotonSampler>(mEyeValue);
	}
private:
	const real mEyeValue;
};

class CenterEyeSample : public SamplerEnum {
public:
	CenterEyeSample() : SamplerEnum(1, 1) {}
	uint64 GetIndex(uint32, uint32, uint32) const override { return 1; }
	real SampleX(uint32, real) const override { return 0.5f; }
	real SampleY(uint32, real) const override { return 0.5f; }
};

bool RenderSppmSample(const Scene& scene, const Ray& ray, int depth, real sample,
	bool traceGlossy, const std::string& filename, Vec3* result) {
	auto film = std::make_shared<Film>(1, 1);
	PinHoleCamera camera(film, ray.mOrig, ray.mOrig + ray.mDir, Vec3(0, 1, 0), 45, 1);
	SPPM integrator(1, 1, depth, 0.25f, 0.7f, true,
		std::make_shared<EyePhotonSampler>(sample), std::make_shared<CenterEyeSample>(), traceGlossy);
	// The point-light photon first hits the emitter or misses. A possible second
	// hit is the mirror, so these fixtures have no gathered indirect flux.
	integrator.Render(scene, camera);
	film->SetFileName(filename);
	if (!film->SaveImage()) return false;
	const auto image = ImageIO::LoadImage(filename, 1, 1);
	if (image.size() != 1) {
		ADD_FAILURE() << "Unexpected SPPM output size: " << filename;
		return false;
	}
	for (int channel = 0; channel < 3; ++channel) {
		if (!std::isfinite(image[0][channel]) || image[0][channel] < 0) {
			ADD_FAILURE() << "Invalid SPPM radiance: " << filename;
			return false;
		}
	}
	*result = image[0];
	return true;
}

void ExpectEmitterRadiance(EmitterView view, bool emissive, real reflectance,
	EmitterIntegrator backend = EmitterIntegrator::Path) {
	const bool sppm = backend == EmitterIntegrator::Sppm || backend == EmitterIntegrator::SppmGlossy;
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	if (sppm && !output) {
		GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test folder.";
	}
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
	const int depth = view == EmitterView::Mirror ? 2 : 1;
	Vec3 mean(0);
	real cumulative = 0;
	const real emission = emissive && view != EmitterView::Back ? 2.f : 0.f;
	for (size_t selected = 0; selected < scene.GetLights().size(); ++selected) {
		real pmf;
		ASSERT_TRUE(scene.LightSelectionPdf(selected, &pmf));
		ASSERT_GT(pmf, 0);
		const real sample = cumulative + 0.5f * pmf;
		ConstantSequence sequence(sample);
		MemoryPool arena;
		Vec3 result;
		if (backend == EmitterIntegrator::Path) {
			PathTracing integrator(1, depth, std::make_shared<RandomSampler>(1), nullptr);
			result = integrator.Li(ray, scene, sequence, arena);
		}
		else if (!sppm) {
			VolPathTracing integrator(1, depth, std::make_shared<RandomSampler>(1), nullptr,
				backend == EmitterIntegrator::EquiAngularVolume);
			result = integrator.Li(ray, scene, sequence, arena);
		}
		else {
			const std::string filename = std::string(output) + "\\emission-" +
				std::to_string(static_cast<int>(backend)) + "-" + std::to_string(static_cast<int>(view)) +
				"-" + std::to_string(emissive) + "-" + std::to_string(reflectance) +
				"-" + std::to_string(selected) + ".hdr";
			ASSERT_TRUE(RenderSppmSample(scene, ray, depth, sample,
				backend == EmitterIntegrator::SppmGlossy, filename, &result));
		}
		const real direct = selected == 1 ? reflectance * INV_PI / pmf : 0;
		// SPPM is read through the existing RGBE writer, not a private-buffer hook.
		const real tolerance = sppm ? 0.01f * std::max(1.f, emission + direct) : 2e-6f;
		for (int channel = 0; channel < 3; ++channel) {
			EXPECT_NEAR(result[channel], emission + direct, tolerance);
		}
		mean += result * pmf;
		cumulative += pmf;
	}
	const real expectedMean = emission + reflectance * INV_PI;
	const real tolerance = sppm ? 0.01f * std::max(1.f, expectedMean) : 2e-6f;
	for (int channel = 0; channel < 3; ++channel) {
		EXPECT_NEAR(mean[channel], expectedMean, tolerance);
	}
	testing::Test::RecordProperty("actual_mean_r", std::to_string(mean.x));
	testing::Test::RecordProperty("expected_mean_r", std::to_string(expectedMean));
}

void ExpectSppmEmissionOnScatteringBranch(bool glossy) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	if (!output) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test folder.";
	std::shared_ptr<Material> material;
	if (glossy) {
		auto roughness = std::make_shared<ConstantTexture<real>>(0.3f);
		material = std::make_shared<RoughnessMaterial>(
			std::make_shared<ConstantTexture<Vec3>>(Vec3(0.5f)), roughness, roughness,
			std::make_shared<ConstantTexture<Vec3>>(Vec3(1.5f)),
			std::make_shared<ConstantTexture<Vec3>>(Vec3(2)));
	}
	else {
		material = std::make_shared<MirrorMaterial>(
			std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
	}
	for (bool traceGlossy : { false, true }) {
		Vec3 radiance[2];
		for (int tagged = 0; tagged < 2; ++tagged) {
			Scene scene{};
			auto triangle = std::make_shared<Triangle>(Transform(), Transform(),
				Vec3(-2, -2, 0), Vec3(2, -2, 0), Vec3(0, 2, 0), Vec3(0, 0, 1));
			auto emitter = std::make_shared<AreaLight>(triangle, Vec3(2));
			scene.AddPrimitive(std::make_shared<Primitive>(triangle, material, tagged ? emitter : nullptr));
			scene.AddLight(emitter);
			scene.AddLight(std::make_shared<PointLight>(Vec3(0, 0, 2), Vec3(4)));
			scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
			scene.Initialize();
			const std::string filename = std::string(output) + "\\emission-sppm-branch-" +
				std::to_string(glossy) + "-" + std::to_string(traceGlossy) + "-" +
				std::to_string(tagged) + ".hdr";
			ASSERT_TRUE(RenderSppmSample(scene, Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)), 2, 0.75f,
				traceGlossy, filename, &radiance[tagged]));
		}
		for (int channel = 0; channel < 3; ++channel) {
			const real tolerance = 0.02f * std::max(1.f, radiance[1][channel]);
			EXPECT_NEAR(radiance[1][channel] - radiance[0][channel], 2, tolerance);
		}
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

class EmissiveSurfaceIntegrators : public testing::TestWithParam<EmitterIntegrator> {};

TEST_P(EmissiveSurfaceIntegrators, PrimaryHitIncludesReflectedDirectLighting) {
	ExpectEmitterRadiance(EmitterView::Front, true, 0.5f, GetParam());
}

TEST_P(EmissiveSurfaceIntegrators, SpecularHitIncludesReflectedDirectLighting) {
	ExpectEmitterRadiance(EmitterView::Mirror, true, 0.5f, GetParam());
}

TEST_P(EmissiveSurfaceIntegrators, NonemittingBackSideStillReflectsDirectLighting) {
	ExpectEmitterRadiance(EmitterView::Back, true, 0.5f, GetParam());
}

TEST_P(EmissiveSurfaceIntegrators, NonemissiveSurfacePreservesDirectLighting) {
	ExpectEmitterRadiance(EmitterView::Front, false, 0.5f, GetParam());
}

TEST_P(EmissiveSurfaceIntegrators, AbsorbingEmitterPreservesEmission) {
	ExpectEmitterRadiance(EmitterView::Front, true, 0, GetParam());
}

TEST_P(EmissiveSurfaceIntegrators, SpecularChainCountsEmissionOnce) {
	ExpectEmitterRadiance(EmitterView::Mirror, true, 0, GetParam());
}

INSTANTIATE_TEST_SUITE_P(VptAndSppm, EmissiveSurfaceIntegrators,
	testing::Values(EmitterIntegrator::Volume, EmitterIntegrator::EquiAngularVolume,
		EmitterIntegrator::Sppm, EmitterIntegrator::SppmGlossy),
	([](const testing::TestParamInfo<EmitterIntegrator>& info) {
		const char* names[] = { "PT", "VPT", "VPTEquiAngular", "SPPM", "SPPMTraceGlossy" };
		return std::string(names[static_cast<int>(info.param)]);
	}));

TEST(SppmSurfaceEmission, DeltaEmitterStillEmits) {
	ExpectSppmEmissionOnScatteringBranch(false);
}

TEST(SppmSurfaceEmission, GlossyEmitterEmitsWithAndWithoutContinuation) {
	ExpectSppmEmissionOnScatteringBranch(true);
}
