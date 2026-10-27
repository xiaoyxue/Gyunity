#include "gtest/gtest.h"
#include "integrator/PT.h"
#include "integrator/VPT.h"
#include "accelerator/BruteForceAccel.h"
#include "camera/PinholeCamera.h"
#include "image/ImageIO.h"
#include "light/AreaLight.h"
#include "light/EnvLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Mirror.h"
#include "shape/Sphere.h"
#include "sampler/SamplerEnum.h"
#include "texture/ConstantTexture.h"
#include "volume/Homogeneous.h"
#include "../../GyunityApp/SceneRenderer.h"
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <limits>

using namespace Gyunity;

namespace {

class AuditSequence : public StateSequence {
public:
	explicit AuditSequence(real value, real first = -1) : mValue(value), mFirst(first) {}
	real Sample() override { return mCursor++ == 0 && mFirst >= 0 ? mFirst : mValue; }
private:
	real mValue, mFirst;
};

// Deliberately inherits Light's unsupported SampleOnePoint implementation.
class AuditEnvironment : public Light {
public:
	bool IsEnvironmentLight() const override { return true; }
	Vec3 Power() const override { return Vec3(4 * PI); }
	Vec3 Emission(const Ray&) const override { return Vec3(1); }
	Vec3 Emission(const Vec3&, const Vec3& = Vec3()) const override { return Vec3(1); }
	real PdfLi(const Intersection&, const Vec3&) const override { return UniformSpherePdf(); }
	Vec3 SampleLi(const Intersection& hit, Vec3* wi, real* pdf,
		Intersection* point, const Vec2& u) const override {
		*wi = UniformSampleSphere(u);
		*pdf = UniformSpherePdf();
		point->mPos = hit.mPos + *wi * 100;
		return Vec3(1);
	}
};

class AuditUnsupportedPoint : public Light {
public:
	bool IsDeltaLight() const override { return true; }
	Vec3 Power() const override { return Vec3(4 * PI); }
	real PdfLi(const Intersection&, const Vec3&) const override { return 0; }
	Vec3 SampleLi(const Intersection& hit, Vec3* wi, real* pdf,
		Intersection* point, const Vec2&) const override {
		point->mPos = Vec3(2, 1, 2);
		*wi = (point->mPos - hit.mPos).Norm();
		*pdf = 1;
		return Vec3(1 / (point->mPos - hit.mPos).Length2());
	}
};

class AuditMixedMaterial : public Material {
public:
	void ComputeScatteringFunction(Intersection* hit, MemoryPool& arena,
		TransportMode = TransportMode::Radiance) const override {
		hit->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*hit);
		hit->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(Vec3(0.5f)));
		hit->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(0.25f)));
	}
};

class AuditCountingMedium : public HomogeneousMedium {
public:
	AuditCountingMedium() : HomogeneousMedium(Vec3(0), Vec3(0.5f), 0) {}
	Vec3 Sample(const Ray& ray, StateSequence& sequence, MemoryPool& arena,
		MediumIntersection* hit) const override {
		++sampleCalls;
		return HomogeneousMedium::Sample(ray, sequence, arena, hit);
	}
	mutable int sampleCalls = 0;
};

class AuditPointDensityShape : public Triangle {
public:
	explicit AuditPointDensityShape(real density) : Triangle(Transform(), Transform(),
		Vec3(-2, -2, 0), Vec3(2, -2, 0), Vec3(0, 2, 0), Vec3(0, 0, -1)), mDensity(density) {}
	Intersection Sample(real* pdf, const Vec2& u) const override {
		Intersection result = Triangle::Sample(pdf, u);
		*pdf = mDensity;
		return result;
	}
private:
	real mDensity;
};

class AuditSelectionSampler : public Sampler {
public:
	explicit AuditSelectionSampler(real selection) : mSelection(selection) {}
	real Sample(unsigned int dimension, unsigned long long) override {
		return dimension == 2 ? mSelection : 0.3f;
	}
	std::shared_ptr<Sampler> Clone(uint32) override {
		return std::make_shared<AuditSelectionSampler>(mSelection);
	}
private:
	real mSelection;
};

SceneRenderOptions AuditCheckedOptions(const std::shared_ptr<Scene>& scene, const std::string& output) {
	SceneRenderOptions options;
	options.width = options.height = 4;
	options.spp = options.maxDepth = 1;
	options.output = output;
	options.nativeScene = std::make_shared<NativeScene>();
	options.nativeScene->scene = scene;
	options.nativeScene->fovY = 30;
	options.nativeScene->cameraToWorld = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, -1, 0, 0, 0, 1 };
	return options;
}

void AuditExpectRejectedOutput(const SceneRenderOptions& options, Integrator& integrator) {
	std::error_code error;
	std::filesystem::remove(options.output, error);
	ASSERT_FALSE(error);
	EXPECT_FALSE(RenderScene(options, integrator));
	EXPECT_FALSE(std::filesystem::exists(options.output, error));
	EXPECT_FALSE(error);
}

std::shared_ptr<Material> AuditDiffuse(real reflectance) {
	return std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(reflectance)));
}

std::shared_ptr<Triangle> AuditPlane(real z, real normalZ) {
	Vec2 uv[] = { Vec2(0, 0), Vec2(1, 0), Vec2(0.5f, 1) };
	return std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-100, -100, z), Vec3(100, -100, z), Vec3(0, 100, z), Vec3(0, 0, normalZ), uv);
}

void AuditInitialize(Scene& scene) {
	scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
	scene.Initialize();
}

Vec3 AuditTrace(const Scene& scene, const Ray& ray, int backend, int depth, real value = 0.3f) {
	AuditSequence sequence(value);
	MemoryPool arena;
	auto sampler = std::make_shared<RandomSampler>(1);
	if (backend == 0) return PathTracing(1, depth, sampler, nullptr).Li(ray, scene, sequence, arena);
	return VolPathTracing(1, depth, sampler, nullptr, backend == 2).Li(ray, scene, sequence, arena);
}

void AuditExpect(const Vec3& value, const Vec3& expected, real tolerance = 2e-6f) {
	for (int c = 0; c < 3; ++c) {
		EXPECT_TRUE(std::isfinite(value[c]));
		EXPECT_NEAR(value[c], expected[c], tolerance) << "channel " << c;
	}
}

void AuditEmitter(Scene& scene, real z, real normalZ) {
	auto shape = AuditPlane(z, normalZ);
	auto light = std::make_shared<AreaLight>(shape, Vec3(1));
	scene.AddPrimitive(std::make_shared<Primitive>(shape, AuditDiffuse(0), light));
	scene.AddLight(light);
}

bool AuditRender(const Scene& scene, int backend, int depth, int spp,
	const Vec3& origin, const Vec3& target, const std::string& filename,
	std::vector<Vec3>* pixels) {
	const int size = 16;
	auto film = std::make_shared<Film>(size, size);
	PinHoleCamera camera(film, origin, target, Vec3(0, 1, 0), 30, 1);
	auto sampler = std::make_shared<RandomSampler>(165);
	auto enumeration = std::make_shared<SamplerEnum>(size, size);
	if (backend == 0) PathTracing(spp, depth, sampler, enumeration).Render(scene, camera);
	else VolPathTracing(spp, depth, sampler, enumeration, backend == 2).Render(scene, camera);
	film->SetFileName(filename);
	if (!film->SaveImage()) return false;
	int width = 0, height = 0;
	if (!ImageIO::LoadImage(filename, *pixels, width, height)) return false;
	return width == size && height == size;
}

Vec3 AuditMean(const std::vector<Vec3>& pixels) {
	Vec3 mean;
	for (const Vec3& pixel : pixels) mean += pixel;
	return mean / static_cast<real>(pixels.size());
}

}

TEST(PathVolumeAudit, PrimaryEmissionAtZeroDepth) {
	for (bool environment : { false, true }) {
		Scene scene{};
		if (environment) scene.AddLight(std::make_shared<AuditEnvironment>());
		else AuditEmitter(scene, 0, 1);
		AuditInitialize(scene);
		for (int backend = 0; backend < 3; ++backend)
			AuditExpect(AuditTrace(scene, Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)), backend, 0), Vec3(1));
	}
}

TEST(PathVolumeAudit, MirrorEmissionAtOneScatteringAndNullBoundaries) {
	for (bool environment : { false, true }) {
		Scene scene{};
		scene.AddPrimitive(std::make_shared<Primitive>(AuditPlane(0, 1),
			std::make_shared<MirrorMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)))));
		scene.AddPrimitive(std::make_shared<Primitive>(AuditPlane(1.5f, -1), nullptr));
		if (environment) scene.AddLight(std::make_shared<AuditEnvironment>());
		else AuditEmitter(scene, 2, -1);
		AuditInitialize(scene);
		for (int backend = 0; backend < 3; ++backend) {
			const Ray ray(Vec3(0, 0, 1), Vec3(0, 0, -1));
			AuditExpect(AuditTrace(scene, ray, backend, 0), Vec3(0));
			for (int depth : { 1, 2 })
				AuditExpect(AuditTrace(scene, ray, backend, depth), Vec3(1));
		}
	}
}

TEST(PathVolumeAudit, DiffuseNeeUsesOneScattering) {
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(AuditPlane(0, 1), AuditDiffuse(0.5f)));
	scene.AddLight(std::make_shared<PointLight>(Vec3(0, 0, 2), Vec3(4)));
	AuditInitialize(scene);
	for (int backend = 0; backend < 3; ++backend) {
		const Ray ray(Vec3(0, 0, 1), Vec3(0, 0, -1));
		AuditExpect(AuditTrace(scene, ray, backend, 0), Vec3(0));
		AuditExpect(AuditTrace(scene, ray, backend, 1), Vec3(0.5f * INV_PI));
	}
}

TEST(PathVolumeAudit, MixedLobesCreditOnlyDeltaContinuation) {
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(AuditPlane(0, 1), std::make_shared<AuditMixedMaterial>()));
	scene.AddLight(std::make_shared<AuditEnvironment>());
	AuditInitialize(scene);
	for (int backend = 0; backend < 3; ++backend) {
		for (real sample : { 0.2f, 0.8f }) {
			const Ray ray(Vec3(0, 0, 1), Vec3(0, 0, -1));
			const Vec3 one = AuditTrace(scene, ray, backend, 1, sample);
			const Vec3 two = AuditTrace(scene, ray, backend, 2, sample);
			AuditExpect(one, two);
		}
	}
}

TEST(PathVolumeAudit, ZeroDepthMediumAttenuatesThroughNullBoundary) {
	HomogeneousMedium medium(Vec3(0.5f), Vec3(0), 0);
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(AuditPlane(0, 1), nullptr, nullptr,
		MediumInterface(&medium, nullptr)));
	scene.AddLight(std::make_shared<AuditEnvironment>());
	AuditInitialize(scene);
	for (int backend : { 1, 2 })
		AuditExpect(AuditTrace(scene, Ray(Vec3(0, 0, -1), Vec3(0, 0, 1), MAX_REAL, &medium),
			backend, 0), Vec3(std::exp(-0.5f)), 1e-5f);
}

TEST(PathVolumeAudit, ZeroExtinctionChannelHasNoCollisionAndMixturePdf) {
	HomogeneousMedium medium(Vec3(0), Vec3(0.5f, 0.5f, 0), 0);
	for (real u : { 0.f, 1e-8f, 0.5f }) {
		AuditSequence sequence(u, 0.9f);
		MemoryPool arena;
		MediumIntersection hit;
		const Vec3 weight = medium.Sample(Ray(Vec3(), Vec3(0, 0, 2), 1), sequence, arena, &hit);
		EXPECT_FALSE(hit.IsValid());
		const Vec3 tr(std::exp(-1.f), std::exp(-1.f), 1);
		AuditExpect(weight, tr / ((2 * std::exp(-1.f) + 1) / 3));
	}
}

TEST(PathVolumeAudit, TinyFreeFlightSampleRetainsPositiveDistance) {
	HomogeneousMedium medium(Vec3(0), Vec3(0.5f), 0);
	AuditSequence sequence(1e-8f, 0.1f);
	MemoryPool arena;
	MediumIntersection hit;
	const Vec3 weight = medium.Sample(Ray(Vec3(), Vec3(0, 0, 1), 1), sequence, arena, &hit);
	ASSERT_TRUE(hit.IsValid());
	EXPECT_GT(hit.mPos.z, 0);
	EXPECT_NEAR(hit.mPos.z, -std::log1p(-1e-8f) / 0.5f, 1e-13f);
	AuditExpect(weight, Vec3(1));
}

TEST(PathVolumeAudit, EveryFreeFlightChannelUsesTheMixtureDensity) {
	const Vec3 sigma(0.25f, 0.5f, 1);
	HomogeneousMedium medium(Vec3(0), sigma, 0);
	for (int channel = 0; channel < 3; ++channel) {
		AuditSequence sequence(0.2f, (channel + 0.5f) / 3);
		MemoryPool arena;
		MediumIntersection hit;
		const Vec3 weight = medium.Sample(Ray(Vec3(), Vec3(0, 0, 2), 10), sequence, arena, &hit);
		ASSERT_TRUE(hit.IsValid());
		const real distance = -std::log1p(-0.2f) / sigma[channel];
		EXPECT_NEAR(hit.mPos.z, distance, 1e-6f);
		const Vec3 density = Exp(-sigma * distance) * sigma;
		AuditExpect(weight, density / ((density.x + density.y + density.z) / 3));
	}
}

TEST(PathVolumeAudit, MediumCollisionsConsumeDepthButTerminalTransmissionDoesNot) {
	AuditCountingMedium medium;
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(
		std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3()), nullptr, nullptr,
		MediumInterface(&medium, nullptr)));
	scene.AddLight(std::make_shared<AuditEnvironment>());
	AuditInitialize(scene);
	for (int backend : { 1, 2 }) {
		for (int depth : { 0, 1, 2 }) {
			medium.sampleCalls = 0;
			const Vec3 value = AuditTrace(scene,
				Ray(Vec3(0, 0, 0.1f), Vec3(1, 0, 0), MAX_REAL, &medium), backend, depth, 0.1f);
			EXPECT_EQ(medium.sampleCalls, depth);
			for (int c = 0; c < 3; ++c) EXPECT_TRUE(std::isfinite(value[c]));
		}
	}
}

TEST(PathVolumeAudit, UnsupportedLightsUseWholeSceneFreeFlight) {
	HomogeneousMedium medium(Vec3(0), Vec3(0.5f), 0);
	for (bool environment : { false, true }) {
		Scene scene{};
		scene.AddPrimitive(std::make_shared<Primitive>(
			std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3()), nullptr, nullptr,
			MediumInterface(&medium, nullptr)));
		if (environment) scene.AddLight(std::make_shared<AuditEnvironment>());
		else scene.AddLight(std::make_shared<AuditUnsupportedPoint>());
		scene.AddLight(std::make_shared<PointLight>(Vec3(2, 0, 1), Vec3(1)));
		AuditInitialize(scene);
		const Ray ray(Vec3(0, 0, 0.1f), Vec3(1, 0, 0), MAX_REAL, &medium);
		for (real value : { 0.1f, 0.3f, 0.7f })
			AuditExpect(AuditTrace(scene, ray, 2, 3, value), AuditTrace(scene, ray, 1, 3, value));
	}
}

TEST(PathVolumeAudit, RenderDepthSupportAndUnchangedControls) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	ASSERT_NE(output, nullptr);
	for (bool mirror : { false, true }) {
		for (bool environment : { false, true }) {
			Scene scene{};
			if (mirror) {
				scene.AddPrimitive(std::make_shared<Primitive>(AuditPlane(0, 1),
					std::make_shared<MirrorMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)))));
			}
			if (environment) scene.AddLight(std::make_shared<AuditEnvironment>());
			else AuditEmitter(scene, mirror ? 2.f : 0.f, mirror ? -1.f : 1.f);
			AuditInitialize(scene);
			for (int backend = 0; backend < 3; ++backend) {
				for (int depth : { 0, 1, 2 }) {
					const std::string name = std::string(output) + "\\depth-" + std::to_string(mirror) +
						"-" + std::to_string(environment) + "-" + std::to_string(backend) +
						"-" + std::to_string(depth) + ".exr";
					std::vector<Vec3> pixels;
					ASSERT_TRUE(AuditRender(scene, backend, depth, 4, Vec3(0, 0, 1), Vec3(), name, &pixels));
					const Vec3 expected(mirror && depth == 0 ? 0.f : 1.f);
					for (const Vec3& pixel : pixels) AuditExpect(pixel, expected);
				}
			}
		}
	}
}

TEST(PathVolumeAudit, RenderConservativeMediumUnderConstantEnvironment) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	ASSERT_NE(output, nullptr);
	const std::string envFile = std::string(output) + "\\constant-environment.exr";
	ASSERT_TRUE(ImageIO::WriteImage(envFile, std::vector<Vec3>(128, Vec3(1)), 16, 8));
	for (bool zeroChannel : { false, true }) {
		HomogeneousMedium medium(Vec3(0), zeroChannel ? Vec3(0.5f, 0.5f, 0) : Vec3(0.5f), 0);
		Scene scene{};
		scene.AddPrimitive(std::make_shared<Primitive>(
			std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3()), nullptr, nullptr,
			MediumInterface(&medium, nullptr)));
		auto environment = std::make_shared<EnvironmentLight>(envFile, 0);
		ASSERT_TRUE(environment->IsValid());
		scene.AddLight(environment);
		AuditInitialize(scene);
		std::vector<Vec3> images[2];
		for (int mode = 0; mode < 2; ++mode) {
			const std::string name = std::string(output) + "\\medium-" + std::to_string(zeroChannel) +
				"-" + std::to_string(mode) + ".exr";
			ASSERT_TRUE(AuditRender(scene, mode + 1, 16, 2048, Vec3(0, 0, 3), Vec3(), name, &images[mode]));
			for (const Vec3& pixel : images[mode])
				for (int c = 0; c < 3; ++c) EXPECT_TRUE(std::isfinite(pixel[c]));
			AuditExpect(AuditMean(images[mode]), Vec3(1), 0.015f);
		}
		ASSERT_EQ(images[0].size(), images[1].size());
		for (size_t i = 0; i < images[0].size(); ++i) AuditExpect(images[0][i], images[1][i], 0);
	}
}

TEST(PathVolumeAudit, RenderMixedFallbackAndSupportedPointControl) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	ASSERT_NE(output, nullptr);
	for (bool mixed : { false, true }) {
		HomogeneousMedium medium(Vec3(0), Vec3(0.5f), 0);
		Scene scene{};
		scene.AddPrimitive(std::make_shared<Primitive>(
			std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3()), nullptr, nullptr,
			MediumInterface(&medium, nullptr)));
		scene.AddLight(std::make_shared<PointLight>(Vec3(2, 1, 2), Vec3(4)));
		if (mixed) scene.AddLight(std::make_shared<AuditEnvironment>());
		AuditInitialize(scene);
		std::vector<Vec3> images[2];
		for (int mode = 0; mode < 2; ++mode) {
			const std::string name = std::string(output) + "\\point-medium-" + std::to_string(mixed) +
				"-" + std::to_string(mode) + ".exr";
			ASSERT_TRUE(AuditRender(scene, mode + 1, mixed ? 16 : 1, 2048,
				Vec3(0, 0, 3), Vec3(), name, &images[mode]));
			for (const Vec3& pixel : images[mode])
				for (int c = 0; c < 3; ++c) EXPECT_TRUE(std::isfinite(pixel[c]));
		}
		if (mixed) {
			for (size_t i = 0; i < images[0].size(); ++i) AuditExpect(images[0][i], images[1][i], 0);
		}
		else {
			EXPECT_GT(AuditMean(images[0]).x, 0);
			AuditExpect(AuditMean(images[0]), AuditMean(images[1]), 0.002f);
		}
	}

}

TEST(PathVolumeAudit, CheckedRenderRejectsInvalidAreaPointDensities) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	ASSERT_NE(output, nullptr);
	int index = 0;
	for (real density : { 0.f, -1.f, std::numeric_limits<real>::infinity(),
		std::numeric_limits<real>::quiet_NaN() }) {
		auto scene = std::make_shared<Scene>();
		auto shape = std::make_shared<AuditPointDensityShape>(density);
		auto light = std::make_shared<AreaLight>(shape, Vec3(1));
		scene->AddPrimitive(std::make_shared<Primitive>(shape, AuditDiffuse(0), light));
		scene->AddLight(light);
		for (const char* extension : { ".exr", ".png" }) {
			VolPathTracing integrator(1, 1, std::make_shared<AuditSelectionSampler>(0.3f),
				std::make_shared<SamplerEnum>(4, 4), true);
			const auto options = AuditCheckedOptions(scene,
				std::string(output) + "\\invalid-point-density-" + std::to_string(index) + extension);
			AuditExpectRejectedOutput(options, integrator);
		}
		++index;
	}
}

TEST(PathVolumeAudit, CheckedRenderRejectsFailedConcretePointLightSelection) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	ASSERT_NE(output, nullptr);
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(AuditPlane(0, -1), AuditDiffuse(0)));
	scene->AddLight(std::make_shared<PointLight>(Vec3(0, 1, -1), Vec3(1)));
	int index = 0;
	for (real selection : { -0.5f, std::numeric_limits<real>::quiet_NaN() }) {
		VolPathTracing integrator(1, 1, std::make_shared<AuditSelectionSampler>(selection),
			std::make_shared<SamplerEnum>(4, 4), true);
		const auto options = AuditCheckedOptions(scene,
			std::string(output) + "\\invalid-light-selection-" + std::to_string(index++) + ".exr");
		AuditExpectRejectedOutput(options, integrator);
	}
}

TEST(PathVolumeAudit, CheckedRenderResetsFailureAndAcceptsUnsupportedLightFallback) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	ASSERT_NE(output, nullptr);
	auto malformed = std::make_shared<Scene>();
	auto shape = std::make_shared<AuditPointDensityShape>(0);
	auto light = std::make_shared<AreaLight>(shape, Vec3(1));
	malformed->AddPrimitive(std::make_shared<Primitive>(shape, AuditDiffuse(0), light));
	malformed->AddLight(light);
	VolPathTracing integrator(1, 1, std::make_shared<AuditSelectionSampler>(0.3f),
		std::make_shared<SamplerEnum>(4, 4), true);
	AuditExpectRejectedOutput(AuditCheckedOptions(malformed, std::string(output) + "\\failed-before-reuse.exr"), integrator);
	auto valid = std::make_shared<Scene>();
	valid->AddPrimitive(std::make_shared<Primitive>(AuditPlane(-2, 1), AuditDiffuse(0)));
	valid->AddLight(std::make_shared<AuditEnvironment>());
	const auto options = AuditCheckedOptions(valid, std::string(output) + "\\checked-fallback-after-failure.exr");
	ASSERT_TRUE(RenderScene(options, integrator));
	std::vector<Vec3> pixels;
	int width = 0, height = 0;
	ASSERT_TRUE(ImageIO::LoadImage(options.output, pixels, width, height));
	ASSERT_EQ(pixels.size(), 16);
	for (const Vec3& pixel : pixels) AuditExpect(pixel, Vec3(1));
}
