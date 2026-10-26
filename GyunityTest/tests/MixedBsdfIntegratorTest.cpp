#include "gtest/gtest.h"
#include "integrator/PT.h"
#include "integrator/VPT.h"
#include "integrator/SPPM.h"
#include "integrator/BDPT.h"
#include "bsdf/BSDF.h"
#include "integrator/LT.h"
#include "accelerator/BruteForceAccel.h"
#include "camera/PinholeCamera.h"
#include "image/ImageIO.h"
#include "light/AreaLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Mirror.h"
#include "shape/Triangle.h"
#include "texture/ConstantTexture.h"
#include <atomic>
#include <cmath>
#include <cstdlib>

using namespace Gyunity;

namespace {

class MixedMaterial : public Material {
public:
	explicit MixedMaterial(bool deltaFirst = false) : mDeltaFirst(deltaFirst) {}
	void ComputeScatteringFunction(Intersection* hit, MemoryPool& arena,
		TransportMode = TransportMode::Radiance) const override {
		hit->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*hit);
		auto diffuse = GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(Vec3(0.5f));
		auto mirror = GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(0.25f));
		if (mDeltaFirst) {
			hit->mBsdf->Add(mirror);
			hit->mBsdf->Add(diffuse);
		} else {
			hit->mBsdf->Add(diffuse);
			hit->mBsdf->Add(mirror);
		}
	}
private:
	bool mDeltaFirst;
};

class ConstantSequence : public StateSequence {
public:
	explicit ConstantSequence(real value) : mValue(value) {}
	real Sample() override { ++mCursor; return mValue; }
private:
	real mValue;
};

std::shared_ptr<Material> Diffuse(real reflectance) {
	return std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(reflectance)));
}

std::shared_ptr<Triangle> Plane(real z, real normalZ) {
	Vec2 uv[] = { Vec2(0, 0), Vec2(1, 0), Vec2(0.5f, 1) };
	return std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-8, -8, z), Vec3(8, -8, z), Vec3(0, 8, z), Vec3(0, 0, normalZ), uv);
}

void Initialize(Scene& scene) {
	scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
	scene.Initialize();
}

void AddMixedFloor(Scene& scene, bool deltaFirst = false) {
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(0, 1), std::make_shared<MixedMaterial>(deltaFirst)));
}

void AddBlackEmitter(Scene& scene) {
	const auto shape = Plane(2, -1);
	const auto light = std::make_shared<AreaLight>(shape, Vec3(2));
	scene.AddPrimitive(std::make_shared<Primitive>(shape, Diffuse(0), light));
	scene.AddLight(light);
}

const Ray kEyeRay(Vec3(0, 0, 1), Vec3(0, 0, -1));

void ExpectColor(const Vec3& actual, const Vec3& expected, real tolerance = 2e-6f) {
	for (int c = 0; c < 3; ++c) {
		EXPECT_TRUE(std::isfinite(actual[c]));
		EXPECT_NEAR(actual[c], expected[c], tolerance) << "channel " << c;
	}
}

Vec3 Trace(const Scene& scene, bool volume, int depth, real sample, bool equiAngular = false) {
	ConstantSequence sequence(sample);
	MemoryPool arena;
	if (volume) {
		VolPathTracing integrator(1, depth, std::make_shared<RandomSampler>(1), nullptr, equiAngular);
		return integrator.Li(kEyeRay, scene, sequence, arena);
	}
	PathTracing integrator(1, depth, std::make_shared<RandomSampler>(1), nullptr);
	return integrator.Li(kEyeRay, scene, sequence, arena);
}

class EyePhotonSampler : public Sampler {
public:
	explicit EyePhotonSampler(real eyeValue) : mEyeValue(eyeValue) {}
	real Sample(unsigned int dimension, unsigned long long instance) override {
		if (instance != 0) return mEyeValue;
		++photonCalls;
		// GeneratePhoton consumes five values. UniformSampleSphere(0,0) emits +z;
		// later .75 samples keep both mixed and diffuse floor continuations nongrazing.
		return dimension < 5 ? 0 : 0.75f;
	}
	std::shared_ptr<Sampler> Clone(uint32) override {
		return std::make_shared<EyePhotonSampler>(mEyeValue);
	}
	std::atomic<int> photonCalls{ 0 };
private:
	real mEyeValue;
};

class CenterEyeSample : public SamplerEnum {
public:
	CenterEyeSample() : SamplerEnum(1, 1) {}
	uint64 GetIndex(uint32, uint32, uint32) const override { return 1; }
	real SampleX(uint32, real) const override { return 0.5f; }
	real SampleY(uint32, real) const override { return 0.5f; }
};

void AddPhotonFixture(Scene& scene, bool mixed, bool mirrorCeiling) {
	if (mixed) AddMixedFloor(scene, true);
	else scene.AddPrimitive(std::make_shared<Primitive>(Plane(0, 1), Diffuse(0.5f)));
	std::shared_ptr<Material> ceiling = Diffuse(0);
	if (mirrorCeiling)
		ceiling = std::make_shared<MirrorMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(2, -1), ceiling));
	scene.AddLight(std::make_shared<PointLight>(Vec3(0, 0, 1), Vec3(1)));
	Initialize(scene);
}

bool RenderSppm(const Scene& scene, real eyeSample, bool traceGlossy, const std::string& filename,
	Vec3* result, int* photonCalls) {
	const auto film = std::make_shared<Film>(1, 1);
	PinHoleCamera camera(film, kEyeRay.mOrig, kEyeRay.mOrig + kEyeRay.mDir, Vec3(0, 1, 0), 45, 1);
	const auto sampler = std::make_shared<EyePhotonSampler>(eyeSample);
	SPPM integrator(1, 1, 3, 0.25f, 0.7f, true, sampler, std::make_shared<CenterEyeSample>(), traceGlossy);
	integrator.Render(scene, camera);
	*photonCalls = sampler->photonCalls.load();
	film->SetFileName(filename);
	if (!film->SaveImage()) return false;
	std::vector<Vec3> image;
	int width = 0, height = 0;
	if (!ImageIO::LoadImage(filename, image, width, height)) return false;
	if (width != 1 || height != 1 || image.size() != 1) {
		ADD_FAILURE() << "Expected a 1x1 SPPM EXR: " << filename;
		return false;
	}
	for (int c = 0; c < 3; ++c) {
		if (!std::isfinite(image[0][c]) || image[0][c] < 0) {
			ADD_FAILURE() << "Invalid SPPM radiance: " << filename;
			return false;
		}
	}
	*result = image[0];
	return true;
}

}

TEST(MixedBsdfIntegrators, PtAndVptReceiveContinuousPointLightNee) {
	Scene scene{};
	AddMixedFloor(scene);
	scene.AddLight(std::make_shared<PointLight>(Vec3(0, 0, 2), Vec3(4)));
	Initialize(scene);
	for (bool volume : { false, true }) {
		for (real sample : { 0.25f, 0.75f }) {
			SCOPED_TRACE(testing::Message() << "volume=" << volume << " sample=" << sample);
			ExpectColor(Trace(scene, volume, 1, sample), Vec3(0.5f * INV_PI));
			if (volume) {
				SCOPED_TRACE("equiangular surface control");
				ExpectColor(Trace(scene, true, 1, sample, true), Vec3(0.5f * INV_PI));
			}
		}
	}
}

TEST(MixedBsdfIntegrators, DirectMisExcludesDeltaInBothSamplingTechniques) {
	Scene scene{};
	AddMixedFloor(scene);
	AddBlackEmitter(scene);
	Initialize(scene);
	MemoryPool arena;
	Intersection mixed;
	ASSERT_TRUE(scene.Intersect(kEyeRay, &mixed));
	scene.QueryIntersectionInfo(kEyeRay, &mixed);
	mixed.ComputeScatteringFunction(arena);
	Intersection diffuse = mixed;
	Diffuse(0.5f)->ComputeScatteringFunction(&diffuse, arena);
	for (bool media : { false, true }) {
		for (real sample : { 0.25f, 0.75f }) {
			ConstantSequence mixedSequence(sample), diffuseSequence(sample);
			const Vec3 actual = Integrator::DirectIllumination(scene, mixed, sample,
				Vec2(sample), Vec3(sample), mixedSequence, media);
			const Vec3 expected = Integrator::DirectIllumination(scene, diffuse, sample,
				Vec2(sample), Vec3(sample), diffuseSequence, media);
			ASSERT_GT(expected.x, 0);
			ExpectColor(actual, expected);
		}
	}
}

TEST(MixedBsdfIntegrators, PtAndVptCreditEmitterOnlyAfterSampledDelta) {
	Scene scene{};
	AddMixedFloor(scene);
	AddBlackEmitter(scene);
	Initialize(scene);
	MemoryPool arena;
	Intersection diffuse;
	ASSERT_TRUE(scene.Intersect(kEyeRay, &diffuse));
	scene.QueryIntersectionInfo(kEyeRay, &diffuse);
	Diffuse(0.5f)->ComputeScatteringFunction(&diffuse, arena);
	for (bool volume : { false, true }) {
		for (real sample : { 0.25f, 0.75f }) {
			SCOPED_TRACE(testing::Message() << "volume=" << volume << " sample=" << sample);
			ConstantSequence sequence(sample);
			const Vec3 nee = Integrator::DirectIllumination(scene, diffuse, sample,
				Vec2(sample), Vec3(sample), sequence, volume);
			ASSERT_GT(nee.x, 0);
			// Diffuse is first: .25 selects diffuse, .75 mirror. Delta throughput
			// is Ks/qdelta=.25/.5, and the black-material emitter contributes Le=2.
			const Vec3 continuation(sample == 0.75f ? 1.f : 0.f);
			ExpectColor(Trace(scene, volume, 2, sample), nee + continuation);
			if (volume) {
				SCOPED_TRACE("equiangular surface control");
				ExpectColor(Trace(scene, true, 2, sample, true), nee + continuation);
			}
		}
	}
}

TEST(MixedBsdfIntegrators, SppmContinuousVisiblePointHasInverseSelectionWeight) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	if (!output) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test folder.";
	Vec3 indirect[2];
	for (int mixed = 0; mixed < 2; ++mixed) {
		Vec3 radiance[2];
		for (int mirror = 0; mirror < 2; ++mirror) {
			Scene scene{};
			AddPhotonFixture(scene, mixed != 0, mirror != 0);
			int photonCalls;
			const std::string filename = std::string(output) + "\\mixed-bsdf-sppm-gather-" +
				std::to_string(mixed) + "-" + std::to_string(mirror) + ".exr";
			ASSERT_TRUE(RenderSppm(scene, 0.75f, false, filename, &radiance[mirror], &photonCalls));
			EXPECT_GT(photonCalls, 5);
		}
		ExpectColor(radiance[0], Vec3((mixed ? 1.f : 0.5f) * INV_PI));
		if (mixed) {
			Scene scene{};
			AddPhotonFixture(scene, true, false);
			Vec3 deltaRadiance;
			int photonCalls;
			const std::string filename = std::string(output) + "\\mixed-bsdf-sppm-direct-delta.exr";
			ASSERT_TRUE(RenderSppm(scene, 0.25f, false, filename, &deltaRadiance, &photonCalls));
			ExpectColor(deltaRadiance, Vec3(0));
			// With an absorbing ceiling, the only direct contribution is at the first floor:
			// continuous draws contribute NEE/qcontinuous; delta draws contribute zero.
			ExpectColor(0.5f * (radiance[0] + deltaRadiance), Vec3(0.5f * INV_PI));
		}
		indirect[mixed] = radiance[1] - radiance[0];
		// One photon has flux 4*pi and reaches the floor after its first-hit mirror.
		// f=.5/pi, r²=.25²; one batch shrink cancels between flux and radius².
		const real expected = (mixed ? 64.f : 32.f) * INV_PI;
		ExpectColor(indirect[mixed], Vec3(expected), 5e-5f);
	}
	ExpectColor(indirect[1], 2 * indirect[0], 5e-5f);
}

TEST(MixedBsdfIntegrators, SppmDeltaOnlyEyePathSkipsEmptyVisiblePointGrid) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	if (!output) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test folder.";
	Scene scene{};
	AddPhotonFixture(scene, true, true);
	for (bool traceGlossy : { false, true }) {
		Vec3 radiance;
		int photonCalls;
		const std::string filename = std::string(output) + "\\mixed-bsdf-sppm-no-visible-point-" +
			std::to_string(traceGlossy) + ".exr";
		ASSERT_TRUE(RenderSppm(scene, 0.25f, traceGlossy, filename, &radiance, &photonCalls));
		// Mirror-first mixture: floor -> pure mirror -> floor. Every selected event
		// is delta, so there is neither continuous NEE nor a VP, and no surface emits.
		ExpectColor(radiance, Vec3(0));
		EXPECT_EQ(photonCalls, 0);
	}
}

TEST(MixedBsdfIntegrators, BdptContinuousConnectionIgnoresUnusedDeltaSelection) {
	Scene scene{};
	const auto shape = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(0, -1, 2), Vec3(2, -1, 2), Vec3(1, 1, 2), Vec3(0, 0, -1));
	const auto light = std::make_shared<AreaLight>(shape, Vec3(2));
	scene.AddLight(light);
	Initialize(scene);
	PinHoleCamera camera(std::make_shared<Film>(16, 16), Vec3(0, 0, 3),
		Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
	std::vector<PathVertex> lightPath(1), cameraPath(2);
	lightPath[0].mType = PathVertexType::Light;
	lightPath[0].mLight = light.get();
	lightPath[0].mIsect.mPos = Vec3(1, 0, 2);
	lightPath[0].mIsect.mNormal = lightPath[0].mIsect.mGeometryNormal = Vec3(0, 0, -1);
	lightPath[0].mThroughput = Vec3(1);
	lightPath[0].mPdfFwd = 0.5f;
	cameraPath[0].mType = PathVertexType::Camera;
	cameraPath[0].mCamera = &camera;
	cameraPath[0].mIsect.mPos = camera.mPos;
	cameraPath[0].mThroughput = Vec3(1);
	cameraPath[0].mPdfFwd = 1;
	PathVertex& surface = cameraPath[1];
	surface.mIsect.mPos = Vec3(0);
	surface.mIsect.mNormal = surface.mIsect.mGeometryNormal = Vec3(0, 0, 1);
	surface.mIsect.mShadingDpDu = Vec3(1, 0, 0);
	surface.mIsect.mShadingDpDv = Vec3(0, 1, 0);
	surface.mIsect.mOutDir = Vec3(0, 0, 1);
	surface.mThroughput = Vec3(1);
	MemoryPool arena;
	MixedMaterial().ComputeScatteringFunction(&surface.mIsect, arena);
	const real cameraDensity = camera.PdfDir(Ray(camera.mPos, Vec3(0, 0, -1))) / 9;
	surface.mPdfFwd = cameraDensity;
	// Three strategy products for this one geometric path: continuous mixture
	// density includes qdiff=.5; the emitter's area and directional PDFs do not.
	const double p0 = double(2.f / (25 * PI)) * cameraDensity;
	const double p1 = 0.5 * cameraDensity;
	const double p2 = 0.5 * (4.f / (25 * PI));
	const real expectedWeight = real(p1 / (p0 + p1 + p2));
	for (real sample : { 0.25f, 0.75f }) {
		Vec3 unusedWi;
		real unusedPdf;
		ScatterEventType selected;
		surface.mIsect.mBsdf->Sample(surface.mIsect.mOutDir, &unusedWi, &unusedPdf,
			Vec3(sample), BSDF_ALL, &selected);
		surface.mIsDelta = (selected & BSDF_SPECULAR) != 0;
		EXPECT_EQ(surface.mIsDelta, sample == 0.75f);
		Vec3 primary, weighted;
		Vec2 raster;
		real weight;
		ASSERT_TRUE(EvaluateConnection(scene, camera, lightPath, cameraPath, 1, 2, &primary, &raster));
		ExpectColor(primary, Vec3(8.f / (25 * PI)));
		ASSERT_TRUE(MisWeight(scene, lightPath, cameraPath, 1, 2, &weight));
		EXPECT_NEAR(weight, expectedWeight, 1e-6f);
		ASSERT_TRUE(ConnectPaths(scene, camera, lightPath, cameraPath, 1, 2, &weighted, &raster));
		ExpectColor(weighted, Vec3(8.f / (25 * PI)) * expectedWeight);
		EXPECT_EQ(surface.mIsDelta, sample == 0.75f);
	}
}

TEST(MixedBsdfIntegrators, LtTraceStoresSampledEventAndFiniteThroughput) {
	Scene scene{};
	AddMixedFloor(scene);
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(2, -1), Diffuse(0)));
	Initialize(scene);
	PinHoleCamera camera(std::make_shared<Film>(16, 16), kEyeRay.mOrig,
		Vec3(0), Vec3(0, 1, 0), 90, 1);
	const int maxDepth = 3;
	LightTracing integrator(std::make_shared<RandomSampler>(1), maxDepth, 1);
	for (real sample : { 0.25f, 0.75f }) {
		SCOPED_TRACE(testing::Message() << "sample=" << sample);
		ConstantSequence sequence(sample);
		MemoryPool arena;
		std::vector<PathVertex> path(maxDepth + 1);
		ASSERT_EQ(integrator.Trace(scene, camera, kEyeRay, arena, sequence, 0, Vec3(1), 1, path), 2);
		ASSERT_NE(path[0].mIsect.mBsdf, nullptr);
		EXPECT_TRUE(path[0].mIsect.mBsdf->HasMixedDeltaLobes());
		EXPECT_FALSE(path[0].mIsect.mBsdf->IsDelta());
		EXPECT_EQ(path[0].mIsDelta, sample == 0.75f);
		EXPECT_FALSE(path[1].mIsDelta);
		EXPECT_NEAR(path[0].mIsect.mPos.z, 0, 1e-6f);
		EXPECT_NEAR(path[1].mIsect.mPos.z, 2, 1e-6f);
		ExpectColor(path[0].mThroughput, Vec3(1));
		// With equal lobe probabilities: diffuse Kd/q=1, mirror Ks/q=.5.
		ExpectColor(path[1].mThroughput, Vec3(sample == 0.75f ? 0.5f : 1.f));
		EXPECT_EQ(path[0].mPdfFwd, 1);
		const real expectedPdf = sample == 0.75f ? 0.5f : std::sqrt(0.75f) * INV_PI / 2;
		EXPECT_NEAR(path[1].mPdfFwd, expectedPdf, 1e-6f);
	}
}

TEST(MixedBsdfIntegrators, LtTraceStopsAtAbsorbingSurface) {
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(0, 1), Diffuse(0)));
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(2, -1), Diffuse(1)));
	Initialize(scene);
	PinHoleCamera camera(std::make_shared<Film>(16, 16), kEyeRay.mOrig,
		Vec3(0), Vec3(0, 1, 0), 90, 1);
	const int maxDepth = 3;
	LightTracing integrator(std::make_shared<RandomSampler>(1), maxDepth, 1);
	for (real sample : { 0.25f, 0.75f }) {
		ConstantSequence sequence(sample);
		MemoryPool arena;
		std::vector<PathVertex> path(maxDepth + 1);
		path[1].mThroughput = Vec3(99);
		path[1].mPdfFwd = 99;
		ASSERT_EQ(integrator.Trace(scene, camera, kEyeRay, arena, sequence, 0, Vec3(1), 1, path), 1);
		ExpectColor(path[0].mThroughput, Vec3(1));
		EXPECT_FALSE(path[0].mIsDelta);
		EXPECT_EQ(path[0].mPdfFwd, 1);
		EXPECT_EQ(path[1].mThroughput, Vec3(99));
		EXPECT_EQ(path[1].mPdfFwd, 99);
	}
}

TEST(MixedBsdfIntegrators, LtContinuousCameraConnectionIgnoresSampledDelta) {
	Scene scene{};
	AddMixedFloor(scene);
	Initialize(scene);
	PinHoleCamera camera(std::make_shared<Film>(16, 16), kEyeRay.mOrig,
		Vec3(0), Vec3(0, 1, 0), 90, 1);
	LightTracing integrator(std::make_shared<RandomSampler>(1), 3, 1);
	MemoryPool arena;
	PathVertex mixed;
	ASSERT_TRUE(scene.Intersect(kEyeRay, &mixed.mIsect));
	scene.QueryIntersectionInfo(kEyeRay, &mixed.mIsect);
	mixed.mIsect.ComputeScatteringFunction(arena, TransportMode::Importance);
	mixed.mThroughput = Vec3(1);
	Vec3 unusedWi;
	real unusedPdf;
	ScatterEventType event;
	mixed.mIsect.mBsdf->Sample(mixed.mIsect.mOutDir, &unusedWi, &unusedPdf,
		Vec3(0.75f), BSDF_ALL, &event);
	mixed.mIsDelta = SurfaceEventClassifier::IsDelta(event);
	ASSERT_TRUE(mixed.mIsDelta);
	ASSERT_GT(unusedPdf, 0);
	PathVertex diffuse = mixed;
	Diffuse(0.5f)->ComputeScatteringFunction(&diffuse.mIsect, arena, TransportMode::Importance);
	diffuse.mIsDelta = false;
	ConstantSequence mixedSequence(0.5f), diffuseSequence(0.5f);
	Vec3 mixedRaster, diffuseRaster;
	bool mixedInScreen = false, diffuseInScreen = false;
	// s=1 is a surface connection, not the separate light-origin (s=0) path.
	const Vec3 actual = integrator.ConnectToCamera(scene, camera, mixedSequence, mixed, 1,
		&mixedRaster, &mixedInScreen);
	const Vec3 expected = integrator.ConnectToCamera(scene, camera, diffuseSequence, diffuse, 1,
		&diffuseRaster, &diffuseInScreen);
	ASSERT_TRUE(mixedInScreen);
	ASSERT_TRUE(diffuseInScreen);
	EXPECT_EQ(mixedRaster, Vec3(8, 8, 0));
	EXPECT_EQ(mixedRaster, diffuseRaster);
	ASSERT_GT(expected.x, 0);
	ExpectColor(actual, expected);
	// Centered camera: distance²=1, cosine=1, unit film distance, film area=4.
	ExpectColor(actual, Vec3(INV_PI / 8));
}
