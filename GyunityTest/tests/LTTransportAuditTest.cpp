#include "gtest/gtest.h"
#include "visual/Scene.h"
#include "accelerator/BruteForceAccel.h"
#include "integrator/LT.h"
#include "camera/PinholeCamera.h"
#include "light/AreaLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Mirror.h"
#include "texture/ConstantTexture.h"
#include "shape/Triangle.h"
#include <limits>

using namespace Gyunity;

namespace {

class ConstantSamples : public StateSequence {
public:
	real Sample() override { return 0.25f; }
};

std::shared_ptr<Triangle> Plane(real z, real normalZ) {
	Vec2 uv[] = { Vec2(0), Vec2(1, 0), Vec2(0, 1) };
	return std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-100, -100, z), Vec3(100, -100, z), Vec3(0, 100, z),
		Vec3(0, 0, normalZ), uv);
}

void Initialize(Scene& scene) {
	scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
	scene.Initialize();
}

PinHoleCamera CameraAt(real z = 2) {
	return PinHoleCamera(std::make_shared<Film>(16, 16), Vec3(0, 0, z),
		Vec3(0), Vec3(0, 1, 0), 90, 1);
}

void ExpectColor(Vec3 value, Vec3 expected) {
	for (int c = 0; c < 3; ++c) {
		EXPECT_TRUE(std::isfinite(value[c]));
		EXPECT_NEAR(value[c], expected[c], 2e-5f);
	}
}

class OriginLight : public Light {
public:
	bool IsAreaLight() const override { return !environment; }
	bool IsEnvironmentLight() const override { return environment; }
	Vec3 Power() const override { return Vec3(1); }
	Vec3 Emission() const override { return Vec3(2); }
	Vec3 Emission(const Intersection& hit, const Vec3& wi) const override {
		return Dot(hit.mGeometryNormal, wi) > 0 ? Emission() : Vec3(0);
	}
	void SampleOnLight(Intersection* hit, Vec3* wi, real* posPdf, real* dirPdf,
		const Vec2&, const Vec2&) const override {
		hit->mPos = Vec3(0);
		hit->mGeometryNormal = hit->mNormal = Vec3(0, 0, 1);
		*wi = Vec3(0, 0, 1);
		*posPdf = positionPdf;
		*dirPdf = directionPdf;
	}
	Vec3 SampleLight(Intersection* hit, Vec3* wi, real* posPdf, real* dirPdf,
		const Vec2& u, const Vec2& v) const override {
		SampleOnLight(hit, wi, posPdf, dirPdf, u, v);
		return Emission();
	}
	Vec3 SampleLi(const Intersection&, Vec3*, real*, Intersection*, const Vec2&) const override {
		return Vec3(0);
	}
	real PdfLi(const Intersection&, const Vec3&) const override { return 0; }
	real positionPdf = 0.5f, directionPdf = 0.25f;
	bool environment = false;
};

Vec3 Connect(const Scene& scene, const Camera& camera, const PathVertex& vertex, bool* inside) {
	ConstantSamples samples;
	Vec3 raster;
	LightTracing tracer(std::make_shared<RandomSampler>(1), 2, 1);
	return tracer.ConnectToCamera(scene, camera, samples, vertex, 0, &raster, inside);
}

}

TEST(LTTransportAudit, OriginUsesCameraGeometryAndPositionSelectionDensitiesOnce) {
	for (int lights : { 1, 2 }) {
		for (real positionPdf : { 0.25f, 0.5f }) {
			for (real directionPdf : { 0.125f, 0.5f }) {
				Scene scene{};
				auto light = std::make_shared<OriginLight>();
				light->positionPdf = positionPdf;
				light->directionPdf = directionPdf;
				for (int i = 0; i < lights; ++i) scene.AddLight(light);
				Initialize(scene);
				auto camera = CameraAt();
				LightTracing tracer(std::make_shared<RandomSampler>(1), 0, 1);
				ConstantSamples samples;
				MemoryPool arena;
				std::vector<PathVertex> path(2);
				ASSERT_EQ(tracer.GenerateLightPath(scene, camera, samples, arena, path), 1);
				EXPECT_EQ(path[0].mThroughput, Vec3(2));
				bool inside = false;
				// f=1, film area=4, distance^2=4, endpoint cosine=1.
				ExpectColor(Connect(scene, camera, path[0], &inside),
					Vec3(2.f * lights / (16 * positionPdf)));
				EXPECT_TRUE(inside);
			}
		}
	}
}

TEST(LTTransportAudit, RejectsBackFacingDeltaAndEnvironmentOrigins) {
	for (int kind = 0; kind < 3; ++kind) {
		Scene scene{};
		auto area = std::make_shared<OriginLight>();
		area->environment = kind == 2;
		std::shared_ptr<Light> light = area;
		if (kind == 1) light = std::make_shared<PointLight>(Vec3(0), Vec3(2));
		scene.AddLight(light);
		Initialize(scene);
		auto camera = CameraAt();
		LightTracing tracer(std::make_shared<RandomSampler>(1), 0, 1);
		MemoryPool arena;
		ConstantSamples samples;
		std::vector<PathVertex> path(2);
		ASSERT_EQ(tracer.GenerateLightPath(scene, camera, samples, arena, path), 1);
		if (kind == 0) path[0].mIsect.mGeometryNormal = Vec3(0, 0, -1);
		bool inside = true;
		ExpectColor(Connect(scene, camera, path[0], &inside), Vec3(0));
	}
}

TEST(LTTransportAudit, OriginUsesGeometricCosineAndInverseSquareCameraGeometry) {
	Scene scene{};
	auto light = std::make_shared<OriginLight>();
	scene.AddLight(light);
	Initialize(scene);
	PathVertex origin;
	origin.mLight = light.get();
	origin.mIsect.mPos = Vec3(0.5f, 0, 0);
	origin.mIsect.mGeometryNormal = Vec3(0.8f, 0, 0.6f);
	origin.mIsect.mNormal = Vec3(0, 0, -1);
	origin.mThroughput = Vec3(2);
	origin.mPdfPos = 0.5f;
	origin.mLightSelectionPdf = 1;
	for (real distance : { 2.f, 4.f }) {
		auto camera = CameraAt(distance);
		bool inside = false;
		// Projected-area Jacobian for the tilted endpoint, with film area 4 and f=1.
		const real expected = 2 / (4 * distance * distance * 0.5f) * (0.6f - 0.8f * 0.5f / distance);
		ExpectColor(Connect(scene, camera, origin, &inside), Vec3(expected));
		EXPECT_TRUE(inside);
	}
}

TEST(LTTransportAudit, UsesFloatingHalfOpenRasterCoordinates) {
	auto camera = CameraAt();
	LightTracing tracer(std::make_shared<RandomSampler>(1), 0, 1);
	for (real x : { -0.25f, 0.f, 0.25f, 15.75f, 16.f, 16.25f }) {
		for (real y : { -0.25f, 0.f, 0.25f, 15.75f, 16.f, 16.25f }) {
			bool inside = false;
			const Vec3 world = camera.mRasterToWorld(Vec3(x, y, -1));
			const Vec3 raster = tracer.WorldToScreen(camera, world, &inside);
			EXPECT_EQ(inside, x >= 0 && x < 16 && y >= 0 && y < 16);
			EXPECT_NEAR(raster.x, x, 2e-5f);
			EXPECT_NEAR(raster.y, y, 2e-5f);
		}
	}
}

TEST(LTTransportAudit, DepthCountsScatteringVerticesAndZeroDoesNotWriteSurface) {
	Scene scene{};
	auto mirror = std::make_shared<MirrorMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(0.5f)));
	scene.AddPrimitive(Plane(0, 1), mirror);
	scene.AddPrimitive(Plane(2, -1), mirror);
	Initialize(scene);
	auto camera = CameraAt();
	for (int depth : { 0, 1, 2, 3, 5 }) {
		LightTracing tracer(std::make_shared<RandomSampler>(1), depth, 1);
		MemoryPool arena;
		ConstantSamples samples;
		std::vector<PathVertex> path(depth + 2);
		for (auto& vertex : path) vertex.mThroughput = Vec3(99);
		const int count = tracer.Trace(scene, camera, Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)),
			arena, samples, 1, Vec3(1), 1, path);
		ASSERT_EQ(count, depth + 1);
		for (int i = 1; i < count; ++i)
			ExpectColor(path[i].mThroughput, Vec3(std::pow(0.5f, real(i - 1))));
		EXPECT_EQ(path[depth + 1].mThroughput, Vec3(99));
	}
}

TEST(LTTransportAudit, ZeroDepthGenerationPreservesGuardVertex) {
	Scene scene{};
	scene.AddLight(std::make_shared<OriginLight>());
	scene.AddPrimitive(Plane(2, -1),
		std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))));
	Initialize(scene);
	auto camera = CameraAt();
	LightTracing tracer(std::make_shared<RandomSampler>(1), 0, 1);
	MemoryPool arena;
	ConstantSamples samples;
	std::vector<PathVertex> path(2);
	path[1].mThroughput = Vec3(99);
	EXPECT_EQ(tracer.GenerateLightPath(scene, camera, samples, arena, path), 1);
	EXPECT_EQ(path[1].mThroughput, Vec3(99));
}

TEST(LTTransportAudit, InvalidOrNullEmissionDirectionKeepsFiniteOriginPrefix) {
	for (real pdf : { 0.f, -1.f, std::numeric_limits<real>::infinity() }) {
		Scene scene{};
		auto light = std::make_shared<OriginLight>();
		light->directionPdf = pdf;
		scene.AddLight(light);
		scene.AddPrimitive(Plane(2, -1),
			std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))));
		Initialize(scene);
		auto camera = CameraAt();
		LightTracing tracer(std::make_shared<RandomSampler>(1), 2, 1);
		MemoryPool arena;
		ConstantSamples samples;
		std::vector<PathVertex> path(3);
		path[1].mThroughput = Vec3(99);
		EXPECT_EQ(tracer.GenerateLightPath(scene, camera, samples, arena, path), 1);
		EXPECT_EQ(path[0].mThroughput, Vec3(2));
		EXPECT_EQ(path[1].mThroughput, Vec3(99));
	}

}

TEST(LTTransportAudit, InvalidContinuationDoesNotAppendNonfiniteVertices) {
	Scene scene{};
	scene.AddPrimitive(Plane(0, 1),
		std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))));
	Initialize(scene);
	auto camera = CameraAt();
	LightTracing tracer(std::make_shared<RandomSampler>(1), 2, 1);
	for (int invalid = 0; invalid < 3; ++invalid) {
		MemoryPool arena;
		ConstantSamples samples;
		std::vector<PathVertex> path(3);
		path[1].mThroughput = Vec3(99);
		const real infinity = std::numeric_limits<real>::infinity();
		const Vec3 weight = invalid == 0 ? Vec3(infinity) : Vec3(1);
		const real pdf = invalid == 1 ? infinity : invalid == 2 ? -1.f : 1.f;
		EXPECT_EQ(tracer.Trace(scene, camera, Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)),
			arena, samples, 1, weight, pdf, path), 1);
		EXPECT_EQ(path[1].mThroughput, Vec3(99));
	}
}
