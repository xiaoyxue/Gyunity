#include "gtest/gtest.h"
#include "integrator/BDPTPath.h"
#include "camera/PinholeCamera.h"
#include "accelerator/BruteForceAccel.h"
#include "material/Diffuse.h"
#include "texture/ConstantTexture.h"
#include "light/AreaLight.h"
#include "shape/Triangle.h"
#include <cmath>
#include <limits>

using namespace Gyunity;

namespace {

class HalfSamples : public StateSequence {
public:
	real Sample() override { ++mCursor; return 0.5f; }
};

std::shared_ptr<Material> WhiteDiffuse() {
	return std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
}

const Vec3 kTilted(0.6f, 0, 0.8f);

// Large z = 0 receiver whose interpolated shading normal is tilted 36.87 degrees toward +x.
std::shared_ptr<Triangle> SmoothReceiver() {
	return std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-10, -10, 0), Vec3(10, -10, 0), Vec3(0, 10, 0), kTilted, kTilted, kTilted);
}

void AddPrimitive(Scene& scene, const std::shared_ptr<Shape>& shape, const std::shared_ptr<Light>& light = nullptr) {
	scene.AddPrimitive(std::make_shared<Primitive>(shape, WhiteDiffuse(), light));
}

// Receiver, a small downward emitter at z = 2 and a wall at x = 3 that catches the
// receiver's sampled direction (the shading normal) without occluding the emitter.
std::shared_ptr<Scene> AdjointScene() {
	auto scene = std::make_shared<Scene>();
	AddPrimitive(*scene, SmoothReceiver());
	AddPrimitive(*scene, std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(3, -20, -1), Vec3(3, 20, -1), Vec3(3, 0, 30), Vec3(-1, 0, 0)));
	auto emitter = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-1, -1, 2), Vec3(1, -1, 2), Vec3(0, 1, 2), Vec3(0, 0, -1));
	auto light = std::make_shared<AreaLight>(emitter, Vec3(1));
	scene->AddLight(light);
	AddPrimitive(*scene, emitter, light);
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
	return scene;
}

std::shared_ptr<Camera> DownCamera() {
	return std::make_shared<PinHoleCamera>(std::make_shared<Film>(16, 16),
		Vec3(0, 0, 1), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
}

Intersection TiltedPoint() {
	Intersection isect;
	isect.mGeometryNormal = Vec3(0, 0, 1);
	isect.mNormal = kTilted;
	return isect;
}

void ExpectOrthonormalFrame(const Intersection& isect) {
	EXPECT_NEAR(isect.mNormal.Length(), 1, 1e-6f);
	EXPECT_NEAR(isect.mShadingDpDu.Length(), 1, 1e-6f);
	EXPECT_NEAR(isect.mShadingDpDv.Length(), 1, 1e-6f);
	EXPECT_NEAR(Dot(isect.mNormal, isect.mShadingDpDu), 0, 1e-6f);
	EXPECT_NEAR(Dot(isect.mNormal, isect.mShadingDpDv), 0, 1e-6f);
	EXPECT_NEAR(Dot(isect.mShadingDpDu, isect.mShadingDpDv), 0, 1e-6f);
	// Right-handed: tangent x bitangent = normal, so the BSDF frame keeps its orientation.
	EXPECT_NEAR(Dot(Cross(isect.mShadingDpDu, isect.mShadingDpDv), isect.mNormal), 1, 1e-6f);
}

}

TEST(BDPTShadingNormal, CombinedCosineUsesTheAdjointDirection) {
	const Intersection isect = TiltedPoint();
	const Vec3 wo = isect.mGeometryNormal;
	const Vec3 wi = isect.mNormal;
	double value;
	value = Bidirectional::ScatteringCosine(isect, wo, wi, TransportMode::Radiance);
	EXPECT_NEAR(value, 1.0, 1e-6);
	// |wo.ns| |wi.ng| / |wo.ng| = 0.8 * 0.8 / 1.
	value = Bidirectional::ScatteringCosine(isect, wo, wi, TransportMode::Importance);
	EXPECT_NEAR(value, 0.64, 1e-6);
	// Swapped: the standalone correction is 1.5625; times the new |wi.ns| = 0.8 gives 1.25.
	value = Bidirectional::ScatteringCosine(isect, wi, wo, TransportMode::Importance);
	EXPECT_NEAR(value, 1.25, 1e-6);
	value = Bidirectional::ScatteringCosine(isect, wi, wo, TransportMode::Radiance);
	EXPECT_NEAR(value, 0.8, 1e-6);

	// Above ns but below ng: outside the support in both modes, for either argument.
	const Vec3 belowGeometry = Vec3(1, 0, -0.1f).Norm();
	ASSERT_GT(Dot(belowGeometry, isect.mNormal), 0);
	for (TransportMode mode : { TransportMode::Radiance, TransportMode::Importance }) {
		value = Bidirectional::ScatteringCosine(isect, wo, belowGeometry, mode);
		EXPECT_DOUBLE_EQ(value, 0.0);
		value = Bidirectional::ScatteringCosine(isect, belowGeometry, wo, mode);
		EXPECT_DOUBLE_EQ(value, 0.0);
	}
	EXPECT_FALSE(Bidirectional::ShadingSupport(isect, belowGeometry));
	EXPECT_TRUE(Bidirectional::ShadingSupport(isect, wo));
	// Exactly tangent to the geometric surface is excluded as well.
	value = Bidirectional::ScatteringCosine(isect, wo, Vec3(1, 0, 0), TransportMode::Importance);
	EXPECT_DOUBLE_EQ(value, 0.0);
}

TEST(BDPTShadingNormal, EqualNormalsReduceToTheGeometricCosine) {
	Intersection isect;
	isect.mGeometryNormal = isect.mNormal = Vec3(0, 0, 1);
	const Vec3 wo = Vec3(0.3f, 0.4f, std::sqrt(0.75f));
	const Vec3 wi = Vec3(-0.6f, 0, 0.8f);
	for (TransportMode mode : { TransportMode::Radiance, TransportMode::Importance }) {
		double value;
		value = Bidirectional::ScatteringCosine(isect, wo, wi, mode);
		EXPECT_NEAR(value, 0.8, 1e-6);
	}
}


TEST(BDPTShadingNormal, AreaDensityDoesNotUseShadingNormal) {
	PathVertex from, to;
	from.mIsect.mPos = Vec3(0);
	to.mIsect.mPos = Vec3(0, 0, 2);
	to.mIsect.mGeometryNormal = Vec3(0, 0, 1);
	to.mIsect.mNormal = kTilted;
	to.mNormalMode = SurfaceNormalMode::Shading;
	real tilted, aligned;
	tilted = Bidirectional::DirectionToArea(0.5f, from, to);
	to.mIsect.mNormal = to.mIsect.mGeometryNormal;
	aligned = Bidirectional::DirectionToArea(0.5f, from, to);
	EXPECT_FLOAT_EQ(tilted, 0.125f);
	EXPECT_FLOAT_EQ(tilted, aligned);
}

TEST(BDPTShadingNormal, GeometricFrameReplacesTheShadingNormal) {
	Intersection isect = TiltedPoint();
	isect.mGeometryNormal = Vec3(0, 0, 3);
	ASSERT_TRUE(Bidirectional::PrepareSurfaceFrame(&isect, SurfaceNormalMode::Geometric));
	EXPECT_EQ(isect.mNormal, Vec3(0, 0, 1));
	ExpectOrthonormalFrame(isect);
}

TEST(BDPTShadingNormal, ShadingFrameKeepsTheTiltedNormalAndIsRayIndependent) {
	Intersection isect = TiltedPoint();
	isect.mNormal = kTilted * 2;
	isect.mGeometryNormal = Vec3(0, 0, 4);
	isect.mDpDu = Vec3(0, 3, 0);
	isect.mOutDir = Vec3(0, 0, 1);
	Intersection reversed = isect;
	reversed.mOutDir = Vec3(0, 0, -1);
	ASSERT_TRUE(Bidirectional::PrepareSurfaceFrame(&isect, SurfaceNormalMode::Shading));
	ASSERT_TRUE(Bidirectional::PrepareSurfaceFrame(&reversed, SurfaceNormalMode::Shading));
	EXPECT_NEAR(isect.mNormal.x, 0.6f, 1e-6f);
	EXPECT_NEAR(isect.mNormal.z, 0.8f, 1e-6f);
	EXPECT_EQ(isect.mGeometryNormal, Vec3(0, 0, 1));
	ExpectOrthonormalFrame(isect);
	// The tangent follows dpdu projected onto the shading plane.
	EXPECT_NEAR(isect.mShadingDpDu.y, 1, 1e-6f);
	EXPECT_EQ(isect.mShadingDpDu, reversed.mShadingDpDu);
	EXPECT_EQ(isect.mShadingDpDv, reversed.mShadingDpDv);

	// A parameterization parallel to ns uses the deterministic fallback basis.
	Intersection degenerate = TiltedPoint();
	degenerate.mDpDu = kTilted;
	ASSERT_TRUE(Bidirectional::PrepareSurfaceFrame(&degenerate, SurfaceNormalMode::Shading));
	ExpectOrthonormalFrame(degenerate);
}

TEST(BDPTShadingNormal, ShadingFrameRejectsNormalsOnOppositeSides) {
	Intersection isect = TiltedPoint();
	isect.mGeometryNormal = Vec3(0, 0, -1);
	EXPECT_FALSE(Bidirectional::PrepareSurfaceFrame(&isect, SurfaceNormalMode::Shading));
	isect = TiltedPoint();
	isect.mNormal = Vec3(0);
	EXPECT_FALSE(Bidirectional::PrepareSurfaceFrame(&isect, SurfaceNormalMode::Shading));
}

TEST(BDPTShadingNormal, SubpathsKeepShadingNormalsOnlyInShadingMode) {
	auto scene = AdjointScene();
	auto camera = DownCamera();
	MemoryPool arena;
	std::vector<PathVertex> path;
	for (SurfaceNormalMode mode : { SurfaceNormalMode::Geometric, SurfaceNormalMode::Shading }) {
		HalfSamples samples;
		ASSERT_GE(GenerateCameraPath(*scene, *camera, samples, arena, Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)), 2, mode, path), 2);
		EXPECT_EQ(path[0].mNormalMode, mode);
		EXPECT_EQ(path[1].mNormalMode, mode);
		EXPECT_EQ(path[1].mIsect.mGeometryNormal, Vec3(0, 0, 1));
		if (mode == SurfaceNormalMode::Geometric) EXPECT_EQ(path[1].mIsect.mNormal, Vec3(0, 0, 1));
		else {
			EXPECT_NEAR(path[1].mIsect.mNormal.x, 0.6f, 1e-5f);
			EXPECT_NEAR(path[1].mIsect.mNormal.z, 0.8f, 1e-5f);
		}
		arena.Reset();
	}
}

TEST(BDPTShadingNormal, ExtensionAppliesTheAdjointCorrectionExactlyOnce) {
	auto scene = AdjointScene();
	auto camera = DownCamera();
	MemoryPool arena;
	std::vector<PathVertex> path;

	// Radiance: f * |wi.ns| / pdf = 1 for a white Lambertian sampled along ns.
	HalfSamples cameraSamples;
	ASSERT_EQ(GenerateCameraPath(*scene, *camera, cameraSamples, arena, Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)), 3,
		SurfaceNormalMode::Shading, path), 3);
	EXPECT_NEAR(path[2].mIsect.mPos.x, 3, 1e-4f);
	for (int c = 0; c < 3; ++c) EXPECT_NEAR(path[2].mThroughput[c] / path[1].mThroughput[c], 1, 1e-5f);

	// Importance from the emitter straight down: wo = ng, wi = ns, so the factor is 0.64.
	HalfSamples lightSamples;
	ASSERT_EQ(GenerateLightPath(*scene, lightSamples, arena, 3, SurfaceNormalMode::Shading, path), 3);
	EXPECT_EQ(path[0].mType, PathVertexType::Light);
	EXPECT_NEAR(path[1].mIsect.mPos.z, 0, 1e-5f);
	EXPECT_NEAR(path[2].mIsect.mPos.x, 3, 1e-4f);
	for (int c = 0; c < 3; ++c) EXPECT_NEAR(path[2].mThroughput[c] / path[1].mThroughput[c], 0.64f, 1e-5f);
	// Densities are untouched by the correction: the reverse density at the receiver is the
	// Lambertian pdf of wo = ng in the shading frame, converted with the emitter's ng.
	EXPECT_NEAR(path[0].mPdfPrev, 0.8f * INV_PI / 4, 1e-5f);

	// Geometric mode samples along ng, straight up into the emitter's front face: there is
	// no adjoint factor, so the legacy ratio f |wi.ng| / pdf = 1.
	HalfSamples geometricSamples;
	ASSERT_EQ(GenerateLightPath(*scene, geometricSamples, arena, 3, SurfaceNormalMode::Geometric, path), 3);
	EXPECT_NEAR(path[2].mIsect.mPos.z, 2, 1e-4f);
	for (int c = 0; c < 3; ++c) EXPECT_NEAR(path[2].mThroughput[c] / path[1].mThroughput[c], 1, 1e-5f);
	arena.Reset();
}

TEST(BDPTShadingNormal, TriangleSamplesUseUnitNormalsOrientedLikeIntersections) {
	// Clockwise winding: the face normal points to -z, opposite the authored vertex normals.
	Triangle triangle(Transform(), Transform(), Vec3(0, 10, 0), Vec3(10, -10, 0), Vec3(-10, -10, 0),
		kTilted * 2, kTilted * 2, kTilted * 2);
	real pdf;
	const Intersection sample = triangle.Sample(&pdf, Vec2(0.3f, 0.6f));
	EXPECT_NEAR(sample.mNormal.Length(), 1, 1e-6f);
	EXPECT_NEAR(sample.mNormal.x, 0.6f, 1e-6f);
	EXPECT_EQ(sample.mGeometryNormal, Vec3(0, 0, 1));
	EXPECT_EQ(sample.mAbsNormal, sample.mGeometryNormal);
	Intersection hit;
	real t;
	const Ray ray(Vec3(0, -2, 1), Vec3(0, 0, -1));
	ASSERT_TRUE(triangle.Intersect(ray, &hit, &t));
	triangle.QueryIntersectionInfo(ray, &hit);
	EXPECT_EQ(hit.mGeometryNormal, sample.mGeometryNormal);
}

TEST(BDPTShadingNormal, AreaEmissionSidednessUsesTheGeometricNormal) {
	auto emitter = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-1, -1, 0), Vec3(1, -1, 0), Vec3(0, 1, 0), Vec3(0, 0, 1));
	AreaLight light(emitter, Vec3(5));
	const Intersection point = TiltedPoint();
	const Vec3 belowGeometry = Vec3(1, 0, -0.2f).Norm();
	ASSERT_GT(Dot(belowGeometry, point.mNormal), 0);
	EXPECT_EQ(light.Emission(point, belowGeometry), Vec3(0));
	EXPECT_EQ(light.Emission(point, Vec3(0, 0, 1)), Vec3(5));
	real pdfPos, pdfDir;
	ASSERT_TRUE(light.PdfLight(point, belowGeometry, &pdfPos, &pdfDir));
	EXPECT_EQ(pdfDir, 0);
	ASSERT_TRUE(light.PdfLight(point, Vec3(0, 0, 1), &pdfPos, &pdfDir));
	EXPECT_NEAR(pdfDir, INV_PI, 1e-6f);
	// Sampled emission directions are cosine-distributed about the geometric normal.
	Intersection sampled;
	Vec3 dir;
	real sampledPos, sampledDir;
	light.SampleLight(&sampled, &dir, &sampledPos, &sampledDir, Vec2(0.2f, 0.3f), Vec2(0.5f, 0.5f));
	EXPECT_NEAR(dir.z, 1, 1e-6f);
	EXPECT_NEAR(sampledDir, INV_PI, 1e-6f);
}

TEST(BDPTShadingNormal, ScatterThroughputPreservesGeometricArithmetic) {
	PathVertex vertex;
	vertex.mIsect.mNormal = vertex.mIsect.mGeometryNormal = Vec3(0, 0, 1);
	const Vec3 wo(0, 0, 1), wi(0.6f, 0, 0.8f), f(0.15f, 0.6f, 0.27f);
	const real pdf = 0.37f;
	Vec3 throughput(10, 0.2f, 5);
	Vec3 expected = throughput;
	expected *= f * std::abs(Dot(vertex.mIsect.mNormal, wi)) / pdf;
	ASSERT_TRUE(Bidirectional::ScatterThroughput(vertex, TransportMode::Importance,
		wo, wi, f, pdf, &throughput));
	EXPECT_EQ(throughput, expected);
}

TEST(BDPTShadingNormal, ScatterThroughputKeepsShadingProductsInDoubleUntilStorage) {
	PathVertex vertex;
	vertex.mIsect = TiltedPoint();
	vertex.mNormalMode = SurfaceNormalMode::Shading;
	const Vec3 wo(0, 0, 1), wi = kTilted, f(0.15f, 0.6f, 0.27f);
	const real pdf = 0.37f;
	const Vec3 before(10, 0.2f, 5);
	Vec3 throughput = before;
	ASSERT_TRUE(Bidirectional::ScatterThroughput(vertex, TransportMode::Importance,
		wo, wi, f, pdf, &throughput));
	const double cosine = double(0.8f) * double(0.8f);
	for (int channel = 0; channel < 3; ++channel) {
		const real expected = real(double(before[channel]) * double(f[channel]) * cosine / double(pdf));
		EXPECT_FLOAT_EQ(throughput[channel], expected);
	}
}

TEST(BDPTShadingNormal, RejectedScatterLeavesThroughputUnchanged) {
	PathVertex vertex;
	vertex.mIsect = TiltedPoint();
	vertex.mNormalMode = SurfaceNormalMode::Shading;
	const Vec3 before(2, 3, 4);
	Vec3 throughput = before;
	ASSERT_FALSE(Bidirectional::ScatterThroughput(vertex, TransportMode::Importance,
		Vec3(0, 0, 1), Vec3(1, 0, -0.1f).Norm(), Vec3(1), 1, &throughput));
	EXPECT_EQ(throughput, before);
}

