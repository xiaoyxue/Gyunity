#include "gtest/gtest.h"
#include "integrator/BDPTPath.h"
#include "camera/PinholeCamera.h"
#include "accelerator/BruteForceAccel.h"
#include "material/Diffuse.h"
#include "material/Mirror.h"
#include "material/Glass.h"
#include "texture/ConstantTexture.h"
#include "light/AreaLight.h"
#include "light/PointLight.h"
#include "light/EnvLight.h"
#include "shape/Sphere.h"
#include "volume/Homogeneous.h"
#include <limits>

using namespace Gyunity;

namespace {

class PathSamples : public StateSequence {
public:
	real Sample() override { ++mCursor; return 0.5f; }
};

std::shared_ptr<Material> Diffuse() {
	return std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
}

std::shared_ptr<Camera> MakeCamera() {
	return std::make_shared<PinHoleCamera>(std::make_shared<Film>(16, 16),
		Vec3(0, 0, 1), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
}

std::shared_ptr<Triangle> Plane(real z, real normalZ) {
	return std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-10, -10, z), Vec3(10, -10, z), Vec3(0, 10, z), Vec3(0, 0, normalZ));
}

void Initialize(const std::shared_ptr<Scene>& scene) {
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
}

struct SamplingRecord {
	TransportMode mode = TransportMode::Importance;
	Vec3 sampledWo, pdfWo, pdfWi;
	int pdfCalls = 0;
};

class RecordedBxdf : public BxDF {
public:
	explicit RecordedBxdf(SamplingRecord* record, bool zeroPdf = false)
		: BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_DIFFUSE)), mRecord(record), mZeroPdf(zeroPdf) {}
	real Pdf(const Vec3& wo, const Vec3& wi) const override {
		mRecord->pdfWo = wo;
		mRecord->pdfWi = wi;
		++mRecord->pdfCalls;
		return 0.25f;
	}
	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3&) const override {
		mRecord->sampledWo = wo;
		*wi = Vec3(0.6f, 0, 0.8f);
		*pdf = mZeroPdf ? 0 : 0.5f;
		return Vec3(1);
	}
	Vec3 Evaluate(const Vec3&, const Vec3&) const override { return Vec3(1); }
private:
	SamplingRecord* mRecord;
	bool mZeroPdf;
};

class RecordedMaterial : public Material {
public:
	explicit RecordedMaterial(SamplingRecord* record, bool zeroPdf = false, bool mixed = false)
		: mRecord(record), mZeroPdf(zeroPdf), mMixed(mixed) {}
	void ComputeScatteringFunction(Intersection* isect, MemoryPool& arena, TransportMode mode) const override {
		mRecord->mode = mode;
		isect->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*isect);
		isect->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, RecordedBxdf)(mRecord, mZeroPdf));
		if (mMixed) isect->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(1)));
	}
private:
	SamplingRecord* mRecord;
	bool mZeroPdf, mMixed;
};

}

TEST(BDPTPath, EndpointMetadataStartsDefined) {
	PathVertex vertex;
	EXPECT_EQ(vertex.mType, PathVertexType::Surface);
	EXPECT_EQ(vertex.mLight, nullptr);
	EXPECT_EQ(vertex.mCamera, nullptr);
	EXPECT_EQ(vertex.mPdfFwd, 0);
	EXPECT_EQ(vertex.mPdfPrev, 0);
	EXPECT_EQ(vertex.mLightSelectionPdf, 0);
	EXPECT_FALSE(vertex.mIsDelta);
}

TEST(BDPTPath, DensityUsesDistanceSquaredAndDestinationNormal) {
	PathVertex from, to;
	from.mIsect.mPos = Vec3(0);
	to.mIsect.mPos = Vec3(0, 0, 2);
	to.mIsect.mGeometryNormal = Vec3(0, 0, 1);
	real pdf;
	ASSERT_TRUE(ConvertDensity(0.5f, from, to, &pdf));
	EXPECT_FLOAT_EQ(pdf, 0.125f);
	to.mIsect.mPos.z = 4;
	ASSERT_TRUE(ConvertDensity(0.5f, from, to, &pdf));
	EXPECT_FLOAT_EQ(pdf, 0.03125f);
	to.mIsect.mGeometryNormal = Vec3(std::sqrt(0.75f), 0, 0.5f);
	ASSERT_TRUE(ConvertDensity(0.5f, from, to, &pdf));
	EXPECT_FLOAT_EQ(pdf, 0.015625f);
}

TEST(BDPTPath, EnvironmentDensityHasNoFiniteDistanceFalloff) {
	PathVertex from, to;
	from.mType = PathVertexType::Environment;
	from.mDirection = Vec3(0, 0, 1);
	to.mIsect.mGeometryNormal = Vec3(std::sqrt(0.75f), 0, 0.5f);
	real nearPdf, farPdf;
	to.mIsect.mPos = Vec3(0, 0, 2);
	ASSERT_TRUE(ConvertDensity(0.25f, from, to, &nearPdf));
	to.mIsect.mPos.z = 1000;
	ASSERT_TRUE(ConvertDensity(0.25f, from, to, &farPdf));
	EXPECT_FLOAT_EQ(nearPdf, 0.125f);
	EXPECT_FLOAT_EQ(farPdf, nearPdf);
	ASSERT_TRUE(ConvertDensity(0.4f, to, from, &farPdf));
	EXPECT_FLOAT_EQ(farPdf, 0.4f);
}

TEST(BDPTPath, DensityRejectsCoincidentAndInvalidInputs) {
	PathVertex from, to;
	real pdf = 1;
	EXPECT_FALSE(ConvertDensity(1, from, to, &pdf));
	EXPECT_EQ(pdf, 0);
	to.mIsect.mPos = Vec3(0, 0, 1);
	EXPECT_FALSE(ConvertDensity(std::numeric_limits<real>::quiet_NaN(), from, to, &pdf));
	EXPECT_EQ(pdf, 0);
	for (real invalid : {-1.f, std::numeric_limits<real>::infinity()}) {
		EXPECT_FALSE(ConvertDensity(invalid, from, to, &pdf));
		EXPECT_EQ(pdf, 0);
	}
}

TEST(BDPTPath, RoundedAdjacentEndpointsHaveZeroContinuousAreaDensity) {
	PathVertex from, to;
	from.mIsect.mPos = Vec3(-1.3118889331817627f, 1.7996618747711182f, 0.8655623197555542f);
	to.mIsect.mPos = Vec3(-1.3118879795074463f, 1.7996618747711182f, 0.8655621409416199f);
	to.mIsect.mGeometryNormal = Vec3(0, -1, 0);
	const Vec3 sampledDirection(0.80235755443573f, 0.4362890124320984f, 0.40727680921554565f);
	EXPECT_GT((to.mIsect.mPos - from.mIsect.mPos).Length2(), 0);
	EXPECT_GT(std::abs(Dot(to.mIsect.mGeometryNormal, sampledDirection)), 0.4f);
	real pdf = 1;
	ASSERT_TRUE(ConvertDensity(0.20677432417869568f, from, to, &pdf));
	EXPECT_EQ(pdf, 0);
}

TEST(BDPTPath, OneUlpNormalDisplacementRetainsPositiveAreaSupport) {
	PathVertex from, to;
	from.mIsect.mPos = Vec3(1, 1, 1);
	to.mIsect.mPos = Vec3(1.000001f, 1, 1);
	to.mIsect.mGeometryNormal = Vec3(0, 1, 0);
	real pdf;
	ASSERT_TRUE(ConvertDensity(0.25f, from, to, &pdf));
	EXPECT_EQ(pdf, 0);
	for (real y : {NextFloatUp(1.f), NextFloatDown(1.f)}) {
		to.mIsect.mPos.y = y;
		ASSERT_TRUE(ConvertDensity(0.25f, from, to, &pdf));
		EXPECT_TRUE(std::isfinite(pdf));
		EXPECT_GT(pdf, 0);
	}
}

TEST(BDPTPath, VerySmallPositiveAreaDensityStillExtendsThePath) {
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0, 1), Diffuse()));
	Initialize(scene);
	const Ray ray(Vec3(0, 0, 1e10f), Vec3(0, 0, -1));
	auto camera = std::make_shared<PinHoleCamera>(std::make_shared<Film>(16, 16),
		ray.mOrig, Vec3(0), Vec3(0, 1, 0), 45, 1);
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	ASSERT_EQ(GenerateCameraPath(*scene, *camera, samples, arena, ray, 2, SurfaceNormalMode::Geometric, path), 2);
	ASSERT_EQ(path.size(), 2u);
	EXPECT_TRUE(std::isfinite(path[1].mPdfFwd));
	EXPECT_GT(path[1].mPdfFwd, 0);
	EXPECT_LT(path[1].mPdfFwd, 1e-18f);
}

TEST(BDPTPath, ZeroAreaDensityTerminatesBeforeAppendingAnInvalidVertex) {
	// CC0 Veach bidir capture: a short hit whose barycentric position rounds onto the ray-origin plane.
	auto triangle = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-3.7068750858306885f, 1.7996619939804077f, 1.3785699605941772f),
		Vec3(-1.0668749809265137f, 1.7996619939804077f, -3.7356009483337402f),
		Vec3(-1.0668749809265137f, 1.7996619939804077f, 1.3785699605941772f),
		Vec3(0, -1, 0));
	const Ray ray(Vec3(-1.3118882179260254f, 1.7996618747711182f, 0.8655620813369751f),
		Vec3(0.80235755443573f, 0.4362890124320984f, 0.40727680921554565f));
	Intersection hit;
	real distance = INF;
	ASSERT_TRUE(triangle->Intersect(ray, &hit, &distance));
	triangle->QueryIntersectionInfo(ray, &hit);
	ASSERT_GT(distance, 0);
	ASSERT_EQ(hit.mPos.y, ray.mOrig.y);
	ASSERT_GT(std::abs(Dot(hit.mGeometryNormal, ray.mDir)), 0.4f);

	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(triangle, Diffuse()));
	Initialize(scene);
	auto camera = std::make_shared<PinHoleCamera>(std::make_shared<Film>(16, 16),
		ray.mOrig, ray.mOrig + ray.mDir, Vec3(0, 1, 0), 45, 1);
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	EXPECT_EQ(GenerateCameraPath(*scene, *camera, samples, arena, ray, 2, SurfaceNormalMode::Geometric, path), 1);
	ASSERT_EQ(path.size(), 1u);
	EXPECT_EQ(path.front().mType, PathVertexType::Camera);
	EXPECT_GT(path.front().mPdfFwd, 0);
	EXPECT_EQ(samples.GetCursor(), 0);
}

TEST(BDPTPath, CameraEndpointAndVertexBudget) {
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0, 1), Diffuse()));
	Initialize(scene);
	auto camera = MakeCamera();
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	const Ray ray(Vec3(0, 0, 1), Vec3(0, 0, -1));
	EXPECT_EQ(GenerateCameraPath(*scene, *camera, samples, arena, ray, 0, SurfaceNormalMode::Geometric, path), 0);
	EXPECT_EQ(GenerateCameraPath(*scene, *camera, samples, arena, ray, 1, SurfaceNormalMode::Geometric, path), 1);
	ASSERT_EQ(path.size(), 1u);
	EXPECT_EQ(path[0].mCamera, camera.get());
	EXPECT_EQ(path[0].mType, PathVertexType::Camera);
	EXPECT_EQ(samples.GetCursor(), 0);
	EXPECT_EQ(GenerateCameraPath(*scene, *camera, samples, arena, ray, 2, SurfaceNormalMode::Geometric, path), 2);
	EXPECT_EQ(path[1].mType, PathVertexType::Surface);
	EXPECT_NEAR(path[1].mPdfFwd, camera->PdfDir(ray), 1e-5f);
	EXPECT_EQ(samples.GetCursor(), 0);
}

TEST(BDPTPath, ReversePdfUsesSavedIncomingDirection) {
	SamplingRecord record;
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0, 1), std::make_shared<RecordedMaterial>(&record)));
	Initialize(scene);
	auto camera = MakeCamera();
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	ASSERT_EQ(GenerateCameraPath(*scene, *camera, samples, arena,
		Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)), 3, SurfaceNormalMode::Geometric, path), 2);
	EXPECT_EQ(record.mode, TransportMode::Radiance);
	EXPECT_EQ(record.pdfCalls, 1);
	EXPECT_EQ(record.pdfWo, Vec3(0.6f, 0, 0.8f));
	EXPECT_EQ(record.pdfWi, record.sampledWo);
	EXPECT_EQ(record.pdfWi, Vec3(0, 0, 1));
	EXPECT_NEAR(path[0].mPdfPrev, 0.25f, 1e-6f);
	EXPECT_EQ(samples.GetCursor(), 3);
}

TEST(BDPTPath, ZeroPdfKeepsReachedVertexWithoutExtending) {
	SamplingRecord record;
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0, 1), std::make_shared<RecordedMaterial>(&record, true)));
	Initialize(scene);
	auto camera = MakeCamera();
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	ASSERT_EQ(GenerateCameraPath(*scene, *camera, samples, arena,
		Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)), 5, SurfaceNormalMode::Geometric, path), 2);
	EXPECT_EQ(path[1].mThroughput, Vec3(1));
	EXPECT_EQ(path[0].mPdfPrev, 0);
	EXPECT_EQ(record.pdfCalls, 0);
}

TEST(BDPTPath, RejectsMixedDeltaAndMediumBoundaries) {
	SamplingRecord record;
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0, 1),
		std::make_shared<RecordedMaterial>(&record, false, true)));
	Initialize(scene);
	auto camera = MakeCamera();
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	const Ray ray(Vec3(0, 0, 1), Vec3(0, 0, -1));
	EXPECT_EQ(GenerateCameraPath(*scene, *camera, samples, arena, ray, 3, SurfaceNormalMode::Geometric, path), -1);
	HomogeneousMedium fog(Vec3(0), Vec3(1), 0);
	auto boundaryScene = std::make_shared<Scene>();
	boundaryScene->AddPrimitive(std::make_shared<Primitive>(Plane(0, 1), nullptr, nullptr, MediumInterface(&fog, nullptr)));
	Initialize(boundaryScene);
	EXPECT_EQ(GenerateCameraPath(*boundaryScene, *camera, samples, arena, ray, 3, SurfaceNormalMode::Geometric, path), -1);
}

TEST(BDPTPath, LightPathStoresJointOriginPmfAndImportanceMode) {
	SamplingRecord record;
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(1, -1), std::make_shared<RecordedMaterial>(&record, true)));
	auto shape = Plane(0, 1);
	shape->mShapeId = 0;
	auto light = std::make_shared<AreaLight>(shape, Vec3(2));
	scene->AddLight(light);
	scene->AddLight(std::make_shared<AreaLight>(shape, Vec3(2)));
	Initialize(scene);
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	ASSERT_EQ(GenerateLightPath(*scene, samples, arena, 3, SurfaceNormalMode::Geometric, path), 2);
	EXPECT_EQ(record.mode, TransportMode::Importance);
	EXPECT_FLOAT_EQ(path[0].mLightSelectionPdf, 0.5f);
	EXPECT_FLOAT_EQ(path[0].mPdfFwd, 0.5f / shape->Area());
	EXPECT_FLOAT_EQ(path[0].mIsect.mGeometryNormal.z, 1);
	EXPECT_NEAR(path[1].mThroughput.x, 2 * shape->Area() * PI / 0.5f, 1e-3f);
	EXPECT_EQ(samples.GetCursor(), 8);
}

TEST(BDPTPath, PointPositionIsNotAContinuousReverseDensity) {
	auto scene = std::make_shared<Scene>();
	auto point = std::make_shared<PointLight>(Vec3(0, 0, 2), Vec3(3));
	scene->AddLight(point);
	Initialize(scene);
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	ASSERT_EQ(GenerateLightPath(*scene, samples, arena, 1, SurfaceNormalMode::Geometric, path), 1);
	EXPECT_EQ(path[0].mType, PathVertexType::Light);
	EXPECT_FALSE(path[0].mIsDelta);
	EXPECT_TRUE(path[0].mLight->IsDeltaLight());
	EXPECT_FLOAT_EQ(path[0].mPdfFwd, 1);
	PathVertex surface;
	surface.mIsect.mPos = Vec3(0);
	real pdf;
	ASSERT_TRUE(ConvertDensity(0.3f, surface, path[0], &pdf));
	EXPECT_EQ(pdf, 0);
}

TEST(BDPTPath, MirrorEventAndEnvironmentEscapeAreRetained) {
	const char* file = "..\\resources\\media\\pictures\\uffizi-large.hdr";
	ASSERT_TRUE(std::ifstream(file).good()) << "Run from GyunityApp.";
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0, 1),
		std::make_shared<MirrorMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)))));
	auto environment = std::make_shared<EnvironmentLight>(file, 0);
	scene->AddLight(environment);
	Initialize(scene);
	auto camera = MakeCamera();
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	ASSERT_EQ(GenerateCameraPath(*scene, *camera, samples, arena,
		Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)), 4, SurfaceNormalMode::Geometric, path), 3);
	EXPECT_TRUE(path[1].mIsDelta);
	EXPECT_EQ(path[2].mType, PathVertexType::Environment);
	EXPECT_EQ(path[2].mLight, environment.get());
	EXPECT_EQ(path[2].mDirection, Vec3(0, 0, 1));
	EXPECT_EQ(path[2].mThroughput, Vec3(1));
	EXPECT_EQ(path[2].mPdfFwd, 0);
	EXPECT_EQ(path[0].mPdfPrev, 0);
}

TEST(BDPTPath, EmptyAndInvalidBudgetsAreExplicit) {
	auto scene = std::make_shared<Scene>();
	Initialize(scene);
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	EXPECT_EQ(GenerateLightPath(*scene, samples, arena, 3, SurfaceNormalMode::Geometric, path), 0);
	EXPECT_EQ(samples.GetCursor(), 0);
	EXPECT_EQ(GenerateLightPath(*scene, samples, arena, -1, SurfaceNormalMode::Geometric, path), -1);
	auto camera = MakeCamera();
	EXPECT_EQ(GenerateCameraPath(*scene, *camera, samples, arena,
		Ray(Vec3(0), Vec3(0)), 3, SurfaceNormalMode::Geometric, path), -1);
}

TEST(BDPTPath, GlassTransmissionUsesRadianceAndImportanceModes) {
	auto scene = std::make_shared<Scene>();
	auto white = std::make_shared<ConstantTexture<Vec3>>(Vec3(1));
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0, 1), std::make_shared<GlassMaterial>(white, white)));
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(-1, 1), Diffuse()));
	auto emitterShape = Plane(1, -1);
	emitterShape->mShapeId = 0;
	scene->AddLight(std::make_shared<AreaLight>(emitterShape, Vec3(1)));
	Initialize(scene);
	auto camera = MakeCamera();
	PathSamples cameraSamples, lightSamples;
	MemoryPool arena;
	std::vector<PathVertex> cameraPath, lightPath;
	ASSERT_EQ(GenerateCameraPath(*scene, *camera, cameraSamples, arena,
		Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)), 3, SurfaceNormalMode::Geometric, cameraPath), 3);
	ASSERT_EQ(GenerateLightPath(*scene, lightSamples, arena, 3, SurfaceNormalMode::Geometric, lightPath), 3);
	EXPECT_TRUE(cameraPath[1].mIsDelta);
	EXPECT_TRUE(lightPath[1].mIsDelta);
	EXPECT_NEAR(cameraPath[2].mThroughput.x, 1 / (1.5f * 1.5f), 1e-5f);
	EXPECT_NEAR(lightPath[2].mThroughput.x, emitterShape->Area() * PI, 1e-3f);
	EXPECT_EQ(cameraPath[2].mPdfFwd, 0);
	EXPECT_EQ(lightPath[2].mPdfFwd, 0);
}

TEST(BDPTPath, EnvironmentLaunchUsesDiskDensityAtFirstSurface) {
	const char* file = "..\\resources\\media\\pictures\\uffizi-large.hdr";
	ASSERT_TRUE(std::ifstream(file).good()) << "Run from GyunityApp.";
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(
		std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3(0)), Diffuse()));
	auto environment = std::make_shared<EnvironmentLight>(file, 0);
	scene->AddLight(environment);
	Initialize(scene);
	PathSamples samples;
	MemoryPool arena;
	std::vector<PathVertex> path;
	ASSERT_EQ(GenerateLightPath(*scene, samples, arena, 2, SurfaceNormalMode::Geometric, path), 2);
	EXPECT_EQ(path[0].mType, PathVertexType::Environment);
	EXPECT_EQ(path[0].mLight, environment.get());
	EXPECT_NEAR(path[0].mPdfPos, 1 / (3 * PI), 1e-6f);
	EXPECT_FLOAT_EQ(path[0].mPdfFwd, path[0].mLightSelectionPdf * path[0].mPdfDir);
	const real expectedSurfacePdf = path[0].mPdfPos *
		std::abs(Dot(path[1].mIsect.mGeometryNormal, path[0].mDirection));
	EXPECT_NEAR(path[1].mPdfFwd, expectedSurfacePdf, 1e-6f);
	EXPECT_NEAR(path[1].mThroughput.x,
		path[0].mThroughput.x / path[0].mPdfPos / path[0].mPdfDir, 1e-4f);
}
