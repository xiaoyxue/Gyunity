#include "gtest/gtest.h"
#include "visual/Scene.h"
#include "camera/PinholeCamera.h"
#include "accelerator/BruteForceAccel.h"
#include "accelerator/BVHAccel.h"
#include "accelerator/KdTreeAccel.h"
#include "shape/Sphere.h"
#include "material/Diffuse.h"
#include "material/Mirror.h"
#include "texture/ConstantTexture.h"
#include "light/AreaLight.h"
#include "light/EnvLight.h"
#include "integrator/PT.h"
#include "integrator/VPT.h"
#include "integrator/LT.h"
#include "sampler/SamplerEnum.h"
#include "volume/Homogeneous.h"
#include <array>
#include <cmath>
#include <cstring>
#include <new>
#include <stdexcept>

using namespace Gyunity;

namespace {

class FixedSequence : public StateSequence {
public:
	real Sample() override { return 0.3f; }
};

bool Finite(const Vec3& value) {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

std::shared_ptr<Triangle> MakePlane() {
	return std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-2, -2, 0), Vec3(2, -2, 0), Vec3(0, 2, 0), Vec3(0, 0, 1));
}

void InitializeScene(const std::shared_ptr<Scene>& scene) {
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
}

class RenderingNumeric : public testing::Test {
protected:
	std::string EnvironmentPath() {
		const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIR");
		if (!directory || !directory[0]) {
			throw std::runtime_error("Set GYUNITY_TEST_OUTPUT_DIR to an existing directory under the main repository's Test folder.");
		}
		const std::string path = std::string(directory) + "\\constant-environment.hdr";
		if (!ImageIO::WriteImage(path, std::vector<Vec3>(16, Vec3(2)), 4, 4)) {
			throw std::runtime_error("Cannot create the numerical environment fixture: " + path);
		}
		return path;
	}

	std::shared_ptr<Scene> EmissiveMirrorScene() {
		auto scene = std::make_shared<Scene>();
		auto shape = MakePlane();
		auto mirror = std::make_shared<MirrorMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
		auto emitter = std::make_shared<AreaLight>(shape, Vec3(3));
		scene->AddPrimitive(std::make_shared<Primitive>(shape, mirror, emitter));
		scene->AddLight(emitter);
		scene->AddLight(std::make_shared<EnvironmentLight>(EnvironmentPath(), 0));
		InitializeScene(scene);
		return scene;
	}
};

// #97: a supplied orthonormal basis must not be replaced by unset look-at fields.
TEST_F(RenderingNumeric, ExplicitAxisCamera) {
	auto film = std::make_shared<Film>(16, 16);
	auto referenceFilm = std::make_shared<Film>(16, 16);
	std::shared_ptr<Camera> camera = std::make_shared<PinHoleCamera>(film, Vec3(0, 0, 3),
		Vec3(0, 0, -1), Vec3(1, 0, 0), Vec3(0, 1, 0), 53.13010235415597f, 1);
	std::shared_ptr<Camera> reference = std::make_shared<PinHoleCamera>(referenceFilm,
		Vec3(0, 0, 3), Vec3(0, 0, 0), Vec3(0, 1, 0), 53.13010235415597f, 1);
	const Ray actual = camera->GenerateRay(8, 8, Vec2(0.5f, 0.5f));
	const Ray expected = reference->GenerateRay(8, 8, Vec2(0.5f, 0.5f));
	RecordProperty("actual_direction_finite", Finite(actual.mDir) ? "true" : "false");
	ASSERT_TRUE(Finite(expected.mDir));
	ASSERT_TRUE(Finite(actual.mDir)) << actual.mDir;
	EXPECT_NEAR(actual.mDir.x, expected.mDir.x, 1e-6f);
	EXPECT_NEAR(actual.mDir.y, expected.mDir.y, 1e-6f);
	EXPECT_NEAR(actual.mDir.z, expected.mDir.z, 1e-6f);
}

// #94: the sphere's sampling and evaluation strategies must describe the same measure.
TEST_F(RenderingNumeric, SphereLightPdf) {
	Sphere sphere(Transform(), Transform(), 1, Vec3(0));
	sphere.mShapeId = 0;
	Intersection reference;
	reference.mPos = Vec3(0, 0, 2);
	real sampledPdf = 0;
	const Intersection point = sphere.Sample(reference, &sampledPdf, Vec2(0.3f, 0.4f));
	const Vec3 direction = (point.mPos - reference.mPos).Norm();
	const real evaluatedPdf = sphere.Pdf(reference, direction);
	const real conePdf = 1 / (2 * PI * (1 - std::sqrt(0.75f)));
	RecordProperty("sampled_pdf", std::to_string(sampledPdf));
	RecordProperty("evaluated_pdf", std::to_string(evaluatedPdf));
	RecordProperty("expected_cone_pdf", std::to_string(conePdf));
	EXPECT_NEAR(sampledPdf, conePdf, 1e-5f);
	EXPECT_NEAR(evaluatedPdf, conePdf, 1e-5f);
}

// #96: agreement between two callers of the same wrong Jacobian is not a reference.
TEST_F(RenderingNumeric, EnvironmentJacobian) {
	EnvironmentMap environment(EnvironmentPath(), 0);
	const Vec2 sample(0.3f, 0.02f);
	real uv[2], uvPdf;
	environment.mDistribution->SampleContinuous(sample.x, sample.y, uv, &uvPdf);
	Vec3 direction;
	real actualPdf;
	const Vec3 radiance = environment.Sample(&direction, &actualPdf, sample);
	const double expectedPdf = uvPdf / (2 * double(PI) * PI * std::sin(double(PI) * uv[1]));
	RecordProperty("latitude_v", std::to_string(uv[1]));
	RecordProperty("actual_pdf", std::to_string(actualPdf));
	RecordProperty("expected_pdf", std::to_string(expectedPdf));
	ASSERT_TRUE(Finite(direction));
	ASSERT_GT(radiance.x, 0);
	ASSERT_GT(expectedPdf, 0);
	EXPECT_NEAR(actualPdf, expectedPdf, expectedPdf * 1e-5);
}

// #100: an emissive mirror supplies a deterministic earlier contribution before escape.
TEST_F(RenderingNumeric, PTAccumulatesEnvironment) {
	auto scene = EmissiveMirrorScene();
	auto sampler = std::make_shared<RandomSampler>(123);
	auto enumeration = std::make_shared<SamplerEnum>();
	PathTracing integrator(1, 3, sampler, enumeration);
	FixedSequence samples;
	MemoryPool arena;
	const Vec3 actual = integrator.Li(Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)), *scene, samples, arena);
	RecordProperty("actual_red", std::to_string(actual.x));
	RecordProperty("expected_red", "5");
	EXPECT_NEAR(actual.x, 5, 1e-5f);
	EXPECT_NEAR(actual.y, 5, 1e-5f);
	EXPECT_NEAR(actual.z, 5, 1e-5f);
}

TEST_F(RenderingNumeric, VPTAccumulatesEnvironment) {
	auto scene = EmissiveMirrorScene();
	auto sampler = std::make_shared<RandomSampler>(123);
	auto enumeration = std::make_shared<SamplerEnum>();
	VolPathTracing integrator(1, 3, sampler, enumeration, false);
	FixedSequence samples;
	MemoryPool arena;
	const Vec3 actual = integrator.Li(Ray(Vec3(0, 0, 1), Vec3(0, 0, -1)), *scene, samples, arena);
	RecordProperty("actual_red", std::to_string(actual.x));
	RecordProperty("expected_red", "5");
	EXPECT_NEAR(actual.x, 5, 1e-5f);
	EXPECT_NEAR(actual.y, 5, 1e-5f);
	EXPECT_NEAR(actual.z, 5, 1e-5f);
}

// #101: examine object representation without evaluating an indeterminate pointer.
TEST_F(RenderingNumeric, DefaultBsdfPointer) {
	alignas(Intersection) unsigned char storage[sizeof(Intersection)];
	std::memset(storage, 0xa5, sizeof(storage));
	auto* intersection = new (storage) Intersection;
	std::array<unsigned char, sizeof(BSDF*)> actual;
	std::memcpy(actual.data(), &intersection->mBsdf, actual.size());
	BSDF* expected = nullptr;
	const bool initializedToNull = std::memcmp(actual.data(), &expected, actual.size()) == 0;
	intersection->~Intersection();
	RecordProperty("initialized_to_null", initializedToNull ? "true" : "false");
	EXPECT_TRUE(initializedToNull);
}

// #95: exiting a homogeneous half-space must not extend attenuation into vacuum.
TEST_F(RenderingNumeric, BoundaryTransmittance) {
	HomogeneousMedium medium(Vec3(0), Vec3(1), 0);
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(MakePlane(), nullptr, nullptr,
		MediumInterface(&medium, nullptr)));
	InitializeScene(scene);
	FixedSequence samples;
	Intersection intersection;
	Ray ray(Vec3(0, 0, -1), Vec3(0, 0, 1), 10, &medium);
	Vec3 transmittance;
	const bool hitMaterial = scene->IntersectTr(ray, samples, &intersection, &transmittance);
	RecordProperty("actual_transmittance", std::to_string(transmittance.x));
	RecordProperty("expected_transmittance", std::to_string(std::exp(-1.f)));
	RecordProperty("exited_to_vacuum", ray.mMedium == nullptr ? "true" : "false");
	EXPECT_FALSE(hitMaterial);
	EXPECT_EQ(ray.mMedium, nullptr);
	EXPECT_NEAR(transmittance.x, std::exp(-1.f), 1e-5f);
}

// #98: emitted-ray sampling returns radiance; the parameterless emission is not a substitute.
TEST_F(RenderingNumeric, EnvironmentLightPathThroughput) {
	auto scene = std::make_shared<Scene>();
	auto diffuse = std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
	scene->AddPrimitive(std::make_shared<Primitive>(MakePlane(), diffuse));
	auto environment = std::make_shared<EnvironmentLight>(EnvironmentPath(), 0);
	scene->AddLight(environment);
	InitializeScene(scene);
	auto film = std::make_shared<Film>(16, 16);
	PinHoleCamera camera(film, Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
	LightTracing integrator(std::make_shared<RandomSampler>(123), 2, 1);
	FixedSequence samples;
	MemoryPool arena;
	std::vector<PathVertex> path(3);
	const int vertices = integrator.GenerateLightPath(*scene, camera, samples, arena, path);
	Intersection emissionPoint;
	Vec3 direction;
	real positionPdf, directionPdf;
	const Vec3 sampledRadiance = environment->SampleLight(&emissionPoint, &direction,
		&positionPdf, &directionPdf, Vec2(0.3f, 0.3f), Vec2(0.3f, 0.3f));
	RecordProperty("sampled_radiance", std::to_string(sampledRadiance.x));
	RecordProperty("light_vertex_throughput", std::to_string(path[0].mThroughput.x));
	ASSERT_GE(vertices, 1);
	ASSERT_GT(sampledRadiance.x, 0);
	EXPECT_NEAR(path[0].mThroughput.x, sampledRadiance.x, 1e-5f);
}

// #93: full-NDF anisotropic samples must be unit normals.
TEST_F(RenderingNumeric, AnisotropicGgxNormals) {
	GGXDistribution distribution(0.2f, 0.5f, false);
	const Vec3 normal = distribution.SampleWh(Vec3(0, 0, 1), Vec2(0.5f, 0));
	RecordProperty("normal_length", std::to_string(normal.Length()));
	EXPECT_NEAR(normal.Length(), 1, 1e-5f);
}

TEST_F(RenderingNumeric, FullNdfGgxPdf) {
	GGXDistribution distribution(0.2f, 0.5f, false);
	const Vec3 wo(0.6f, 0, 0.8f), normal(0, 0, 1);
	const real actualPdf = distribution.Pdf(wo, normal);
	const real expectedPdf = distribution.D(normal) * std::abs(normal.z);
	RecordProperty("actual_pdf", std::to_string(actualPdf));
	RecordProperty("expected_pdf", std::to_string(expectedPdf));
	EXPECT_NEAR(actualPdf, expectedPdf, 1e-5f);
}

// #99: accelerator choice must not change a valid triangle-edge intersection.
TEST_F(RenderingNumeric, BvhParallelBoundary) {
	auto shape = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(0, 0, 0), Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1));
	shape->mShapeId = 0;
	auto primitive = std::make_shared<Primitive>(shape);
	primitive->mPrimId = 0;
	const std::vector<std::shared_ptr<Primitive>> primitives = { primitive };
	BruteForce bruteForce(primitives);
	BVHAccel bvh(primitives);
	KdTreeAccel kd(primitives);
	const Ray source(Vec3(0, 0.25f, 1), Vec3(0, 0, -1));
	Intersection intersection;
	real distance;
	ASSERT_TRUE(shape->Intersect(source, &intersection, &distance));
	EXPECT_FLOAT_EQ(distance, 1);
	Ray bruteRay = source, bvhRay = source, kdRay = source;
	const bool bruteHit = bruteForce.Intersect(bruteRay, &intersection);
	const bool bvhHit = bvh.Intersect(bvhRay, &intersection);
	const bool kdHit = kd.Intersect(kdRay, &intersection);
	RecordProperty("brute_hit", bruteHit ? "true" : "false");
	RecordProperty("bvh_hit", bvhHit ? "true" : "false");
	RecordProperty("kd_hit", kdHit ? "true" : "false");
	EXPECT_TRUE(bruteHit);
	EXPECT_TRUE(kdHit);
	EXPECT_TRUE(bvhHit);
}

}
