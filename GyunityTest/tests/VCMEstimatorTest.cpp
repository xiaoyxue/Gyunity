#include "gtest/gtest.h"
#include "integrator/VCMPath.h"
#include "integrator/VCMPhotonMap.h"
#include "VCMReference.h"
#include "accelerator/BruteForceAccel.h"
#include "camera/PinholeCamera.h"
#include "image/ImageIO.h"
#include "light/AreaLight.h"
#include "light/EnvLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Glass.h"
#include "material/Mirror.h"
#include "texture/ConstantTexture.h"
#include "visual/Rng.h"
#include "volume/Homogeneous.h"
#include <array>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

using namespace Gyunity;

namespace {

using Paths = std::vector<std::vector<PathVertex>>;
using Vcm::Mode;
using Vcm::Technique;
const double Pi = std::acos(-1.0);
const Vec3 Up(0, 0, 1);
double comparisonMaximum = 0;

class FixedCamera : public PinHoleCamera {
public:
	FixedCamera() : PinHoleCamera(std::make_shared<Film>(4, 4), Up, Vec3(0), Vec3(0, 1, 0), 70, 1) {}
	real PdfDir(const Ray&) const override { return 0.5f; }
};

class FixedLight : public AreaLight {
public:
	explicit FixedLight(const std::shared_ptr<Shape>& shape) : AreaLight(shape, Vec3(1)) {}
	bool PdfLight(const Intersection&, const Vec3&, real* position, real* direction) const override {
		*position = 0.5f;
		*direction = 0.25f;
		return true;
	}
};

class RecordingLobe : public BxDF {
public:
	mutable int evaluations = 0;
	mutable Vec3 lastWo, lastWi;
	Vec3 value;
	real pdf = 0.125f;
	bool directional = false;
	explicit RecordingLobe(Vec3 value = Vec3(0.125f))
		: BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_GLOSSY)), value(value) {}
	real Pdf(const Vec3&, const Vec3&) const override { return pdf; }
	Vec3 Sample(const Vec3&, Vec3*, real*, const Vec3&) const override { return Vec3(0); }
	Vec3 Evaluate(const Vec3& wo, const Vec3& wi) const override {
		++evaluations;
		lastWo = wo;
		lastWi = wi;
		return value * (directional ? 1 + 0.4f * wi.x + 0.3f * wi.z : 1);
	}
};

// Snapshot the original storage, including all intersection and sampling metadata.
// Do not compare padding in independently copied PathVertex objects.
std::vector<unsigned char> Snapshot(const std::vector<PathVertex>& path) {
	std::vector<unsigned char> result(path.size() * sizeof(PathVertex));
	if (!result.empty()) std::memcpy(result.data(), path.data(), result.size());
	return result;
}

double RelativeError(const Vec3& actual, const std::array<double, 3>& expected) {
	double error = 0;
	for (int c = 0; c < 3; ++c)
		error = std::max(error, std::abs(double(actual[c]) - expected[c]) / std::max(1e-30, std::abs(expected[c])));
	return error;
}

void Close(const Vec3& actual, const std::array<double, 3>& expected, double tolerance = 2e-6) {
	const double error = RelativeError(actual, expected);
	comparisonMaximum = std::max(comparisonMaximum, error);
	EXPECT_LE(error, tolerance);
	for (int c = 0; c < 3; ++c) EXPECT_TRUE(std::isfinite(actual[c]));
}

class VCMEstimator : public testing::Test {
protected:
	Scene scene{};
	MemoryPool arena;
	FixedCamera camera;
	std::shared_ptr<FixedLight> light;
	Paths paths;
	std::vector<PathVertex> eye;
	Vcm::PhotonMap map;
	RecordingLobe cameraLobe, photonLobe{ Vec3(7) };
	Vec3 result;

	PathVertex Surface(const Vec3& position, BxDF* lobe, Vec3 normal = Up) {
		PathVertex vertex;
		auto& hit = vertex.mIsect;
		hit.mPos = position;
		hit.mNormal = hit.mGeometryNormal = hit.mAbsNormal = normal;
		hit.mOutDir = Up;
		hit.mPrimitive = nullptr;
		hit.mPrimId = 3;
		hit.mShapeId = 7;
		hit.mB1 = 0.2f; hit.mB2 = 0.3f;
		CoordinateSystem(normal, &hit.mShadingDpDu, &hit.mShadingDpDv);
		hit.mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(hit);
		hit.mBsdf->Add(lobe);
		vertex.mThroughput = Vec3(1);
		vertex.mPdfFwd = vertex.mPdfPrev = 0.125f;
		return vertex;
	}

	void SetUp() override {
		comparisonMaximum = 0;
		auto shape = std::make_shared<Triangle>(Transform(), Transform(), Vec3(-1, -1, 1),
			Vec3(1, -1, 1), Vec3(0, 1, 1), -Up);
		light = std::make_shared<FixedLight>(shape);
		scene.AddLight(light);
		scene.SetAccelerator(std::make_shared<BruteForce>());
		scene.Initialize();
		PathVertex origin;
		origin.mType = PathVertexType::Light;
		origin.mLight = light.get();
		origin.mIsect.mPos = Up;
		origin.mIsect.mNormal = origin.mIsect.mGeometryNormal = -Up;
		origin.mPdfFwd = 0.5f;
		origin.mThroughput = Vec3(123); // Not another factor in the local estimator.
		paths = { { origin, Surface(Vec3(0), &photonLobe) } };
		paths[0][1].mPdfFwd = 0.25f;
		paths[0][1].mThroughput = Vec3(3, 5, 7);
		PathVertex sensor;
		sensor.mType = PathVertexType::Camera;
		sensor.mCamera = &camera;
		sensor.mIsect.mPos = Up;
		sensor.mPdfFwd = 1;
		eye = { sensor, Surface(Vec3(0), &cameraLobe) };
		eye[1].mPdfFwd = 0.5f;
		eye[1].mThroughput = Vec3(2, 4, 6);
	}

	void TearDown() override {
		std::ostringstream text;
		text << std::setprecision(17) << comparisonMaximum;
		RecordProperty("max_checked_relative_error", text.str());
	}

	bool Pair(Mode mode = Mode::MergingOnly, double radius = 0.5, int64 count = 4, int s = 2, int t = 2) {
		return Vcm::EvaluateMerging(scene, Vcm::PathContext(paths[0], eye,
			{ Technique::Merging, s, t }, radius, count, mode), &result);
	}

	bool Gather(int t = 2, int depth = 1, Mode mode = Mode::MergingOnly) {
		return Vcm::GatherMerging(scene, map, paths, eye, t, depth, mode, &result);
	}

	std::array<double, 3> Expected(double scattering, double weight, double radius, int64 count) const {
		std::array<double, 3> expected;
		for (int c = 0; c < 3; ++c)
			expected[c] = double(eye.back().mThroughput[c]) * paths[0].back().mThroughput[c] *
				scattering * weight / (double(count) * Pi * radius * radius);
		return expected;
	}
};

TEST_F(VCMEstimator, IndependentLambertianHasSeparateBrdfAndKernelPi) {
	const Vec3 rho(0.25f, 0.5f, 0.75f);
	auto* diffuse = GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(rho);
	const auto throughput = eye[1].mThroughput;
	eye[1] = Surface(Vec3(0), diffuse);
	eye[1].mThroughput = throughput;
	eye[1].mPdfFwd = 0.5f;
	const auto lightBefore = Snapshot(paths[0]), eyeBefore = Snapshot(eye);
	for (double radius : { 0.125, 0.5, 2.0 }) for (int64 count : { 1, 4, 17 }) {
		ASSERT_TRUE(Pair(Mode::MergingOnly, radius, count));
		std::array<double, 3> expected;
		for (int c = 0; c < 3; ++c)
			expected[c] = double(throughput[c]) * paths[0][1].mThroughput[c] * rho[c] /
				(double(count) * Pi * Pi * radius * radius);
		Close(result, expected);
	}
	EXPECT_EQ(Snapshot(paths[0]), lightBefore);
	EXPECT_EQ(Snapshot(eye), eyeBefore);
	EXPECT_EQ(photonLobe.evaluations, 0);
	RecordProperty("independent_lambertian_cases", 9);
}

TEST_F(VCMEstimator, ManualDepthOneJointScoresAndModes) {
	// Unit edges: VC scores 1/16, 4/16, 2/16; selected VM score N*A/16.
	for (double radius : { 0.25, 0.5, 2.0 }) for (int64 count : { 1, 4, 16 }) {
		const double eta = count * Pi * radius * radius;
		for (Mode mode : { Mode::Combined, Mode::MergingOnly, Mode::ConnectionOnly }) {
			ASSERT_TRUE(Pair(mode, radius, count));
			const double weight = mode == Mode::Combined ? eta / (7 + eta) :
				mode == Mode::MergingOnly ? 1 : 0;
			Close(result, Expected(0.125, weight, radius, count));
		}
	}
	EXPECT_EQ(cameraLobe.evaluations, 18);
	EXPECT_EQ(photonLobe.evaluations, 0);
	RecordProperty("manual_joint_cases", 27);
}

TEST_F(VCMEstimator, ManualTwoVmStrategiesShareOneJointDenominator) {
	eye[0].mIsect.mPos = Vec3(0);
	eye[0].mPdfPrev = 0.125f;
	eye[1].mIsect.mPos = Up;
	eye.push_back(Surface(Vec3(0), &cameraLobe));
	// Four logical vertices: sum VC=15/128, each of two VM scores=N*A/128.
	for (Mode mode : { Mode::Combined, Mode::MergingOnly }) for (int64 count : { 1, 8 }) {
		const double eta = count * Pi * 0.25;
		ASSERT_TRUE(Pair(mode, 0.5, count, 2, 3));
		Close(result, Expected(0.125, mode == Mode::Combined ? eta / (15 + 2 * eta) : 0.5, 0.5, count));
	}
	RecordProperty("manual_two_vm_cases", 4);
}

TEST_F(VCMEstimator, CameraMaterialAndOriginalDirectionNotMovedPredecessorControlScatter) {
	cameraLobe.directional = true;
	paths[0][1].mIsect.mOutDir = Vec3(0.6f, 0, 0.8f);
	for (real offset : { 0.f, 0.2f, 0.4f }) {
		paths[0][1].mIsect.mPos = Vec3(offset, 0, 0);
		const auto before = Snapshot(paths[0]);
		ASSERT_TRUE(Pair());
		const Vec3 original = eye[1].mIsect.mBsdf->WorldToLocal(paths[0][1].mIsect.mOutDir);
		EXPECT_EQ(cameraLobe.lastWi, original);
		EXPECT_EQ(cameraLobe.lastWo, eye[1].mIsect.mBsdf->WorldToLocal(eye[1].mIsect.mOutDir));
		EXPECT_GT(((paths[0][0].mIsect.mPos - paths[0][1].mIsect.mPos).Norm() -
			paths[0][1].mIsect.mOutDir).Length(), 0.1f);
		Close(result, Expected(0.125 * (1 + 0.4 * original.x + 0.3 * original.z), 1, 0.5, 4));
		EXPECT_EQ(Snapshot(paths[0]), before);
	}
	EXPECT_EQ(cameraLobe.evaluations, 3);
	EXPECT_EQ(photonLobe.evaluations, 0);
}

TEST_F(VCMEstimator, NoExtraCosineDistanceOrVisibilityAcrossTheDisk) {
	auto white = std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
	auto blocker = std::make_shared<Triangle>(Transform(), Transform(), Vec3(0.1f, -1, -1),
		Vec3(0.1f, 1, -1), Vec3(0.1f, 0, 1), Vec3(-1, 0, 0));
	scene.AddPrimitive(std::make_shared<Primitive>(blocker, white));
	scene.Initialize();
	paths[0][1].mIsect.mPos = Vec3(0.2f, 0, 0);
	paths[0][1].mIsect.mOutDir = Vec3(0.8f, 0, 0.6f);
	ASSERT_TRUE(scene.Intersect(Ray(eye[1].mIsect.mPos, Vec3(1, 0, 0), 0.2f)));
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather());
	Close(result, Expected(0.125, 1, 0.5, 1));
	// Both distances/cosines change but the single-strategy constant-scatter estimate does not.
	const Vec3 previous = result;
	paths[0][1].mIsect.mPos = Vec3(0.4f, 0, 0);
	paths[0][1].mIsect.mOutDir = Up;
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather());
	EXPECT_EQ(previous, result);
	EXPECT_EQ(photonLobe.evaluations, 0);
}

TEST_F(VCMEstimator, DuplicateEmittedPathsAndMissesUseAttemptCountNotHits) {
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather());
	const Vec3 base = result;
	paths.push_back(paths[0]);
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather());
	EXPECT_EQ(base, result);
	paths.push_back({});
	paths.push_back({ paths[0][0] }); // An emitted ray which missed every surface.
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather());
	EXPECT_EQ(base * 0.5f, result);
	paths.push_back(paths[0]);
	paths.back()[1].mIsect.mPos = Vec3(5, 0, 0); // Stored, but not a query hit.
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather());
	for (int c = 0; c < 3; ++c) EXPECT_NEAR(result[c], base[c] * 0.4f, 1e-6);
	EXPECT_EQ(map.RecordCount(), 3u);
	RecordProperty("attempts_records_hits", "5,3,2");
}

TEST_F(VCMEstimator, CountChangesJointMisRatherThanAssumingCombinedInvariance) {
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather(2, 1, Mode::Combined));
	Close(result, Expected(0.125, Pi * 0.25 / (7 + Pi * 0.25), 0.5, 1));
	const Vec3 previous = result;
	paths.push_back(paths[0]);
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather(2, 1, Mode::Combined));
	Close(result, Expected(0.125, Pi * 0.5 / (7 + Pi * 0.5), 0.5, 1));
	EXPECT_GT(result.x, previous.x);
}

TEST_F(VCMEstimator, FirstLightSurfaceAndEveryCameraVertexRespectPhysicalDepth) {
	eye[0].mIsect.mPos = Vec3(0);
	eye[0].mPdfPrev = 0.125f;
	eye[1].mIsect.mPos = Up;
	eye.push_back(Surface(Vec3(0), &cameraLobe));
	paths[0][1].mIsect.mPos = Vec3(0.1f, 0, 0.5f);
	ASSERT_TRUE(map.Build(paths, 0.75f));
	int positive = 0;
	for (int t : { 2, 3 }) for (int depth : { 0, 1, 2, 3 }) {
		ASSERT_TRUE(Gather(t, depth));
		if (depth < t - 1) EXPECT_EQ(result, Vec3(0));
		else { EXPECT_GT(result.x, 0); ++positive; }
	}
	EXPECT_EQ(positive, 5);
	RecordProperty("positive_camera_prefixes", 2);
}

TEST_F(VCMEstimator, EmptyMapAndNonmergeableCameraVerticesAreNormalZero) {
	paths = { {}, { paths[0][0] } };
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather());
	EXPECT_EQ(result, Vec3(0));
	ASSERT_TRUE(Gather(1));
	EXPECT_EQ(result, Vec3(0));
	auto* delta = GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(1));
	eye[1] = Surface(Vec3(0), delta);
	ASSERT_FALSE(eye[1].mIsDelta); // Terminal type, not sampled-event marker, decides eligibility.
	ASSERT_TRUE(Gather());
	EXPECT_EQ(result, Vec3(0));
}

TEST_F(VCMEstimator, ConnectionOnlyNeedsNoBuiltMapOrActivePathInputs) {
	Paths noPaths;
	std::vector<PathVertex> noCamera;
	ASSERT_TRUE(Vcm::GatherMerging(scene, map, noPaths, noCamera, -1, 0, Mode::ConnectionOnly, &result));
	EXPECT_EQ(result, Vec3(0));
	ASSERT_TRUE(Vcm::EvaluateMerging(scene, Vcm::PathContext(noCamera, noCamera,
		{ Technique::Merging, -1, -1 }, 0, 0, Mode::ConnectionOnly), &result));
	EXPECT_EQ(result, Vec3(0));
}

TEST_F(VCMEstimator, DirectPairIgnoresUnselectedSuffixesAndTerminalContinuationMasses) {
	ASSERT_TRUE(Pair());
	const auto expected = result;
	PathVertex unused;
	unused.mPdfFwd = std::numeric_limits<real>::quiet_NaN();
	paths[0].push_back(unused);
	eye.push_back(unused);
	paths[0][1].mDeltaPdf = eye[1].mDeltaReversePdf = std::numeric_limits<real>::quiet_NaN();
	const auto lightBefore = Snapshot(paths[0]), eyeBefore = Snapshot(eye);
	ASSERT_TRUE(Pair());
	EXPECT_EQ(expected, result);
	EXPECT_EQ(Snapshot(paths[0]), lightBefore);
	EXPECT_EQ(Snapshot(eye), eyeBefore);
}

TEST_F(VCMEstimator, GatherIgnoresUnselectedCameraSuffix) {
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather());
	const auto expected = result;
	PathVertex unused;
	unused.mPdfFwd = unused.mDeltaPdf = std::numeric_limits<real>::quiet_NaN();
	eye.push_back(unused);
	const auto before = Snapshot(eye);
	ASSERT_TRUE(Gather());
	EXPECT_EQ(expected, result);
	EXPECT_EQ(Snapshot(eye), before);
}

TEST_F(VCMEstimator, DoubleProductsAvoidIntermediateFloatOverflowAndUnderflow) {
	eye[1].mThroughput = Vec3(1e30f);
	paths[0][1].mThroughput = Vec3(1e30f);
	cameraLobe.value = Vec3(1e-20f);
	ASSERT_TRUE(Pair(Mode::MergingOnly, 1e10, 4));
	Close(result, Expected(1e-20f, 1, 1e10, 4));
	eye[1].mThroughput = Vec3(1e-30f);
	paths[0][1].mThroughput = Vec3(1e-30f);
	cameraLobe.value = Vec3(1e30f);
	ASSERT_TRUE(Pair(Mode::MergingOnly, 1e-10, 4));
	Close(result, Expected(1e30f, 1, 1e-10, 4));
}

TEST_F(VCMEstimator, ZeroScatterAndThroughputStillValidateActiveMetadata) {
	cameraLobe.value = Vec3(0);
	ASSERT_TRUE(Pair());
	EXPECT_EQ(result, Vec3(0));
	cameraLobe.value = Vec3(0.125f);
	eye[1].mThroughput = Vec3(0);
	ASSERT_TRUE(Pair());
	EXPECT_EQ(result, Vec3(0));
}

TEST_F(VCMEstimator, ActualRoughTransmissionUsesRadianceModeCameraBsdf) {
	MicrofacetTransmissionBSDF radiance(new GGXDistribution(0.25f), Vec3(0.7f), 1, 1.5f, TransportMode::Radiance);
	MicrofacetTransmissionBSDF importance(new GGXDistribution(0.25f), Vec3(0.7f), 1, 1.5f, TransportMode::Importance);
	class RoughMaterial : public Material {
	public:
		mutable TransportMode observed = TransportMode::Importance;
		mutable std::vector<std::unique_ptr<MicrofacetTransmissionBSDF>> lobes;
		void ComputeScatteringFunction(Intersection* hit, MemoryPool& arena, TransportMode mode) const override {
			observed = mode;
			lobes.emplace_back(new MicrofacetTransmissionBSDF(new GGXDistribution(0.25f), Vec3(0.7f), 1, 1.5f, mode));
			hit->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*hit);
			hit->mBsdf->Add(lobes.back().get());
		}
	};
	class ConstantSamples : public StateSequence {
	public:
		real Sample() override { ++mCursor; return 0.5f; }
	} samples;
	auto material = std::make_shared<RoughMaterial>();
	auto floor = std::make_shared<Triangle>(Transform(), Transform(), Vec3(-3, -3, 0),
		Vec3(3, -3, 0), Vec3(0, 3, 0), Up);
	scene.AddPrimitive(std::make_shared<Primitive>(floor, material));
	scene.Initialize();
	ASSERT_EQ(GenerateCameraPath(scene, camera, samples, arena, Ray(Up, -Up), 2, SurfaceNormalMode::Geometric, eye), 2);
	EXPECT_EQ(material->observed, TransportMode::Radiance);
	ASSERT_FALSE(eye[1].mIsect.mBsdf->IsDelta());
	const auto before = Snapshot(eye);
	paths[0][1].mIsect.mOutDir = -Up;
	paths[0][0].mIsect.mPos = -Up;
	paths[0][0].mIsect.mNormal = paths[0][0].mIsect.mGeometryNormal = Up;
	const Vec3 fRadiance = radiance.Evaluate(Up, -Up);
	const Vec3 fImportance = importance.Evaluate(Up, -Up);
	ASSERT_GT(fRadiance.x, 0);
	EXPECT_NEAR(fImportance.x / fRadiance.x, 2.25f, 1e-5);
	ASSERT_TRUE(Pair());
	Close(result, Expected(fRadiance.x, 1, 0.5, 4));
	EXPECT_EQ(photonLobe.evaluations, 0);
	EXPECT_EQ(Snapshot(eye), before);
}

// Expected negative probes are a separate suite/filter, not silent fallbacks.
class VCMEstimatorNegative : public VCMEstimator {};

TEST_F(VCMEstimatorNegative, RejectsInvalidCountsModesOutputsAndNormalization) {
	for (int64 count : { int64(0), int64(-1) }) { EXPECT_FALSE(Pair(Mode::Combined, 0.5, count)); EXPECT_EQ(result, Vec3(0)); }
	for (double radius : { 0.0, -1.0, std::numeric_limits<double>::infinity(),
		std::numeric_limits<double>::quiet_NaN(), 1e-300, 1e154 }) {
		EXPECT_FALSE(Pair(Mode::Combined, radius));
		EXPECT_EQ(result, Vec3(0));
	}
	EXPECT_FALSE(Pair(Mode(99)));
	EXPECT_FALSE(Pair(Mode::Combined, 1e150, (std::numeric_limits<int64>::max)()));
	EXPECT_EQ(result, Vec3(0));
	EXPECT_FALSE(Pair(Mode::Combined, 0.5, 4, 1, 2));
	EXPECT_FALSE(Pair(Mode::Combined, 0.5, 4, 2, 3));
	EXPECT_FALSE(Pair(Mode::Combined, 0.5, 4, (std::numeric_limits<int>::max)(), 2));
	EXPECT_FALSE(Vcm::EvaluateMerging(scene, Vcm::PathContext(paths[0], eye,
		{ Technique::Connection, 2, 2 }, 0.5, 4), &result));
	EXPECT_FALSE(Vcm::EvaluateMerging(scene, Vcm::PathContext(paths[0], eye,
		{ Technique::Merging, 2, 2 }, 0.5, 4), nullptr));
	EXPECT_FALSE(Gather(2, -1, Mode::ConnectionOnly));
	EXPECT_FALSE(Gather(2, 1, Mode(99)));
	EXPECT_FALSE(Vcm::GatherMerging(scene, map, paths, eye, 2, 1, Mode::Combined, nullptr));
}

TEST_F(VCMEstimatorNegative, UnbuiltEmptyAttemptBatchAndDifferentBatchAreRejected) {
	EXPECT_FALSE(Gather());
	ASSERT_TRUE(map.Build(paths, 0.5f));
	const auto copy = paths;
	EXPECT_FALSE(Vcm::GatherMerging(scene, map, copy, eye, 2, 1, Mode::Combined, &result));
	EXPECT_EQ(result, Vec3(0));
	paths.clear();
	ASSERT_TRUE(map.Build(paths, 0.5f));
	EXPECT_FALSE(Gather());
	map.Clear();
	EXPECT_FALSE(Gather());
}

TEST_F(VCMEstimatorNegative, InvalidCandidatePrefixDiscardsEarlierAccumulationWithoutMutation) {
	paths.push_back(paths[0]);
	paths[1][0].mLight = nullptr; // Build validates surfaces; MIS must validate the selected origin.
	const auto first = Snapshot(paths[0]), second = Snapshot(paths[1]), cameraBefore = Snapshot(eye);
	ASSERT_TRUE(map.Build(paths, 0.5f));
	EXPECT_FALSE(Gather());
	EXPECT_EQ(result, Vec3(0));
	EXPECT_EQ(cameraLobe.evaluations, 1);
	EXPECT_EQ(photonLobe.evaluations, 0);
	EXPECT_EQ(Snapshot(paths[0]), first);
	EXPECT_EQ(Snapshot(paths[1]), second);
	EXPECT_EQ(Snapshot(eye), cameraBefore);
}

TEST_F(VCMEstimatorNegative, DirectPairRequiresInRadiusCompatibleContinuousEndpoints) {
	const auto original = paths[0][1];
	for (real offset : { 0.50001f, 1.0f }) {
		paths[0][1].mIsect.mPos = Vec3(offset, 0, 0);
		EXPECT_FALSE(Pair());
		EXPECT_EQ(result, Vec3(0));
		ASSERT_TRUE(map.Build(paths, 0.5f));
		ASSERT_TRUE(Gather());
		EXPECT_EQ(result, Vec3(0)); // Gather rejects via its kernel, direct API requires accepted inputs.
	}
	paths[0][1] = Surface(Vec3(0), &photonLobe, -Up);
	EXPECT_FALSE(Pair());
	ASSERT_TRUE(map.Build(paths, 0.5f));
	ASSERT_TRUE(Gather());
	EXPECT_EQ(result, Vec3(0));
	paths[0][1] = original;
	auto* delta = GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(1));
	eye[1] = Surface(Vec3(0), delta);
	EXPECT_FALSE(Pair());
}

TEST_F(VCMEstimatorNegative, InvalidSelectedMetadataNeverBecomesAZeroFallback) {
	const auto original = eye[1];
	HomogeneousMedium medium(Vec3(0), Vec3(0.1f), 0);
	for (int defect = 0; defect < 8; ++defect) {
		eye[1] = original;
		switch (defect) {
		case 0: eye[1].mIsect.mBsdf = nullptr; break;
		case 1: eye[1].mIsect.mMediumInterface = MediumInterface(&medium); break;
		case 2: eye[1].mPdfFwd = -1; break;
		case 3: eye[1].mIsect.mOutDir = Vec3(2); break;
		case 4: eye[1].mCamera = &camera; break;
		case 5: eye[1].mIsDelta = true; break;
		case 6: eye[1].mIsect.mNormal = Vec3(0, 1, 0); break;
		case 7: eye[1].mThroughput.x = std::numeric_limits<real>::infinity(); break;
		}
		const auto before = Snapshot(eye);
		EXPECT_FALSE(Pair());
		EXPECT_EQ(result, Vec3(0));
		EXPECT_EQ(Snapshot(eye), before);
	}
	eye[1] = original;
	eye[1].mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(1)));
	EXPECT_FALSE(Pair());
	// Even no hits/depth zero must validate an active camera prefix.
	paths = { {} };
	ASSERT_TRUE(map.Build(paths, 0.5f));
	EXPECT_FALSE(Gather(2, 0));
	EXPECT_EQ(result, Vec3(0));
}

TEST_F(VCMEstimatorNegative, NonfiniteScatterPairRangeAndGatherRangeFailAtomically) {
	for (real value : { std::numeric_limits<real>::infinity(), std::numeric_limits<real>::quiet_NaN() }) {
		cameraLobe.value = Vec3(value);
		EXPECT_FALSE(Pair());
		EXPECT_EQ(result, Vec3(0));
	}
	cameraLobe.value = Vec3(1e30f);
	eye[1].mThroughput = paths[0][1].mThroughput = Vec3(1e30f);
	EXPECT_FALSE(Pair());
	EXPECT_EQ(result, Vec3(0));
	// Each accepted pair is representable; their sum is not. ONE attempt.
	cameraLobe.value = Vec3(1);
	eye[1].mThroughput = Vec3(1);
	paths[0][1].mThroughput = Vec3(2.1e38f);
	paths[0].push_back(Surface(Up, &photonLobe));
	paths[0].back().mThroughput = Vec3(2.1e38f);
	paths[0].push_back(Surface(Vec3(0), &photonLobe));
	paths[0].back().mThroughput = Vec3(2.1e38f);
	ASSERT_TRUE(Pair(Mode::MergingOnly, 0.5, 1, 2, 2));
	ASSERT_TRUE(Pair(Mode::MergingOnly, 0.5, 1, 4, 2));
	ASSERT_TRUE(map.Build(paths, 0.5f));
	EXPECT_FALSE(Gather(2, 4));
	EXPECT_EQ(result, Vec3(0));
}

// Independent finite-radius pair set; no map or production acceptance helpers.
bool Accept(const PathVertex& x, const PathVertex& y, double radius) {
	if (x.mType != PathVertexType::Surface || y.mType != PathVertexType::Surface ||
		x.mIsect.mBsdf->IsDelta() || y.mIsect.mBsdf->IsDelta()) return false;
	double distance2 = 0, nx = 0, ny = 0, dot = 0;
	for (int c = 0; c < 3; ++c) {
		const double d = double(x.mIsect.mPos[c]) - y.mIsect.mPos[c];
		distance2 += d * d;
		nx += double(x.mIsect.mGeometryNormal[c]) * x.mIsect.mGeometryNormal[c];
		ny += double(y.mIsect.mGeometryNormal[c]) * y.mIsect.mGeometryNormal[c];
		dot += double(x.mIsect.mGeometryNormal[c]) * y.mIsect.mGeometryNormal[c];
	}
	return distance2 <= radius * radius && dot / std::sqrt(nx * ny) >= 0.9;
}

double Jacobian(const PathVertex& source, const PathVertex& target) {
	if (target.mType == PathVertexType::Environment) return 1;
	if (source.mType == PathVertexType::Environment)
		return std::abs(Dot(target.mIsect.mGeometryNormal, source.mDirection));
	double d2 = 0, cosineNumerator = 0;
	for (int c = 0; c < 3; ++c) {
		const double d = double(target.mIsect.mPos[c]) - source.mIsect.mPos[c];
		d2 += d * d;
		cosineNumerator += target.mIsect.mGeometryNormal[c] * d;
	}
	if (target.mType == PathVertexType::Camera || (target.mLight && target.mLight->IsDeltaLight())) return 1 / d2;
	return std::abs(cosineNumerator) / (d2 * std::sqrt(d2));
}

double EventPdf(const PathVertex& source, const Vec3& wo, const Vec3& wi) {
	if (!source.mIsect.mBsdf->IsDelta()) return source.mIsect.mBsdf->Pdf(wo, wi);
	const bool reflection = Dot(wo, source.mIsect.mNormal) * Dot(wi, source.mIsect.mNormal) > 0;
	Vec3 sampled;
	real mass;
	const auto value = source.mIsect.mBsdf->Sample(wo, &sampled, &mass,
		Vec3(0.5f, reflection ? 0.f : ONE_MINUS_EPSILON, 0.5f));
	EXPECT_NE(value, Vec3(0));
	EXPECT_LT((sampled - wi).Length(), 2e-5);
	return mass;
}

// Original sampled events and two distinct geometric endpoints supply the
// report oracle. Never calls production ComputePathPdf/ComputeMisWeight.
VcmReference::Edges MergeEdges(const std::vector<PathVertex>& light,
	const std::vector<PathVertex>& eye, int s, int t) {
	const int n = s + t - 1, j = s - 1;
	std::vector<const PathVertex*> left;
	for (int i = 0; i < s; ++i) left.push_back(&light[i]);
	for (int i = t - 2; i >= 0; --i) left.push_back(&eye[i]);
	auto right = left;
	right[j] = &eye[t - 1];
	VcmReference::Edges e;
	e.f.resize(n); e.r.resize(n); e.gf.resize(n, 1); e.gr.resize(n, 1); e.delta.resize(n, false);
	e.pointOrigin = light[0].mLight->IsDeltaLight();
	for (int i = 1; i + 1 < n; ++i) e.delta[i] = left[i]->mIsect.mBsdf->IsDelta();
	for (int i = 1; i < n; ++i) {
		const auto& chain = i <= j ? left : right;
		e.gf[i] = Jacobian(*chain[i - 1], *chain[i]);
	}
	for (int i = 0; i + 1 < n; ++i) {
		const auto& chain = i < j ? left : right;
		e.gr[i] = Jacobian(*chain[i + 1], *chain[i]);
	}
	auto forward = [](const std::vector<PathVertex>& path, int i) -> double {
		if (i <= 1 || !path[i - 1].mIsect.mBsdf->IsDelta()) return path[i].mPdfFwd;
		return EventPdf(path[i - 1], path[i - 1].mIsect.mOutDir, -path[i].mIsect.mOutDir) *
			Jacobian(path[i - 1], path[i]);
	};
	auto reverse = [](const std::vector<PathVertex>& path, int i) {
		return EventPdf(path[i + 1], -path[i + 2].mIsect.mOutDir, path[i + 1].mIsect.mOutDir) *
			Jacobian(path[i + 1], path[i]);
	};
	for (int i = 0; i < n; ++i) {
		if (i < s) e.f[i] = forward(light, i);
		else if (i == s) e.f[i] = eye[t - 1].mIsect.mBsdf->Pdf(light[s - 1].mIsect.mOutDir,
			eye[t - 1].mIsect.mOutDir) * e.gf[i];
		else e.f[i] = reverse(eye, n - 1 - i);
		if (i >= j) e.r[i] = forward(eye, n - 1 - i);
		else if (i == j - 1) e.r[i] = eye[t - 1].mIsect.mBsdf->Pdf(eye[t - 1].mIsect.mOutDir,
			light[s - 1].mIsect.mOutDir) * e.gr[i];
		else e.r[i] = reverse(light, i);
	}
	if (e.pointOrigin) e.r[0] = 0;
	return e;
}

std::array<double, 3> ReferencePair(const std::vector<PathVertex>& light,
	const std::vector<PathVertex>& eye, int s, int t, double radius, size_t count, Mode mode) {
	const auto e = MergeEdges(light, eye, s, t);
	const bool vc = mode != Mode::MergingOnly, vm = mode != Mode::ConnectionOnly;
	const double area = Pi * radius * radius;
	const double weight = VcmReference::Weight(e, true, s, area, double(count), vc, vm);
	const auto direct = VcmReference::DirectWeights(e, area, double(count), vc, vm);
	EXPECT_NEAR(weight, direct[e.f.size() + s - 2], 1e-10);
	const auto& x = eye[t - 1];
	const auto& y = light[s - 1];
	const Vec3 f = x.mIsect.mBsdf->Evaluate(x.mIsect.mOutDir, y.mIsect.mOutDir);
	std::array<double, 3> value;
	for (int c = 0; c < 3; ++c)
		value[c] = double(x.mThroughput[c]) * f[c] * y.mThroughput[c] * weight / (count * area);
	return value;
}

TEST_F(VCMEstimator, BruteForceGatherRetainsDistinctCoincidentRecordsAndFiltersNormalsRadiusDepth) {
	paths.resize(6, paths[0]);
	paths[1][1].mIsect.mPos = Vec3(0.5f, 0, 0); // Exact radius.
	paths[2][1].mIsect.mPos = Vec3(std::nextafter(0.5f, 1.f), 0, 0);
	paths[3][1] = Surface(Vec3(0), &photonLobe, -Up);
	paths[4][1] = Surface(Vec3(0), &photonLobe, Vec3(0.3f, 0, std::sqrt(0.91f)));
	paths[5].push_back(Surface(Up, &photonLobe));
	paths[5].push_back(Surface(Vec3(0), &photonLobe));
	paths[5][0].mPdfPrev = 0.125f;
	ASSERT_TRUE(map.Build(paths, 0.5f));
	int cases = 0;
	for (Mode mode : { Mode::Combined, Mode::MergingOnly }) for (int depth : { 0, 1, 2, 3, 4 }) {
		SCOPED_TRACE(testing::Message() << int(mode) << "," << depth);
		std::array<double, 3> expected{};
		for (const auto& path : paths) for (size_t v = 1; v < path.size(); ++v) {
			if (int(v) > depth || !Accept(eye[1], path[v], 0.5)) continue;
			const auto value = ReferencePair(path, eye, int(v) + 1, 2, 0.5, paths.size(), mode);
			for (int c = 0; c < 3; ++c) expected[c] += value[c];
		}
		ASSERT_TRUE(Gather(2, depth, mode));
		Close(result, expected, 2e-5);
		++cases;
	}
	RecordProperty("brute_force_gathers", cases);
}

TEST_F(VCMEstimatorNegative, DoubleDistanceMatchesMapAtFloatSubtractionBoundary) {
	// float(y-x) rounds to radius, while exact double(y)-double(x) is outside.
	eye[1].mIsect.mPos = Vec3(-0.50000006f, 0, 0);
	paths[0][1].mIsect.mPos = Vec3(0.5f, 0, 0);
	ASSERT_TRUE(map.Build(paths, 1));
	ASSERT_TRUE(Gather());
	EXPECT_EQ(result, Vec3(0));
	EXPECT_FALSE(Pair(Mode::MergingOnly, 1));
	EXPECT_EQ(result, Vec3(0));
}

class Samples : public StateSequence {
public:
	explicit Samples(uint64 seed) : rng(seed) {}
	real Sample() override { ++mCursor; return rng.UniformFloat(); }
private:
	Pcg32 rng;
};

TEST(VCMEstimatorGenerated, MixedOriginsAndSpecularTransportMatchIndependentReportAndBruteForce) {
	const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	ASSERT_NE(directory, nullptr) << "Set GYUNITY_TEST_OUTPUT_DIRECTORY to an existing main-root Test subdirectory.";
	ASSERT_NE(*directory, '\0');
	const std::string filename = std::string(directory) + "\\estimator-environment.hdr";
	ASSERT_TRUE(ImageIO::WriteImage(filename, std::vector<Vec3>(16, Vec3(2)), 4, 4));
	std::ofstream report(std::string(directory) + "\\estimator-generated.csv");
	ASSERT_TRUE(report.is_open()) << "Cannot open requested estimator CSV.";
	report << "camera_sample,light_path,s,t,mode,origin,actual_r,expected_r,relative_error\n" << std::setprecision(17);
	Scene scene{};
	auto white = std::make_shared<ConstantTexture<Vec3>>(Vec3(0.8f));
	auto diffuse = std::make_shared<DiffuseMaterial>(white);
	auto mirror = std::make_shared<MirrorMaterial>(white);
	auto glass = std::make_shared<GlassMaterial>(white, white);
	for (int axis = 0; axis < 3; ++axis) for (int sign : { -1, 1 }) {
		// An open side admits environment photons to the camera-visible interior.
		// A closed inward-facing box would only test black exterior deposits.
		if (axis == 0 && sign == 1) continue;
		Vec3 center(0), u(0), v(0), normal(0);
		center[axis] = real(2 * sign);
		normal[axis] = real(-sign);
		u[(axis + 1) % 3] = 2;
		v[(axis + 2) % 3] = 2;
		for (int half = 0; half < 2; ++half) {
			auto shape = std::make_shared<Triangle>(Transform(), Transform(), center - u - v,
				half == 0 ? center + u - v : center + u + v,
				half == 0 ? center + u + v : center - u + v, normal);
			std::shared_ptr<AreaLight> emitter;
			if (axis == 2 && sign == 1) {
				emitter = std::make_shared<AreaLight>(shape, Vec3(2));
				scene.AddLight(emitter);
			}
			const std::shared_ptr<Material> material = axis == 0 ? std::static_pointer_cast<Material>(mirror) :
				axis == 1 ? std::static_pointer_cast<Material>(glass) : std::static_pointer_cast<Material>(diffuse);
			scene.AddPrimitive(std::make_shared<Primitive>(shape, material, emitter));
		}
	}
	scene.AddLight(std::make_shared<PointLight>(Vec3(0.3f, 0.4f, 0.5f), Vec3(3)));
	scene.AddLight(std::make_shared<EnvironmentLight>(filename, 0));
	scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
	scene.Initialize();
	PinHoleCamera camera(std::make_shared<Film>(8, 8), Vec3(0), Up, Vec3(0, 1, 0), 120, 1);
	// Preserve coverage gates under main's corrected environment power/selection PMF.
	constexpr int count = 256, maxDepth = 6;
	constexpr real radius = 1.5f;
	MemoryPool lightArena;
	Paths paths(count);
	std::vector<std::vector<unsigned char>> beforeLight;
	std::array<int, 3> attempts{}, positives{};
	int offsetVertices = 0, differentDirections = 0;
	double maxDirectionDifference = 0;
	auto checkOriginal = [&](const std::vector<PathVertex>& path) {
		for (size_t i = 1; i < path.size(); ++i) {
			if (path[i].mType != PathVertexType::Surface) continue;
			if (path[i].mIsect.mPointError.Length2() > 0) ++offsetVertices;
			if (path[i - 1].mType == PathVertexType::Environment) continue;
			const double difference = ((path[i - 1].mIsect.mPos - path[i].mIsect.mPos).Norm() -
				path[i].mIsect.mOutDir).Length();
			if (difference > 0) ++differentDirections;
			maxDirectionDifference = std::max(maxDirectionDifference, difference);
		}
	};
	for (int p = 0; p < count; ++p) {
		Samples samples(0x3920u + p);
		ASSERT_GE(GenerateLightPath(scene, samples, lightArena, maxDepth + 1, SurfaceNormalMode::Geometric, paths[p]), 1);
		beforeLight.push_back(Snapshot(paths[p]));
		const auto& origin = paths[p][0];
		++attempts[origin.mType == PathVertexType::Environment ? 2 : origin.mLight->IsDeltaLight() ? 1 : 0];
		checkOriginal(paths[p]);
	}
	Vcm::PhotonMap map;
	ASSERT_TRUE(map.Build(paths, radius));
	int queries = 0, accepted = 0, positive = 0, zero = 0, escapes = 0, deltaQueries = 0;
	int afterMirror = 0, afterGlass = 0, firstSurface = 0, laterCamera = 0;
	std::array<int, maxDepth + 1> depths{};
	double maxPairError = 0, maxGatherError = 0;
	for (int sample = 0; sample < count; ++sample) {
		MemoryPool eyeArena;
		Samples samples(0x7392u + sample);
		std::vector<PathVertex> eye;
		ASSERT_GE(GenerateCameraPath(scene, camera, samples, eyeArena,
			camera.GenerateRay(sample % 8, (sample / 8) % 8, Vec2(0.37f, 0.61f), 0), maxDepth + 2, SurfaceNormalMode::Geometric, eye), 2);
		const auto beforeEye = Snapshot(eye);
		checkOriginal(eye);
		for (int t = 1; t <= int(eye.size()); ++t) {
			const auto& x = eye[t - 1];
			if (x.mType == PathVertexType::Environment) ++escapes;
			if (x.mType == PathVertexType::Surface && x.mIsect.mBsdf->IsDelta()) ++deltaQueries;
			for (Mode mode : { Mode::Combined, Mode::MergingOnly }) {
				SCOPED_TRACE(testing::Message() << "sample=" << sample << " t=" << t << " mode=" << int(mode));
				std::array<double, 3> expected{};
				for (int p = 0; p < count; ++p) for (int s = 2; s <= int(paths[p].size()); ++s) {
					if (s + t - 3 > maxDepth || !Accept(x, paths[p][s - 1], radius)) continue;
					SCOPED_TRACE(testing::Message() << "photon=" << p << " s=" << s);
					const auto value = ReferencePair(paths[p], eye, s, t, radius, count, mode);
					Vec3 actual;
					ASSERT_TRUE(Vcm::EvaluateMerging(scene, Vcm::PathContext(paths[p], eye,
						{ Technique::Merging, s, t }, radius, count, mode), &actual));
					const double error = RelativeError(actual, value);
					maxPairError = std::max(maxPairError, error);
					Close(actual, value, 3e-5);
					for (int c = 0; c < 3; ++c) expected[c] += value[c];
					const int origin = paths[p][0].mType == PathVertexType::Environment ? 2 :
						paths[p][0].mLight->IsDeltaLight() ? 1 : 0;
					report << sample << ',' << p << ',' << s << ',' << t << ',' << int(mode) << ',' << origin
						<< ',' << actual.x << ',' << value[0] << ',' << error << '\n';
					if (mode != Mode::Combined) continue;
					++accepted;
					if (actual == Vec3(0)) { ++zero; continue; }
					++positive;
					++positives[origin];
					++depths[s + t - 3];
					if (s == 2) ++firstSurface;
					if (t > 2) ++laterCamera;
					bool hasMirror = false, hasGlass = false;
					for (auto prefix : { std::make_pair(&paths[p], s), std::make_pair(&eye, t) })
						for (int v = 1; v + 1 < prefix.second; ++v) {
							const auto& vertex = (*prefix.first)[v];
							if (!vertex.mIsDelta) continue;
							const auto* material = vertex.mIsect.mPrimitive->GetMaterial();
							hasMirror |= dynamic_cast<const MirrorMaterial*>(material) != nullptr;
							hasGlass |= dynamic_cast<const GlassMaterial*>(material) != nullptr;
						}
					if (hasMirror) ++afterMirror;
					if (hasGlass) ++afterGlass;
				}
				Vec3 gathered;
				ASSERT_TRUE(Vcm::GatherMerging(scene, map, paths, eye, t, maxDepth, mode, &gathered));
				++queries;
				maxGatherError = std::max(maxGatherError, RelativeError(gathered, expected));
				Close(gathered, expected, 3e-5);
			}
		}
		EXPECT_EQ(Snapshot(eye), beforeEye);
	}
	for (int p = 0; p < count; ++p) EXPECT_EQ(Snapshot(paths[p]), beforeLight[p]);
	EXPECT_GE(positive, 100);
	EXPECT_GE(firstSurface, 20);
	EXPECT_GE(laterCamera, 20);
	EXPECT_GE(afterMirror, 10);
	EXPECT_GE(afterGlass, 10);
	EXPECT_GE(escapes, 10);
	EXPECT_GE(deltaQueries, 20);
	EXPECT_GE(offsetVertices, 100);
	EXPECT_GE(differentDirections, 100);
	for (int origin = 0; origin < 3; ++origin) {
		EXPECT_GE(attempts[origin], 4);
		EXPECT_GE(positives[origin], 10);
		testing::Test::RecordProperty("attempts_origin_" + std::to_string(origin), attempts[origin]);
		testing::Test::RecordProperty("positive_origin_" + std::to_string(origin), positives[origin]);
	}
	for (int depth = 1; depth <= maxDepth; ++depth) {
		EXPECT_GT(depths[depth], 0);
		testing::Test::RecordProperty("positive_depth_" + std::to_string(depth), depths[depth]);
	}
	report.close();
	ASSERT_FALSE(report.fail()) << "Writing requested estimator CSV failed.";
	testing::Test::RecordProperty("generated_light_attempts", count);
	testing::Test::RecordProperty("generated_camera_paths", count);
	testing::Test::RecordProperty("brute_force_gathers", queries);
	testing::Test::RecordProperty("accepted_pairs_combined", accepted);
	testing::Test::RecordProperty("positive_pairs_combined", positive);
	testing::Test::RecordProperty("zero_pairs_combined", zero);
	testing::Test::RecordProperty("positive_first_light_surface", firstSurface);
	testing::Test::RecordProperty("positive_later_camera_vertex", laterCamera);
	testing::Test::RecordProperty("positive_after_mirror", afterMirror);
	testing::Test::RecordProperty("positive_after_glass", afterGlass);
	testing::Test::RecordProperty("camera_escapes", escapes);
	testing::Test::RecordProperty("delta_camera_queries", deltaQueries);
	testing::Test::RecordProperty("offset_surface_vertices", offsetVertices);
	testing::Test::RecordProperty("different_original_directions", differentDirections);
	auto property = [](const char* name, double value) {
		std::ostringstream text;
		text << std::setprecision(17) << value;
		testing::Test::RecordProperty(name, text.str());
	};
	property("max_original_direction_difference", maxDirectionDifference);
	property("max_pair_relative_error", maxPairError);
	property("max_gather_relative_error", maxGatherError);
}

TEST_F(VCMEstimatorNegative, RotatedEnvironmentAndUnregisteredOriginFailEvenWithZeroContribution) {
	const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	ASSERT_NE(directory, nullptr);
	ASSERT_NE(*directory, '\0');
	const std::string filename = std::string(directory) + "\\estimator-negative-environment.hdr";
	ASSERT_TRUE(ImageIO::WriteImage(filename, std::vector<Vec3>(16, Vec3(2)), 4, 4));
	PointLight unregistered(Up, Vec3(1));
	paths[0][0].mLight = &unregistered;
	cameraLobe.value = Vec3(0);
	EXPECT_FALSE(Pair());
	EXPECT_EQ(result, Vec3(0));
	eye[1].mType = PathVertexType::Light;
	eye[1].mLight = &unregistered;
	paths = { {} };
	ASSERT_TRUE(map.Build(paths, 0.5f));
	EXPECT_FALSE(Gather());
	EXPECT_EQ(result, Vec3(0));
	scene.AddLight(std::make_shared<EnvironmentLight>(filename, 0.25f));
	scene.Initialize();
	cameraLobe.value = Vec3(0);
	EXPECT_FALSE(Pair());
	paths = { {} };
	ASSERT_TRUE(map.Build(paths, 0.5f));
	EXPECT_FALSE(Gather());
	EXPECT_EQ(result, Vec3(0));
}

}
