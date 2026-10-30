#include "gtest/gtest.h"
#include "integrator/VCM.h"
#include "integrator/VCMRecursive.h"
#include "integrator/VCMRecursiveMis.h"
#include "accelerator/BruteForceAccel.h"
#include "camera/PinholeCamera.h"
#include "image/ImageIO.h"
#include "light/AreaLight.h"
#include "light/EnvLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Glass.h"
#include "material/Mirror.h"
#include "sampler/Sampler.h"
#include "texture/ConstantTexture.h"
#include "visual/Rng.h"
#include <array>
#include <cmath>
#include <cstdio>

using namespace Gyunity;

namespace {

using Vcm::Mode;
using Vcm::Strategy;
using Vcm::Technique;

const double Pi = std::acos(-1.0);
const std::array<Mode, 3> Modes{ Mode::Combined, Mode::ConnectionOnly, Mode::MergingOnly };

class Samples : public StateSequence {
public:
	explicit Samples(uint64 seed) : rng(seed) {}
	real Sample() override { ++mCursor; return rng.UniformFloat(); }
private:
	Pcg32 rng;
};

std::string OutputDirectory() {
	const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	EXPECT_NE(directory, nullptr) << "Set GYUNITY_TEST_OUTPUT_DIRECTORY to an existing main-root Test subdirectory.";
	return directory ? directory : "";
}

// A box with mirror (x), glass (y) and diffuse (z) walls, a ceiling emitter and a point light.
// With an environment the +x side is open, so environment photons reach the interior.
void BuildScene(Scene& scene, const std::string& environment) {
	auto white = std::make_shared<ConstantTexture<Vec3>>(Vec3(0.8f));
	auto diffuse = std::make_shared<DiffuseMaterial>(white);
	auto mirror = std::make_shared<MirrorMaterial>(white);
	auto glass = std::make_shared<GlassMaterial>(white, white);
	for (int axis = 0; axis < 3; ++axis) for (int sign : { -1, 1 }) {
		if (!environment.empty() && axis == 0 && sign == 1) continue;
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
	if (!environment.empty()) scene.AddLight(std::make_shared<EnvironmentLight>(environment, 0));
	scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
	scene.Initialize();
}

std::string EnvironmentImage(const std::string& directory) {
	const std::string filename = directory + "\\vcm-recursive-environment.hdr";
	EXPECT_TRUE(ImageIO::WriteImage(filename, std::vector<Vec3>(16, Vec3(2)), 4, 4));
	return filename;
}

bool HasDelta(const std::vector<PathVertex>& path) {
	for (const auto& vertex : path)
		if (vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf->IsDelta()) return true;
	return false;
}

bool Continuous(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface && !vertex.mIsect.mBsdf->IsDelta();
}

struct Coverage {
	int connections = 0, merges = 0, deltaPaths = 0, emitterHits = 0, lightTracing = 0, pointOrigins = 0,
		environmentOrigins = 0;
};

// Every feasible strategy weight of one light/camera pair against the log-domain reference.
void CompareWeights(const Scene& scene, const std::vector<PathVertex>& light, const std::vector<PathVertex>& camera,
	double radius, int64 nVm, Mode mode, int maxDepth, Coverage& coverage) {
	const double eta = Vcm::UseVm(mode) ? double(nVm) * Pi * radius * radius : 0;
	std::vector<Vcm::MisVertex> lightMis, cameraMis;
	Vcm::PrepareLightMis(light, eta, mode, lightMis);
	Vcm::PrepareCameraMis(camera, eta, mode, cameraMis);
	const auto feasible = [&](const Vcm::PathContext& path) {
		double logPdf;
		EXPECT_TRUE(Vcm::ComputePathPdf(scene, path, path.selected, &logPdf));
		return std::isfinite(logPdf);
	};
	for (int t = 1; t <= int(camera.size()); ++t) {
		for (int s = 0; s <= int(light.size()); ++s) {
			if (s + t - 2 < 0 || s + t - 2 > maxDepth ||
				!Bidirectional::ValidConnection(s ? &light[s - 1] : nullptr, camera[t - 1], s, t)) continue;
			const Strategy strategy{ Technique::Connection, s, t };
			const Vcm::PathContext path(light, camera, strategy, radius, nVm, mode);
			if (!Vcm::IsStrategyEnabled(path, strategy) || !feasible(path)) continue;
			SCOPED_TRACE(testing::Message() << "connection " << s << "," << t << " mode " << int(mode));
			real expected, actual;
			ASSERT_TRUE(Vcm::ComputeMisWeight(scene, path, strategy, &expected));
			ASSERT_TRUE(Vcm::RecursiveConnectionWeight(scene, light, lightMis, camera, cameraMis, s, t, eta, mode, &actual));
			EXPECT_NEAR(actual, expected, 1e-6);
			++coverage.connections;
			coverage.emitterHits += s == 0;
			coverage.lightTracing += t == 1;
		}
		if (!Vcm::UseVm(mode) || t < 2 || !Continuous(camera[t - 1])) continue;
		for (int s = 2; s <= int(light.size()); ++s) {
			if (s + t - 3 > maxDepth || !Continuous(light[s - 1])) continue;
			const Strategy strategy{ Technique::Merging, s, t };
			const Vcm::PathContext path(light, camera, strategy, radius, nVm, mode);
			if (!feasible(path)) continue;
			SCOPED_TRACE(testing::Message() << "merge " << s << "," << t << " mode " << int(mode));
			real expected, actual;
			ASSERT_TRUE(Vcm::ComputeMisWeight(scene, path, strategy, &expected));
			ASSERT_TRUE(Vcm::RecursiveMergeWeight(light, lightMis, camera, cameraMis, s, t, eta, &actual));
			EXPECT_NEAR(actual, expected, 1e-6);
			++coverage.merges;
		}
	}
}

std::vector<Vec3> LoadFilm(const Camera& camera, const std::string& filename) {
	const auto film = camera.GetFilm();
	film->SetFileName(filename);
	EXPECT_TRUE(film->SaveImage());
	std::vector<Vec3> image;
	int width = 0, height = 0;
	EXPECT_TRUE(ImageIO::LoadImage(filename, image, width, height));
	EXPECT_EQ(width, film->mResX);
	EXPECT_EQ(height, film->mResY);
	return image;
}

}

TEST(VCMRecursive, EveryStrategyWeightMatchesTheLogDomainReference) {
	const std::string directory = OutputDirectory();
	ASSERT_FALSE(directory.empty());
	const std::string environment = EnvironmentImage(directory);
	constexpr int count = 256, maxDepth = 6;
	constexpr real radius = 0.6f;
	for (bool open : { false, true }) for (auto normals : { SurfaceNormalMode::Geometric, SurfaceNormalMode::Shading }) {
		SCOPED_TRACE(testing::Message() << "open " << open << " normals " << int(normals));
		Scene scene{};
		BuildScene(scene, open ? environment : std::string());
		PinHoleCamera camera(std::make_shared<Film>(8, 8), Vec3(0), Vec3(0, 0, 1), Vec3(0, 1, 0), 120, 1);
		Coverage coverage;
		for (int k = 0; k < count; ++k) {
			MemoryPool lightArena, cameraArena;
			Samples lightSamples(0x51a7u + k), cameraSamples(0xc0deu + k);
			std::vector<PathVertex> light, eye;
			ASSERT_GE(GenerateLightPath(scene, lightSamples, lightArena, maxDepth + 1, normals, light), 1);
			const Ray ray = camera.GenerateRay(k % 8, (k / 8) % 8, Vec2(0.37f, 0.61f), 0);
			ASSERT_GE(GenerateCameraPath(scene, camera, cameraSamples, cameraArena, ray, maxDepth + 2, normals, eye), 1);
			coverage.deltaPaths += HasDelta(light) || HasDelta(eye);
			coverage.pointOrigins += light[0].mLight->IsDeltaLight();
			coverage.environmentOrigins += light[0].mType == PathVertexType::Environment;
			for (Mode mode : Modes) CompareWeights(scene, light, eye, radius, count, mode, maxDepth, coverage);
		}
		// Each camera path pairs with its own light path; the open box has fewer continuous vertices.
		EXPECT_GE(coverage.connections, 1000);
		EXPECT_GE(coverage.merges, 100);
		EXPECT_GE(coverage.deltaPaths, 10);
		EXPECT_GE(coverage.emitterHits, 5);
		EXPECT_GE(coverage.lightTracing, 20);
		// The environment dominates light selection in the open box; the closed box covers point origins.
		if (open) EXPECT_GE(coverage.environmentOrigins, 2);
		else EXPECT_GE(coverage.pointOrigins, 2);
		std::printf("open %d normals %d: %d connections, %d merges, %d delta paths, %d emitter hits, %d light tracing, %d point and %d environment origins\n",
			int(open), int(normals), coverage.connections, coverage.merges, coverage.deltaPaths, coverage.emitterHits,
			coverage.lightTracing, coverage.pointOrigins, coverage.environmentOrigins);
		RecordProperty(std::string("compared_") + (open ? "open_" : "closed_") + std::to_string(int(normals)),
			coverage.connections + coverage.merges);
	}
}

TEST(VCMRecursive, RendersMatchTheReferenceIntegratorInEveryMode) {
	const std::string directory = OutputDirectory();
	ASSERT_FALSE(directory.empty());
	Scene scene{};
	BuildScene(scene, EnvironmentImage(directory));
	for (Mode mode : Modes) for (int frequency : { 1, 3 }) {
		SCOPED_TRACE(testing::Message() << "mode " << int(mode) << " frequency " << frequency);
		const auto name = directory + "\\vcm-recursive-" + std::to_string(int(mode)) + "-" + std::to_string(frequency);
		PinHoleCamera referenceCamera(std::make_shared<Film>(12, 9), Vec3(0), Vec3(0, 0, 1), Vec3(0, 1, 0), 120, 1);
		PinHoleCamera recursiveCamera(std::make_shared<Film>(12, 9), Vec3(0), Vec3(0, 0, 1), Vec3(0, 1, 0), 120, 1);
		VertexConnectionMerging reference(std::make_shared<RandomSampler>(17), 5, 6, 0.4f, real(2.0 / 3), mode, 1,
			false, frequency);
		RecursiveVertexConnectionMerging recursive(std::make_shared<RandomSampler>(17), 5, 6, 0.4f, real(2.0 / 3),
			mode, 1, false, frequency);
		ASSERT_TRUE(reference.RenderChecked(scene, referenceCamera));
		ASSERT_TRUE(recursive.RenderChecked(scene, recursiveCamera));
		EXPECT_EQ(recursive.GetStatistics().mCameraSamples, reference.GetStatistics().mCameraSamples);
		EXPECT_EQ(recursive.GetStatistics().mPeakPhotons, reference.GetStatistics().mPeakPhotons);
		const auto expected = LoadFilm(referenceCamera, name + "-reference.exr");
		const auto actual = LoadFilm(recursiveCamera, name + "-recursive.exr");
		ASSERT_EQ(actual.size(), expected.size());
		double total = 0;
		for (size_t i = 0; i < actual.size(); ++i) for (int c = 0; c < 3; ++c) {
			EXPECT_NEAR(actual[i][c], expected[i][c], 1e-4 * (1 + std::abs(expected[i][c]))) << "pixel " << i;
			total += expected[i][c];
		}
		EXPECT_GT(total, 0);
	}
}

TEST(VCMRecursive, RejectsInvalidParametersLikeTheReference) {
	Scene scene{};
	BuildScene(scene, std::string());
	PinHoleCamera camera(std::make_shared<Film>(4, 4), Vec3(0), Vec3(0, 0, 1), Vec3(0, 1, 0), 120, 1);
	const auto sampler = std::make_shared<RandomSampler>(3);
	EXPECT_FALSE(RecursiveVertexConnectionMerging(nullptr, 3, 1, 0.1f).RenderChecked(scene, camera));
	EXPECT_FALSE(RecursiveVertexConnectionMerging(sampler, 3, 0, 0.1f).RenderChecked(scene, camera));
	EXPECT_FALSE(RecursiveVertexConnectionMerging(sampler, 3, 1, 0).RenderChecked(scene, camera));
	EXPECT_FALSE(RecursiveVertexConnectionMerging(sampler, 3, 1, 0.1f, 1).RenderChecked(scene, camera));
	EXPECT_FALSE(RecursiveVertexConnectionMerging(sampler, 3, 2, 0.1f, real(2.0 / 3), Mode::Combined, 1, false, 3)
		.RenderChecked(scene, camera));
}
