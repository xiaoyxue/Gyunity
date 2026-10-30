#include "stb/stb_image.h"
#include "gtest/gtest.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sys/stat.h>
#include "../../GyunityApp/Examples.h"
#include "../../GyunityApp/SceneCommandLine.h"
#include <set>

namespace {

Examples::Options RenderOptions(const std::string& output, int width, int height, int spp) {
	Examples::Options options;
	options.output = output;
	options.width = width;
	options.height = height;
	options.spp = spp;
	return options;
}

Examples::Options VcmOptions(const std::string& output, int width = 32, int height = 24,
	int iterations = 4, int depth = 4) {
	Examples::Options options;
	options.output = output;
	options.width = width;
	options.height = height;
	options.iterations = iterations;
	options.maxDepth = depth;
	return options;
}

TEST(MainExamples, SceneCliRemainsIndependentOfTheExamplesDispatcher) {
	char app[] = "GyunityApp";
	char list[] = "--list-scenes";
	char help[] = "--help";
	char invalid[] = "--invalid";
	char* argv[] = { app, list };
	std::string expected;
	for (const auto& scene : GetSceneRegistry()) expected += std::string(scene.name) + '\n';

	testing::internal::CaptureStdout();
	const int listResult = RunSceneCommandLine(2, argv);
	const auto listed = testing::internal::GetCapturedStdout();
	EXPECT_EQ(listResult, 0);
	EXPECT_EQ(listed, expected);

	argv[1] = help;
	testing::internal::CaptureStdout();
	const int helpResult = RunSceneCommandLine(2, argv);
	const auto usage = testing::internal::GetCapturedStdout();
	EXPECT_EQ(helpResult, 0);
	EXPECT_NE(usage.find("builder-based PathTracing example"), std::string::npos);
	EXPECT_NE(usage.find("Test\\CornellBoxPtEnvmap.bmp in the main checkout"), std::string::npos);
	EXPECT_NE(usage.find("SPPM, LightTracing and VolPathTracing"), std::string::npos);

	argv[1] = invalid;
	testing::internal::CaptureStderr();
	const int invalidResult = RunSceneCommandLine(2, argv);
	const auto error = testing::internal::GetCapturedStderr();
	EXPECT_EQ(invalidResult, 1);
	EXPECT_NE(error.find("Unknown option"), std::string::npos);
}

TEST(MainExamples, DefaultOutputUsesTheMainCheckoutTestDirectory) {
	for (const auto* filename : { "CornellBoxPtEnvmap.bmp", "CornellBoxSppmTest.bmp",
		"CornellBoxLT.bmp", "MediaTestFogEquiAngular.bmp", "CornellBoxBdpt.bmp", "CornellBoxVcm.hdr" }) {
		std::string output;
		ASSERT_TRUE(Examples::OutputPath(filename, output));
		const auto suffix = std::string("\\Test\\") + filename;
		ASSERT_GE(output.size(), suffix.size());
		EXPECT_EQ(output.substr(output.size() - suffix.size()), suffix);
		const auto root = output.substr(0, output.size() - suffix.size());
		struct _stat info;
		ASSERT_EQ(_stat((root + "\\.git").c_str(), &info), 0);
		EXPECT_NE(info.st_mode & _S_IFDIR, 0);
		ASSERT_EQ(_stat((root + "\\Test").c_str(), &info), 0);
		EXPECT_NE(info.st_mode & _S_IFDIR, 0);
	}
}

TEST(MainExamples, SharedRendererRejectsInvalidSetup) {
	class UnusedIntegrator : public Integrator {
	public:
		bool called = false;
		void Render(const Scene&, const Camera&) override { called = true; }
	} integrator;
	SceneRenderOptions options;
	EXPECT_FALSE(RenderScene(options, integrator));
	options.scene = FindScene("empty");
	EXPECT_FALSE(RenderScene(options, integrator));
	options.output = "unused.png";
	options.width = 0;
	EXPECT_FALSE(RenderScene(options, integrator));
	options.width = -1;
	EXPECT_FALSE(RenderScene(options, integrator));
	options.width = 2147483647;
	EXPECT_FALSE(RenderScene(options, integrator));
	EXPECT_FALSE(integrator.called);
}

class MainExampleRenders : public testing::Test {
protected:
	std::string outputDirectory;

	void SetUp() override {
		const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
		if (!directory || !*directory) {
			GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY to an existing directory under the main checkout's Test.";
		}
		outputDirectory = directory;
	}

	std::string Output(const char* name) const {
		return outputDirectory + '\\' + name;
	}

	void ExpectImage(const std::string& filename) {
		int width = 0, height = 0, channels = 0;
		std::unique_ptr<float, decltype(&stbi_image_free)> pixels(
			stbi_loadf(filename.c_str(), &width, &height, &channels, 3), stbi_image_free);
		ASSERT_NE(pixels, nullptr) << filename;
		ASSERT_EQ(width, 32);
		ASSERT_EQ(height, 24);
		bool hasLight = false;
		for (int i = 0; i < width * height * 3; ++i) {
			ASSERT_TRUE(std::isfinite(pixels.get()[i])) << filename << " channel " << i;
			hasLight = hasLight || pixels.get()[i] > 0;
		}
		EXPECT_TRUE(hasLight) << filename;
	}
};

TEST_F(MainExampleRenders, PathTracing) {
	const auto output = Output("path-tracing.hdr");
	ASSERT_TRUE(Examples::PathTracing(RenderOptions(output, 32, 24, 4)));
	ExpectImage(output);
}

TEST_F(MainExampleRenders, Sppm) {
	const auto output = Output("sppm.hdr");
	auto options = RenderOptions(output, 32, 24, 4);
	options.iterations = 2;
	options.photonsPerStage = 4096;
	ASSERT_TRUE(Examples::Sppm(options));
	ExpectImage(output);
}

TEST_F(MainExampleRenders, LightTracing) {
	const auto output = Output("light-tracing.hdr");
	ASSERT_TRUE(Examples::LightTracing(RenderOptions(output, 32, 24, 8)));
	ExpectImage(output);
}

TEST_F(MainExampleRenders, VolumePathTracing) {
	const auto output = Output("volume-path-tracing.hdr");
	ASSERT_TRUE(Examples::VolumePathTracing(RenderOptions(output, 32, 24, 4)));
	ExpectImage(output);
}

TEST_F(MainExampleRenders, ImageWriteErrorsPropagate) {
	EXPECT_FALSE(Examples::LightTracing(RenderOptions(Output("nonexistent-main-example-subdirectory\\failed.png"), 8, 8, 1)));
}

TEST(MainExamples, BidirectionalExampleRejectsVolumeAndUnknownScenes) {
	EXPECT_FALSE(Examples::BidirectionalPathTracing("fog", RenderOptions("unused.png", 16, 16, 4)));
	EXPECT_FALSE(Examples::BidirectionalPathTracing("missing", RenderOptions("unused.png", 16, 16, 4)));
}

TEST_F(MainExampleRenders, BidirectionalAllSurfaceScenes) {
	int count = 0;
	for (const auto& scene : GetSceneRegistry()) {
		if (scene.usesVolume || scene.preparation) continue;
		SCOPED_TRACE(scene.name);
		for (auto accelerator : { SceneAccelerator::Bvh, SceneAccelerator::Kd }) {
			const std::string filename = outputDirectory + "\\bdpt-" + scene.name +
				(accelerator == SceneAccelerator::Bvh ? "-bvh.hdr" : "-kd.hdr");
			auto options = RenderOptions(filename, 16, 16, 4);
			options.maxDepth = 4;
			options.accelerator = accelerator;
			ASSERT_TRUE(Examples::BidirectionalPathTracing(scene.name, options));
			int width = 0, height = 0, channels = 0;
			std::unique_ptr<float, decltype(&stbi_image_free)> pixels(
				stbi_loadf(filename.c_str(), &width, &height, &channels, 3), stbi_image_free);
			ASSERT_NE(pixels, nullptr);
			ASSERT_EQ(width, 16);
			ASSERT_EQ(height, 16);
			bool nonblack = false;
			for (int i = 0; i < width * height * 3; ++i) {
				ASSERT_TRUE(std::isfinite(pixels.get()[i]));
				nonblack = nonblack || pixels.get()[i] > 0;
			}
			EXPECT_TRUE(nonblack);
		}
		++count;
	}
	EXPECT_EQ(count, std::count_if(GetSceneRegistry().begin(), GetSceneRegistry().end(),
		[](const SceneDescription& scene) { return !scene.usesVolume && !scene.preparation; }));
}

TEST(MainExamples, NamedSelectionListsEveryFeatureAndRejectsBadRequests) {
	const auto names = Examples::Names();
	EXPECT_EQ(std::set<std::string>(names.begin(), names.end()).size(), names.size());
	for (const auto& scene : GetSceneRegistry())
		if (scene.file) EXPECT_NE(std::find(names.begin(), names.end(), scene.name), names.end());
	for (const auto* name : {"sppm", "light-tracing", "volume-path-tracing", "feature-scenes", "all"})
		EXPECT_NE(std::find(names.begin(), names.end(), name), names.end());
	EXPECT_FALSE(Examples::Named("missing"));
	auto invalid = RenderOptions("unused.png", 0, 8, 1);
	EXPECT_FALSE(Examples::PathTracing(invalid));
	invalid.width = 8;
	EXPECT_FALSE(Examples::FeatureScenes(invalid));
	std::string output;
	EXPECT_FALSE(Examples::OutputPath("..\\outside.png", output));
}

TEST_F(MainExampleRenders, VeachBidirHelperAndNamedExampleDefaultToBdpt) {
	auto options = RenderOptions(Output("bidir-default.hdr"), 32, 24, 4);
	options.maxDepth = 4;
	testing::internal::CaptureStdout();
	const bool helper = Examples::VeachBidir(options);
	const auto helperLog = testing::internal::GetCapturedStdout();
	ASSERT_TRUE(helper);
	EXPECT_NE(helperLog.find("veach-bidir (bdpt)"), std::string::npos);
	ExpectImage(options.output);
	options.output.clear();
	options.outputDirectory = outputDirectory;
	testing::internal::CaptureStdout();
	const bool named = Examples::Named("veach-bidir", options);
	const auto namedLog = testing::internal::GetCapturedStdout();
	ASSERT_TRUE(named);
	EXPECT_NE(namedLog.find("veach-bidir (bdpt)"), std::string::npos);
	ExpectImage(Output("veach-bidir-bdpt.png"));
}

TEST_F(MainExampleRenders, OriginalFeatureHelpersUseTheNativeRenderingPath) {
	for (auto method : {Examples::SurfaceIntegrator::Path, Examples::SurfaceIntegrator::Bdpt, Examples::SurfaceIntegrator::Sppm}) {
		const auto filename = method == Examples::SurfaceIntegrator::Path ? "feature-path.hdr" :
			method == Examples::SurfaceIntegrator::Bdpt ? "feature-bdpt.hdr" : "feature-sppm.hdr";
		auto options = RenderOptions(Output(filename), 32, 24, 4);
		options.maxDepth = 4;
		options.iterations = 2;
		options.photonsPerStage = 4096;
		ASSERT_TRUE(Examples::CornellBoxStandard(options, method));
		ExpectImage(options.output);
		ASSERT_TRUE(Examples::VeachMis(options, method));
		ExpectImage(options.output);
		ASSERT_TRUE(Examples::VeachBidir(options, method));
		ExpectImage(options.output);
	}
}

TEST_F(MainExampleRenders, EveryLocalMaterialPreviewIsDirectlyCallable) {
	if (!std::getenv("GYUNITY_MATPREVIEW_SCENES"))
		GTEST_SKIP() << "Prepare local registered Matpreview assets and set GYUNITY_MATPREVIEW_SCENES.";
	for (const auto& scene : GetSceneRegistry()) {
		if (!scene.preparation) continue;
		for (auto method : {Examples::SurfaceIntegrator::Path, Examples::SurfaceIntegrator::Bdpt, Examples::SurfaceIntegrator::Sppm}) {
			const std::string name(scene.name);
			const auto filename = name + (method == Examples::SurfaceIntegrator::Path ? "-path.hdr" :
				method == Examples::SurfaceIntegrator::Bdpt ? "-bdpt.hdr" : "-sppm.hdr");
			auto options = RenderOptions(outputDirectory + '\\' + filename, 32, 24, 4);
			options.maxDepth = 4;
			options.iterations = 2;
			options.photonsPerStage = 4096;
			ASSERT_TRUE(Examples::Matpreview(name.substr(std::string("matpreview-").size()), options, method));
			ExpectImage(options.output);
		}
	}
}

TEST_F(MainExampleRenders, ExplicitAllBatchUsesSeparateOutputNames) {
	if (!std::getenv("GYUNITY_MATPREVIEW_SCENES"))
		GTEST_SKIP() << "Prepare local registered Matpreview assets and set GYUNITY_MATPREVIEW_SCENES.";
	Examples::Options options;
	options.width = options.height = 8;
	options.spp = 2;
	options.maxDepth = 3;
	options.iterations = 1;
	options.photonsPerStage = 1024;
	options.outputDirectory = outputDirectory;
	ASSERT_TRUE(Examples::Named("all", options));
	for (const auto& scene : GetSceneRegistry()) {
		if (!scene.file) continue;
		for (const auto* method : {"path", "bdpt", "sppm"}) {
			const auto filename = outputDirectory + '\\' + scene.name + '-' + method + ".png";
			EXPECT_TRUE(std::ifstream(filename, std::ios::binary).good()) << filename;
		}
	}
	for (const auto* filename : {"CornellBoxPtEnvmap.bmp", "CornellBoxSppmTest.bmp", "CornellBoxLT.bmp",
		"MediaTestFogEquiAngular.bmp", "CornellBoxBdpt.bmp"})
		EXPECT_TRUE(std::ifstream(Output(filename), std::ios::binary).good()) << filename;
}

TEST_F(MainExampleRenders, FailedCheckedIntegratorDoesNotWriteImage) {
	class FailedIntegrator : public Integrator {
	public:
		void Render(const Scene&, const Camera&) override { ADD_FAILURE() << "Unchecked rendering was called."; }
		bool RenderChecked(const Scene&, const Camera&) override { return false; }
	} integrator;
	SceneRenderOptions options;
	options.scene = FindScene("empty");
	options.width = options.height = 4;
	options.output = Output("must-not-write-failed-render.hdr");
	EXPECT_FALSE(RenderScene(options, integrator));
	EXPECT_FALSE(std::ifstream(options.output).good());
}

TEST_F(MainExampleRenders, VcmRenderSceneSavesOnlyAfterCheckedSuccess) {
	SceneRenderOptions options;
	options.scene = FindScene("empty");
	options.width = options.height = 4;
	options.output = Output("vcm-render-scene.hdr");
	VertexConnectionMerging render(std::make_shared<RandomSampler>(17), 2, 2, 0.1f,
		real(2.0 / 3), Vcm::Mode::Combined, 1, false);
	ASSERT_TRUE(RenderScene(options, render));
	EXPECT_TRUE(std::ifstream(options.output).good());
	options.output = Output("vcm-map-must-not-save.hdr");
	std::remove(options.output.c_str());
	VertexConnectionMerging invalidMap(std::make_shared<RandomSampler>(17), 1, 1, 1e-12f,
		real(2.0 / 3), Vcm::Mode::Combined, 1);
	EXPECT_FALSE(RenderScene(options, invalidMap));
	EXPECT_EQ(invalidMap.GetStatistics().mCameraSamples, 0);
	EXPECT_FALSE(std::ifstream(options.output).good());
	options.output = Output("vcm-must-not-save.hdr");
	std::remove(options.output.c_str());
	VertexConnectionMerging invalid(std::make_shared<RandomSampler>(17), 2, 0, 0.1f);
	EXPECT_FALSE(RenderScene(options, invalid));
	EXPECT_FALSE(std::ifstream(options.output).good());
}

TEST_F(MainExampleRenders, VertexConnectionMergingRejectsVolumeUnknownSceneAndAccelerator) {
	const auto filename = Output("vcm-rejected-scene-must-not-save.hdr");
	std::remove(filename.c_str());
	auto options = VcmOptions(filename, 4, 4);
	VertexConnectionMerging::Statistics statistics;
	statistics.mSucceeded = true;
	EXPECT_FALSE(Examples::VertexConnectionMerging("fog", options));
	EXPECT_FALSE(Examples::VertexConnectionMerging("missing", options));
	options.accelerator = static_cast<SceneAccelerator>(99);
	EXPECT_FALSE(Examples::VertexConnectionMerging("empty", options, 0.1f,
		real(2.0 / 3), Vcm::Mode::Combined, 1, false, &statistics));
	EXPECT_FALSE(statistics.mSucceeded);
	EXPECT_EQ(statistics.mLightAttempts, 0);
	EXPECT_FALSE(std::ifstream(filename).good());
}

TEST_F(MainExampleRenders, VertexConnectionMergingLegacySurfaceScenesAndModes) {
	int count = 0;
	for (const auto& scene : GetSceneRegistry()) {
		if (scene.usesVolume || scene.file) continue;
		for (auto mode : { Vcm::Mode::Combined, Vcm::Mode::ConnectionOnly, Vcm::Mode::MergingOnly }) {
			SCOPED_TRACE(std::string(scene.name) + "-" + std::to_string(int(mode)));
			const auto filename = outputDirectory + "\\vcm-example-" + scene.name + "-" + std::to_string(int(mode)) + ".hdr";
			VertexConnectionMerging::Statistics statistics;
			ASSERT_TRUE(Examples::VertexConnectionMerging(scene.name, VcmOptions(filename), 0.15f,
				real(2.0 / 3), mode, 1, false, &statistics));
			EXPECT_TRUE(statistics.mSucceeded);
			EXPECT_EQ(statistics.mCameraSamples, 32 * 24 * 4);
			ExpectImage(filename);
		}
		++count;
	}
	EXPECT_EQ(count, 12);
}

TEST_F(MainExampleRenders, VertexConnectionMergingKdAndExplicitConcurrency) {
	for (int concurrency : { 1, 2 }) {
		const auto filename = Output(("vcm-example-kd-" + std::to_string(concurrency) + ".hdr").c_str());
		auto options = VcmOptions(filename, 32, 24, 2);
		options.accelerator = SceneAccelerator::Kd;
		VertexConnectionMerging::Statistics statistics;
		ASSERT_TRUE(Examples::VertexConnectionMerging("mesh", options, 0.15f,
			0.7f, Vcm::Mode::Combined, concurrency, false, &statistics));
		EXPECT_TRUE(statistics.mSucceeded);
		ExpectImage(filename);
	}
}

TEST_F(MainExampleRenders, VertexConnectionMergingSupportsNativeSceneAndShadingNormals) {
	const auto filename = Output("vcm-native-cornell.hdr");
	ASSERT_TRUE(Examples::VertexConnectionMerging("CornellBoxStandard", VcmOptions(filename, 32, 24, 2),
		0.15f, real(2.0 / 3), Vcm::Mode::Combined, 1));
	ExpectImage(filename);
	for (auto mode : { Vcm::Mode::Combined, Vcm::Mode::ConnectionOnly, Vcm::Mode::MergingOnly }) {
		const auto shading = Output(("vcm-shading-" + std::to_string(int(mode)) + ".hdr").c_str());
		std::remove(shading.c_str());
		VertexConnectionMerging::Statistics statistics;
		ASSERT_TRUE(Examples::VertexConnectionMerging("lucy-bunny", VcmOptions(shading, 32, 24, 1), 0.1f,
			real(2.0 / 3), mode, 1, true, &statistics));
		EXPECT_TRUE(statistics.mSucceeded);
		EXPECT_EQ(statistics.mLightAttempts, 32 * 24);
		EXPECT_EQ(statistics.mCameraSamples, 32 * 24);
		ExpectImage(shading);
	}
}

TEST_F(MainExampleRenders, VertexConnectionMergingRejectsParametersWithoutSaving) {
	const auto filename = Output("vcm-example-must-not-save.hdr");
	std::remove(filename.c_str());
	const real nan = std::numeric_limits<real>::quiet_NaN(), inf = std::numeric_limits<real>::infinity();
	for (int iterations : { 0, -1 })
		EXPECT_FALSE(Examples::VertexConnectionMerging("empty", VcmOptions(filename, 4, 4, iterations)));
	for (int depth : { -1, 410 })
		EXPECT_FALSE(Examples::VertexConnectionMerging("empty", VcmOptions(filename, 4, 4, 1, depth)));
	const auto valid = VcmOptions(filename, 4, 4, 1, 2);
	for (real radius : { 0.f, -1.f, nan, inf, 1e-12f })
		EXPECT_FALSE(Examples::VertexConnectionMerging("empty", valid, radius));
	for (real alpha : { 0.f, 1.f, nan, inf })
		EXPECT_FALSE(Examples::VertexConnectionMerging("empty", valid, 0.1f, alpha));
	EXPECT_FALSE(Examples::VertexConnectionMerging("empty", valid, 0.1f,
		real(2.0 / 3), static_cast<Vcm::Mode>(99)));
	for (int concurrency : { 0, -2 })
		EXPECT_FALSE(Examples::VertexConnectionMerging("empty", valid, 0.1f,
			real(2.0 / 3), Vcm::Mode::Combined, concurrency));
	EXPECT_FALSE(Examples::VertexConnectionMerging("empty", VcmOptions(filename, 0, 4, 1, 2)));
	EXPECT_FALSE(Examples::VertexConnectionMerging("empty", VcmOptions(filename, 4, -1, 1, 2)));
	EXPECT_FALSE(Examples::VertexConnectionMerging("empty", VcmOptions(filename, INT_MAX, INT_MAX, 1, 2)));
	auto invalid = valid;
	invalid.spp = 1;
	EXPECT_FALSE(Examples::VertexConnectionMerging("empty", invalid));
	invalid = valid;
	invalid.photonsPerStage = 1;
	EXPECT_FALSE(Examples::VertexConnectionMerging("empty", invalid));
	for (int frequency : { 0, -1, 2 }) {
		invalid = valid;
		invalid.stageFrequency = frequency;
		EXPECT_FALSE(Examples::VertexConnectionMerging("empty", invalid));
	}
	EXPECT_FALSE(std::ifstream(filename).good());
	EXPECT_FALSE(Examples::VertexConnectionMerging("empty",
		VcmOptions(Output("nonexistent-vcm-example\\failed.hdr"), 4, 4, 1, 2)));
	const auto direct = Output("vcm-example-depth-zero.hdr");
	ASSERT_TRUE(Examples::VertexConnectionMerging("empty", VcmOptions(direct, 4, 4, 1, 0)));
	EXPECT_TRUE(std::ifstream(direct).good());
}

TEST_F(MainExampleRenders, VertexConnectionMergingStageFrequencySplitsEachFilmPass) {
	// 32 x 24 = 768 pixels. Each iteration traces ceil(768 / f) light and camera paths, and f
	// iterations cover every pixel; f = 5 rounds up to 154 paths.
	for (const auto& [frequency, paths] : std::vector<std::pair<int, int64>>{ { 1, 768 }, { 3, 256 }, { 5, 154 } }) {
		const auto name = "vcm-stage-frequency-" + std::to_string(frequency) + ".hdr";
		auto options = VcmOptions(Output(name.c_str()), 32, 24, frequency, 3);
		options.stageFrequency = frequency;
		VertexConnectionMerging::Statistics statistics;
		ASSERT_TRUE(Examples::VertexConnectionMerging("CornellBoxStandard", options, 0.1f,
			real(2.0 / 3), Vcm::Mode::Combined, ThreadedTaskManager::DefaultConcurrency(), false, &statistics));
		EXPECT_TRUE(statistics.mSucceeded);
		EXPECT_EQ(statistics.mIterations, frequency);
		EXPECT_EQ(statistics.mCameraSamples, paths * frequency);
		EXPECT_EQ(statistics.mLightAttempts, paths * frequency);
		ExpectImage(options.output);
	}
	const auto names = Examples::Names();
	EXPECT_NE(std::find(names.begin(), names.end(), "veach-bidir-vcm"), names.end());
	auto options = VcmOptions(Output("veach-vcm-named.hdr"), 32, 24, 2, 1);
	options.stageFrequency = 2;
	ASSERT_TRUE(Examples::Named("veach-bidir-vcm", options));
	ExpectImage(options.output);
}

}
