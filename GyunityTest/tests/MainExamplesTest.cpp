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
		"CornellBoxLT.bmp", "MediaTestFogEquiAngular.bmp", "CornellBoxBdpt.bmp" }) {
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

}
