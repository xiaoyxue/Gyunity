#include "stb/stb_image.h"
#include "gtest/gtest.h"
#include <cmath>
#include <cstdlib>
#include "integrator/VCM.h"

// Exercise the real example functions without replacing the test runner's entry point.
#define main GyunityAppMain
#include "../../GyunityApp/main.cpp"
#undef main

namespace {

TEST(MainExamples, ExplicitCommandsStillUseTheSceneCli) {
	char app[] = "GyunityApp";
	char list[] = "--list-scenes";
	char help[] = "--help";
	char invalid[] = "--invalid";
	char* argv[] = { app, list };
	std::string expected;
	for (const auto& scene : GetSceneRegistry()) expected += std::string(scene.name) + '\n';

	testing::internal::CaptureStdout();
	const int listResult = GyunityAppMain(2, argv);
	const auto listed = testing::internal::GetCapturedStdout();
	EXPECT_EQ(listResult, 0);
	EXPECT_EQ(listed, expected);

	argv[1] = help;
	testing::internal::CaptureStdout();
	const int helpResult = GyunityAppMain(2, argv);
	const auto usage = testing::internal::GetCapturedStdout();
	EXPECT_EQ(helpResult, 0);
	EXPECT_NE(usage.find("builder-based PathTracing example"), std::string::npos);
	EXPECT_NE(usage.find("Test\\CornellBoxPtEnvmap.bmp in the main checkout"), std::string::npos);
	EXPECT_NE(usage.find("SPPM, LightTracing and VolPathTracing"), std::string::npos);

	argv[1] = invalid;
	testing::internal::CaptureStderr();
	const int invalidResult = GyunityAppMain(2, argv);
	const auto error = testing::internal::GetCapturedStderr();
	EXPECT_EQ(invalidResult, 1);
	EXPECT_NE(error.find("Unknown option"), std::string::npos);
}

TEST(MainExamples, DefaultOutputUsesTheMainCheckoutTestDirectory) {
	for (const auto* filename : { "CornellBoxPtEnvmap.bmp", "CornellBoxSppmTest.bmp",
		"CornellBoxLT.bmp", "MediaTestFogEquiAngular.bmp", "CornellBoxBdpt.bmp" }) {
		const auto output = ExampleOutputPath(filename);
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
	EXPECT_THROW(RenderScene(options, integrator), std::invalid_argument);
	options.scene = &FindScene("empty");
	EXPECT_THROW(RenderScene(options, integrator), std::invalid_argument);
	options.output = "unused.png";
	options.width = 0;
	EXPECT_THROW(RenderScene(options, integrator), std::invalid_argument);
	options.width = -1;
	EXPECT_THROW(RenderScene(options, integrator), std::invalid_argument);
	options.width = 2147483647;
	EXPECT_THROW(RenderScene(options, integrator), std::invalid_argument);
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
	ASSERT_NO_THROW(TestPathTracing(output, 32, 24, 4));
	ExpectImage(output);
}

TEST_F(MainExampleRenders, Sppm) {
	const auto output = Output("sppm.hdr");
	ASSERT_NO_THROW(TestSPPM(output, 32, 24, 2, 4096));
	ExpectImage(output);
}

TEST_F(MainExampleRenders, LightTracing) {
	const auto output = Output("light-tracing.hdr");
	ASSERT_NO_THROW(TestLightTracing(output, 32, 24, 8));
	ExpectImage(output);
}

TEST_F(MainExampleRenders, VolumePathTracing) {
	const auto output = Output("volume-path-tracing.hdr");
	ASSERT_NO_THROW(TestVolPathTracing(output, 32, 24, 4));
	ExpectImage(output);
}

TEST_F(MainExampleRenders, ImageWriteErrorsPropagate) {
	EXPECT_THROW(TestLightTracing(Output("nonexistent-main-example-subdirectory\\failed.png"), 8, 8, 1),
		std::runtime_error);
}

TEST(MainExamples, BidirectionalExampleRejectsVolumeAndUnknownScenes) {
	EXPECT_FALSE(TestBidirectionalPathTracing("fog", "unused.png", 16, 16, 4));
	EXPECT_FALSE(TestBidirectionalPathTracing("missing", "unused.png", 16, 16, 4));
}

TEST_F(MainExampleRenders, BidirectionalAllSurfaceScenes) {
	int count = 0;
	for (const auto& scene : GetSceneRegistry()) {
		if (scene.usesVolume) continue;
		SCOPED_TRACE(scene.name);
		for (auto accelerator : { SceneAccelerator::Bvh, SceneAccelerator::Kd }) {
			const std::string filename = outputDirectory + "\\bdpt-" + scene.name +
				(accelerator == SceneAccelerator::Bvh ? "-bvh.hdr" : "-kd.hdr");
			ASSERT_TRUE(TestBidirectionalPathTracing(scene.name, filename, 16, 16, 4, 4, accelerator));
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
	EXPECT_EQ(count, 10);
}

TEST_F(MainExampleRenders, VcmRenderSceneSavesOnlyAfterCheckedSuccess) {
	SceneRenderOptions options;
	options.scene = &FindScene("empty");
	options.width = options.height = 4;
	options.output = Output("vcm-render-scene.hdr");
	VertexConnectionMerging render(std::make_shared<RandomSampler>(17), 2, 2, 0.1f,
		real(2.0 / 3), Vcm::Mode::Combined, 1);
	ASSERT_TRUE(RenderScene(options, render));
	EXPECT_TRUE(std::ifstream(options.output).good());
	options.output = Output("vcm-map-must-not-save.hdr");
	std::remove(options.output.c_str());
	VertexConnectionMerging invalidMap(std::make_shared<RandomSampler>(17), 1, 1, 1e-12f,
		real(2.0 / 3), Vcm::Mode::Combined, 1);
	EXPECT_FALSE(RenderScene(options, invalidMap));
	EXPECT_EQ(invalidMap.GetStatistics().mLastMapPhase.mStatus, ThreadedTaskManager::Status::Failed);
	EXPECT_FALSE(std::ifstream(options.output).good());
	options.output = Output("vcm-must-not-save.hdr");
	std::remove(options.output.c_str());
	VertexConnectionMerging invalid(std::make_shared<RandomSampler>(17), 2, 0, 0.1f);
	EXPECT_FALSE(RenderScene(options, invalid));
	EXPECT_FALSE(std::ifstream(options.output).good());
}

TEST_F(MainExampleRenders, FailedCheckedIntegratorDoesNotWriteImage) {
	class FailedIntegrator : public Integrator {
	public:
		void Render(const Scene&, const Camera&) override { ADD_FAILURE() << "Unchecked rendering was called."; }
		bool RenderChecked(const Scene&, const Camera&) override { return false; }
	} integrator;
	SceneRenderOptions options;
	options.scene = &FindScene("empty");
	options.width = options.height = 4;
	options.output = Output("must-not-write-failed-render.hdr");
	EXPECT_FALSE(RenderScene(options, integrator));
	EXPECT_FALSE(std::ifstream(options.output).good());
}

}
