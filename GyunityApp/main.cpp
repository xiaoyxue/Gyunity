#include "SceneCommandLine.h"
#include "integrator/LT.h"
#include "integrator/BDPT.h"
#include "integrator/PT.h"
#include "integrator/SPPM.h"
#include "integrator/VPT.h"
#include "integrator/VCM.h"
#include "sampler/Halton.h"
#include "sampler/HaltonEnum.h"
#include "sampler/SamplerEnum.h"
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <direct.h>
#include <sys/stat.h>

namespace {

std::string ExampleOutputPath(const char* filename) {
	char* executable = nullptr;
	if (_get_pgmptr(&executable) != 0 || !executable) {
		throw std::runtime_error("Cannot locate the executable for the example output directory.");
	}
	std::string directory(executable);
	for (auto separator = directory.find_last_of("\\/"); separator != std::string::npos;
		separator = directory.find_last_of("\\/")) {
		directory.erase(separator);
		const auto gitDirectory = directory + "\\.git";
		struct _stat info;
		const int result = _stat(gitDirectory.c_str(), &info);
		if (result != 0 && errno != ENOENT && errno != ENOTDIR) {
			throw std::runtime_error("Cannot inspect '" + gitDirectory + "': " + std::strerror(errno));
		}
		// Nested worktrees have .git files; the main checkout has the .git directory.
		if (result == 0 && (info.st_mode & _S_IFDIR)) {
			const auto outputDirectory = directory + "\\Test";
			if ((_mkdir(outputDirectory.c_str()) != 0 && errno != EEXIST) ||
				_stat(outputDirectory.c_str(), &info) != 0 || !(info.st_mode & _S_IFDIR)) {
				throw std::runtime_error("Cannot create example output directory '" + outputDirectory + "'.");
			}
			return outputDirectory + '\\' + filename;
		}
	}
	throw std::runtime_error("Cannot find the main checkout for example output. Pass an explicit output path.");
}

SceneRenderOptions ExampleOptions(const char* scene, const std::string& output, int width, int height) {
	SceneRenderOptions options;
	options.scene = &FindScene(scene);
	options.width = width;
	options.height = height;
	options.output = output;
	return options;
}

void TestPathTracing(const std::string& output = ExampleOutputPath("CornellBoxPtEnvmap.bmp"),
	int width = 1024, int height = 1024, int spp = 100) {
	const auto options = ExampleOptions("env", output, width, height);
	auto sampler = std::make_shared<RandomSampler>(123);
	auto samplerEnum = std::make_shared<SamplerEnum>();
	PathTracing integrator(spp, 10, sampler, samplerEnum);
	RenderScene(options, integrator);
}

void TestSPPM(const std::string& output = ExampleOutputPath("CornellBoxSppmTest.bmp"),
	int width = 1024, int height = 1024, int iterations = 256, int photonsPerStage = 300000) {
	const auto options = ExampleOptions("mesh", output, width, height);
	auto sampler = std::make_shared<HaltonSampler>(width, height);
	auto samplerEnum = std::make_shared<HaltonEnum>(static_cast<unsigned>(width), static_cast<unsigned>(height));
	const real alpha = 0.66666667;
	SPPM integrator(iterations, photonsPerStage, 20, 0.05, alpha, false, sampler, samplerEnum, true);
	RenderScene(options, integrator);
}

void TestLightTracing(const std::string& output = ExampleOutputPath("CornellBoxLT.bmp"),
	int width = 1024, int height = 1024, int spp = 512) {
	const auto options = ExampleOptions("mesh", output, width, height);
	auto sampler = std::make_shared<RandomSampler>(123);
	LightTracing integrator(sampler, 15, spp);
	RenderScene(options, integrator);
}

void TestVolPathTracing(const std::string& output = ExampleOutputPath("MediaTestFogEquiAngular.bmp"),
	int width = 1024, int height = 1024, int spp = 2000) {
	const auto options = ExampleOptions("fog", output, width, height);
	auto sampler = std::make_shared<RandomSampler>(123);
	auto samplerEnum = std::make_shared<SamplerEnum>();
	VolPathTracing integrator(spp, 8, sampler, samplerEnum, false);
	RenderScene(options, integrator);
}

bool TestBidirectionalPathTracing(const std::string& sceneName = "mesh",
	const std::string& output = ExampleOutputPath("CornellBoxBdpt.bmp"),
	int width = 1024, int height = 1024, int spp = 256, int maxDepth = 8,
	SceneAccelerator accelerator = SceneAccelerator::Bvh) {
	const SceneDescription* selected = nullptr;
	for (const auto& scene : GetSceneRegistry()) {
		if (sceneName == scene.name) selected = &scene;
	}
	if (!selected || selected->usesVolume) {
		GetDefaultLogger()->Error("BDPT requires a registered non-volume scene.", false);
		return false;
	}
	SceneRenderOptions options;
	options.scene = selected;
	options.width = width;
	options.height = height;
	options.output = output;
	options.accelerator = accelerator;
	auto sampler = std::make_shared<RandomSampler>(123);
	BidirectionalPathTracing integrator(sampler, maxDepth, spp);
	return RenderScene(options, integrator);
}

bool TestVertexConnectionMerging(const std::string& sceneName = "mesh",
	const std::string& output = ExampleOutputPath("CornellBoxVcm.hdr"),
	int width = 128, int height = 128, int iterations = 16, int maxDepth = 8,
	real initialRadius = 0.05f, real alpha = real(2.0 / 3.0), Vcm::Mode mode = Vcm::Mode::Combined,
	SceneAccelerator accelerator = SceneAccelerator::Bvh,
	int concurrency = ThreadedTaskManager::DefaultConcurrency(),
	VertexConnectionMerging::Statistics* statistics = nullptr) {
	if (statistics) *statistics = {};
	const SceneDescription* selected = nullptr;
	for (const auto& scene : GetSceneRegistry()) {
		if (sceneName == scene.name) selected = &scene;
	}
	if (!selected || selected->usesVolume) {
		GetDefaultLogger()->Error("VCM requires a registered non-volume scene.", false);
		return false;
	}
	if (accelerator != SceneAccelerator::Bvh && accelerator != SceneAccelerator::Kd) {
		GetDefaultLogger()->Error("VCM example requires BVH or KD acceleration.", false);
		return false;
	}
	SceneRenderOptions options;
	options.scene = selected;
	options.width = width;
	options.height = height;
	options.output = output;
	options.accelerator = accelerator;
	auto sampler = std::make_shared<RandomSampler>(123);
	VertexConnectionMerging integrator(sampler, maxDepth, iterations, initialRadius, alpha, mode, concurrency);
	const bool success = RenderScene(options, integrator);
	if (statistics) *statistics = integrator.GetStatistics();
	return success;
}

}

int main(int argc, char* argv[]) {
	if (argc > 1) return RunSceneCommandLine(argc, argv);

	try {
		const auto begin = std::chrono::steady_clock::now();
		TestPathTracing();
		const std::chrono::duration<double, std::ratio<60>> elapsed = std::chrono::steady_clock::now() - begin;
		std::cout << "Cost time: " << elapsed.count() << " min\n";
		return 0;
	}
	catch (const std::exception& error) {
		std::cerr << "GyunityApp: " << error.what() << '\n';
		return 1;
	}
}
