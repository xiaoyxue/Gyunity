#include "SceneCommandLine.h"
#include "integrator/LT.h"
#include "integrator/BDPT.h"
#include "integrator/PT.h"
#include "integrator/SPPM.h"
#include "integrator/VPT.h"
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

bool ExampleError(const std::string& message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool ExampleOutputPath(const char* filename, std::string& output) {
	char* executable = nullptr;
	if (_get_pgmptr(&executable) != 0 || !executable) {
		return ExampleError("Cannot locate the executable for the example output directory.");
	}
	std::string directory(executable);
	for (auto separator = directory.find_last_of("\\/"); separator != std::string::npos;
		separator = directory.find_last_of("\\/")) {
		directory.erase(separator);
		const auto gitDirectory = directory + "\\.git";
		struct _stat info;
		const int result = _stat(gitDirectory.c_str(), &info);
		if (result != 0 && errno != ENOENT && errno != ENOTDIR) {
			return ExampleError("Cannot inspect '" + gitDirectory + "': " + std::strerror(errno));
		}
		// Nested worktrees have .git files; the main checkout has the .git directory.
		if (result == 0 && (info.st_mode & _S_IFDIR)) {
			const auto outputDirectory = directory + "\\Test";
			if ((_mkdir(outputDirectory.c_str()) != 0 && errno != EEXIST) ||
				_stat(outputDirectory.c_str(), &info) != 0 || !(info.st_mode & _S_IFDIR)) {
				return ExampleError("Cannot create example output directory '" + outputDirectory + "'.");
			}
			output = outputDirectory + '\\' + filename;
			return true;
		}
	}
	return ExampleError("Cannot find the main checkout for example output. Pass an explicit output path.");
}

bool ExampleOptions(const char* scene, const std::string& output, int width, int height, SceneRenderOptions& options) {
	options.scene = FindScene(scene);
	if (!options.scene) return false;
	if (width <= 0 || height <= 0 || int64(width) * height > std::numeric_limits<int>::max() / 3 || output.empty())
		return ExampleError("Example rendering requires valid dimensions and a nonempty output path.");
	options.width = width;
	options.height = height;
	options.output = output;
	return true;
}

bool TestPathTracing(const std::string& output,
	int width = 1024, int height = 1024, int spp = 100) {
	SceneRenderOptions options;
	if (!ExampleOptions("env", output, width, height, options)) return false;
	auto sampler = std::make_shared<RandomSampler>(123);
	auto samplerEnum = std::make_shared<SamplerEnum>();
	PathTracing integrator(spp, 10, sampler, samplerEnum);
	return RenderScene(options, integrator);
}

bool TestSPPM(const std::string& output,
	int width = 1024, int height = 1024, int iterations = 256, int photonsPerStage = 300000) {
	SceneRenderOptions options;
	if (!ExampleOptions("mesh", output, width, height, options)) return false;
	auto sampler = std::make_shared<HaltonSampler>(width, height);
	auto samplerEnum = std::make_shared<HaltonEnum>(static_cast<unsigned>(width), static_cast<unsigned>(height));
	const real alpha = 0.66666667;
	SPPM integrator(iterations, photonsPerStage, 20, 0.05, alpha, false, sampler, samplerEnum, true);
	return RenderScene(options, integrator);
}

bool TestLightTracing(const std::string& output,
	int width = 1024, int height = 1024, int spp = 512) {
	SceneRenderOptions options;
	if (!ExampleOptions("mesh", output, width, height, options)) return false;
	auto sampler = std::make_shared<RandomSampler>(123);
	LightTracing integrator(sampler, 15, spp);
	return RenderScene(options, integrator);
}

bool TestVolPathTracing(const std::string& output,
	int width = 1024, int height = 1024, int spp = 2000) {
	SceneRenderOptions options;
	if (!ExampleOptions("fog", output, width, height, options)) return false;
	auto sampler = std::make_shared<RandomSampler>(123);
	auto samplerEnum = std::make_shared<SamplerEnum>();
	VolPathTracing integrator(spp, 8, sampler, samplerEnum, false);
	return RenderScene(options, integrator);
}

bool TestBidirectionalPathTracing(const std::string& sceneName,
	const std::string& output,
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

bool TestPathTracing() {
	std::string output;
	return ExampleOutputPath("CornellBoxPtEnvmap.bmp", output) && TestPathTracing(output);
}
bool TestSPPM() {
	std::string output;
	return ExampleOutputPath("CornellBoxSppmTest.bmp", output) && TestSPPM(output);
}
bool TestLightTracing() {
	std::string output;
	return ExampleOutputPath("CornellBoxLT.bmp", output) && TestLightTracing(output);
}
bool TestVolPathTracing() {
	std::string output;
	return ExampleOutputPath("MediaTestFogEquiAngular.bmp", output) && TestVolPathTracing(output);
}
bool TestBidirectionalPathTracing(const std::string& sceneName = "mesh") {
	std::string output;
	return ExampleOutputPath("CornellBoxBdpt.bmp", output) && TestBidirectionalPathTracing(sceneName, output);
}

}

int main(int argc, char* argv[]) {
	try {
		if (argc > 1) return RunSceneCommandLine(argc, argv);
		const auto begin = std::chrono::steady_clock::now();
		if (!TestPathTracing()) return 1;
		const std::chrono::duration<double, std::ratio<60>> elapsed = std::chrono::steady_clock::now() - begin;
		std::cout << "Cost time: " << elapsed.count() << " min\n";
		return 0;
	}
	catch (const std::exception& error) {
		std::cerr << "GyunityApp: unexpected failure: " << error.what() << '\n';
		return 1;
	}
}
