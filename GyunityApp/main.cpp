#include "SceneCommandLine.h"
#include "integrator/LT.h"
#include "integrator/PT.h"
#include "integrator/SPPM.h"
#include "integrator/VPT.h"
#include "sampler/Halton.h"
#include "sampler/HaltonEnum.h"
#include "sampler/SamplerEnum.h"
#include <chrono>

namespace {

SceneRenderOptions ExampleOptions(const char* scene, const std::string& output, int width, int height) {
	SceneRenderOptions options;
	options.scene = &FindScene(scene);
	options.width = width;
	options.height = height;
	options.output = output;
	return options;
}

void TestPathTracing(const std::string& output = "CornellBoxPtEnvmap.bmp",
	int width = 1024, int height = 1024, int spp = 100) {
	const auto options = ExampleOptions("env", output, width, height);
	auto sampler = std::make_shared<RandomSampler>(123);
	auto samplerEnum = std::make_shared<SamplerEnum>();
	PathTracing integrator(spp, 10, sampler, samplerEnum);
	RenderScene(options, integrator);
}

void TestSPPM(const std::string& output = "CornellBoxSppmTest3.bmp",
	int width = 1024, int height = 1024, int iterations = 256, int photonsPerStage = 300000) {
	const auto options = ExampleOptions("mesh", output, width, height);
	auto sampler = std::make_shared<HaltonSampler>(width, height);
	auto samplerEnum = std::make_shared<HaltonEnum>(static_cast<unsigned>(width), static_cast<unsigned>(height));
	const real alpha = 0.66666667;
	SPPM integrator(iterations, photonsPerStage, 20, 0.05, alpha, false, sampler, samplerEnum, true);
	RenderScene(options, integrator);
}

void TestLightTracing(const std::string& output = "CornellBoxLT16.bmp",
	int width = 1024, int height = 1024, int spp = 512) {
	const auto options = ExampleOptions("mesh", output, width, height);
	auto sampler = std::make_shared<RandomSampler>(123);
	LightTracing integrator(sampler, 15, spp);
	RenderScene(options, integrator);
}

void TestVolPathTracing(const std::string& output = "MediaTestFogEquiAngular12.bmp",
	int width = 1024, int height = 1024, int spp = 2000) {
	const auto options = ExampleOptions("fog", output, width, height);
	auto sampler = std::make_shared<RandomSampler>(123);
	auto samplerEnum = std::make_shared<SamplerEnum>();
	VolPathTracing integrator(spp, 8, sampler, samplerEnum, false);
	RenderScene(options, integrator);
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
