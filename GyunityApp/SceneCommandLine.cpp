#include "SceneCommandLine.h"
#include "accelerator/BVHAccel.h"
#include "accelerator/KdTreeAccel.h"
#include "integrator/PT.h"
#include "integrator/VPT.h"
#include "integrator/BDPT.h"
#include "sampler/SamplerEnum.h"
#include <limits>
#include <set>

namespace {
bool Error(const std::string& message) {
	std::cerr << "GyunityApp: " << message << '\n';
	return false;
}

bool ParseInteger(const std::string& option, const std::string& value, int minimum, int& output) {
	int result = 0;
	if (value.empty()) return Error("Missing integer for " + option);
	for (char digit : value) {
		if (digit < '0' || digit > '9' || result > (std::numeric_limits<int>::max() - (digit - '0')) / 10)
			return Error("Invalid integer for " + option + ": " + value);
		result = result * 10 + digit - '0';
	}
	if (result < minimum) return Error(option + " must be at least " + std::to_string(minimum));
	output = result;
	return true;
}

bool RenderSceneCommand(const SceneRenderOptions& options) {
	auto sampler = std::make_shared<RandomSampler>(options.nativeScene ? options.nativeScene->seed : 123);
	auto samplerEnum = std::make_shared<SamplerEnum>();
	if (options.integrator == "bdpt") {
		BidirectionalPathTracing integrator(sampler, options.maxDepth, options.spp);
		return RenderScene(options, integrator);
	}
	if (options.scene && options.scene->usesVolume) {
		VolPathTracing integrator(options.spp, options.maxDepth, sampler, samplerEnum, false);
		return RenderScene(options, integrator);
	}
	PathTracing integrator(options.spp, options.maxDepth, sampler, samplerEnum);
	return RenderScene(options, integrator);
}
}

bool RenderScene(const SceneRenderOptions& options, Integrator& integrator) {
	if (!options.scene && !options.nativeScene) return Error("A registered or native scene is required.");
	if (options.width <= 0 || options.height <= 0 ||
		static_cast<int64>(options.width) * options.height > std::numeric_limits<int>::max() / 3)
		return Error("Image dimensions exceed the supported pixel count or are not positive.");
	if (options.output.empty()) return Error("An output filename is required.");
	for (const auto* resource : options.scene ? options.scene->resources : std::array<const char*, 3>{}) {
		if (resource && !std::ifstream(SceneBuilder::Resource(resource), std::ios::binary).good())
			return Error("Cannot read resource '" + SceneBuilder::Resource(resource) + "'. Run from GyunityApp.");
	}
	auto native = options.nativeScene;
	if (!native && options.scene && options.scene->file) {
		native = LoadNativeScene(SceneBuilder::Resource(options.scene->file));
		if (!native) return false;
	}
	Filter* filter;
	if (native && native->filter == "tent") filter = new TentFilter();
	else if (native && native->filter == "gaussian") filter = new GaussianFilter();
	else filter = new BoxFilter();
	auto film = std::make_shared<Film>(options.width, options.height, filter);
	auto scene = native ? native->scene : std::make_shared<Scene>();
	auto camera = native ? native->CreateCamera(film) : options.scene->camera.Create(film);
	if (!native) {
		SceneBuilder builder(scene);
		options.scene->build(builder);
	}
	for (const auto& light : scene->GetLights()) {
		const auto* environment = dynamic_cast<const EnvironmentLight*>(light.get());
		if (environment && !environment->IsValid()) return Error("Cannot render an invalid environment map.");
	}
	if (options.accelerator == SceneAccelerator::Kd)
		scene->SetAccelerator(std::make_shared<KdTreeAccel>(scene->GetPrimitives()));
	else
		scene->SetAccelerator(std::make_shared<BVHAccel>(scene->GetPrimitives()));
	scene->Initialize();
	film->SetFileName(options.output);
	if (!integrator.RenderChecked(*scene, *camera)) return false;
	if (!film->SaveImage()) return Error("Cannot write image '" + options.output + "'. Check its directory and permissions.");
	return true;
}

bool ParseSceneCommandLine(int argc, const char* const* argv, SceneRenderOptions& output) {
	SceneRenderOptions options;
	if (argc <= 1) { output = options; return true; }
	if (argc == 2 && std::string(argv[1]) == "--help") {
		options.command = SceneCommand::Help;
		output = options;
		return true;
	}
	if (argc == 2 && std::string(argv[1]) == "--list-scenes") {
		options.command = SceneCommand::List;
		output = options;
		return true;
	}
	options.command = SceneCommand::Render;
	std::set<std::string> seen;
	std::string nativeFile;
	for (int i = 1; i < argc; ++i) {
		const std::string option = argv[i];
		if (option != "--scene" && option != "--scene-file" && option != "--integrator" &&
			option != "--accel" && option != "--width" && option != "--height" &&
			option != "--spp" && option != "--max-depth" && option != "--out")
			return Error("Unknown option '" + option + "'. Use --help.");
		if (!seen.insert(option).second) return Error("Duplicate option: " + option);
		if (++i == argc || std::string(argv[i]).empty() || std::string(argv[i]).find("--") == 0)
			return Error("Missing value for " + option);
		const std::string value = argv[i];
		if (option == "--scene") {
			options.scene = FindScene(value);
			if (!options.scene) return false;
		}
		else if (option == "--scene-file") nativeFile = value;
		else if (option == "--integrator") {
			if (value != "path" && value != "bdpt") return Error("Use --integrator path|bdpt.");
			options.integrator = value;
		}
		else if (option == "--out") options.output = value;
		else if (option == "--accel") {
			if (value == "bvh") options.accelerator = SceneAccelerator::Bvh;
			else if (value == "kd") options.accelerator = SceneAccelerator::Kd;
			else return Error("Unknown accelerator '" + value + "'. Use bvh or kd.");
		}
		else if (option == "--width" && !ParseInteger(option, value, 1, options.width)) return false;
		else if (option == "--height" && !ParseInteger(option, value, 1, options.height)) return false;
		else if (option == "--spp" && !ParseInteger(option, value, 1, options.spp)) return false;
		else if (option == "--max-depth" && !ParseInteger(option, value, 1, options.maxDepth)) return false;
	}
	if (options.scene && !nativeFile.empty()) return Error("--scene and --scene-file are mutually exclusive.");
	if (options.scene && options.scene->file) nativeFile = SceneBuilder::Resource(options.scene->file);
	if (!nativeFile.empty()) {
		options.nativeScene = LoadNativeScene(nativeFile);
		if (!options.nativeScene) return false;
		const auto& native = *options.nativeScene;
		if (options.scene && native.name != options.scene->name) return Error("Native scene name does not match the registry.");
		if (!seen.count("--width")) options.width = native.width;
		if (!seen.count("--height")) options.height = native.height;
		if (!seen.count("--spp")) options.spp = native.spp;
		if (!seen.count("--max-depth")) options.maxDepth = native.maxDepth;
		if (!seen.count("--integrator")) options.integrator = native.integrator;
		if (!seen.count("--accel")) options.accelerator = native.accelerator == "bvh" ? SceneAccelerator::Bvh : SceneAccelerator::Kd;
	}
	if (!options.scene && !options.nativeScene) return Error("--scene or --scene-file is required.");
	if (options.scene && options.scene->usesVolume && !options.integrator.empty())
		return Error("Surface integrator overrides are not supported for volume scenes.");
	if (options.output.empty()) return Error("--out is required.");
	if (options.output.size() < 4) return Error("--out must end in .png, .bmp, .hdr or .exr.");
	const auto suffix = options.output.substr(options.output.size() - 4);
	if (suffix != ".png" && suffix != ".bmp" && suffix != ".hdr" && suffix != ".exr")
		return Error("--out must end in .png, .bmp, .hdr or .exr.");
	if (static_cast<int64>(options.width) * options.height > std::numeric_limits<int>::max() / 3)
		return Error("Image dimensions exceed the supported pixel count.");
	output = std::move(options);
	return true;
}

int RunSceneCommandLine(int argc, const char* const* argv) {
	SceneRenderOptions options;
	if (!ParseSceneCommandLine(argc, argv, options)) return 1;
	if (options.command == SceneCommand::Help) {
		std::cout << "Usage: GyunityApp --scene NAME --out FILE [options]\n"
			"  --scene-file PATH      Load native Gyunity JSON instead of a named scene\n"
			"  --integrator path|bdpt Override the surface integrator\n"
			"  --list-scenes          List registered scene names\n"
			"  --width N --height N    Resolution (default 512 x 512)\n"
			"  --spp N                Samples per pixel (default 64)\n"
			"  --max-depth N          Path depth (default 8)\n"
			"  --accel bvh|kd         Accelerator (default bvh)\n"
			"  --out FILE             Required .png, .bmp, .hdr or .exr output\n"
			"Native scenes supply defaults; explicit flags override them.\n"
			"Fog uses volume path tracing. Run from GyunityApp for named scene assets.\n"
			"With no arguments, the builder-based PathTracing example runs the env scene\n"
			"at 1024 x 1024, 100 spp, depth 10, BVH;\n"
			"output: Test\\CornellBoxPtEnvmap.bmp in the main checkout (also from worktrees).\n"
			"main.cpp also contains SPPM, LightTracing and VolPathTracing examples.\n";
		return 0;
	}
	if (options.command == SceneCommand::List) {
		for (const auto& scene : GetSceneRegistry()) std::cout << scene.name << '\n';
		return 0;
	}
	if (options.command == SceneCommand::Render) return RenderSceneCommand(options) ? 0 : 1;
	Error("No scene command supplied.");
	return 1;
}
