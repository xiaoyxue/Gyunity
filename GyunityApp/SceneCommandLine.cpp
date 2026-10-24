#include "SceneCommandLine.h"
#include "accelerator/BVHAccel.h"
#include "accelerator/KdTreeAccel.h"
#include "integrator/PT.h"
#include "integrator/VPT.h"
#include "sampler/SamplerEnum.h"
#include <limits>
#include <set>

namespace {

int ParseInteger(const std::string& option, const std::string& value, int minimum) {
	int result = 0;
	if (value.empty()) throw std::invalid_argument("Missing integer for " + option);
	for (char digit : value) {
		if (digit < '0' || digit > '9' ||
			result > (std::numeric_limits<int>::max() - (digit - '0')) / 10) {
			throw std::invalid_argument("Invalid integer for " + option + ": " + value);
		}
		result = result * 10 + digit - '0';
	}
	if (result < minimum) throw std::invalid_argument(option + " must be at least " + std::to_string(minimum));
	return result;
}

void RenderSceneCommand(const SceneRenderOptions& options) {
	auto sampler = std::make_shared<RandomSampler>(123);
	auto samplerEnum = std::make_shared<SamplerEnum>();
	if (options.scene->usesVolume) {
		VolPathTracing integrator(options.spp, options.maxDepth, sampler, samplerEnum, false);
		RenderScene(options, integrator);
	}
	else {
		PathTracing integrator(options.spp, options.maxDepth, sampler, samplerEnum);
		RenderScene(options, integrator);
	}
}

}

void RenderScene(const SceneRenderOptions& options, Integrator& integrator) {
	if (!options.scene) throw std::invalid_argument("A registered scene is required.");
	if (options.width <= 0 || options.height <= 0 ||
		static_cast<int64>(options.width) * options.height > std::numeric_limits<int>::max() / 3) {
		throw std::invalid_argument("Image dimensions exceed the supported pixel count or are not positive.");
	}
	if (options.output.empty()) throw std::invalid_argument("An output filename is required.");
	for (const auto* resource : options.scene->resources) {
		if (resource && !std::ifstream(SceneBuilder::Resource(resource), std::ios::binary).good()) {
			throw std::runtime_error("Cannot read resource '" + SceneBuilder::Resource(resource) +
				"'. Run from the GyunityApp directory.");
		}
	}

	auto film = std::make_shared<Film>(options.width, options.height, new BoxFilter());
	auto scene = std::make_shared<Scene>();
	auto camera = options.scene->camera.Create(film);

	SceneBuilder builder(scene);
	options.scene->build(builder);
	if (options.accelerator == SceneAccelerator::Kd) {
		scene->SetAccelerator(std::make_shared<KdTreeAccel>(scene->GetPrimitives()));
	}
	else {
		scene->SetAccelerator(std::make_shared<BVHAccel>(scene->GetPrimitives()));
	}
	scene->Initialize();
	film->SetFileName(options.output);
	integrator.Render(*scene, *camera);
	if (!film->SaveImage()) {
		throw std::runtime_error("Cannot write image '" + options.output + "'. Check its directory and permissions.");
	}
}

SceneRenderOptions ParseSceneCommandLine(int argc, const char* const* argv) {
	SceneRenderOptions options;
	if (argc <= 1) return options;
	if (argc == 2 && std::string(argv[1]) == "--help") {
		options.command = SceneCommand::Help;
		return options;
	}
	if (argc == 2 && std::string(argv[1]) == "--list-scenes") {
		options.command = SceneCommand::List;
		return options;
	}
	options.command = SceneCommand::Render;
	std::set<std::string> seen;
	for (int i = 1; i < argc; ++i) {
		const std::string option = argv[i];
		if (option != "--scene" && option != "--accel" && option != "--width" && option != "--height" &&
			option != "--spp" && option != "--max-depth" && option != "--out") {
			throw std::invalid_argument("Unknown option '" + option + "'. Use --help.");
		}
		if (!seen.insert(option).second) throw std::invalid_argument("Duplicate option: " + option);
		if (++i == argc || std::string(argv[i]).empty() || std::string(argv[i]).find("--") == 0) {
			throw std::invalid_argument("Missing value for " + option);
		}
		const std::string value = argv[i];
		if (option == "--scene") options.scene = &FindScene(value);
		else if (option == "--out") options.output = value;
		else if (option == "--accel") {
			if (value == "bvh") options.accelerator = SceneAccelerator::Bvh;
			else if (value == "kd") options.accelerator = SceneAccelerator::Kd;
			else throw std::invalid_argument("Unknown accelerator '" + value + "'. Use bvh or kd.");
		}
		else if (option == "--width") options.width = ParseInteger(option, value, 1);
		else if (option == "--height") options.height = ParseInteger(option, value, 1);
		else if (option == "--spp") options.spp = ParseInteger(option, value, 1);
		else if (option == "--max-depth") options.maxDepth = ParseInteger(option, value, 1);
	}
	if (!options.scene) throw std::invalid_argument("--scene is required. Use --list-scenes.");
	if (options.output.empty()) throw std::invalid_argument("--out is required.");
	if (options.output.size() < 4) throw std::invalid_argument("--out must end in .png, .bmp or .hdr.");
	const std::string suffix = options.output.substr(options.output.size() - 4);
	if (suffix != ".png" && suffix != ".bmp" && suffix != ".hdr") {
		throw std::invalid_argument("--out must end in .png, .bmp or .hdr.");
	}
	// Film and image writers use signed pixel/channel indices.
	if (static_cast<int64>(options.width) * options.height > std::numeric_limits<int>::max() / 3) {
		throw std::invalid_argument("Image dimensions exceed the supported pixel count.");
	}
	return options;
}

int RunSceneCommandLine(int argc, const char* const* argv) {
	try {
		const auto options = ParseSceneCommandLine(argc, argv);
		if (options.command == SceneCommand::Help) {
			std::cout << "Usage: GyunityApp --scene NAME --out FILE [options]\n"
				"  --list-scenes          List registered scene names\n"
				"  --width N --height N    Resolution (default 512 x 512)\n"
				"  --spp N                Samples per pixel (default 64)\n"
				"  --max-depth N          Path depth (default 8)\n"
				"  --accel bvh|kd         Accelerator (default bvh)\n"
				"  --out FILE             Required .png, .bmp or .hdr output\n"
				"Fog uses volume path tracing; other scenes use path tracing.\n"
				"Run from GyunityApp for resource paths. Output directories must exist.\n"
				"With no arguments, the builder-based PathTracing example runs the env scene\n"
				"at 1024 x 1024, 100 spp, depth 10, BVH; output: CornellBoxPtEnvmap.bmp.\n"
				"main.cpp also contains SPPM, LightTracing and VolPathTracing examples.\n";
		}
		else if (options.command == SceneCommand::List) {
			for (const auto& scene : GetSceneRegistry()) std::cout << scene.name << '\n';
		}
		else if (options.command == SceneCommand::Render) {
			RenderSceneCommand(options);
		}
		else {
			throw std::invalid_argument("No scene command supplied.");
		}
		return 0;
	}
	catch (const std::exception& error) {
		std::cerr << "GyunityApp: " << error.what() << '\n';
		return 1;
	}
}
