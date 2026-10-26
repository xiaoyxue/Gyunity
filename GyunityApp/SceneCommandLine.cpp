#include "SceneCommandLine.h"
#include <cmath>
#include <cstdlib>
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

bool ParsePositiveReal(const std::string& option, const std::string& value, Gyunity::real& output) {
	char* end = nullptr;
	const double result = std::strtod(value.c_str(), &end);
	if (value.empty() || end != value.c_str() + value.size() || !std::isfinite(result) || result <= 0)
		return Error(option + " requires a positive finite number: " + value);
	output = static_cast<Gyunity::real>(result);
	return true;
}

bool LowDynamicRange(const std::string& output) {
	if (output.size() < 4) return false;
	const auto suffix = output.substr(output.size() - 4);
	return suffix == ".png" || suffix == ".bmp";
}

}

bool ParseSceneCommandLine(int argc, const char* const* argv, SceneCommandLineOptions& output) {
	SceneCommandLineOptions parsed;
	auto& options = parsed.render;
	if (argc <= 1) { output = parsed; return true; }
	if (argc == 2 && std::string(argv[1]) == "--help") {
		parsed.command = SceneCommand::Help;
		output = parsed;
		return true;
	}
	if (argc == 2 && std::string(argv[1]) == "--list-scenes") {
		parsed.command = SceneCommand::List;
		output = parsed;
		return true;
	}
	parsed.command = SceneCommand::Render;
	std::set<std::string> seen;
	std::string nativeFile;
	for (int i = 1; i < argc; ++i) {
		const std::string option = argv[i];
		if (option != "--scene" && option != "--scene-file" && option != "--integrator" &&
			option != "--accel" && option != "--width" && option != "--height" &&
			option != "--spp" && option != "--max-depth" && option != "--out" &&
			option != "--iterations" && option != "--photons-per-stage" &&
			option != "--bdpt-normals" && option != "--tonemap" && option != "--max-luminance" &&
			option != "--tonemap-scale" && option != "--convert")
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
			if (value != "path" && value != "bdpt" && value != "sppm") return Error("Use --integrator path|bdpt|sppm.");
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
		else if (option == "--iterations" && !ParseInteger(option, value, 1, options.sppm.iterations)) return false;
		else if (option == "--photons-per-stage" && !ParseInteger(option, value, 1, options.sppm.photonsPerStage)) return false;
		else if (option == "--bdpt-normals") {
			if (value == "shading") options.bdptShadingNormals = true;
			else if (value == "geometric") options.bdptShadingNormals = false;
			else return Error("Use --bdpt-normals geometric|shading.");
		}
		else if (option == "--tonemap") {
			if (!Gyunity::ParseToneMapOperator(value, &options.toneMap.op)) return Error("Use --tonemap none|reinhard|aces.");
		}
		else if (option == "--max-luminance" && !ParsePositiveReal(option, value, options.toneMap.maxLuminance)) return false;
		else if (option == "--tonemap-scale" && !ParsePositiveReal(option, value, options.toneMap.scale)) return false;
		else if (option == "--convert") parsed.convertInput = value;
	}
	const bool toneMapped = seen.count("--tonemap") || seen.count("--max-luminance") || seen.count("--tonemap-scale");
	if (seen.count("--convert")) {
		for (const auto& option : seen) {
			if (option != "--convert" && option != "--out" && option != "--tonemap" &&
				option != "--max-luminance" && option != "--tonemap-scale")
				return Error("--convert accepts only --out, --tonemap, --max-luminance and --tonemap-scale.");
		}
		if (!LowDynamicRange(options.output)) return Error("--convert requires a .png or .bmp --out file.");
		parsed.command = SceneCommand::Convert;
		output = std::move(parsed);
		return true;
	}
	if (toneMapped && !LowDynamicRange(options.output))
		return Error("--tonemap, --max-luminance and --tonemap-scale apply to .png or .bmp output only.");
	if (options.scene && !nativeFile.empty()) return Error("--scene and --scene-file are mutually exclusive.");
	const auto overrides = options;
	if (!LoadSceneDefaults(options, nativeFile)) return false;
	if (seen.count("--width")) options.width = overrides.width;
	if (seen.count("--height")) options.height = overrides.height;
	if (seen.count("--spp")) options.spp = overrides.spp;
	if (seen.count("--max-depth")) options.maxDepth = overrides.maxDepth;
	if (seen.count("--integrator")) options.integrator = overrides.integrator;
	if (seen.count("--accel")) options.accelerator = overrides.accelerator;
	if (seen.count("--iterations")) options.sppm.iterations = overrides.sppm.iterations;
	if (seen.count("--photons-per-stage")) options.sppm.photonsPerStage = overrides.sppm.photonsPerStage;
	if (!options.scene && !options.nativeScene) return Error("--scene or --scene-file is required.");
	if (options.scene && options.scene->usesVolume && !options.integrator.empty())
		return Error("Surface integrator overrides are not supported for volume scenes.");
	if (options.integrator == "sppm" && seen.count("--spp"))
		return Error("SPPM uses --iterations and --photons-per-stage, not --spp.");
	if (options.integrator != "sppm" && (seen.count("--iterations") || seen.count("--photons-per-stage")))
		return Error("--iterations and --photons-per-stage require the sppm integrator.");
	if (options.integrator != "bdpt" && seen.count("--bdpt-normals"))
		return Error("--bdpt-normals requires the bdpt integrator.");
	if (options.output.empty()) return Error("--out is required.");
	if (options.output.size() < 4) return Error("--out must end in .png, .bmp, .hdr or .exr.");
	const auto suffix = options.output.substr(options.output.size() - 4);
	if (suffix != ".png" && suffix != ".bmp" && suffix != ".hdr" && suffix != ".exr")
		return Error("--out must end in .png, .bmp, .hdr or .exr.");
	if (static_cast<int64>(options.width) * options.height > std::numeric_limits<int>::max() / 3)
		return Error("Image dimensions exceed the supported pixel count.");
	output = std::move(parsed);
	return true;
}

int RunSceneCommandLine(int argc, const char* const* argv) {
	SceneCommandLineOptions options;
	if (!ParseSceneCommandLine(argc, argv, options)) return 1;
	if (options.command == SceneCommand::Help) {
		std::cout << "Usage: GyunityApp --scene NAME --out FILE [options]\n"
			"  --scene-file PATH      Load native Gyunity JSON instead of a named scene\n"
			"  --integrator path|bdpt|sppm  Override the surface integrator\n"
			"  --list-scenes          List registered scene names\n"
			"  --list-examples        List callable application examples\n"
			"  --example NAME         Run an example with its documented defaults\n"
			"                         feature-scenes runs native scenes with PT, BDPT and SPPM;\n"
			"                         all also runs the existing integrator examples\n"
			"  --width N --height N    Resolution (default 512 x 512)\n"
			"  --spp N                Samples per pixel (default 64)\n"
			"  --iterations N         SPPM iterations (default 256)\n"
			"  --photons-per-stage N  SPPM photon budget per iteration (default 300000)\n"
			"  --max-depth N          Path depth (default 8)\n"
			"  --accel bvh|kd         Accelerator (default bvh)\n"
			"  --bdpt-normals geometric|shading  BDPT surface normals (default geometric);\n"
			"                         shading uses adjoint-corrected smooth normals (#148)\n"
			"  --tonemap none|reinhard|aces  PBRT imgtool tone mapping for .png/.bmp output\n"
			"  --max-luminance N      Reinhard white luminance (default 1)\n"
			"  --tonemap-scale S      Linear scale before tone mapping (default 1)\n"
			"  --convert IN           Convert IN (.exr/.hdr/...) to the --out .png/.bmp with the\n"
			"                         tone-mapping options and 8-bit sRGB encoding\n"
			"  --out FILE             Required .png, .bmp, .hdr or .exr output\n"
			"Native scenes supply defaults; explicit flags override them.\n"
			"Fog uses volume path tracing. Run from GyunityApp for named scene assets.\n"
			"With no arguments, the builder-based PathTracing example runs the env scene\n"
			"at 1024 x 1024, 100 spp, depth 10, BVH;\n"
			"output: Test\\CornellBoxPtEnvmap.bmp in the main checkout (also from worktrees).\n"
			"Examples.h/.cpp also contain SPPM, LightTracing and VolPathTracing examples.\n"
			"Prepare local Matpreview assets once with the documented uv import command.\n";
		return 0;
	}
	if (options.command == SceneCommand::List) {
		for (const auto& scene : GetSceneRegistry()) std::cout << scene.name << '\n';
		return 0;
	}
	if (options.command == SceneCommand::Render) return RenderScene(options.render) ? 0 : 1;
	if (options.command == SceneCommand::Convert)
		return ConvertImage(options.convertInput, options.render.output, options.render.toneMap) ? 0 : 1;
	Error("No scene command supplied.");
	return 1;
}
