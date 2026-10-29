#include "Examples.h"
#include "integrator/LT.h"
#include "integrator/BDPT.h"
#include "integrator/PT.h"
#include "integrator/VPT.h"
#include "sampler/SamplerEnum.h"
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <direct.h>
#include <filesystem>
#include <sys/stat.h>

namespace Examples {
namespace {

bool Error(const std::string& message) {
	GetDefaultLogger()->Error("Example: " + message, false);
	return false;
}

bool Validate(const Options& options, bool allowZeroDepth = false) {
	if (!options.output.empty() && !options.outputDirectory.empty())
		return Error("Choose an output filename or an output directory, not both.");
	if (!options.outputDirectory.empty()) {
		std::error_code error;
		if (!std::filesystem::is_directory(options.outputDirectory, error) || error)
			return Error("Cannot access example output directory '" + options.outputDirectory + "'.");
	}
	for (const auto value : {options.width, options.height, options.spp,
		options.iterations, options.photonsPerStage}) {
		if (value && *value <= 0) return Error("Explicit dimensions and sampling/depth budgets must be positive.");
	}
	if (options.maxDepth && *options.maxDepth < (allowZeroDepth ? 0 : 1))
		return Error(allowZeroDepth ? "Explicit depth must be nonnegative." : "Explicit depth must be positive.");
	return true;
}

bool Prepare(const std::string& name, const Options& overrides, const std::string& filename,
	SceneRenderOptions& options, int legacySpp = 0, int legacyDepth = 0, bool allowZeroDepth = false) {
	if (!Validate(overrides, allowZeroDepth)) return false;
	options.scene = FindScene(name);
	if (!options.scene || !LoadSceneDefaults(options)) return false;
	if (legacySpp) {
		options.width = options.height = 1024;
		options.spp = legacySpp;
		options.maxDepth = legacyDepth;
	}
	if (overrides.width) options.width = *overrides.width;
	if (overrides.height) options.height = *overrides.height;
	if (overrides.spp) options.spp = *overrides.spp;
	if (overrides.maxDepth) options.maxDepth = *overrides.maxDepth;
	if (overrides.accelerator) options.accelerator = *overrides.accelerator;
	if (overrides.iterations) options.sppm.iterations = *overrides.iterations;
	if (overrides.photonsPerStage) options.sppm.photonsPerStage = *overrides.photonsPerStage;
	if (int64(options.width) * options.height > std::numeric_limits<int>::max() / 3)
		return Error("Image dimensions exceed the supported pixel count.");
	const auto outputName = filename.empty() ? name + '-' + options.integrator + ".png" : filename;
	if (overrides.output.empty()) {
		if (!overrides.outputDirectory.empty()) options.output = overrides.outputDirectory + '\\' + outputName;
		else if (!OutputPath(outputName, options.output)) return false;
	}
	else options.output = overrides.output;
	return true;
}

bool Native(const std::string& name, const Options& overrides,
	std::optional<SurfaceIntegrator> integrator = std::nullopt) {
	SceneRenderOptions options;
	std::string method;
	if (integrator) {
		switch (*integrator) {
		case SurfaceIntegrator::Path: method = "path"; break;
		case SurfaceIntegrator::Bdpt: method = "bdpt"; break;
		case SurfaceIntegrator::Sppm: method = "sppm"; break;
		default: return Error("Unsupported surface integrator.");
		}
	}
	if (!Prepare(name, overrides, integrator ? name + '-' + method + ".png" : "", options)) return false;
	if (!options.nativeScene || options.scene->usesVolume) return Error("A native surface scene is required.");
	if (integrator) options.integrator = method;
	std::cout << "Rendering " << name << " (" << options.integrator << ") -> " << options.output << '\n';
	return ::RenderScene(options);
}

struct Entry {
	const char* name;
	bool (*render)(const Options&);
};

const Entry legacyExamples[] = {
	{"path-tracing", PathTracing}, {"sppm", Sppm}, {"light-tracing", LightTracing},
	{"volume-path-tracing", VolumePathTracing},
	{"bidirectional-path-tracing", [](const Options& options) { return BidirectionalPathTracing("mesh", options); }}
};

bool BatchOptions(const Options& options) {
	if (!Validate(options)) return false;
	if (!options.output.empty()) return Error("Batch examples use separate default filenames; a single output path is not allowed.");
	for (const auto& scene : GetSceneRegistry()) {
		if (scene.file && !std::ifstream(SceneBuilder::Resource(scene.file), std::ios::binary).good())
			return Error("Cannot read '" + SceneBuilder::Resource(scene.file) + "'. " +
				(scene.preparation ? scene.preparation : "Run from GyunityApp and check the scene assets."));
	}
	return true;
}

}

bool OutputPath(const std::string& filename, std::string& output) {
	if (filename.empty() || filename.find_first_of("\\/:") != std::string::npos || filename == "." || filename == "..")
		return Error("Example output requires a filename, not a directory or traversal path.");
	char* executable = nullptr;
	if (_get_pgmptr(&executable) != 0 || !executable)
		return Error("Cannot locate the executable for the example output directory.");
	std::string directory(executable);
	for (auto separator = directory.find_last_of("\\/"); separator != std::string::npos;
		separator = directory.find_last_of("\\/")) {
		directory.erase(separator);
		const auto gitDirectory = directory + "\\.git";
		struct _stat info;
		const int result = _stat(gitDirectory.c_str(), &info);
		if (result != 0 && errno != ENOENT && errno != ENOTDIR)
			return Error("Cannot inspect '" + gitDirectory + "': " + std::strerror(errno));
		// Worktrees have .git files; the main checkout has the .git directory.
		if (result == 0 && (info.st_mode & _S_IFDIR)) {
			const auto outputDirectory = directory + "\\Test";
			if ((_mkdir(outputDirectory.c_str()) != 0 && errno != EEXIST) ||
				_stat(outputDirectory.c_str(), &info) != 0 || !(info.st_mode & _S_IFDIR))
				return Error("Cannot create example output directory '" + outputDirectory + "'.");
			output = outputDirectory + '\\' + filename;
			return true;
		}
	}
	return Error("Cannot find the main checkout for example output. Pass an explicit output path.");
}

bool PathTracing(const Options& overrides) {
	SceneRenderOptions options;
	if (!Prepare("env", overrides, "CornellBoxPtEnvmap.bmp", options, 100, 10)) return false;
	Gyunity::PathTracing integrator(options.spp, options.maxDepth, std::make_shared<RandomSampler>(123),
		std::make_shared<SamplerEnum>());
	return ::RenderScene(options, integrator);
}

bool Sppm(const Options& overrides) {
	SceneRenderOptions options;
	if (!Prepare("mesh", overrides, "CornellBoxSppmTest.bmp", options, 64, 20)) return false;
	options.integrator = "sppm";
	return ::RenderScene(options);
}

bool LightTracing(const Options& overrides) {
	SceneRenderOptions options;
	if (!Prepare("mesh", overrides, "CornellBoxLT.bmp", options, 512, 15)) return false;
	Gyunity::LightTracing integrator(std::make_shared<RandomSampler>(123), options.maxDepth, options.spp);
	return ::RenderScene(options, integrator);
}

bool VolumePathTracing(const Options& overrides) {
	SceneRenderOptions options;
	if (!Prepare("fog", overrides, "MediaTestFogEquiAngular.bmp", options, 2000, 8)) return false;
	VolPathTracing integrator(options.spp, options.maxDepth, std::make_shared<RandomSampler>(123),
		std::make_shared<SamplerEnum>(), false);
	return ::RenderScene(options, integrator);
}

bool BidirectionalPathTracing(const std::string& scene, const Options& overrides) {
	SceneRenderOptions options;
	if (!Prepare(scene, overrides, "CornellBoxBdpt.bmp", options, 256, 8)) return false;
	if (options.scene->usesVolume) return Error("BDPT requires a registered non-volume scene.");
	Gyunity::BidirectionalPathTracing integrator(
		std::make_shared<RandomSampler>(options.nativeScene ? options.nativeScene->seed : 123),
		options.maxDepth, options.spp);
	return ::RenderScene(options, integrator);
}

bool VertexConnectionMerging(const std::string& scene, const Options& overrides,
	real initialRadius, real alpha, Vcm::Mode mode, int concurrency, bool useShadingNormals,
	Gyunity::VertexConnectionMerging::Statistics* statistics) {
	if (statistics) *statistics = {};
	const auto* selected = FindScene(scene);
	if (!selected || selected->usesVolume) return Error("VCM requires a registered non-volume scene.");
	if (overrides.spp || overrides.photonsPerStage)
		return Error("VCM uses iterations, not spp or an independent photon budget.");
	if (overrides.accelerator && *overrides.accelerator != SceneAccelerator::Bvh &&
		*overrides.accelerator != SceneAccelerator::Kd)
		return Error("VCM example requires BVH or KD acceleration.");
	auto settings = overrides;
	if (!settings.width) settings.width = 128;
	if (!settings.height) settings.height = 128;
	if (!settings.iterations) settings.iterations = 16;
	if (!settings.maxDepth) settings.maxDepth = 8;
	SceneRenderOptions options;
	if (!Prepare(scene, settings, "CornellBoxVcm.hdr", options, 0, 0, true)) return false;
	Gyunity::VertexConnectionMerging integrator(
		std::make_shared<RandomSampler>(options.nativeScene ? options.nativeScene->seed : 123),
		options.maxDepth, *settings.iterations, initialRadius, alpha, mode, concurrency, useShadingNormals);
	const bool success = ::RenderScene(options, integrator);
	if (statistics) *statistics = integrator.GetStatistics();
	return success;
}

bool CornellBoxStandard(const Options& options, SurfaceIntegrator integrator) {
	return Native("CornellBoxStandard", options, integrator);
}

bool VeachMis(const Options& options, SurfaceIntegrator integrator) {
	return Native("veach-mis", options, integrator);
}

bool VeachBidir(const Options& options, SurfaceIntegrator integrator) {
	return Native("veach-bidir", options, integrator);
}

// SmallVCM's veach-bidir comparison settings. Its radius is 0.003 times the scene AABB's
// world-space half-diagonal; that half-diagonal is scene-specific and precomputed here, not
// derived from the loaded scene.
bool VeachBidirVcm(const Options& options, Gyunity::VertexConnectionMerging::Statistics* statistics) {
	constexpr double sceneHalfDiagonal = 6.956186780406363;
	auto settings = options;
	if (!settings.width) settings.width = 1024;
	if (!settings.height) settings.height = 1024;
	if (!settings.iterations) settings.iterations = 512;
	if (!settings.maxDepth) settings.maxDepth = 7;
	if (settings.output.empty()) {
		const auto filename = "veach-bidir-vcm-shading-" + std::to_string(*settings.width) + 'x' +
			std::to_string(*settings.height) + "-iters" + std::to_string(*settings.iterations) +
			"-depth" + std::to_string(*settings.maxDepth) + "-smallvcm-radius.exr";
		if (!OutputPath(filename, settings.output)) return false;
	}
	return VertexConnectionMerging("veach-bidir", settings, real(0.003 * sceneHalfDiagonal), real(0.75),
		Vcm::Mode::Combined, ThreadedTaskManager::DefaultConcurrency(), true, statistics);
}

bool Matpreview(const std::string& profile, const Options& options, SurfaceIntegrator integrator) {
	return Native("matpreview-" + profile, options, integrator);
}

bool FeatureScenes(const Options& options) {
	if (!BatchOptions(options)) return false;
	for (const auto& scene : GetSceneRegistry()) {
		if (!scene.file) continue;
		for (auto integrator : {SurfaceIntegrator::Path, SurfaceIntegrator::Bdpt, SurfaceIntegrator::Sppm})
			if (!Native(scene.name, options, integrator)) return false;
	}
	return true;
}

bool All(const Options& options) {
	if (!BatchOptions(options)) return false;
	for (const auto& example : legacyExamples) {
		std::cout << "Rendering example " << example.name << '\n';
		if (!example.render(options)) return false;
	}
	return FeatureScenes(options);
}

bool Named(const std::string& name, const Options& options) {
	if (name == "all") return All(options);
	if (name == "feature-scenes") return FeatureScenes(options);
	for (const auto& example : legacyExamples)
		if (name == example.name) return example.render(options);
	for (const auto& scene : GetSceneRegistry())
		if (scene.file && name == scene.name) return Native(name, options);
	return Error("Unknown example '" + name + "'. Use --list-examples.");
}

std::vector<std::string> Names() {
	std::vector<std::string> names;
	for (const auto& example : legacyExamples) names.emplace_back(example.name);
	for (const auto& scene : GetSceneRegistry()) if (scene.file) names.emplace_back(scene.name);
	names.emplace_back("feature-scenes");
	names.emplace_back("all");
	return names;
}

}
