#include "SceneRenderer.h"
#include "accelerator/BVHAccel.h"
#include "accelerator/KdTreeAccel.h"
#include "integrator/PT.h"
#include "integrator/VPT.h"
#include "integrator/BDPT.h"
#include "integrator/SPPM.h"
#include "sampler/Halton.h"
#include "sampler/HaltonEnum.h"
#include "sampler/SamplerEnum.h"
#include "image/ImageIO.h"
#include <limits>

namespace {
bool Error(const std::string& message) {
	std::cerr << "GyunityApp: " << message << '\n';
	return false;
}

bool ValidateSetup(const SceneRenderOptions& options) {
	if (!options.scene && !options.nativeScene) return Error("A registered or native scene is required.");
	if (options.width <= 0 || options.height <= 0 ||
		static_cast<int64>(options.width) * options.height > std::numeric_limits<int>::max() / 3)
		return Error("Image dimensions exceed the supported pixel count or are not positive.");
	if (options.output.empty()) return Error("An output filename is required.");
	return true;
}
}

bool RenderScene(const SceneRenderOptions& options) {
	if (!ValidateSetup(options)) return false;
	if (options.integrator != "" && options.integrator != "path" && options.integrator != "bdpt" && options.integrator != "sppm")
		return Error("Use surface integrator path|bdpt|sppm.");
	if (options.scene && options.scene->usesVolume && !options.integrator.empty())
		return Error("Surface integrator overrides are not supported for volume scenes.");
	if (options.maxDepth <= 0) return Error("Path depth must be positive.");
	if (options.integrator == "sppm") {
		if (options.sppm.iterations <= 0 || options.sppm.photonsPerStage <= 0)
			return Error("SPPM iterations and photons per stage must be positive.");
		auto sampler = std::make_shared<HaltonSampler>(options.width, options.height);
		auto samplerEnum = std::make_shared<HaltonEnum>(static_cast<unsigned>(options.width), static_cast<unsigned>(options.height));
		SPPM integrator(options.sppm.iterations, options.sppm.photonsPerStage, options.maxDepth,
			0.05f, 0.66666667f, false, sampler, samplerEnum, true);
		return RenderScene(options, integrator);
	}
	if (options.spp <= 0) return Error("Samples per pixel must be positive.");
	auto sampler = std::make_shared<RandomSampler>(options.nativeScene ? options.nativeScene->seed : 123);
	auto samplerEnum = std::make_shared<SamplerEnum>();
	if (options.integrator == "bdpt") {
		BidirectionalPathTracing integrator(sampler, options.maxDepth, options.spp, options.bdptShadingNormals);
		return RenderScene(options, integrator);
	}
	if (options.scene && options.scene->usesVolume) {
		VolPathTracing integrator(options.spp, options.maxDepth, sampler, samplerEnum, false);
		return RenderScene(options, integrator);
	}
	PathTracing integrator(options.spp, options.maxDepth, sampler, samplerEnum);
	return RenderScene(options, integrator);
}

bool LoadSceneDefaults(SceneRenderOptions& options, const std::string& filename) {
	const auto file = options.scene && options.scene->file ? SceneBuilder::Resource(options.scene->file) : filename;
	if (file.empty()) return true;
	if (options.scene && options.scene->preparation && !std::ifstream(file, std::ios::binary).good())
		return Error("Cannot read local scene '" + file + "'. " + options.scene->preparation);
	auto native = LoadNativeScene(file);
	if (!native) return false;
	if (options.scene && native->name != options.scene->name) return Error("Native scene name does not match the registry.");
	options.width = native->width;
	options.height = native->height;
	options.spp = native->spp;
	options.maxDepth = native->maxDepth;
	options.integrator = native->integrator;
	options.accelerator = native->accelerator == "bvh" ? SceneAccelerator::Bvh : SceneAccelerator::Kd;
	options.nativeScene = std::move(native);
	options.sppm = options.nativeScene->sppm;
	return true;
}

bool RenderScene(const SceneRenderOptions& options, Integrator& integrator) {
	if (!ValidateSetup(options)) return false;
	for (const auto* resource : options.scene ? options.scene->resources : std::array<const char*, 3>{}) {
		if (resource && !std::ifstream(SceneBuilder::Resource(resource), std::ios::binary).good())
			return Error("Cannot read resource '" + SceneBuilder::Resource(resource) + "'. Run from GyunityApp.");
	}
	auto native = options.nativeScene;
	if (!native && options.scene && options.scene->file) {
		auto defaults = options;
		if (!LoadSceneDefaults(defaults)) return false;
		native = defaults.nativeScene;
	}
	Filter* filter;
	if (native && native->filter == "tent") filter = new TentFilter();
	else if (native && native->filter == "gaussian") filter = new GaussianFilter();
	else filter = new BoxFilter();
	auto film = std::make_shared<Film>(options.width, options.height, filter);
	auto scene = native ? native->scene : std::make_shared<Scene>();
	auto camera = native ? native->CreateCamera(film) : options.scene->camera.Create(film);
	if (!camera || !camera->IsValid()) return Error("Cannot render an invalid camera.");
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
	if (!film->SaveImage(options.toneMap)) return Error("Cannot write image '" + options.output + "'. Check its directory and permissions.");
	return true;
}

bool ConvertImage(const std::string& input, const std::string& output, const ToneMapSettings& toneMap) {
	std::vector<Vec3> image;
	int width = 0, height = 0;
	if (!ImageIO::LoadImage(input, image, width, height) || width <= 0 || height <= 0)
		return Error("Cannot read image '" + input + "'.");
	if (!ApplyToneMap(image, toneMap)) return Error("Cannot tone map '" + input + "'.");
	if (!ImageIO::WriteSrgbLdrImage(output, image, uint32(width), uint32(height)))
		return Error("Cannot write image '" + output + "'. Use a .png or .bmp output.");
	return true;
}
