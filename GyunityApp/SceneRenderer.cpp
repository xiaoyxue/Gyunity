#include <stb/stb_image.h>
#include <stb/stb_image_write.h>
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
#include <cmath>
#include <limits>

namespace {
bool Error(const std::string& message) {
	std::cerr << "GyunityApp: " << message << '\n';
	return false;
}

class BakingImageTexture : public ImageTexture<Vec3> {
public:
	BakingImageTexture(const Array2D<Vec3>& color, const Array2D<real>& alpha)
		: ImageTexture<Vec3>(color), mAlpha(alpha) {}

	Vec3 Sample(const Vec2& uv) const override {
		const real alpha = mAlpha.Sample(uv);
		return alpha > 0 ? ImageTexture<Vec3>::Sample(uv) / alpha : Vec3(0);
	}

private:
	ImageTexture<real> mAlpha;
};

std::shared_ptr<ImageTexture<Vec3>> CreateBakingTexture(const unsigned char* rgba, int width, int height) {
	std::vector<Vec3> color(size_t(width) * height);
	std::vector<real> alpha(color.size());
	bool visible = false;
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; ++x) {
			const size_t source = (size_t(y) * width + x) * 4;
			const size_t target = size_t(height - 1 - y) * width + x;
			const real a = rgba[source + 3] / 255.f;
			alpha[target] = a;
			visible = visible || a > 0;
			// Alpha-weighted filtering prevents hidden RGB from bleeding into visible edges.
			color[target] = a * Vec3(std::pow(rgba[source] / 255.f, 2.2f),
				std::pow(rgba[source + 1] / 255.f, 2.2f), std::pow(rgba[source + 2] / 255.f, 2.2f));
		}
	}
	if (!visible) {
		Error("Texture baking input has no visible pixels.");
		return nullptr;
	}
	return std::make_shared<BakingImageTexture>(
		Array2D<Vec3>(color, height, width), Array2D<real>(alpha, height, width));
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

bool RenderTextureBaking(const TextureBakingOptions& options) {
	if (options.input.empty()) return Error("Texture baking requires an input image.");
	if (options.output.size() < 4 || options.output.substr(options.output.size() - 4) != ".png")
		return Error("Texture baking requires a .png output to preserve alpha.");
	if (!std::isfinite(options.roughness) || options.roughness <= 0 || options.roughness > 1)
		return Error("Texture baking roughness must be in (0, 1].");
	if (options.spp <= 0 || options.maxDepth <= 0)
		return Error("Texture baking sampling and depth budgets must be positive.");
	if (options.accelerator != SceneAccelerator::Bvh && options.accelerator != SceneAccelerator::Kd)
		return Error("Texture baking requires BVH or KD acceleration.");
	if (stbi_is_hdr(options.input.c_str()) || stbi_is_16_bit(options.input.c_str()))
		return Error("Texture baking requires an 8-bit input image.");

	int width = 0, height = 0, channels = 0;
	std::unique_ptr<unsigned char, decltype(&stbi_image_free)> rgba(
		stbi_load(options.input.c_str(), &width, &height, &channels, 4), stbi_image_free);
	if (!rgba) {
		const char* reason = stbi_failure_reason();
		return Error("Cannot read texture baking input '" + options.input + "': " +
			(reason ? reason : "unknown decoding error"));
	}
	if (width <= 0 || height <= 0 || int64(width) * height > std::numeric_limits<int>::max() / 4)
		return Error("Texture baking input exceeds the supported pixel count.");
	auto texture = CreateBakingTexture(rgba.get(), width, height);
	if (!texture) return false;
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	if (!CornellBoxScenes::TextureBaking(builder, texture, options.roughness)) return false;
	auto film = std::make_shared<Film>(width, height, new BoxFilter());
	auto camera = SceneCamera{}.Create(film);
	if (!camera || !film->HasValidStorage()) return Error("Cannot create the texture baking camera or film.");
	if (options.accelerator == SceneAccelerator::Kd)
		scene->SetAccelerator(std::make_shared<KdTreeAccel>(scene->GetPrimitives()));
	else
		scene->SetAccelerator(std::make_shared<BVHAccel>(scene->GetPrimitives()));
	scene->Initialize();
	PathTracing integrator(options.spp, options.maxDepth,
		std::make_shared<RandomSampler>(123), std::make_shared<SamplerEnum>());
	if (!integrator.RenderChecked(*scene, *camera) || !film->HasFiniteSamples())
		return Error("Texture baking failed or produced nonfinite samples.");
	const auto& image = film->GetImage();
	for (size_t i = 0; i < image.size(); ++i) {
		for (int channel = 0; channel < 3; ++channel)
			rgba.get()[i * 4 + channel] = rgba.get()[i * 4 + 3]
				? static_cast<unsigned char>(ImageIO::ToInt(image[i][channel], 2.2f)) : 0;
	}
	if (!stbi_write_png(options.output.c_str(), width, height, 4, rgba.get(), width * 4))
		return Error("Cannot write texture baking output '" + options.output + "'. Check its directory and permissions.");
	return true;
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
		BidirectionalPathTracing integrator(sampler, options.maxDepth, options.spp, options.bdptShadingNormals,
			options.bdptLogPdfMis ? BdptMis::LogPdf : BdptMis::Ratio);
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
