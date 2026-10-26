#pragma once
#include "scenes/SceneRegistry.h"
#include "integrator/Integrator.h"
#include "image/ToneMap.h"
#include "NativeScene.h"

enum class SceneAccelerator { Bvh, Kd };

struct SceneRenderOptions {
	const SceneDescription* scene = nullptr;
	SceneAccelerator accelerator = SceneAccelerator::Bvh;
	int width = 512;
	int height = 512;
	int spp = 64;
	int maxDepth = 8;
	std::string output;
	std::string integrator;
	std::shared_ptr<NativeScene> nativeScene;
	SppmSettings sppm;
	// BDPT only: smooth shading normals with adjoint-corrected transport (#148).
	bool bdptShadingNormals = false;
	// Low-dynamic-range outputs only: pbrt imgtool tone mapping with 8-bit sRGB encoding.
	Gyunity::ToneMapSettings toneMap;
};

bool LoadSceneDefaults(SceneRenderOptions& options, const std::string& filename = {});
bool RenderScene(const SceneRenderOptions& options);
bool RenderScene(const SceneRenderOptions& options, Gyunity::Integrator& integrator);
// Like pbrt imgtool convert: reads an image, applies scale and tone mapping, writes 8-bit sRGB.
bool ConvertImage(const std::string& input, const std::string& output, const Gyunity::ToneMapSettings& toneMap);
