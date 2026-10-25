#pragma once
#include "scenes/SceneRegistry.h"
#include "integrator/Integrator.h"
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
};

bool LoadSceneDefaults(SceneRenderOptions& options, const std::string& filename = {});
bool RenderScene(const SceneRenderOptions& options);
bool RenderScene(const SceneRenderOptions& options, Gyunity::Integrator& integrator);
