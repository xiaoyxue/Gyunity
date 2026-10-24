#pragma once
#include "SceneRegistry.h"
#include "integrator/Integrator.h"

enum class SceneCommand { Example, Help, List, Render };
enum class SceneAccelerator { Bvh, Kd };

struct SceneRenderOptions {
	SceneCommand command = SceneCommand::Example;
	const SceneDescription* scene = nullptr;
	SceneAccelerator accelerator = SceneAccelerator::Bvh;
	int width = 512;
	int height = 512;
	int spp = 64;
	int maxDepth = 8;
	std::string output;
};

SceneRenderOptions ParseSceneCommandLine(int argc, const char* const* argv);
void RenderScene(const SceneRenderOptions& options, Gyunity::Integrator& integrator);
int RunSceneCommandLine(int argc, const char* const* argv);
