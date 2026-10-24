#pragma once
#include "SceneRegistry.h"

enum class SceneCommand { Legacy, Help, List, Render };
enum class SceneAccelerator { Bvh, Kd };

struct SceneRenderOptions {
	SceneCommand command = SceneCommand::Legacy;
	const SceneDescription* scene = nullptr;
	SceneAccelerator accelerator = SceneAccelerator::Bvh;
	int width = 512;
	int height = 512;
	int spp = 64;
	int maxDepth = 8;
	std::string output;
};

SceneRenderOptions ParseSceneCommandLine(int argc, const char* const* argv);
int RunSceneCommandLine(int argc, const char* const* argv);
