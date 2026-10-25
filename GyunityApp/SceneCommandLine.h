#pragma once
#include "scenes/SceneRegistry.h"
#include "integrator/Integrator.h"
#include "NativeScene.h"

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
	std::string integrator;
	std::shared_ptr<NativeScene> nativeScene;
};

bool ParseSceneCommandLine(int argc, const char* const* argv, SceneRenderOptions& output);
bool RenderScene(const SceneRenderOptions& options, Gyunity::Integrator& integrator);
int RunSceneCommandLine(int argc, const char* const* argv);
