#pragma once
#include "SceneRenderer.h"

enum class SceneCommand { Example, Help, List, Render };

struct SceneCommandLineOptions {
	SceneCommand command = SceneCommand::Example;
	SceneRenderOptions render;
};

bool ParseSceneCommandLine(int argc, const char* const* argv, SceneCommandLineOptions& output);
int RunSceneCommandLine(int argc, const char* const* argv);
