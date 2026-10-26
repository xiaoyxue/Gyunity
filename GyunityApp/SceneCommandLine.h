#pragma once
#include "SceneRenderer.h"

enum class SceneCommand { Example, Help, List, Render, Convert };

struct SceneCommandLineOptions {
	SceneCommand command = SceneCommand::Example;
	SceneRenderOptions render;
	std::string convertInput;
};

bool ParseSceneCommandLine(int argc, const char* const* argv, SceneCommandLineOptions& output);
int RunSceneCommandLine(int argc, const char* const* argv);
