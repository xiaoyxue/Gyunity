#pragma once
#include "scenes/SceneBuilder.h"
#include "camera/PinholeCamera.h"
#include <array>

struct SppmSettings {
	int iterations = 256;
	int photonsPerStage = 300000;
};

struct NativeScene {
	std::string name, integrator, accelerator, filter;
	int width, height, spp, maxDepth;
	uint32 seed;
	real fovY;
	std::array<real, 16> cameraToWorld;
	std::shared_ptr<Scene> scene;
	SppmSettings sppm;

	std::shared_ptr<Camera> CreateCamera(const std::shared_ptr<Film>& film) const;
};

// Failure is logged and returned as nullptr; callers must check before rendering.
std::shared_ptr<NativeScene> LoadNativeScene(const std::string& filename);
