#pragma once
#include "scenes/SceneBuilder.h"
#include "camera/PinholeCamera.h"
#include <array>

struct NativeScene {
	std::string name, integrator, accelerator, filter;
	int width, height, spp, maxDepth;
	uint32 seed;
	real fovY;
	std::array<real, 16> cameraToWorld;
	std::shared_ptr<Scene> scene;

	std::shared_ptr<Camera> CreateCamera(const std::shared_ptr<Film>& film) const;
};

std::shared_ptr<NativeScene> LoadNativeScene(const std::string& filename);
