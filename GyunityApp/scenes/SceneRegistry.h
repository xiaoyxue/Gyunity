#pragma once
#include "CornellBoxScenes.h"
#include "MisScenes.h"
#include "camera/PinholeCamera.h"
#include "BdptRoomScenes.h"
#include <array>
#include <stdexcept>

struct SceneCamera {
	Vec3 pos = Vec3(0, 0, 3);
	Vec3 lookAt = Vec3(0, 0, 0);
	Vec3 up = Vec3(0, 1, 0);
	real fovy = 53.13010235415597f;
	real filmDistance = 1;

	std::shared_ptr<Camera> Create(const std::shared_ptr<Film>& film) const {
		return std::make_shared<PinHoleCamera>(film, pos, lookAt, up, fovy, filmDistance);
	}
};

struct SceneDescription {
	const char* name;
	void (*build)(SceneBuilder&);
	SceneCamera camera;
	bool usesVolume;
	std::array<const char*, 3> resources;
};

inline const std::array<SceneDescription, 14>& GetSceneRegistry() {
	static const SceneCamera standardCamera;
	static const SceneCamera sphereCamera = {
		Vec3(50, 52, 295.6f), Vec3(50, 52, 295.6f) + Vec3(0, -0.042612f, -1),
		Vec3(0, 1, 0), 28.7993f, 1
	};
	static const SceneCamera roomCamera = { Vec3(0, 1.6f, 5.5f), Vec3(-0.2f, 1.15f, -0.6f), Vec3(0, 1, 0), 53.13010235415597f, 1 };
	static const SceneCamera misCamera = { Vec3(0, 4, 8), Vec3(0, 1, 0), Vec3(0, 1, 0), 53.13010235415597f, 1 };
	static const std::array<SceneDescription, 14> scenes = {{
		{ "bdpt-room", BdptRoomScenes::Diffuse, roomCamera, false, {} },
		{ "bdpt-room-glossy", BdptRoomScenes::Glossy, roomCamera, false, {} },
		{ "mesh", CornellBoxScenes::GlassBunny, standardCamera, false, { "meshs\\bunny3.obj" } },
		{ "slope", CornellBoxScenes::Slope, standardCamera, false, {} },
		{ "sphere", CornellBoxScenes::SphereBox, sphereCamera, false, {} },
		{ "env", CornellBoxScenes::Environment, standardCamera, false,
			{ "meshs\\bunny3.obj", "meshs\\Alucy.obj", "media\\pictures\\uffizi-large.hdr" } },
		{ "triangle", CornellBoxScenes::TriangleBox, sphereCamera, false, {} },
		{ "textured", CornellBoxScenes::TexturedSpheres, standardCamera, false,
			{ "texture_images\\sphere_mapping_8k.jpg" } },
		{ "empty", CornellBoxScenes::Empty, standardCamera, false, {} },
		{ "diffuse", CornellBoxScenes::DiffuseBunny, standardCamera, false, { "meshs\\bunny3.obj" } },
		{ "water", CornellBoxScenes::Water, standardCamera, false, { "meshs\\water.obj", "meshs\\Alucy.obj" } },
		{ "lucy-bunny", CornellBoxScenes::RoughGlassLucyAndPurpleBunny, standardCamera, false,
			{ "meshs\\Alucy.obj", "meshs\\bunny3.obj" } },
		{ "fog", CornellBoxScenes::FogBunny, standardCamera, true, { "meshs\\bunny3.obj" } },
		{ "mis-glossy", MisScenes::Glossy, misCamera, false, {} }
	}};
	return scenes;
}

inline const SceneDescription& FindScene(const std::string& name) {
	for (const auto& scene : GetSceneRegistry()) {
		if (name == scene.name) return scene;
	}
	throw std::invalid_argument("Unknown scene '" + name + "'. Use --list-scenes.");
}
