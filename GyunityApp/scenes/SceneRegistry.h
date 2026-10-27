#pragma once
#include "CornellBoxScenes.h"
#include "camera/PinholeCamera.h"
#include <array>

struct SceneCamera {
	Vec3 pos = Vec3(0, 0, 3);
	Vec3 lookAt = Vec3(0, 0, 0);
	Vec3 up = Vec3(0, 1, 0);
	real fovy = 53.13010235415597f;
	real filmDistance = 1;

	std::shared_ptr<Camera> Create(const std::shared_ptr<Film>& film) const {
		return PinHoleCamera::Create(film, pos, lookAt, up, fovy, filmDistance);
	}
};

struct SceneDescription {
	const char* name;
	void (*build)(SceneBuilder&);
	SceneCamera camera;
	bool usesVolume;
	std::array<const char*, 3> resources;
	const char* file = nullptr;
	const char* preparation = nullptr;
};

inline const std::array<SceneDescription, 23>& GetSceneRegistry() {
	static const SceneCamera standardCamera;
	static const SceneCamera sphereCamera = {
		Vec3(50, 52, 295.6f), Vec3(50, 52, 295.6f) + Vec3(0, -0.042612f, -1),
		Vec3(0, 1, 0), 28.7993f, 1
	};
	static const char* previewPreparation =
		"From the repository root, run: uv run --project tools\\gyscene --locked --extra mitsuba "
		"gyscene import-matpreview <path-to-matpreview.zip> --profile all --max-depth 8 --output-root resources\\scenes";
	static const std::array<SceneDescription, 23> scenes = {{
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
		{ "CornellBoxStandard", nullptr, {}, false, {}, "scenes\\mitsuba\\cornell-box\\scene.json" },
		{ "veach-mis", nullptr, {}, false, {}, "scenes\\mitsuba\\veach-mis\\scene.json" },
		{ "veach-bidir", nullptr, {}, false, {}, "scenes\\mitsuba\\veach-bidir\\scene.json" },
		{ "matpreview-diffuse", nullptr, {}, false, {}, "scenes\\matpreview\\matpreview-diffuse.json", previewPreparation },
		{ "matpreview-glass", nullptr, {}, false, {}, "scenes\\matpreview\\matpreview-glass.json", previewPreparation },
		{ "matpreview-copper", nullptr, {}, false, {}, "scenes\\matpreview\\matpreview-copper.json", previewPreparation },
		{ "matpreview-checkerboard", nullptr, {}, false, {}, "scenes\\matpreview\\matpreview-checkerboard.json", previewPreparation },
		{ "matpreview-mirror", nullptr, {}, false, {}, "scenes\\matpreview\\matpreview-mirror.json", previewPreparation },
		{ "matpreview-rough-glass", nullptr, {}, false, {}, "scenes\\matpreview\\matpreview-rough-glass.json", previewPreparation },
		{ "matpreview-copper-beckmann", nullptr, {}, false, {}, "scenes\\matpreview\\matpreview-copper-beckmann.json", previewPreparation },
		{ "matpreview-copper-anisotropic", nullptr, {}, false, {}, "scenes\\matpreview\\matpreview-copper-anisotropic.json", previewPreparation },
		{ "matpreview-plastic", nullptr, {}, false, {}, "scenes\\matpreview\\matpreview-plastic.json", previewPreparation }
	}};
	return scenes;
}

inline const SceneDescription* FindScene(const std::string& name) {
	for (const auto& scene : GetSceneRegistry()) {
		if (name == scene.name) return &scene;
	}
	GetDefaultLogger()->Error("Unknown scene '" + name + "'. Use --list-scenes.", false);
	return nullptr;
}
