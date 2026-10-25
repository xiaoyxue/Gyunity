#include "../../GyunityApp/SceneCommandLine.h"
#include "gtest/gtest.h"
#include <set>

namespace {

SceneRenderOptions Parse(std::initializer_list<const char*> arguments) {
	std::vector<const char*> argv = { "GyunityApp" };
	argv.insert(argv.end(), arguments.begin(), arguments.end());
	return ParseSceneCommandLine(static_cast<int>(argv.size()), argv.data());
}

void ExpectVector(const Vec3& actual, const Vec3& expected) {
	EXPECT_EQ(actual.x, expected.x);
	EXPECT_EQ(actual.y, expected.y);
	EXPECT_EQ(actual.z, expected.z);
}

TEST(SceneRegistry, NamesMapToAllBuilders) {
	const std::pair<const char*, void (*)(SceneBuilder&)> expected[] = {
		{ "mesh", CornellBoxScenes::GlassBunny }, { "slope", CornellBoxScenes::Slope },
		{ "sphere", CornellBoxScenes::SphereBox }, { "env", CornellBoxScenes::Environment },
		{ "triangle", CornellBoxScenes::TriangleBox }, { "textured", CornellBoxScenes::TexturedSpheres },
		{ "empty", CornellBoxScenes::Empty }, { "diffuse", CornellBoxScenes::DiffuseBunny },
		{ "water", CornellBoxScenes::Water }, { "fog", CornellBoxScenes::FogBunny },
		{ "lucy-bunny", CornellBoxScenes::RoughGlassLucyAndPurpleBunny },
		{ "CornellBoxStandard", nullptr }, { "veach-mis", nullptr }, { "veach-bidir", nullptr }
	};
	ASSERT_EQ(GetSceneRegistry().size(), sizeof(expected) / sizeof(expected[0]));
	std::set<std::string> names;
	for (const auto& scene : GetSceneRegistry()) EXPECT_TRUE(names.insert(scene.name).second);
	for (const auto& entry : expected) {
		EXPECT_EQ(FindScene(entry.first).build, entry.second);
		EXPECT_EQ(FindScene(entry.first).usesVolume, std::string(entry.first) == "fog");
	}
	EXPECT_THROW(FindScene("not-a-scene"), std::invalid_argument);
}

TEST(SceneRegistry, CamerasMatchOriginalRegressionRays) {
	for (const auto& scene : GetSceneRegistry()) {
		if (scene.file) continue;
		SCOPED_TRACE(scene.name);
		for (const auto& size : { std::pair<int, int>(512, 512), std::pair<int, int>(640, 360) }) {
			auto film = std::make_shared<Film>(size.first, size.second);
			auto referenceFilm = std::make_shared<Film>(size.first, size.second);
			auto camera = scene.camera.Create(film);
			std::shared_ptr<Camera> reference;
			if (std::string(scene.name) == "sphere" || std::string(scene.name) == "triangle") {
				const Vec3 pos(50, 52, 295.6f);
				reference = std::make_shared<PinHoleCamera>(referenceFilm, pos,
					pos + Vec3(0, -0.042612f, -1), Vec3(0, 1, 0), 28.7993f, 1);
			}
			else {
				reference = std::make_shared<PinHoleCamera>(referenceFilm, Vec3(0, 0, 3),
					Vec3(0, 0, 0), Vec3(0, 1, 0), 53.13010235415597f, 1);
			}
			for (int x : { 0, size.first / 2, size.first - 1 }) {
				for (int y : { 0, size.second / 2, size.second - 1 }) {
					const Ray ray = camera->GenerateRay(x, y, Vec2(0.2f, 0.7f));
					const Ray expected = reference->GenerateRay(x, y, Vec2(0.2f, 0.7f));
					ExpectVector(ray.mOrig, expected.mOrig);
					ExpectVector(ray.mDir, expected.mDir);
				}
			}
		}
	}
}

TEST(SceneCommandLine, NoArgumentsSelectTheMainExample) {
	EXPECT_EQ(Parse({}).command, SceneCommand::Example);
}

TEST(SceneCommandLine, InformationalCommandsDoNotRequireSceneOrOutput) {
	EXPECT_EQ(Parse({ "--help" }).command, SceneCommand::Help);
	EXPECT_EQ(Parse({ "--list-scenes" }).command, SceneCommand::List);
}

TEST(SceneCommandLine, RenderDefaultsMatchRegressionSettings) {
	const auto options = Parse({ "--scene", "mesh", "--out", "image.png" });
	EXPECT_EQ(options.command, SceneCommand::Render);
	EXPECT_EQ(options.scene, &FindScene("mesh"));
	EXPECT_EQ(options.accelerator, SceneAccelerator::Bvh);
	EXPECT_EQ(options.width, 512);
	EXPECT_EQ(options.height, 512);
	EXPECT_EQ(options.spp, 64);
	EXPECT_EQ(options.maxDepth, 8);
	EXPECT_EQ(options.output, "image.png");
}

TEST(SceneCommandLine, OverridesAreParsedIndependentlyOfOrder) {
	const auto options = Parse({ "--out", "folder with spaces\\image.hdr", "--height", "360",
		"--spp", "2", "--max-depth", "4", "--accel", "kd", "--width", "640", "--scene", "fog" });
	EXPECT_EQ(options.scene, &FindScene("fog"));
	EXPECT_EQ(options.width, 640);
	EXPECT_EQ(options.height, 360);
	EXPECT_EQ(options.spp, 2);
	EXPECT_EQ(options.maxDepth, 4);
	EXPECT_EQ(options.accelerator, SceneAccelerator::Kd);
	EXPECT_EQ(options.output, "folder with spaces\\image.hdr");
}

TEST(SceneCommandLine, RejectsMissingUnknownAndDuplicateOptions) {
	EXPECT_THROW(Parse({ "--scene" }), std::invalid_argument);
	EXPECT_THROW(Parse({ "--scene", "--out", "image.png" }), std::invalid_argument);
	EXPECT_THROW(Parse({ "--out", "image.png" }), std::invalid_argument);
	EXPECT_THROW(Parse({ "--scene", "mesh" }), std::invalid_argument);
	EXPECT_THROW(Parse({ "--scene", "missing", "--out", "image.png" }), std::invalid_argument);
	EXPECT_THROW(Parse({ "--scene", "mesh", "--out", "image.png", "--accel", "other" }), std::invalid_argument);
	EXPECT_THROW(Parse({ "--scene", "mesh", "--out", "image.png", "--unknown", "1" }), std::invalid_argument);
	EXPECT_THROW(Parse({ "--scene", "mesh", "--scene", "fog", "--out", "image.png" }), std::invalid_argument);
	EXPECT_THROW(Parse({ "--scene", "mesh", "--out", "" }), std::invalid_argument);
	EXPECT_THROW(Parse({ "--help", "--scene", "mesh" }), std::invalid_argument);
}

TEST(SceneCommandLine, RejectsInvalidNumbersAndPixelOverflow) {
	for (const auto* option : { "--width", "--height", "--spp", "--max-depth" }) {
		for (const auto* value : { "0", "-1", "+1", "1.5", "12junk", " 5", "2147483648" }) {
			SCOPED_TRACE(option);
			SCOPED_TRACE(value);
			EXPECT_THROW(Parse({ "--scene", "empty", "--out", "image.png", option, value }), std::invalid_argument);
		}
	}
	EXPECT_THROW(Parse({ "--scene", "empty", "--out", "image.png",
		"--width", "2147483647", "--height", "2" }), std::invalid_argument);
}

TEST(SceneCommandLine, OnlySupportedOutputFormatsAreAccepted) {
	for (const auto* file : { "a", "image.jpg", "image.PNG", "image.png.bak" }) {
		EXPECT_THROW(Parse({ "--scene", "empty", "--out", file }), std::invalid_argument);
	}
	for (const auto* file : { "image.png", "image.bmp", "image.hdr" }) {
		EXPECT_NO_THROW(Parse({ "--scene", "empty", "--out", file }));
	}
}

}
