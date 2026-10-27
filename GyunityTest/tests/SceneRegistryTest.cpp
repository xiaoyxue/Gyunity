#include "../../GyunityApp/SceneCommandLine.h"
#include "gtest/gtest.h"
#include <set>

namespace {

bool ParseInto(std::initializer_list<const char*> arguments, SceneCommandLineOptions& output) {
	std::vector<const char*> argv = { "GyunityApp" };
	argv.insert(argv.end(), arguments.begin(), arguments.end());
	return ParseSceneCommandLine(static_cast<int>(argv.size()), argv.data(), output);
}

SceneCommandLineOptions Parse(std::initializer_list<const char*> arguments) {
	SceneCommandLineOptions output;
	EXPECT_TRUE(ParseInto(arguments, output));
	return output;
}

bool Parses(std::initializer_list<const char*> arguments) {
	SceneCommandLineOptions output;
	return ParseInto(arguments, output);
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
		{ "CornellBoxStandard", nullptr }, { "veach-mis", nullptr }, { "veach-bidir", nullptr },
		{ "matpreview-diffuse", nullptr }, { "matpreview-glass", nullptr }, { "matpreview-copper", nullptr },
		{ "matpreview-checkerboard", nullptr }, { "matpreview-mirror", nullptr }, { "matpreview-rough-glass", nullptr },
		{ "matpreview-copper-beckmann", nullptr }, { "matpreview-copper-anisotropic", nullptr }, { "matpreview-plastic", nullptr }
	};
	ASSERT_EQ(GetSceneRegistry().size(), sizeof(expected) / sizeof(expected[0]));
	std::set<std::string> names;
	for (const auto& scene : GetSceneRegistry()) EXPECT_TRUE(names.insert(scene.name).second);
	for (const auto& entry : expected) {
		const auto* scene = FindScene(entry.first);
		ASSERT_NE(scene, nullptr);
		EXPECT_EQ(scene->build, entry.second);
		EXPECT_EQ(scene->usesVolume, std::string(entry.first) == "fog");
		if (std::string(entry.first).find("matpreview-") == 0) {
			ASSERT_NE(scene->preparation, nullptr);
			EXPECT_NE(std::string(scene->preparation).find("--profile all"), std::string::npos);
			EXPECT_EQ(std::string(scene->file), "scenes\\matpreview\\" + std::string(scene->name) + ".json");
		}
	}
	EXPECT_EQ(FindScene("not-a-scene"), nullptr);
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

TEST(SceneRegistry, MissingLocalAssetsExplainHowToPrepareThem) {
	auto description = *FindScene("matpreview-copper");
	description.file = "scenes\\matpreview\\__missing-local-preview-fixture__.json";
	SceneRenderOptions options;
	options.scene = &description;
	testing::internal::CaptureStderr();
	EXPECT_FALSE(LoadSceneDefaults(options));
	const auto message = testing::internal::GetCapturedStderr();
	EXPECT_NE(message.find("Cannot read local scene"), std::string::npos);
	EXPECT_NE(message.find("--profile all"), std::string::npos);
	EXPECT_EQ(options.nativeScene, nullptr);
}

TEST(SceneCommandLine, InformationalCommandsDoNotRequireSceneOrOutput) {
	EXPECT_EQ(Parse({ "--help" }).command, SceneCommand::Help);
	EXPECT_EQ(Parse({ "--list-scenes" }).command, SceneCommand::List);
}

TEST(SceneCommandLine, RenderDefaultsMatchRegressionSettings) {
	const auto parsed = Parse({ "--scene", "mesh", "--out", "image.png" });
	EXPECT_EQ(parsed.command, SceneCommand::Render);
	const auto& options = parsed.render;
	EXPECT_EQ(options.scene, FindScene("mesh"));
	EXPECT_EQ(options.accelerator, SceneAccelerator::Bvh);
	EXPECT_EQ(options.width, 512);
	EXPECT_EQ(options.height, 512);
	EXPECT_EQ(options.spp, 64);
	EXPECT_EQ(options.maxDepth, 8);
	EXPECT_EQ(options.output, "image.png");
}

TEST(SceneCommandLine, OverridesAreParsedIndependentlyOfOrder) {
	const auto options = Parse({ "--out", "folder with spaces\\image.hdr", "--height", "360",
		"--spp", "2", "--max-depth", "4", "--accel", "kd", "--width", "640", "--scene", "fog" }).render;
	EXPECT_EQ(options.scene, FindScene("fog"));
	EXPECT_EQ(options.width, 640);
	EXPECT_EQ(options.height, 360);
	EXPECT_EQ(options.spp, 2);
	EXPECT_EQ(options.maxDepth, 4);
	EXPECT_EQ(options.accelerator, SceneAccelerator::Kd);
	EXPECT_EQ(options.output, "folder with spaces\\image.hdr");
}

TEST(SceneCommandLine, VeachBidirDefaultsToBdptButAllowsAnExplicitPathOverride) {
	const auto defaults = Parse({"--scene", "veach-bidir", "--out", "unused.png"});
	EXPECT_EQ(defaults.render.integrator, "bdpt");
	const auto path = Parse({"--scene", "veach-bidir", "--integrator", "path", "--out", "unused.png"});
	EXPECT_EQ(path.render.integrator, "path");
	const auto file = Parse({"--scene-file", "..\\resources\\scenes\\mitsuba\\veach-bidir\\scene.json", "--out", "unused.png"});
	EXPECT_EQ(file.render.integrator, "bdpt");
}

TEST(SceneCommandLine, ShadingNormalsRemainAnExplicitBdptOption) {
	const auto defaults = Parse({ "--scene", "empty", "--integrator", "bdpt", "--out", "unused.exr" });
	EXPECT_FALSE(defaults.render.bdptShadingNormals);
	const auto shading = Parse({ "--scene", "empty", "--integrator", "bdpt",
		"--bdpt-normals", "shading", "--out", "unused.exr" });
	EXPECT_TRUE(shading.render.bdptShadingNormals);
	const auto geometric = Parse({ "--scene", "empty", "--integrator", "bdpt",
		"--bdpt-normals", "geometric", "--out", "unused.exr" });
	EXPECT_FALSE(geometric.render.bdptShadingNormals);
	EXPECT_FALSE(Parses({ "--scene", "empty", "--integrator", "path",
		"--bdpt-normals", "shading", "--out", "unused.png" }));
}

TEST(SceneCommandLine, ExistingDisplayOperatorsKeepTheirNamesAndSettings) {
	for (const auto& entry : {
		std::make_pair("none", Gyunity::ToneMapOperator::None),
		std::make_pair("reinhard", Gyunity::ToneMapOperator::Reinhard),
		std::make_pair("aces", Gyunity::ToneMapOperator::AcesFilmic) }) {
		const auto converted = Parse({ "--convert", "unused.exr", "--tonemap", entry.first,
			"--max-luminance", "4", "--tonemap-scale", "2", "--out", "unused.png" });
		EXPECT_EQ(converted.command, SceneCommand::Convert);
		EXPECT_EQ(converted.render.toneMap.op, entry.second);
		EXPECT_EQ(converted.render.toneMap.maxLuminance, 4);
		EXPECT_EQ(converted.render.toneMap.scale, 2);
	}
	EXPECT_FALSE(Parses({ "--convert", "unused.exr", "--tonemap", "filmic", "--out", "unused.png" }));
	EXPECT_FALSE(Parses({ "--scene", "empty", "--tonemap", "aces", "--out", "unused.exr" }));
}

TEST(SceneCommandLine, SppmBudgetsAreSeparateAndOrderIndependent) {
	const auto options = Parse({"--iterations", "2", "--photons-per-stage", "512",
		"--scene", "empty", "--integrator", "sppm", "--out", "unused.png"}).render;
	EXPECT_EQ(options.integrator, "sppm");
	EXPECT_EQ(options.sppm.iterations, 2);
	EXPECT_EQ(options.sppm.photonsPerStage, 512);
	EXPECT_EQ(options.spp, 64);
	const auto defaults = Parse({"--scene", "CornellBoxStandard", "--integrator", "sppm", "--out", "unused.png"}).render;
	EXPECT_EQ(defaults.sppm.iterations, 256);
	EXPECT_EQ(defaults.sppm.photonsPerStage, 300000);
	EXPECT_FALSE(Parses({"--scene", "empty", "--integrator", "sppm", "--spp", "4", "--out", "unused.png"}));
	EXPECT_FALSE(Parses({"--scene", "empty", "--iterations", "2", "--out", "unused.png"}));
	EXPECT_FALSE(Parses({"--scene", "empty", "--integrator", "bdpt", "--photons-per-stage", "512", "--out", "unused.png"}));
	EXPECT_FALSE(Parses({"--scene", "fog", "--integrator", "sppm", "--out", "unused.png"}));
	for (const auto* option : {"--iterations", "--photons-per-stage"}) {
		for (const auto* value : {"0", "-1", "1.5", "2147483648"})
			EXPECT_FALSE(Parses({"--scene", "empty", "--integrator", "sppm", option, value, "--out", "unused.png"}));
	}
}

TEST(SceneCommandLine, RejectsMissingUnknownAndDuplicateOptions) {
	EXPECT_FALSE(Parses({ "--scene" }));
	EXPECT_FALSE(Parses({ "--scene", "--out", "image.png" }));
	EXPECT_FALSE(Parses({ "--out", "image.png" }));
	EXPECT_FALSE(Parses({ "--scene", "mesh" }));
	EXPECT_FALSE(Parses({ "--scene", "missing", "--out", "image.png" }));
	EXPECT_FALSE(Parses({ "--scene", "mesh", "--out", "image.png", "--accel", "other" }));
	EXPECT_FALSE(Parses({ "--scene", "mesh", "--out", "image.png", "--unknown", "1" }));
	EXPECT_FALSE(Parses({ "--scene", "mesh", "--scene", "fog", "--out", "image.png" }));
	EXPECT_FALSE(Parses({ "--scene", "mesh", "--out", "" }));
	EXPECT_FALSE(Parses({ "--help", "--scene", "mesh" }));
}

TEST(SceneCommandLine, RejectsInvalidNumbersAndPixelOverflow) {
	for (const auto* option : { "--width", "--height", "--spp", "--max-depth" }) {
		for (const auto* value : { "0", "-1", "+1", "1.5", "12junk", " 5", "2147483648" }) {
			SCOPED_TRACE(option);
			SCOPED_TRACE(value);
			EXPECT_FALSE(Parses({ "--scene", "empty", "--out", "image.png", option, value }));
		}
	}
	EXPECT_FALSE(Parses({ "--scene", "empty", "--out", "image.png",
		"--width", "2147483647", "--height", "2" }));
}

TEST(SceneCommandLine, OnlySupportedOutputFormatsAreAccepted) {
	for (const auto* file : { "a", "image.jpg", "image.PNG", "image.png.bak" }) {
		EXPECT_FALSE(Parses({ "--scene", "empty", "--out", file }));
	}
	for (const auto* file : { "image.png", "image.bmp", "image.hdr", "image.exr" }) {
		EXPECT_TRUE(Parses({ "--scene", "empty", "--out", file }));
	}
}

}
