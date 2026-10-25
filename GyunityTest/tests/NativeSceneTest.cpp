#include "stb/stb_image.h"
#include "../../GyunityApp/SceneCommandLine.h"
#include "json/single_include/nlohmann/json.hpp"
#include "integrator/BDPT.h"
#include "integrator/PT.h"
#include "sampler/SamplerEnum.h"
#include "gtest/gtest.h"
#include <cstdlib>

namespace {
using Json = nlohmann::json;

TEST(NativeScene, CornellSourceParametersAndGeometry) {
	const auto native = LoadNativeScene(SceneBuilder::Resource("scenes\\mitsuba\\cornell-box\\scene.json"));
	ASSERT_NE(native, nullptr);
	EXPECT_EQ(native->name, "CornellBoxStandard");
	EXPECT_EQ(native->width, 1024);
	EXPECT_EQ(native->height, 1024);
	EXPECT_EQ(native->filter, "tent");
	EXPECT_FLOAT_EQ(native->fovY, 19.5f);
	EXPECT_EQ(native->scene->GetPrimitives().size(), 36u);
	ASSERT_EQ(native->scene->GetLights().size(), 2u);
	for (const auto& light : native->scene->GetLights()) {
		EXPECT_EQ(light->Emission(), Vec3(17, 12, 4));
		real pdf;
		const auto point = light->GetShapePtr()->Sample(&pdf, Vec2(0.3f, 0.4f));
		EXPECT_LT(point.mNormal.y, -0.999f);
	}
	const auto camera = native->CreateCamera(std::make_shared<Film>(32, 32));
	EXPECT_EQ(camera->mPos, Vec3(0, 1, 6.8f));
	const Ray ray = camera->GenerateRay(16, 16, Vec2(0));
	EXPECT_TRUE(std::isfinite(ray.mDir.x));
	EXPECT_LT(ray.mDir.z, -0.99f);
	EXPECT_GT(camera->mCx.x, 0.99f);
}

TEST(NativeScene, FileAndNamedCliUseSourceDefaultsAndExplicitOverrides) {
	const char* defaults[] = { "app", "--scene", "CornellBoxStandard", "--out", "unused.hdr" };
	SceneCommandLineOptions originalCommand;
	ASSERT_TRUE(ParseSceneCommandLine(5, defaults, originalCommand));
	const auto& original = originalCommand.render;
	ASSERT_NE(original.nativeScene, nullptr);
	EXPECT_EQ(original.width, 1024);
	EXPECT_EQ(original.maxDepth, 65);
	const char* overrides[] = { "app", "--width", "64", "--height", "48", "--spp", "8", "--max-depth", "4",
		"--integrator", "bdpt", "--scene", "CornellBoxStandard", "--out", "unused.hdr" };
	SceneCommandLineOptions changedCommand;
	ASSERT_TRUE(ParseSceneCommandLine(15, overrides, changedCommand));
	const auto& changed = changedCommand.render;
	EXPECT_EQ(changed.width, 64);
	EXPECT_EQ(changed.height, 48);
	EXPECT_EQ(changed.spp, 8);
	EXPECT_EQ(changed.maxDepth, 4);
	EXPECT_EQ(changed.integrator, "bdpt");
	const char* file[] = { "app", "--scene-file", "..\\resources\\scenes\\native-demo\\scene.json", "--out", "unused.hdr" };
	SceneCommandLineOptions parsed;
	ASSERT_TRUE(ParseSceneCommandLine(5, file, parsed));
	ASSERT_NE(parsed.render.nativeScene, nullptr);
	EXPECT_EQ(parsed.render.nativeScene->name, "NativeDemo");
	const char* both[] = { "app", "--scene", "empty", "--scene-file", "anything.json", "--out", "unused.hdr" };
	EXPECT_FALSE(ParseSceneCommandLine(7, both, parsed));
	const char* fog[] = { "app", "--scene", "fog", "--integrator", "bdpt", "--out", "unused.hdr" };
	EXPECT_FALSE(ParseSceneCommandLine(7, fog, parsed));
}

TEST(NativeScene, TentFilterHasTheSourceSupportAndWeights) {
	TentFilter filter;
	EXPECT_EQ(filter.GetRadius(), 1);
	EXPECT_EQ(filter.Evaluate(0, 0), 1);
	EXPECT_EQ(filter.Evaluate(0.5f, -0.5f), 0.25f);
	EXPECT_EQ(filter.Evaluate(1, 0), 0);
	EXPECT_EQ(filter.Evaluate(2, 0), 0);
}

TEST(NativeBeckmann, NormalizationRawAlphaAndSamplingMoments) {
	for (const Vec2& alpha : { Vec2(0.01f), Vec2(0.05f), Vec2(0.1f), Vec2(0.25f), Vec2(0.08f, 0.2f) }) {
		BeckmannDistribution distribution(alpha.x, alpha.y);
		EXPECT_NEAR(distribution.D(Vec3(0, 0, 1)), 1 / (PI * alpha.x * alpha.y), 0.01f);
		EXPECT_EQ(distribution.G1(Vec3(0, 0, 1), Vec3(0, 0, 1)), 1);
		EXPECT_EQ(distribution.G1(Vec3(1, 0, 0), Vec3(0, 0, 1)), 0);
		double mass = 0, momentX = 0, momentY = 0;
		const int count = 160;
		for (int y = 0; y < count; ++y) {
			for (int x = 0; x < count; ++x) {
				const real sx = alpha.x * (12.f * (x + 0.5f) / count - 6);
				const real sy = alpha.y * (12.f * (y + 0.5f) / count - 6);
				const Vec3 normal = Vec3(sx, sy, 1).Norm();
				mass += distribution.D(normal) * std::pow(double(normal.z), 4) * (12 * alpha.x / count) * (12 * alpha.y / count);
				const Vec3 sample = distribution.SampleWh(Vec3(0, 0, 1), Vec2((x + 0.5f) / count, (y + 0.5f) / count));
				const real pdf = distribution.Pdf(Vec3(0, 0, 1), sample);
				ASSERT_TRUE(std::isfinite(pdf));
				EXPECT_GT(pdf, 0);
				momentX += (sample.x / sample.z) * (sample.x / sample.z);
				momentY += (sample.y / sample.z) * (sample.y / sample.z);
			}
		}
		EXPECT_NEAR(mass, 1, 1e-4);
		EXPECT_NEAR(momentX / (count * count), alpha.x * alpha.x / 2, alpha.x * alpha.x * 0.01);
		EXPECT_NEAR(momentY / (count * count), alpha.y * alpha.y / 2, alpha.y * alpha.y * 0.01);
	}
}

class NativeSceneFiles : public testing::Test {
protected:
	std::string directory;
	void SetUp() override {
		const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
		if (!output || !*output) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test.";
		directory = output;
	}
};

TEST_F(NativeSceneFiles, InvalidNativeDataFailsExplicitly) {
	std::ifstream source(SceneBuilder::Resource("scenes\\native-demo\\scene.json"));
	const Json original = Json::parse(source);
	const std::string filename = directory + "\\invalid-native.json";
	for (int variant = 0; variant < 8; ++variant) {
		Json data = original;
		switch (variant) {
		case 0: data["schema_version"] = 2; break;
		case 1: data["unknown"] = true; break;
		case 2: data["shapes"][0]["material"] = "missing"; break;
		case 3: data["render"]["width"] = 0; break;
		case 4: data["camera"]["to_world"] = Json::array({ 0 }); break;
		case 5: data["materials"]["white"]["two_sided"] = "false"; break;
		case 6: data["shapes"][1]["radius"] = -1; break;
		case 7: data["shapes"][1]["id"] = data["shapes"][0]["id"]; break;
		}
		{ std::ofstream file(filename); file << data.dump(); }
		EXPECT_EQ(LoadNativeScene(filename), nullptr) << variant;
	}
	{ std::ofstream file(filename); file << "{\"name\":\"first\",\"name\":\"second\"}"; }
	EXPECT_EQ(LoadNativeScene(filename), nullptr);
}

TEST_F(NativeSceneFiles, PythonAuthoredAndOriginalCornellRenderWithPtAndBdpt) {
	for (const char* file : { "scenes\\native-demo\\scene.json", "scenes\\mitsuba\\cornell-box\\scene.json" }) {
		for (auto accelerator : { SceneAccelerator::Bvh, SceneAccelerator::Kd }) {
			for (bool bdpt : { false, true }) {
				SceneRenderOptions options;
				options.nativeScene = LoadNativeScene(SceneBuilder::Resource(file));
				ASSERT_NE(options.nativeScene, nullptr);
				options.width = options.height = 32;
				options.spp = 8;
				options.maxDepth = 6;
				options.accelerator = accelerator;
				options.output = directory + '\\' + options.nativeScene->name + (bdpt ? "-bdpt-" : "-pt-") +
					(accelerator == SceneAccelerator::Bvh ? "bvh.hdr" : "kd.hdr");
				auto sampler = std::make_shared<RandomSampler>(123);
				if (bdpt) {
					BidirectionalPathTracing integrator(sampler, options.maxDepth, options.spp);
					ASSERT_TRUE(RenderScene(options, integrator));
				}
				else {
					PathTracing integrator(options.spp, options.maxDepth, sampler, std::make_shared<SamplerEnum>());
					ASSERT_TRUE(RenderScene(options, integrator));
				}
				int width, height, channels;
				std::unique_ptr<float, decltype(&stbi_image_free)> pixels(
					stbi_loadf(options.output.c_str(), &width, &height, &channels, 3), stbi_image_free);
				ASSERT_NE(pixels, nullptr);
				ASSERT_EQ(width, 32);
				ASSERT_EQ(height, 32);
				bool nonblack = false;
				for (int i = 0; i < width * height * 3; ++i) {
					ASSERT_TRUE(std::isfinite(pixels.get()[i]));
					EXPECT_GE(pixels.get()[i], 0);
					nonblack = nonblack || pixels.get()[i] > 0;
				}
				EXPECT_TRUE(nonblack);
			}
		}
	}
}

TEST_F(NativeSceneFiles, AddedMaterialsReuseTheExistingBsdfImplementations) {
	std::ifstream source(SceneBuilder::Resource("scenes\\native-demo\\scene.json"));
	const Json original = Json::parse(source, nullptr, false);
	ASSERT_FALSE(original.is_discarded());
	const std::vector<std::pair<Json, std::shared_ptr<Material>>> cases = {
		{Json{{"type","mirror"},{"reflection",{0.8,0.7,0.6}}}, SceneBuilder::Mirror(Vec3(0.8f,0.7f,0.6f))},
		{Json{{"type","rough_dielectric"},{"roughness",{0.1,0.3}},{"eta_inside",1.9},
			{"reflection",{0.8,0.7,0.6}},{"transmission",{0.6,0.7,0.8}}},
			SceneBuilder::RoughDielectric(Vec3(0.8f,0.7f,0.6f), Vec3(0.6f,0.7f,0.8f), 0.1f, 0.3f, 1.9f)},
		{Json{{"type","plastic"},{"diffuse",{0.2,0.7,0.3}},{"specular",{0.8,0.8,0.8}},{"roughness",0.2}},
			SceneBuilder::Plastic(Vec3(0.2f,0.7f,0.3f), Vec3(0.8f), 0.2f)}
	};
	for (const auto& entry : cases) {
		Json data = original;
		data["materials"]["white"] = entry.first;
		const auto filename = directory + "\\native-material-" + entry.first.at("type").get<std::string>() + ".json";
		{ std::ofstream file(filename); file << data.dump(); }
		const auto native = LoadNativeScene(filename);
		ASSERT_NE(native, nullptr);
		const auto actual = native->scene->GetPrimitives()[0]->GetMaterial();
		EXPECT_EQ(typeid(*actual), typeid(*entry.second));
		for (auto mode : {TransportMode::Radiance, TransportMode::Importance}) {
			Intersection hit, reference;
			hit.mNormal = hit.mGeometryNormal = Vec3(0,0,1);
			hit.mShadingDpDu = Vec3(1,0,0);
			hit.mShadingDpDv = Vec3(0,1,0);
			reference = hit;
			MemoryPool arena, referenceArena;
			actual->ComputeScatteringFunction(&hit, arena, mode);
			entry.second->ComputeScatteringFunction(&reference, referenceArena, mode);
			const Vec3 wo = Vec3(0.2f,0.1f,1).Norm();
			for (const Vec3& wi : {Vec3(0.3f,-0.1f,1).Norm(), Vec3(0.1f,0.2f,-1).Norm()}) {
				EXPECT_EQ(hit.mBsdf->Evaluate(wo, wi), reference.mBsdf->Evaluate(wo, wi));
				EXPECT_EQ(hit.mBsdf->Pdf(wo, wi), reference.mBsdf->Pdf(wo, wi));
			}
		}
	}
}

TEST_F(NativeSceneFiles, SppmJsonDefaultsAndExplicitBudgetsAreChecked) {
	std::ifstream source(SceneBuilder::Resource("scenes\\native-demo\\scene.json"));
	Json data = Json::parse(source, nullptr, false);
	ASSERT_FALSE(data.is_discarded());
	data["render"]["integrator"] = "sppm";
	data["render"]["spp"] = 13;
	data["render"]["sppm"] = {{"iterations", 2}, {"photons_per_stage", 512}};
	const auto filename = directory + "\\native-sppm.json";
	{ std::ofstream file(filename); file << data.dump(); }
	const auto native = LoadNativeScene(filename);
	ASSERT_NE(native, nullptr);
	EXPECT_EQ(native->integrator, "sppm");
	EXPECT_EQ(native->spp, 13);
	EXPECT_EQ(native->sppm.iterations, 2);
	EXPECT_EQ(native->sppm.photonsPerStage, 512);
	SceneCommandLineOptions parsed;
	const char* arguments[] = {"app", "--scene-file", filename.c_str(), "--iterations", "3", "--out", "unused.png"};
	ASSERT_TRUE(ParseSceneCommandLine(7, arguments, parsed));
	EXPECT_EQ(parsed.render.sppm.iterations, 3);
	EXPECT_EQ(parsed.render.sppm.photonsPerStage, 512);
	for (const auto& invalid : std::vector<Json>{
		{{"iterations", 0}}, {{"photons_per_stage", -1}}, {{"iterations", true}},
		{{"photons_per_stage", 2147483648LL}}, {{"spp", 4}}}) {
		data["render"]["sppm"] = invalid;
		{ std::ofstream file(filename); file << data.dump(); }
		EXPECT_EQ(LoadNativeScene(filename), nullptr);
	}
	data["render"].erase("sppm");
	{ std::ofstream file(filename); file << data.dump(); }
	const auto defaults = LoadNativeScene(filename);
	ASSERT_NE(defaults, nullptr);
	EXPECT_EQ(defaults->sppm.iterations, 256);
	EXPECT_EQ(defaults->sppm.photonsPerStage, 300000);
}

TEST_F(NativeSceneFiles, AddedMaterialParametersRejectInvalidAndUnsupportedValues) {
	std::ifstream source(SceneBuilder::Resource("scenes\\native-demo\\scene.json"));
	const Json original = Json::parse(source, nullptr, false);
	ASSERT_FALSE(original.is_discarded());
	const std::vector<Json> invalid = {
		{{"type","mirror"},{"reflection",{2,1,1}}},
		{{"type","mirror"},{"roughness",0.1}},
		{{"type","rough_dielectric"},{"roughness",{0,0.1}}},
		{{"type","rough_dielectric"},{"roughness",{0.1,1.1}}},
		{{"type","rough_dielectric"},{"roughness",{0.1}}},
		{{"type","rough_dielectric"},{"roughness",{0.1,0.1}},{"eta_inside",0}},
		{{"type","rough_dielectric"},{"roughness",{0.1,0.1}},{"eta_outside",1.3}},
		{{"type","plastic"},{"diffuse",{0.2,0.2,0.2}},{"roughness",0}},
		{{"type","plastic"},{"diffuse",{0.2,0.2,0.2}},{"roughness",1.1}},
		{{"type","plastic"},{"diffuse",{0.2,0.2,0.2}},{"roughness",true}},
		{{"type","plastic"},{"diffuse",{0.2,0.2,0.2}},{"roughness",0.1},{"distribution","beckmann"}}
	};
	const auto filename = directory + "\\invalid-native-material.json";
	for (const auto& material : invalid) {
		Json data = original;
		data["materials"]["white"] = material;
		{ std::ofstream file(filename); file << data.dump(); }
		EXPECT_EQ(LoadNativeScene(filename), nullptr) << material.dump();
	}
}
}
