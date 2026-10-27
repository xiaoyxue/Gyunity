#include "gtest/gtest.h"
#include "integrator/BDPT.h"
#include "accelerator/BruteForceAccel.h"
#include "camera/PinholeCamera.h"
#include "light/EnvLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Mirror.h"
#include "material/Glass.h"
#include "texture/ConstantTexture.h"
#include "image/ImageIO.h"
#include "../../GyunityApp/scenes/CornellBoxScenes.h"
#include <array>

using namespace Gyunity;

namespace {

class EnvironmentBDPT : public testing::Test {
protected:
	std::shared_ptr<Scene> scene;
	std::shared_ptr<EnvironmentLight> environment;
	std::shared_ptr<Camera> camera;
	MemoryPool arena;
	std::string output;

	void SetUp() override {
		const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
		if (!directory) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main repository Test folder.";
		output = directory;
		const std::string filename = output + "\\bdpt-constant-environment.hdr";
		ASSERT_TRUE(ImageIO::WriteImage(filename, std::vector<Vec3>(16, Vec3(2)), 4, 4));
		environment = std::make_shared<EnvironmentLight>(filename, 0);
		BuildScene(std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))));
	}

	void BuildScene(const std::shared_ptr<Material>& material) {
		scene = std::make_shared<Scene>();
		auto plane = std::make_shared<Triangle>(Transform(), Transform(),
			Vec3(-10, -10, 0), Vec3(10, -10, 0), Vec3(0, 10, 0), Vec3(0, 0, 1));
		scene->AddPrimitive(std::make_shared<Primitive>(plane, material));
		scene->AddLight(environment);
		Initialize();
		camera = std::make_shared<PinHoleCamera>(std::make_shared<Film>(16, 16),
			Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
	}

	void Initialize() {
		scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
		scene->Initialize();
	}

	void BuildPaths(int s, std::vector<PathVertex>* light, std::vector<PathVertex>* eye) {
		std::array<PathVertex, 3> path;
		path[0].mType = PathVertexType::Environment;
		path[0].mLight = environment.get();
		path[0].mDirection = Vec3(0, 0, 1);
		path[0].mIsect.mPos = Vec3(0, 0, 100);
		path[0].mThroughput = Vec3(2);
		path[1].mIsect.mPos = Vec3(0);
		path[1].mIsect.mNormal = path[1].mIsect.mGeometryNormal = Vec3(0, 0, 1);
		path[1].mIsect.mShadingDpDu = Vec3(1, 0, 0);
		path[1].mIsect.mShadingDpDv = Vec3(0, 1, 0);
		path[1].mIsect.mOutDir = Vec3(0, 0, 1);
		DiffuseMaterial(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))).ComputeScatteringFunction(&path[1].mIsect, arena);
		path[1].mThroughput = Vec3(1);
		path[2].mType = PathVertexType::Camera;
		path[2].mCamera = camera.get();
		path[2].mIsect.mPos = camera->mPos;
		path[2].mThroughput = Vec3(1);
		const real rowAverage = (std::sin(PI / 8) + std::sin(3 * PI / 8)) / 2;
		const real environmentPdf = (std::sin(5 * PI / 8) / rowAverage) / (2 * PI * PI);
		real selectionPdf;
		ASSERT_TRUE(scene->LightSelectionPdf(0, &selectionPdf));
		const std::array<real, 3> f = { selectionPdf * environmentPdf, 1 / (200 * PI), 1 / (9 * PI) };
		const std::array<real, 3> r = { INV_PI, camera->PdfDir(Ray(camera->mPos, Vec3(0, 0, -1))) / 9, 1 };
		for (int i = 0; i < s; ++i) {
			path[i].mPdfFwd = f[i];
			path[i].mPdfPrev = r[i];
			light->push_back(path[i]);
		}
		for (int i = 2; i >= s; --i) {
			path[i].mPdfFwd = r[i];
			path[i].mPdfPrev = f[i];
			eye->push_back(path[i]);
		}
	}

	void CheckWeightProducts() {
		const double average = (std::sin(double(PI) / 8) + std::sin(3 * double(PI) / 8)) / 2;
		const double mapPdf = (std::sin(5 * double(PI) / 8) / average) / (2 * double(PI) * PI);
		real pmf;
		ASSERT_TRUE(scene->LightSelectionPdf(0, &pmf));
		const double cameraPdf = camera->PdfDir(Ray(camera->mPos, Vec3(0, 0, -1))) / 9;
		const double products[] = { INV_PI * cameraPdf, pmf * mapPdf * cameraPdf, pmf * mapPdf / (200 * PI) };
		const double total = products[0] + products[1] + products[2];
		double sum = 0;
		for (int s = 0; s < 3; ++s) {
			std::vector<PathVertex> light, eye;
			BuildPaths(s, &light, &eye);
			real weight;
			ASSERT_TRUE(MisWeight(*scene, light, eye, s, 3 - s, &weight));
			EXPECT_NEAR(weight, products[s] / total, 1e-6);
			sum += weight;
		}
		EXPECT_NEAR(sum, 1, 1e-6);
	}

	double RenderMean(const std::string& name, int spp, real fovy = 53.13010235415597f) {
		auto film = std::make_shared<Film>(16, 16);
		PinHoleCamera renderCamera(film, Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), fovy, 1);
		BidirectionalPathTracing integrator(std::make_shared<RandomSampler>(123), 1, spp);
		if (!integrator.RenderChecked(*scene, renderCamera)) { ADD_FAILURE() << "BDPT render failed."; return -1; }
		const std::string filename = output + '\\' + name + ".hdr";
		film->SetFileName(filename);
		if (!film->SaveImage()) { ADD_FAILURE() << "Cannot save BDPT numerical render."; return -1; }
		std::vector<Vec3> image;
		int width = 0, height = 0;
		if (!ImageIO::LoadImage(filename, image, width, height)) { ADD_FAILURE() << "Cannot load numerical render."; return -1; }
		double sum = 0;
		for (const auto& pixel : image) {
			if (!std::isfinite(pixel.x)) { ADD_FAILURE() << "Nonfinite radiance."; return -1; }
			sum += pixel.x;
		}
		return sum / image.size();
	}
};

}

TEST_F(EnvironmentBDPT, WeightsMatchDirectionalAndProjectedDiskProducts) {
	CheckWeightProducts();
}

TEST_F(EnvironmentBDPT, MixedLightSelectionPmfIsIncludedExactlyOnce) {
	scene->AddLight(std::make_shared<PointLight>(Vec3(1, 2, 3), Vec3(37)));
	Initialize();
	CheckWeightProducts();
}

TEST_F(EnvironmentBDPT, PrimaryStrategiesAreIndependentOfMisDensities) {
	const real average = (std::sin(PI / 8) + std::sin(3 * PI / 8)) / 2;
	const real mapPdf = (std::sin(5 * PI / 8) / average) / (2 * PI * PI);
	const std::array<real, 3> expected = { 4, 2 / (PI * mapPdf), 1 / (9 * PI) };
	for (int s = 0; s < 3; ++s) {
		SCOPED_TRACE(s);
		std::vector<PathVertex> light, eye;
		BuildPaths(s, &light, &eye);
		Vec3 primary, contribution;
		Vec2 raster, primaryRaster;
		real weight;
		ASSERT_TRUE(EvaluateConnection(*scene, *camera, light, eye, s, 3 - s, &primary, &primaryRaster));
		EXPECT_NEAR(primary.x, expected[s], 1e-5f);
		ASSERT_TRUE(MisWeight(*scene, light, eye, s, 3 - s, &weight));
		ASSERT_TRUE(ConnectPaths(*scene, *camera, light, eye, s, 3 - s, &contribution, &raster));
		EXPECT_EQ(contribution, primary * weight);
		eye[0].mPdfFwd = -1;
		ASSERT_TRUE(EvaluateConnection(*scene, *camera, light, eye, s, 3 - s, &contribution, &raster));
		EXPECT_EQ(contribution, primary);
		EXPECT_FALSE(ConnectPaths(*scene, *camera, light, eye, s, 3 - s, &contribution, &raster));
		EXPECT_EQ(contribution, Vec3(0));
		EXPECT_EQ(raster.x, primaryRaster.x);
		EXPECT_EQ(raster.y, primaryRaster.y);
		EXPECT_EQ(eye[0].mPdfFwd, -1);
		const auto& endpoint = s == 0 ? eye.back() : light.front();
		EXPECT_EQ(endpoint.mDirection, Vec3(0, 0, 1));
		EXPECT_EQ(endpoint.mIsect.mPos, Vec3(0, 0, 100));
	}
}

TEST_F(EnvironmentBDPT, SharedHelpersPreserveEnvironmentDirectionAndLaunchMeasures) {
	scene->AddLight(std::make_shared<PointLight>(Vec3(1, 2, 3), Vec3(37)));
	Initialize();
	std::vector<PathVertex> light, eye;
	BuildPaths(1, &light, &eye);
	const real average = (std::sin(PI / 8) + std::sin(3 * PI / 8)) / 2;
	const real mapPdf = (std::sin(5 * PI / 8) / average) / (2 * PI * PI);
	real pmf, originPdf, surfacePdf;
	ASSERT_TRUE(scene->LightSelectionPdf(0, &pmf));
	ASSERT_TRUE(Bidirectional::LightOriginPdf(*scene, light[0], eye.back(), &originPdf));
	ASSERT_TRUE(Bidirectional::PdfToward(light[0], nullptr, eye.back(), &surfacePdf));
	EXPECT_NEAR(originPdf, pmf * mapPdf, 1e-7f);
	EXPECT_NEAR(surfacePdf, 1 / (200 * PI), 1e-8f);
	Vec3 dir, primary, distantPrimary;
	Vec2 raster;
	ASSERT_TRUE(Bidirectional::Direction(light[0], eye.back(), &dir));
	EXPECT_EQ(dir, Vec3(0, 0, -1));
	ASSERT_TRUE(Bidirectional::Direction(eye.back(), light[0], &dir));
	EXPECT_EQ(dir, Vec3(0, 0, 1));
	EXPECT_TRUE(Bidirectional::ValidConnection(&light[0], eye.back(), 1, 2));
	EXPECT_TRUE(Bidirectional::ValidConnection(nullptr, light[0], 0, 2));
	EXPECT_FALSE(Bidirectional::ValidConnection(&light[0], light[0], 1, 2));
	ASSERT_TRUE(EvaluateConnection(*scene, *camera, light, eye, 1, 2, &primary, &raster));
	EXPECT_NEAR(primary.x, 2 / (pmf * mapPdf * PI), 1e-4f);
	light[0].mIsect.mPos = Vec3(0, 0, 100000);
	ASSERT_TRUE(EvaluateConnection(*scene, *camera, light, eye, 1, 2, &distantPrimary, &raster));
	EXPECT_EQ(primary, distantPrimary);
	EXPECT_FALSE(Bidirectional::Direction(light[0], light[0], &dir));
	light[0].mDirection = Vec3(0, 0, 2);
	EXPECT_FALSE(Bidirectional::Direction(light[0], eye.back(), &dir));
}

TEST_F(EnvironmentBDPT, BlockedEnvironmentConnectionSkipsMis) {
	scene->AddPrimitive(std::make_shared<Primitive>(
		std::make_shared<Sphere>(Transform(), Transform(), 0.2f, Vec3(0, 0, 1))));
	Initialize();
	std::vector<PathVertex> light, eye;
	BuildPaths(1, &light, &eye);
	eye[0].mPdfFwd = -1;
	for (auto evaluate : { &EvaluateConnection, &ConnectPaths }) {
		Vec3 contribution(7);
		Vec2 raster(7);
		ASSERT_TRUE(evaluate(*scene, *camera, light, eye, 1, 2, &contribution, &raster));
		EXPECT_EQ(contribution, Vec3(0));
		EXPECT_EQ(raster.x, 0);
		EXPECT_EQ(raster.y, 0);
	}
}

TEST_F(EnvironmentBDPT, DirectConnectionDoesNotDependOnHelperEndpointDistance) {
	std::vector<PathVertex> light, eye;
	BuildPaths(1, &light, &eye);
	Vec3 nearValue, farValue;
	Vec2 raster;
	ASSERT_TRUE(ConnectPaths(*scene, *camera, light, eye, 1, 2, &nearValue, &raster));
	light[0].mIsect.mPos = Vec3(0, 0, 100000);
	ASSERT_TRUE(ConnectPaths(*scene, *camera, light, eye, 1, 2, &farValue, &raster));
	EXPECT_GT(nearValue.x, 0);
	EXPECT_EQ(nearValue, farValue);
}

TEST_F(EnvironmentBDPT, BackgroundHasUnitWeightAndCannotBeAConnectionSurface) {
	std::vector<PathVertex> light, eye;
	BuildPaths(1, &light, &eye);
	std::vector<PathVertex> escaped = { eye[0], light[0] };
	escaped[1].mThroughput = Vec3(1);
	Vec3 contribution;
	Vec2 raster;
	ASSERT_TRUE(ConnectPaths(*scene, *camera, light, escaped, 0, 2, &contribution, &raster));
	EXPECT_EQ(contribution, Vec3(2));
	ASSERT_TRUE(ConnectPaths(*scene, *camera, light, escaped, 1, 2, &contribution, &raster));
	EXPECT_EQ(contribution, Vec3(0));
	ASSERT_TRUE(EvaluateConnection(*scene, *camera, light, escaped, 0, 2, &contribution, &raster));
	EXPECT_EQ(contribution, Vec3(2));
	ASSERT_TRUE(EvaluateConnection(*scene, *camera, light, escaped, 1, 2, &contribution, &raster));
	EXPECT_EQ(contribution, Vec3(0));
}

TEST_F(EnvironmentBDPT, ConstantEnvironmentDiffuseMatchesAnalyticRadiance) {
	const double mean = RenderMean("bdpt-diffuse-environment", 256);
	RecordProperty("mean_radiance", std::to_string(mean));
	EXPECT_NEAR(mean, 2, 0.08);
}

TEST_F(EnvironmentBDPT, ConstantEnvironmentMirrorAndGlassRespectTransport) {
	auto white = std::make_shared<ConstantTexture<Vec3>>(Vec3(1));
	BuildScene(std::make_shared<MirrorMaterial>(white));
	EXPECT_NEAR(RenderMean("bdpt-mirror-environment", 4), 2, 1e-5);
	BuildScene(std::make_shared<GlassMaterial>(white, white));
	const double expected = 2 * (0.04 + 0.96 / (1.5 * 1.5));
	const double mean = RenderMean("bdpt-glass-environment", 128, 0.1f);
	RecordProperty("glass_mean_radiance", std::to_string(mean));
	EXPECT_NEAR(mean, expected, 0.02);
}

TEST_F(EnvironmentBDPT, EnvironmentAndPointLightMatchAnalyticSum) {
	scene->AddLight(std::make_shared<PointLight>(Vec3(0, 0, 2), Vec3(8)));
	Initialize();
	double direct = 0;
	for (int y = 0; y < 128; ++y) {
		for (int x = 0; x < 128; ++x) {
			const double px = 3 * ((x + 0.5) / 128 - 0.5);
			const double py = 3 * ((y + 0.5) / 128 - 0.5);
			const double distance2 = px * px + py * py + 4;
			direct += 16 / (double(PI) * std::pow(distance2, 1.5));
		}
	}
	const double expected = 2 + direct / (128 * 128);
	const double mean = RenderMean("bdpt-mixed-environment-point", 256);
	RecordProperty("mixed_mean_radiance", std::to_string(mean));
	RecordProperty("mixed_expected_radiance", std::to_string(expected));
	EXPECT_NEAR(mean, expected, 0.1);
}

TEST(BDPTSceneComposition, LucyAndPurpleBunnyFitInsideTheCornellBox) {
	for (const auto* path : { "meshs\\Alucy.obj", "meshs\\bunny3.obj" }) {
		ASSERT_TRUE(std::ifstream(SceneBuilder::Resource(path)).good()) << "Run from GyunityApp.";
	}
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxScenes::RoughGlassLucyAndPurpleBunny(builder);
	ASSERT_EQ(scene->GetLights().size(), 2u);
	EXPECT_EQ(scene->GetEnvironmentLight(), nullptr);
	for (const auto& light : scene->GetLights()) EXPECT_TRUE(light->IsAreaLight());
	for (const auto& primitive : scene->GetPrimitives()) {
		const AABB bound = primitive->WorldBound();
		for (int axis = 0; axis < 3; ++axis) {
			ASSERT_GE(bound.minPoint[axis], -1.00001f);
			ASSERT_LE(bound.maxPoint[axis], 1.00001f);
		}
	}
}
