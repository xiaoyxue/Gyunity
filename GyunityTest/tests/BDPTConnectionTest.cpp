#include "gtest/gtest.h"
#include "integrator/BDPT.h"
#include "integrator/PT.h"
#include "accelerator/BruteForceAccel.h"
#include "camera/PinholeCamera.h"
#include "light/AreaLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Mirror.h"
#include "sampler/SamplerEnum.h"
#include "texture/ConstantTexture.h"
#include "image/ImageIO.h"
#include "../../GyunityApp/scenes/SceneBuilder.h"
#include <array>

using namespace Gyunity;

namespace {

class FiniteBDPT : public testing::Test {
protected:
	std::shared_ptr<Scene> scene;
	std::shared_ptr<Camera> camera;
	MemoryPool arena;
	std::array<PathVertex, 3> full;
	std::array<real, 3> forward, reverse;

	void SetUp() override {
		scene = std::make_shared<Scene>();
		camera = std::make_shared<PinHoleCamera>(std::make_shared<Film>(16, 16),
			Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
		auto shape = std::make_shared<Triangle>(Transform(), Transform(),
			Vec3(0, -1, 2), Vec3(2, -1, 2), Vec3(1, 1, 2), Vec3(0, 0, -1));
		shape->mShapeId = 0;
		auto light = std::make_shared<AreaLight>(shape, Vec3(1));
		scene->AddLight(light);
		scene->SetAccelerator(std::make_shared<BruteForce>());
		scene->Initialize();
		full[0].mType = PathVertexType::Light;
		full[0].mLight = light.get();
		full[0].mIsect.mPos = Vec3(1, 0, 2);
		full[0].mIsect.mNormal = full[0].mIsect.mGeometryNormal = Vec3(0, 0, -1);
		full[0].mThroughput = Vec3(1);
		full[1].mIsect.mPos = Vec3(0);
		full[1].mIsect.mNormal = full[1].mIsect.mGeometryNormal = Vec3(0, 0, 1);
		full[1].mIsect.mShadingDpDu = Vec3(1, 0, 0);
		full[1].mIsect.mShadingDpDv = Vec3(0, 1, 0);
		DiffuseMaterial(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))).ComputeScatteringFunction(
			&full[1].mIsect, arena);
		full[1].mThroughput = Vec3(1);
		full[2].mType = PathVertexType::Camera;
		full[2].mCamera = camera.get();
		full[2].mIsect.mPos = camera->mPos;
		full[2].mThroughput = Vec3(1);
		const real cameraPdf = camera->PdfDir(Ray(camera->mPos, Vec3(0, 0, -1)));
		// Independent products: emitter area=2, light distance²=5, camera distance²=9.
		forward = { 0.5f, 4.f / (25 * PI), 1.f / (9 * PI) };
		reverse = { 4.f / (25 * PI), cameraPdf / 9, 1 };
	}

	void Split(int s, std::vector<PathVertex>* lightPath, std::vector<PathVertex>* cameraPath) {
		lightPath->clear();
		cameraPath->clear();
		for (int i = 0; i < s; ++i) {
			auto vertex = full[i];
			vertex.mPdfFwd = forward[i];
			vertex.mPdfPrev = reverse[i];
			if (i > 0) vertex.mIsect.mOutDir = (full[i - 1].mIsect.mPos - vertex.mIsect.mPos).Norm();
			lightPath->push_back(vertex);
		}
		for (int i = 2; i >= s; --i) {
			auto vertex = full[i];
			vertex.mPdfFwd = reverse[i];
			vertex.mPdfPrev = forward[i];
			if (i < 2) vertex.mIsect.mOutDir = (full[i + 1].mIsect.mPos - vertex.mIsect.mPos).Norm();
			cameraPath->push_back(vertex);
		}
	}

	void CheckMixedChain(bool sampledDelta, int split) {
		auto chainCamera = std::make_shared<PinHoleCamera>(std::make_shared<Film>(16, 16),
			Vec3(1, 0, -1), Vec3(1, 0, 1), Vec3(0, 1, 0), 53.13010235415597f, 1);
		std::array<PathVertex, 4> chain = { full[0], full[1], full[1], full[2] };
		chain[0].mIsect.mPos = Vec3(-1, 0, 1);
		chain[0].mIsect.mNormal = chain[0].mIsect.mGeometryNormal = Vec3(1, 0, -1).Norm();
		chain[1].mIsect.mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(chain[1].mIsect);
		chain[1].mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(Vec3(0.5f)));
		chain[1].mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(0.5f)));
		ASSERT_TRUE(chain[1].mIsect.mBsdf->HasMixedDeltaLobes());
		chain[1].mIsDelta = sampledDelta;
		chain[2].mIsect.mPos = Vec3(1, 0, 1);
		chain[2].mIsect.mNormal = chain[2].mIsect.mGeometryNormal = Vec3(0, 0, -1);
		chain[2].mIsect.mShadingDpDv = Vec3(0, -1, 0);
		DiffuseMaterial(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))).ComputeScatteringFunction(
			&chain[2].mIsect, arena);
		chain[3].mIsect.mPos = chainCamera->mPos;
		chain[3].mCamera = chainCamera.get();

		// L--M--D--C: distances squared are 2, 2, 4; M has equal lobe probabilities.
		// A delta event at M permits only splits 0 and 3. Continuous M permits all four.
		const std::array<real, 4> f = { 0.5f, 1 / (2 * std::sqrt(2.f) * PI),
			sampledDelta ? 0 : 1 / (8 * PI), 1 / (4 * PI) };
		const std::array<real, 4> r = { sampledDelta ? 0 : 1 / (4 * std::sqrt(2.f) * PI),
			1 / (4 * PI), chainCamera->PdfDir(Ray(chainCamera->mPos, Vec3(0, 0, 1))) / 4, 1 };
		const double probabilities[] = {
			(sampledDelta ? 1 : double(r[0])) * r[1] * r[2],
			sampledDelta ? 0 : double(f[0]) * r[1] * r[2],
			sampledDelta ? 0 : double(f[0]) * f[1] * r[2],
			double(f[0]) * f[1] * (sampledDelta ? 1 : double(f[2]))
		};
		const double total = probabilities[0] + probabilities[1] + probabilities[2] + probabilities[3];
		std::vector<PathVertex> lightPath, cameraPath;
		for (int i = 0; i < split; ++i) {
			auto vertex = chain[i];
			vertex.mPdfFwd = f[i];
			vertex.mPdfPrev = r[i];
			lightPath.push_back(vertex);
		}
		for (int i = 3; i >= split; --i) {
			auto vertex = chain[i];
			vertex.mPdfFwd = r[i];
			vertex.mPdfPrev = f[i];
			cameraPath.push_back(vertex);
		}
		real weight;
		ASSERT_TRUE(MisWeight(*scene, lightPath, cameraPath, split, 4 - split, &weight));
		EXPECT_NEAR(weight, probabilities[split] / total, 1e-6) << "split=" << split;
		for (const auto& path : { lightPath, cameraPath }) {
			for (const auto& vertex : path) {
				if (vertex.mIsect.mBsdf == chain[1].mIsect.mBsdf)
					EXPECT_EQ(vertex.mIsDelta, sampledDelta);
			}
		}
	}
};

}

TEST_F(FiniteBDPT, InternalCameraMixedDeltaExcludesAdjacentSplits) {
	CheckMixedChain(true, 0);
}

TEST_F(FiniteBDPT, InternalLightMixedDeltaExcludesAdjacentSplits) {
	CheckMixedChain(true, 3);
}

TEST_F(FiniteBDPT, InternalMixedContinuousKeepsAllFourSplits) {
	for (int split = 0; split < 4; ++split) CheckMixedChain(false, split);
}

TEST_F(FiniteBDPT, MixedDeltaReflectionHasOnlyCameraHitStrategy) {
	camera = std::make_shared<PinHoleCamera>(std::make_shared<Film>(16, 16),
		Vec3(-1.5f, 0, 3), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
	full[2].mCamera = camera.get();
	full[2].mIsect.mPos = camera->mPos;
	const Vec3 direction = -camera->mPos.Norm();
	reverse[1] = camera->PdfDir(Ray(camera->mPos, direction)) *
		std::abs(direction.z) / camera->mPos.Length2();
	full[1].mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(1)));
	full[1].mIsDelta = true;
	reverse[0] = 0;
	std::vector<PathVertex> lightPath, cameraPath;
	Split(0, &lightPath, &cameraPath);
	real weight;
	ASSERT_TRUE(MisWeight(*scene, lightPath, cameraPath, 0, 3, &weight));
	EXPECT_FLOAT_EQ(weight, 1);
	EXPECT_TRUE(cameraPath[1].mIsDelta);
	EXPECT_FLOAT_EQ(cameraPath[2].mPdfFwd, 0);
}

TEST_F(FiniteBDPT, BalanceWeightsMatchIndependentProductsAndSumToOne) {
	const double probabilities[] = {
		double(reverse[0]) * reverse[1] * reverse[2],
		double(forward[0]) * reverse[1] * reverse[2],
		double(forward[0]) * forward[1] * reverse[2]
	};
	const double total = probabilities[0] + probabilities[1] + probabilities[2];
	double weightSum = 0;
	for (int s = 0; s < 3; ++s) {
		std::vector<PathVertex> lightPath, cameraPath;
		Split(s, &lightPath, &cameraPath);
		const auto originalLight = lightPath;
		const auto originalCamera = cameraPath;
		real weight, repeated;
		ASSERT_TRUE(MisWeight(*scene, lightPath, cameraPath, s, 3 - s, &weight));
		ASSERT_TRUE(MisWeight(*scene, lightPath, cameraPath, s, 3 - s, &repeated));
		EXPECT_FLOAT_EQ(weight, repeated);
		EXPECT_NEAR(weight, probabilities[s] / total, 1e-6);
		weightSum += weight;
		for (const auto& pair : { std::make_pair(&lightPath, &originalLight), std::make_pair(&cameraPath, &originalCamera) }) {
			ASSERT_EQ(pair.first->size(), pair.second->size());
			for (size_t i = 0; i < pair.first->size(); ++i) {
				const auto& actual = (*pair.first)[i];
				const auto& original = (*pair.second)[i];
				EXPECT_FLOAT_EQ(actual.mPdfFwd, original.mPdfFwd);
				EXPECT_FLOAT_EQ(actual.mPdfPrev, original.mPdfPrev);
				EXPECT_EQ(actual.mIsDelta, original.mIsDelta);
				EXPECT_EQ(actual.mIsect.mPos, original.mIsect.mPos);
				EXPECT_EQ(actual.mIsect.mOutDir, original.mIsect.mOutDir);
				EXPECT_EQ(actual.mThroughput, original.mThroughput);
				EXPECT_EQ(actual.mLight, original.mLight);
				EXPECT_EQ(actual.mCamera, original.mCamera);
			}
		}
	}
	EXPECT_NEAR(weightSum, 1, 1e-6);
}

TEST_F(FiniteBDPT, DirectConnectionIncludesGeometryAndItsOwnMisWeight) {
	std::vector<PathVertex> lightPath, cameraPath;
	Split(1, &lightPath, &cameraPath);
	Vec3 contribution;
	Vec2 raster;
	real weight;
	ASSERT_TRUE(MisWeight(*scene, lightPath, cameraPath, 1, 2, &weight));
	ASSERT_TRUE(ConnectPaths(*scene, *camera, lightPath, cameraPath, 1, 2, &contribution, &raster));
	EXPECT_NEAR(contribution.x, (8.f / (25 * PI)) * weight, 1e-6f);
}

TEST_F(FiniteBDPT, PrimaryConnectionsReceiveMisExactlyOnce) {
	for (SurfaceNormalMode mode : { SurfaceNormalMode::Geometric, SurfaceNormalMode::Shading }) {
		for (auto& vertex : full) vertex.mNormalMode = mode;
		for (int s = 0; s < 3; ++s) {
			std::vector<PathVertex> lightPath, cameraPath;
			Split(s, &lightPath, &cameraPath);
			Vec3 primary, weighted;
			Vec2 primaryRaster, weightedRaster;
			real weight;
			ASSERT_TRUE(EvaluateConnection(*scene, *camera, lightPath, cameraPath, s, 3 - s, &primary, &primaryRaster));
			ASSERT_TRUE(MisWeight(*scene, lightPath, cameraPath, s, 3 - s, &weight));
			ASSERT_TRUE(ConnectPaths(*scene, *camera, lightPath, cameraPath, s, 3 - s, &weighted, &weightedRaster));
			EXPECT_GT(primary.x, 0);
			EXPECT_EQ(weighted, primary * weight);
			EXPECT_EQ(primaryRaster, weightedRaster);
		}
	}
}

TEST_F(FiniteBDPT, PrimaryConnectionDoesNotDependOnMisSnapshots) {
	std::vector<PathVertex> lightPath, cameraPath;
	Split(1, &lightPath, &cameraPath);
	for (auto& vertex : lightPath) vertex.mPdfFwd = 0;
	for (auto& vertex : cameraPath) vertex.mPdfFwd = 0;
	Vec3 primary, weighted(99);
	Vec2 raster;
	ASSERT_TRUE(EvaluateConnection(*scene, *camera, lightPath, cameraPath, 1, 2, &primary, &raster));
	EXPECT_NEAR(primary.x, 8.f / (25 * PI), 1e-6f);
	EXPECT_FALSE(ConnectPaths(*scene, *camera, lightPath, cameraPath, 1, 2, &weighted, &raster));
	EXPECT_EQ(weighted, Vec3(0));
}

TEST_F(FiniteBDPT, PrimaryConnectionRejectsMixedNormalPolicies) {
	std::vector<PathVertex> lightPath, cameraPath;
	Split(1, &lightPath, &cameraPath);
	lightPath.back().mNormalMode = SurfaceNormalMode::Shading;
	Vec3 primary(99);
	Vec2 raster;
	EXPECT_FALSE(EvaluateConnection(*scene, *camera, lightPath, cameraPath, 1, 2, &primary, &raster));
	EXPECT_EQ(primary, Vec3(0));
}

TEST_F(FiniteBDPT, GeometryUsesInverseSquareAndNormalizedDirections) {
	PathVertex first, second;
	first.mIsect.mNormal = first.mIsect.mGeometryNormal = Vec3(0, 0, 1);
	second.mIsect.mNormal = second.mIsect.mGeometryNormal = Vec3(0, 0, -1);
	second.mIsect.mPos = Vec3(0, 0, 2);
	real nearValue, farValue;
	ASSERT_TRUE(GeometryTerm(first, second, &nearValue));
	second.mIsect.mPos.z = 4;
	ASSERT_TRUE(GeometryTerm(first, second, &farValue));
	EXPECT_FLOAT_EQ(nearValue, 0.25f);
	EXPECT_FLOAT_EQ(farValue, 0.0625f);
}

TEST_F(FiniteBDPT, BlockedConnectionContributesZero) {
	auto blocker = std::make_shared<Sphere>(Transform(), Transform(), 0.2f, Vec3(0.5f, 0, 1));
	scene->AddPrimitive(std::make_shared<Primitive>(blocker));
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	std::vector<PathVertex> lightPath, cameraPath;
	Split(1, &lightPath, &cameraPath);
	Vec3 contribution;
	Vec2 raster;
	ASSERT_TRUE(EvaluateConnection(*scene, *camera, lightPath, cameraPath, 1, 2, &contribution, &raster));
	EXPECT_EQ(contribution, Vec3(0));
	ASSERT_TRUE(ConnectPaths(*scene, *camera, lightPath, cameraPath, 1, 2, &contribution, &raster));
	EXPECT_EQ(contribution, Vec3(0));
}

TEST_F(FiniteBDPT, SpecularConnectionEndpointIsNotConnectible) {
	MirrorMaterial(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))).ComputeScatteringFunction(
		&full[1].mIsect, arena);
	std::vector<PathVertex> lightPath, cameraPath;
	Split(1, &lightPath, &cameraPath);
	Vec3 contribution;
	Vec2 raster;
	real weight;
	ASSERT_TRUE(MisWeight(*scene, lightPath, cameraPath, 1, 2, &weight));
	EXPECT_EQ(weight, 0);
	ASSERT_TRUE(ConnectPaths(*scene, *camera, lightPath, cameraPath, 1, 2, &contribution, &raster));
	EXPECT_EQ(contribution, Vec3(0));
}

TEST_F(FiniteBDPT, CameraConnectionRejectsOffFilmBeforeSplatClamping) {
	full[1].mIsect.mPos = Vec3(-100, 0, 0);
	std::vector<PathVertex> lightPath, cameraPath;
	Split(2, &lightPath, &cameraPath);
	Vec3 contribution;
	Vec2 raster;
	ASSERT_TRUE(ConnectPaths(*scene, *camera, lightPath, cameraPath, 2, 1, &contribution, &raster));
	EXPECT_EQ(contribution, Vec3(0));
}

TEST_F(FiniteBDPT, PointLightCannotBeAHitEmitterStrategy) {
	auto point = std::make_shared<PointLight>(full[0].mIsect.mPos, Vec3(1));
	full[0].mLight = point.get();
	scene = std::make_shared<Scene>();
	scene->AddLight(point);
	scene->SetAccelerator(std::make_shared<BruteForce>());
	scene->Initialize();
	forward[0] = 1;
	forward[1] = INV_4PI * (2 / std::sqrt(5.f)) / 5;
	reverse[0] = 0;
	double sum = 0;
	for (int s = 0; s < 3; ++s) {
		std::vector<PathVertex> lightPath, cameraPath;
		Split(s, &lightPath, &cameraPath);
		real weight;
		ASSERT_TRUE(MisWeight(*scene, lightPath, cameraPath, s, 3 - s, &weight));
		if (s == 0) EXPECT_EQ(weight, 0);
		sum += weight;
	}
	EXPECT_NEAR(sum, 1, 1e-6);
}

TEST_F(FiniteBDPT, UnequalLightPmfAppearsExactlyOnceInStrategyProducts) {
	auto extraShape = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(0, -1, 2), Vec3(2, -1, 2), Vec3(1, 1, 2), Vec3(0, 0, -1));
	scene->AddLight(std::make_shared<AreaLight>(extraShape, Vec3(3)));
	scene->Initialize();
	forward[0] *= 0.25f;
	const double probabilities[] = {
		double(reverse[0]) * reverse[1],
		double(forward[0]) * reverse[1],
		double(forward[0]) * forward[1]
	};
	const double total = probabilities[0] + probabilities[1] + probabilities[2];
	for (int s = 0; s < 3; ++s) {
		std::vector<PathVertex> lightPath, cameraPath;
		Split(s, &lightPath, &cameraPath);
		real weight;
		ASSERT_TRUE(MisWeight(*scene, lightPath, cameraPath, s, 3 - s, &weight));
		EXPECT_NEAR(weight, probabilities[s] / total, 1e-6);
	}
}

TEST_F(FiniteBDPT, OrdinaryZeroProbabilityIsNotRemappedAsSpecular) {
	reverse[0] = 0;
	std::vector<PathVertex> lightPath, cameraPath;
	Split(0, &lightPath, &cameraPath);
	const auto originalCamera = cameraPath;
	real weight;
	EXPECT_FALSE(MisWeight(*scene, lightPath, cameraPath, 0, 3, &weight));
	EXPECT_EQ(weight, 0);
	for (size_t i = 0; i < cameraPath.size(); ++i) {
		EXPECT_FLOAT_EQ(cameraPath[i].mPdfFwd, originalCamera[i].mPdfFwd);
		EXPECT_FLOAT_EQ(cameraPath[i].mPdfPrev, originalCamera[i].mPdfPrev);
		EXPECT_EQ(cameraPath[i].mIsDelta, originalCamera[i].mIsDelta);
	}
}

TEST_F(FiniteBDPT, MirrorChainKeepsOnlyTheTwoValidSplits) {
	auto chainCamera = std::make_shared<PinHoleCamera>(std::make_shared<Film>(16, 16),
		Vec3(1, 0, -1), Vec3(1, 0, 1), Vec3(0, 1, 0), 53.13010235415597f, 1);
	std::array<PathVertex, 4> chain;
	chain[0] = full[0];
	chain[0].mIsect.mPos = Vec3(-1, 0, 1);
	chain[0].mIsect.mNormal = chain[0].mIsect.mGeometryNormal = Vec3(1, 0, -1).Norm();
	chain[1] = full[1];
	chain[1].mIsect.mPos = Vec3(0);
	MirrorMaterial(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))).ComputeScatteringFunction(
		&chain[1].mIsect, arena);
	chain[1].mIsDelta = true;
	chain[2] = full[1];
	chain[2].mIsect.mPos = Vec3(1, 0, 1);
	chain[2].mIsect.mNormal = chain[2].mIsect.mGeometryNormal = Vec3(0, 0, -1);
	chain[2].mIsect.mShadingDpDv = Vec3(0, -1, 0);
	DiffuseMaterial(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))).ComputeScatteringFunction(
		&chain[2].mIsect, arena);
	chain[3] = full[2];
	chain[3].mIsect.mPos = chainCamera->mPos;
	chain[3].mCamera = chainCamera.get();
	const std::array<real, 4> f = { 0.5f, 1 / (2 * std::sqrt(2.f) * PI), 0, 1 / (4 * PI) };
	const std::array<real, 4> r = { 0, 1 / (4 * PI),
		chainCamera->PdfDir(Ray(chainCamera->mPos, Vec3(0, 0, 1))) / 4, 1 };
	const double cameraProduct = double(r[1]) * r[2];
	const double lightProduct = double(f[0]) * f[1];
	double sum = 0;
	for (int split = 0; split < 4; ++split) {
		std::vector<PathVertex> lightPath, cameraPath;
		for (int i = 0; i < split; ++i) {
			auto vertex = chain[i];
			vertex.mPdfFwd = f[i];
			vertex.mPdfPrev = r[i];
			lightPath.push_back(vertex);
		}
		for (int i = 3; i >= split; --i) {
			auto vertex = chain[i];
			vertex.mPdfFwd = r[i];
			vertex.mPdfPrev = f[i];
			cameraPath.push_back(vertex);
		}
		real weight;
		ASSERT_TRUE(MisWeight(*scene, lightPath, cameraPath, split, 4 - split, &weight));
		const double expected = split == 0 ? cameraProduct / (cameraProduct + lightProduct) :
			split == 3 ? lightProduct / (cameraProduct + lightProduct) : 0;
		EXPECT_NEAR(weight, expected, 1e-6);
		sum += weight;
	}
	EXPECT_NEAR(sum, 1, 1e-6);
}

TEST(FiniteBDPTRender, DirectEmissionIsIndependentOfSampleCount) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	if (!output) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main repository Test folder.";
	for (int spp : { 1, 4 }) {
		auto scene = std::make_shared<Scene>();
		auto shape = std::make_shared<Triangle>(Transform(), Transform(),
			Vec3(-10, -10, 0), Vec3(10, -10, 0), Vec3(0, 10, 0), Vec3(0, 0, 1));
		auto light = std::make_shared<AreaLight>(shape, Vec3(3));
		auto black = std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(0)));
		scene->AddPrimitive(std::make_shared<Primitive>(shape, black, light));
		scene->AddLight(light);
		scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
		scene->Initialize();
		auto film = std::make_shared<Film>(16, 16);
		PinHoleCamera camera(film, Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
		BidirectionalPathTracing integrator(std::make_shared<RandomSampler>(123), 0, spp);
		ASSERT_TRUE(integrator.RenderChecked(*scene, camera));
		const std::string path = std::string(output) + "\\bdpt-emission-" + std::to_string(spp) + ".hdr";
		film->SetFileName(path);
		ASSERT_TRUE(film->SaveImage());
		std::vector<Vec3> image;
		int width = 0, height = 0;
		ASSERT_TRUE(ImageIO::LoadImage(path, image, width, height));
		for (const auto& pixel : image) {
			EXPECT_NEAR(pixel.x, 3, 1e-5f);
			EXPECT_NEAR(pixel.y, 3, 1e-5f);
			EXPECT_NEAR(pixel.z, 3, 1e-5f);
		}
	}
}

TEST(FiniteBDPTRender, MixedPlasticMirrorHighlightMatchesAnalyticRadiance) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	if (!output) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main repository Test folder.";
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	const auto plastic = SceneBuilder::Plastic(Vec3(0.05f), Vec3(1), 1, 1.5f, false);
	ASSERT_NE(plastic, nullptr);
	builder.AddQuad(Vec3(-10, -10, 0), Vec3(10, -10, 0), Vec3(10, 10, 0), Vec3(-10, 10, 0),
		Vec3(0, 0, 1), plastic);
	const real angle = 50 * PI / 180;
	const real offset = std::tan(angle);
	builder.AddQuadLight(Vec3(-0.1f, offset - 0.1f, 1), Vec3(-0.1f, offset + 0.1f, 1),
		Vec3(0.1f, offset + 0.1f, 1), Vec3(0.1f, offset - 0.1f, 1), Vec3(0, 0, -1), Vec3(50));
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
	// A one-degree pixel lies wholly inside the mirror image of the small emitter.
	// Independent Fresnel formula: dominant radiance F(50 degrees)*Le, plus < .02 diffuse.
	const double ci = std::cos(double(angle)), ct = std::sqrt(1 - (1 - ci * ci) / 2.25);
	const double rs = (ci - 1.5 * ct) / (ci + 1.5 * ct);
	const double rp = (1.5 * ci - ct) / (1.5 * ci + ct);
	const double expected = 25 * (rs * rs + rp * rp);
	std::array<Vec3, 2> radiance;
	for (int bdpt = 0; bdpt < 2; ++bdpt) {
		auto film = std::make_shared<Film>(1, 1);
		PinHoleCamera camera(film, Vec3(0, -offset, 1), Vec3(0), Vec3(0, 0, 1), 1, 1);
		auto sampler = std::make_shared<RandomSampler>(123);
		if (bdpt) {
			BidirectionalPathTracing integrator(sampler, 3, 16384);
			ASSERT_TRUE(integrator.RenderChecked(*scene, camera));
		} else {
			PathTracing integrator(16384, 3, sampler, std::make_shared<SamplerEnum>());
			ASSERT_TRUE(integrator.RenderChecked(*scene, camera));
		}
		const std::string path = std::string(output) + "\\mixed-plastic-highlight-" +
			(bdpt ? "bdpt.exr" : "pt.exr");
		film->SetFileName(path);
		ASSERT_TRUE(film->SaveImage());
		std::vector<Vec3> image;
		int width = 0, height = 0;
		ASSERT_TRUE(ImageIO::LoadImage(path, image, width, height));
		ASSERT_EQ(image.size(), 1);
		radiance[bdpt] = image[0];
		for (int channel = 0; channel < 3; ++channel) {
			EXPECT_GE(image[0][channel], expected * 0.96);
			EXPECT_LE(image[0][channel], expected * 1.04 + 0.02);
		}
	}
	EXPECT_NEAR(radiance[1].x / radiance[0].x, 1, 0.06);
}

TEST(FiniteBDPTRender, TinyCornellBoxRunsBothSubpathsAndConnections) {
	const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	if (!output) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main repository Test folder.";
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	AddCornellBox(builder);
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
	auto film = std::make_shared<Film>(16, 16);
	PinHoleCamera camera(film, Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
	BidirectionalPathTracing integrator(std::make_shared<RandomSampler>(123), 3, 4);
	ASSERT_TRUE(integrator.RenderChecked(*scene, camera));
	const std::string path = std::string(output) + "\\bdpt-cornell.hdr";
	film->SetFileName(path);
	ASSERT_TRUE(film->SaveImage());
	std::vector<Vec3> image;
	int width = 0, height = 0;
	ASSERT_TRUE(ImageIO::LoadImage(path, image, width, height));
	double sum = 0;
	for (const auto& pixel : image) {
		ASSERT_TRUE(std::isfinite(pixel.x) && std::isfinite(pixel.y) && std::isfinite(pixel.z));
		sum += pixel.x + pixel.y + pixel.z;
	}
	EXPECT_GT(sum, 0);
}

TEST(FiniteBDPTRender, ProgressIsMonotonicAndCompletesOnlyOnce) {
	auto scene = std::make_shared<Scene>();
	scene->SetAccelerator(std::make_shared<BruteForce>());
	scene->Initialize();
	PinHoleCamera camera(std::make_shared<Film>(64, 32), Vec3(0, 0, 3), Vec3(0),
		Vec3(0, 1, 0), 53.13010235415597f, 1);
	BidirectionalPathTracing integrator(std::make_shared<RandomSampler>(123), 0, 1);
	testing::internal::CaptureStderr();
	const bool rendered = integrator.RenderChecked(*scene, camera);
	const std::string output = testing::internal::GetCapturedStderr();
	ASSERT_TRUE(rendered);
	EXPECT_NE(output.find("Rendering BDPT (64 x 32, 1 spp, depth 0)"), std::string::npos);
	const std::string label = "Percentage:";
	size_t cursor = 0;
	double previous = -1;
	int reports = 0, completions = 0;
	while ((cursor = output.find(label, cursor)) != std::string::npos) {
		cursor += label.size();
		char* end = nullptr;
		const double percentage = std::strtod(output.c_str() + cursor, &end);
		ASSERT_NE(end, output.c_str() + cursor);
		ASSERT_EQ(*end, '%');
		EXPECT_GT(percentage, previous);
		EXPECT_LE(percentage, 100);
		if (reports == 0) EXPECT_EQ(percentage, 0);
		if (percentage == 100) ++completions;
		previous = percentage;
		++reports;
	}
	EXPECT_GT(reports, 2);
	EXPECT_LE(reports, 1001);
	EXPECT_EQ(completions, 1);
	EXPECT_EQ(previous, 100);
	EXPECT_EQ(output.back(), '\n');
}

TEST(FiniteBDPTRender, FailedRenderNeverReportsCompletion) {
	class InvalidRayCamera : public PinHoleCamera {
	public:
		using PinHoleCamera::PinHoleCamera;
		Ray GenerateRay(int, int, const Vec2&, real) const override {
			return Ray(Vec3(0), Vec3(0));
		}
	};
	auto scene = std::make_shared<Scene>();
	scene->SetAccelerator(std::make_shared<BruteForce>());
	scene->Initialize();
	InvalidRayCamera camera(std::make_shared<Film>(8, 8), Vec3(0, 0, 3), Vec3(0),
		Vec3(0, 1, 0), 53.13010235415597f, 1);
	BidirectionalPathTracing integrator(std::make_shared<RandomSampler>(123), 0, 1);
	testing::internal::CaptureStderr();
	const bool rendered = integrator.RenderChecked(*scene, camera);
	const std::string output = testing::internal::GetCapturedStderr();
	EXPECT_FALSE(rendered);
	EXPECT_EQ(output.find("100.0%"), std::string::npos);
	EXPECT_NE(output.find("failed after 0 of 1 work units."), std::string::npos);
}

TEST(FiniteBDPTRender, TilesCoverPartialEdgesAndCloneOneSamplerPerTile) {
	struct SampleCounts {
		std::array<std::atomic<int>, 4> clones;
		std::vector<std::atomic<int>> pixels;
		std::atomic<int> invalid{0};
		SampleCounts() : pixels(35 * 33) {
			for (auto& count : clones) count = 0;
			for (auto& count : pixels) count = 0;
		}
	};
	class CountingSampler : public Sampler {
	public:
		explicit CountingSampler(std::shared_ptr<SampleCounts> counts) : mCounts(counts) {}
		real Sample(unsigned int dimension, unsigned long long instance) override {
			if (dimension == 0) {
				const auto pixel = instance / 3;
				if (pixel >= mCounts->pixels.size()) ++mCounts->invalid;
				else ++mCounts->pixels[static_cast<size_t>(pixel)];
			}
			return 0.5f;
		}
		std::shared_ptr<Sampler> Clone(uint32 seed) override {
			if (seed >= mCounts->clones.size()) ++mCounts->invalid;
			else ++mCounts->clones[seed];
			return std::make_shared<CountingSampler>(mCounts);
		}
	private:
		std::shared_ptr<SampleCounts> mCounts;
	};
	auto scene = std::make_shared<Scene>();
	scene->SetAccelerator(std::make_shared<BruteForce>());
	scene->Initialize();
	PinHoleCamera camera(std::make_shared<Film>(35, 33), Vec3(0, 0, 3), Vec3(0),
		Vec3(0, 1, 0), 53.13010235415597f, 1);
	auto counts = std::make_shared<SampleCounts>();
	BidirectionalPathTracing integrator(std::make_shared<CountingSampler>(counts), 0, 3);
	ASSERT_TRUE(integrator.RenderChecked(*scene, camera));
	EXPECT_EQ(counts->invalid.load(), 0);
	for (const auto& count : counts->clones) EXPECT_EQ(count.load(), 1);
	for (const auto& count : counts->pixels) EXPECT_EQ(count.load(), 3);
}
