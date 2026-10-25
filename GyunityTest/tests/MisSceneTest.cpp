#include "stb/stb_image.h"
#include "../../GyunityApp/SceneCommandLine.h"
#include "accelerator/BVHAccel.h"
#include "accelerator/KdTreeAccel.h"
#include "integrator/BDPT.h"
#include "integrator/PT.h"
#include "sampler/SamplerEnum.h"
#include "gtest/gtest.h"
#include <cmath>
#include <cstdlib>

namespace {

std::shared_ptr<Scene> BuildMisScene(SceneAccelerator accelerator) {
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	FindScene("mis-glossy").build(builder);
	if (accelerator == SceneAccelerator::Bvh)
		scene->SetAccelerator(std::make_shared<BVHAccel>(scene->GetPrimitives()));
	else
		scene->SetAccelerator(std::make_shared<KdTreeAccel>(scene->GetPrimitives()));
	scene->Initialize();
	return scene;
}

TEST(MisScene, AssetFreeLayoutAndEqualPowerEmitters) {
	const auto& description = FindScene("mis-glossy");
	EXPECT_FALSE(description.usesVolume);
	for (const auto* resource : description.resources) EXPECT_EQ(resource, nullptr);
	const char* argv[] = { "GyunityApp", "--scene", "mis-glossy", "--out", "mis.hdr" };
	EXPECT_EQ(ParseSceneCommandLine(5, argv).scene, &description);

	const auto scene = BuildMisScene(SceneAccelerator::Bvh);
	ASSERT_EQ(scene->GetPrimitives().size(), 16u);
	ASSERT_EQ(scene->GetLights().size(), 4u);
	EXPECT_EQ(scene->GetEnvironmentLight(), nullptr);
	const real radii[] = { 0.06f, 0.12f, 0.24f, 0.48f };
	for (int i = 0; i < 4; ++i) {
		const auto& light = scene->GetLights()[i];
		EXPECT_TRUE(light->IsAreaLight());
		EXPECT_EQ(scene->GetPrimitives()[12 + i]->GetLight(), light.get());
		EXPECT_NEAR(light->GetShapePtr()->Area(), 4 * PI * radii[i] * radii[i], 1e-5f);
		EXPECT_NEAR(light->Emission().x, 1.5f / (radii[i] * radii[i]), 1e-4f);
		EXPECT_NEAR(light->Power().x, 6 * PI * PI, 1e-4f);
		real selectionPdf = 0;
		ASSERT_TRUE(scene->LightSelectionPdf(i, &selectionPdf));
		EXPECT_NEAR(selectionPdf, 0.25f, 1e-6f);
	}
	for (const auto& primitive : scene->GetPrimitives()) {
		const auto bounds = primitive->GetShape()->WorldBould();
		for (int axis = 0; axis < 3; ++axis) {
			EXPECT_TRUE(std::isfinite(bounds.minPoint[axis]));
			EXPECT_TRUE(std::isfinite(bounds.maxPoint[axis]));
			EXPECT_LE(bounds.minPoint[axis], bounds.maxPoint[axis]);
		}
		EXPECT_GT(primitive->GetShape()->Area(), 0);
	}
}

TEST(MisScene, StripsHaveDistinctRoughnessAndVisibleLightSamples) {
	const real roughness[] = { 0.025f, 0.07f, 0.18f, 0.4f };
	for (auto accelerator : { SceneAccelerator::Bvh, SceneAccelerator::Kd }) {
		const auto scene = BuildMisScene(accelerator);
		for (int i = 0; i < 4; ++i) {
			SCOPED_TRACE(i);
			// Probe a triangle interior, not the shared diagonal of the two strip triangles.
			const Vec3 point(0.17f, 0.4f, -1.35f + 0.9f * i);
			const Vec3 camera = FindScene("mis-glossy").camera.pos;
			const Ray ray(camera, (point - camera).Norm());
			Intersection hit;
			ASSERT_TRUE(scene->Intersect(ray, &hit));
			scene->QueryIntersectionInfo(ray, &hit);
			EXPECT_LT((hit.mPos - point).Length(), 1e-4f);
			EXPECT_GT(hit.mNormal.y, 0);
			MemoryPool arena, referenceArena;
			hit.ComputeScatteringFunction(arena);
			ASSERT_NE(dynamic_cast<const RoughnessMaterial*>(hit.mPrimitive->GetMaterial()), nullptr);
			Intersection reference = hit;
			SceneBuilder::RoughConductor(Vec3(1), roughness[i], roughness[i], Vec3(0.2f), Vec3(3))
				->ComputeScatteringFunction(&reference, referenceArena);
			const Vec3 actual = hit.mBsdf->Evaluate(hit.mNormal, hit.mNormal);
			const Vec3 expected = reference.mBsdf->Evaluate(hit.mNormal, hit.mNormal);
			EXPECT_GT(actual.x, 0);
			EXPECT_FLOAT_EQ(actual.x, expected.x);

			for (const auto& light : scene->GetLights()) {
				for (const Vec2& sample : { Vec2(0.2f, 0.3f), Vec2(0.7f, 0.6f) }) {
					Vec3 wi;
					real pdf = 0;
					Intersection lightPoint;
					EXPECT_GT(light->SampleLi(hit, &wi, &pdf, &lightPoint, sample).x, 0);
					ASSERT_TRUE(std::isfinite(pdf));
					ASSERT_GT(pdf, 0);
					EXPECT_NEAR(pdf, light->PdfLi(hit, wi), pdf * 1e-5f);
					EXPECT_GT(Dot(hit.mNormal, wi), 0);
					EXPECT_FALSE(scene->Intersect(hit.SpawnTo(lightPoint)));
				}
			}
		}
	}
}

TEST(MisScene, DefaultCameraFramesStripsAndEmitters) {
	for (const auto& size : { Vec2i(160, 120), Vec2i(128, 128) }) {
		const auto camera = FindScene("mis-glossy").camera.Create(std::make_shared<Film>(size.x, size.y));
		const Transform worldToRaster = camera->mCameraToRaster * camera->mWorldToCamera;
		for (int i = 0; i < 4; ++i) {
			for (const Vec3& point : { Vec3(0, 0.4f, -1.35f + 0.9f * i),
				Vec3(-2.25f + 1.5f * i, 2.6f, -2.3f) }) {
				const Vec3 raster = worldToRaster(point);
				EXPECT_GT(raster.x, 0);
				EXPECT_LT(raster.x, size.x);
				EXPECT_GT(raster.y, 0);
				EXPECT_LT(raster.y, size.y);
			}
		}
	}
}

TEST(MisSceneRenders, PtAndBdptWithBothAccelerators) {
	const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	if (!directory || !*directory) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test folder.";
	for (auto accelerator : { SceneAccelerator::Bvh, SceneAccelerator::Kd }) {
		for (bool bidirectional : { false, true }) {
			SceneRenderOptions options;
			options.scene = &FindScene("mis-glossy");
			options.accelerator = accelerator;
			options.width = 32;
			options.height = 24;
			options.spp = 16;
			options.maxDepth = 4;
			options.output = std::string(directory) + (bidirectional ? "\\mis-bdpt-" : "\\mis-pt-") +
				(accelerator == SceneAccelerator::Bvh ? "bvh.hdr" : "kd.hdr");
			auto sampler = std::make_shared<RandomSampler>(123);
			if (bidirectional) {
				BidirectionalPathTracing integrator(sampler, options.maxDepth, options.spp);
				ASSERT_TRUE(RenderScene(options, integrator));
			}
			else {
				PathTracing integrator(options.spp, options.maxDepth, sampler, std::make_shared<SamplerEnum>());
				ASSERT_TRUE(RenderScene(options, integrator));
			}
			int width = 0, height = 0, channels = 0;
			std::unique_ptr<float, decltype(&stbi_image_free)> pixels(
				stbi_loadf(options.output.c_str(), &width, &height, &channels, 3), stbi_image_free);
			ASSERT_NE(pixels, nullptr);
			ASSERT_EQ(width, options.width);
			ASSERT_EQ(height, options.height);
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
