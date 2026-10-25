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

std::shared_ptr<Scene> BuildRoom(const char* name, SceneAccelerator accelerator) {
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	FindScene(name).build(builder);
	if (accelerator == SceneAccelerator::Bvh)
		scene->SetAccelerator(std::make_shared<BVHAccel>(scene->GetPrimitives()));
	else
		scene->SetAccelerator(std::make_shared<KdTreeAccel>(scene->GetPrimitives()));
	scene->Initialize();
	return scene;
}

bool Blocked(const Scene& scene, const Vec3& a, const Vec3& b) {
	return scene.Intersect(Ray(a, (b - a).Norm(), (b - a).Length() - 1e-3f));
}

TEST(BdptRoomScene, VariantsAreAssetFreeAndShareGeometry) {
	const auto diffuse = BuildRoom("bdpt-room", SceneAccelerator::Bvh);
	const auto glossy = BuildRoom("bdpt-room-glossy", SceneAccelerator::Bvh);
	ASSERT_EQ(diffuse->GetPrimitives().size(), 212u);
	ASSERT_EQ(glossy->GetPrimitives().size(), diffuse->GetPrimitives().size());
	for (size_t i = 0; i < diffuse->GetPrimitives().size(); ++i) {
		const auto* a = diffuse->GetPrimitives()[i]->GetShape();
		const auto* b = glossy->GetPrimitives()[i]->GetShape();
		EXPECT_FLOAT_EQ(a->Area(), b->Area());
		EXPECT_GT(a->Area(), 0);
		const auto bounds = a->WorldBould();
		for (int axis = 0; axis < 3; ++axis) {
			EXPECT_TRUE(std::isfinite(bounds.minPoint[axis]));
			EXPECT_TRUE(std::isfinite(bounds.maxPoint[axis]));
			EXPECT_FLOAT_EQ(bounds.minPoint[axis], b->WorldBould().minPoint[axis]);
			EXPECT_FLOAT_EQ(bounds.maxPoint[axis], b->WorldBould().maxPoint[axis]);
		}
	}
	for (const char* name : { "bdpt-room", "bdpt-room-glossy" }) {
		const auto& description = FindScene(name);
		EXPECT_FALSE(description.usesVolume);
		for (const auto* resource : description.resources) EXPECT_EQ(resource, nullptr);
		const char* argv[] = { "GyunityApp", "--scene", name, "--out", "room.hdr" };
		EXPECT_EQ(ParseSceneCommandLine(5, argv).scene, &description);
		const auto scene = BuildRoom(name, SceneAccelerator::Bvh);
		ASSERT_EQ(scene->GetLights().size(), 2u);
		EXPECT_EQ(scene->GetEnvironmentLight(), nullptr);
		int glossyPrimitives = 0;
		for (const auto& primitive : scene->GetPrimitives()) {
			EXPECT_EQ(primitive->mMediumInterface.inside, nullptr);
			EXPECT_EQ(primitive->mMediumInterface.outside, nullptr);
			if (dynamic_cast<const RoughnessMaterial*>(primitive->GetMaterial())) ++glossyPrimitives;
			else EXPECT_NE(dynamic_cast<const DiffuseMaterial*>(primitive->GetMaterial()), nullptr);
		}
		EXPECT_EQ(glossyPrimitives, std::string(name) == "bdpt-room" ? 0 : 3);
		for (const auto& light : scene->GetLights()) {
			real pdf = 0;
			const auto point = light->GetShapePtr()->Sample(&pdf, Vec2(0.3f, 0.4f));
			EXPECT_FLOAT_EQ(point.mNormal.y, -1);
			EXPECT_FLOAT_EQ(light->Emission().x, 45);
			EXPECT_NEAR(light->GetShapePtr()->Area(), 0.36f, 1e-6f);
		}
	}
}

TEST(BdptRoomScene, DoorwayAndBafflePreserveAnIndirectLightPath) {
	for (auto accelerator : { SceneAccelerator::Bvh, SceneAccelerator::Kd }) {
		for (const char* name : { "bdpt-room", "bdpt-room-glossy" }) {
			SCOPED_TRACE(name);
			const auto scene = BuildRoom(name, accelerator);
			EXPECT_FALSE(Blocked(*scene, Vec3(-2.3f, 0.6f, -0.1f), Vec3(-1.6f, 0.6f, -0.1f)));
			for (const Vec3& offset : { Vec3(0, 0, -1.2f), Vec3(0, 1.9f, 0), Vec3(0, -0.55f, 0) })
				EXPECT_TRUE(Blocked(*scene, Vec3(-2.3f, 0.6f, -0.1f) + offset, Vec3(-1.6f, 0.6f, -0.1f) + offset));
			EXPECT_TRUE(Blocked(*scene, Vec3(-3, 2, 0), Vec3(-2.4f, 2, 0)));
			EXPECT_FALSE(Blocked(*scene, Vec3(-3, 0.6f, 0), Vec3(-2.4f, 0.6f, 0)));

			const Vec3 source(-3.4f, 2.849f, -1.15f);
			const Vec3 bounce(-3.3f, 0.001f, 0.7f);
			const Vec3 receiver(-0.65f, 1.4f, -1.999f);
			EXPECT_TRUE(Blocked(*scene, source, receiver));
			EXPECT_FALSE(Blocked(*scene, source, bounce));
			EXPECT_FALSE(Blocked(*scene, bounce, receiver));
			EXPECT_FALSE(Blocked(*scene, FindScene(name).camera.pos, receiver));
			for (const Vec3& point : { bounce, receiver }) {
				const Vec3 direction = point.y < 0.01f ? Vec3(0, -1, 0) : Vec3(0, 0, -1);
				Intersection hit;
				ASSERT_TRUE(scene->Intersect(Ray(point, direction, 0.01f), &hit));
				scene->QueryIntersectionInfo(Ray(point, direction), &hit);
				MemoryPool arena;
				hit.ComputeScatteringFunction(arena);
				ASSERT_NE(hit.mBsdf, nullptr);
				const Vec3 incoming = point.y < 0.01f ? (source - bounce).Norm() : (bounce - receiver).Norm();
				const Vec3 outgoing = point.y < 0.01f ? (receiver - bounce).Norm() : (FindScene(name).camera.pos - receiver).Norm();
				EXPECT_GT(hit.mBsdf->Evaluate(incoming, outgoing).x, 0);
			}
		}
	}
}

TEST(BdptRoomScene, DefaultCameraSeesTheRoomButNotTheEmitter) {
	for (auto accelerator : { SceneAccelerator::Bvh, SceneAccelerator::Kd }) {
		const auto scene = BuildRoom("bdpt-room", accelerator);
		const auto camera = FindScene("bdpt-room").camera.Create(std::make_shared<Film>(32, 24));
		int hits = 0;
		for (int y = 0; y < 24; ++y) {
			for (int x = 0; x < 32; ++x) {
				const Ray ray = camera->GenerateRay(x, y, Vec2(0.5f));
				EXPECT_TRUE(std::isfinite(ray.mDir.x));
				EXPECT_TRUE(std::isfinite(ray.mDir.y));
				EXPECT_TRUE(std::isfinite(ray.mDir.z));
				Intersection hit;
				if (scene->Intersect(ray, &hit)) {
					EXPECT_FALSE(hit.mPrimitive->IsLight());
					++hits;
				}
			}
		}
		EXPECT_GT(hits, 32 * 24 / 2);
	}
}

TEST(BdptRoomSceneRenders, BothVariantsWithPtBdptAndBothAccelerators) {
	const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	if (!directory || !*directory) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test folder.";
	for (const char* name : { "bdpt-room", "bdpt-room-glossy" }) {
		for (auto accelerator : { SceneAccelerator::Bvh, SceneAccelerator::Kd }) {
			for (bool bidirectional : { false, true }) {
				SceneRenderOptions options;
				options.scene = &FindScene(name);
				options.accelerator = accelerator;
				options.width = 32;
				options.height = 24;
				options.spp = 32;
				options.maxDepth = 6;
				options.output = std::string(directory) + '\\' + name + (bidirectional ? "-bdpt-" : "-pt-") +
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

}
