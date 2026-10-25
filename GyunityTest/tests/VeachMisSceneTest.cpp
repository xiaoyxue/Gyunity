#include "stb/stb_image.h"
#include "../../GyunityApp/SceneCommandLine.h"
#include "accelerator/BVHAccel.h"
#include "json/single_include/nlohmann/json.hpp"
#include "gtest/gtest.h"
#include <sstream>

namespace {
using Json = nlohmann::json;

TEST(VeachMisScene, OriginalDataUsesThreeSourceEmittersAndRawBeckmannParameters) {
	const auto native = LoadNativeScene(SceneBuilder::Resource("scenes\\mitsuba\\veach-mis\\scene.json"));
	EXPECT_EQ(native->name, "veach-mis");
	EXPECT_EQ(native->width, 1280);
	EXPECT_EQ(native->height, 720);
	EXPECT_NEAR(native->fovY, 2 * std::atan(std::tan(35 * PI / 360) * 720 / 1280) * 180 / PI, 1e-5f);
	EXPECT_EQ(native->scene->GetPrimitives().size(), 55u);
	ASSERT_EQ(native->scene->GetLights().size(), 3u);
	native->scene->SetAccelerator(std::make_shared<BVHAccel>(native->scene->GetPrimitives()));
	native->scene->Initialize();
	const real radii[] = { 1, 0.5f, 0.05f };
	const real radiance[] = { 7.59909f, 30.3964f, 3039.64f };
	const auto camera = native->CreateCamera(std::make_shared<Film>(1280, 720));
	const auto worldToRaster = camera->mCameraToRaster * camera->mWorldToCamera;
	real previousX = 1280;
	for (int i = 0; i < 3; ++i) {
		const auto& light = native->scene->GetLights()[i];
		EXPECT_NEAR(light->GetShapePtr()->Area(), 4 * PI * radii[i] * radii[i], 1e-5f);
		EXPECT_FLOAT_EQ(light->Emission().x, radiance[i]);
		EXPECT_NEAR(light->Power().x, native->scene->GetLights()[0]->Power().x, 0.001f);
		const auto bounds = light->GetShapePtr()->WorldBould();
		const Vec3 center = (bounds.minPoint + bounds.maxPoint) * 0.5f;
		EXPECT_FLOAT_EQ(center.y, 6.5f);
		const Vec3 raster = worldToRaster(center);
		EXPECT_GT(raster.x, 0);
		EXPECT_LT(raster.x, previousX);
		EXPECT_GT(raster.y, 0);
		EXPECT_LT(raster.y, 720);
		previousX = raster.x;
	}

	const int indices[] = { 0, 12, 24, 43 };
	const real alpha[] = { 0.01f, 0.05f, 0.1f, 0.25f };
	FresnelConductor fresnel(Vec3(1), Vec3(0.200438f, 0.924033f, 1.10221f), Vec3(3.91295f, 2.45285f, 2.14219f));
	for (int i = 0; i < 4; ++i) {
		Intersection hit;
		hit.mNormal = hit.mGeometryNormal = Vec3(0, 0, 1);
		hit.mShadingDpDu = Vec3(1, 0, 0);
		hit.mShadingDpDv = Vec3(0, 1, 0);
		MemoryPool arena;
		native->scene->GetPrimitives()[indices[i]]->GetMaterial()->ComputeScatteringFunction(&hit, arena);
		const Vec3 f = hit.mBsdf->Evaluate(Vec3(0, 0, 1), Vec3(0, 0, 1));
		const Vec3 expected = Vec3(0.3f) * fresnel.Evaluate(1) / (4 * PI * alpha[i] * alpha[i]);
		EXPECT_NEAR(f.x, expected.x, expected.x * 1e-5f);
	}
}

}
