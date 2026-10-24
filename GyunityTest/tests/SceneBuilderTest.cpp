#include "../../GyunityApp/SceneBuilder.h"
#include "../../GyunityApp/CornellBox.h"
#include "gtest/gtest.h"

namespace {

void ExpectVectorEqual(const Vec3& actual, const Vec3& expected) {
	EXPECT_EQ(actual.x, expected.x);
	EXPECT_EQ(actual.y, expected.y);
	EXPECT_EQ(actual.z, expected.z);
}

void ExpectShapeEqual(const Shape& actual, const Shape& expected) {
	ExpectVectorEqual(actual.WorldBould().minPoint, expected.WorldBould().minPoint);
	ExpectVectorEqual(actual.WorldBould().maxPoint, expected.WorldBould().maxPoint);
	EXPECT_EQ(actual.Area(), expected.Area());
	// Corner samples also detect cyclic vertex permutations, not just winding.
	for (const Vec2& sample : { Vec2(0, 0), Vec2(1, 0), Vec2(1, 1), Vec2(0.36f, 0.6f) }) {
		real actualPdf, expectedPdf;
		Intersection actualPoint = actual.Sample(&actualPdf, sample);
		Intersection expectedPoint = expected.Sample(&expectedPdf, sample);
		ExpectVectorEqual(actualPoint.mPos, expectedPoint.mPos);
		ExpectVectorEqual(actualPoint.mNormal, expectedPoint.mNormal);
		EXPECT_EQ(actualPdf, expectedPdf);
	}
}

void ExpectMaterialEqual(const Material& actual, const Material& expected) {
	for (TransportMode mode : { TransportMode::Radiance, TransportMode::Importance }) {
		MemoryPool actualPool, expectedPool;
		Intersection actualPoint;
		actualPoint.mNormal = actualPoint.mGeometryNormal = Vec3(0, 0, 1);
		actualPoint.mShadingDpDu = Vec3(1, 0, 0);
		actualPoint.mShadingDpDv = Vec3(0, 1, 0);
		Intersection expectedPoint = actualPoint;
		actual.ComputeScatteringFunction(&actualPoint, actualPool, mode);
		expected.ComputeScatteringFunction(&expectedPoint, expectedPool, mode);
		const Vec3 wo = Vec3(0.3f, 0.2f, 0.9f).Norm();
		for (const Vec3& sample : { Vec3(0.2f, 0.4f, 0.01f), Vec3(0.6f, 0.3f, 0.9f) }) {
			Vec3 actualWi, expectedWi;
			real actualPdf = 0, expectedPdf = 0;
			ExpectVectorEqual(actualPoint.mBsdf->Sample(wo, &actualWi, &actualPdf, sample),
				expectedPoint.mBsdf->Sample(wo, &expectedWi, &expectedPdf, sample));
			EXPECT_EQ(actualPdf, expectedPdf);
			if (actualPdf > 0 && expectedPdf > 0) {
				ExpectVectorEqual(actualWi, expectedWi);
				ExpectVectorEqual(actualPoint.mBsdf->Evaluate(wo, actualWi),
					expectedPoint.mBsdf->Evaluate(wo, expectedWi));
			}
		}
	}
}

TEST(SceneBuilder, ConstantTexturesAndResourcePaths) {
	Intersection point;
	ExpectVectorEqual(SceneBuilder::ConstantColor(Vec3(0.2f, 0.4f, 0.6f))->Sample(point), Vec3(0.2f, 0.4f, 0.6f));
	EXPECT_EQ(SceneBuilder::ConstantValue(0.17f)->Sample(point), 0.17f);
	EXPECT_EQ(SceneBuilder::Resource("meshs\\bunny3.obj"), "..\\resources\\meshs\\bunny3.obj");
}

TEST(SceneBuilder, MaterialsMatchDirectConstruction) {
	const Vec3 kr(0.8f, 0.6f, 0.4f), kt(0.2f, 0.5f, 0.9f), eta(1.5f, 1.6f, 1.7f), k(2, 3, 4);
	auto krTexture = std::make_shared<ConstantTexture<Vec3>>(kr);
	auto ktTexture = std::make_shared<ConstantTexture<Vec3>>(kt);
	auto roughnessX = std::make_shared<ConstantTexture<real>>(0.06f);
	auto roughnessY = std::make_shared<ConstantTexture<real>>(0.02f);
	ExpectMaterialEqual(*SceneBuilder::Diffuse(kr), DiffuseMaterial(krTexture));
	ExpectMaterialEqual(*SceneBuilder::Mirror(kr), MirrorMaterial(krTexture));
	ExpectMaterialEqual(*SceneBuilder::Glass(kr, kt), GlassMaterial(krTexture, ktTexture));
	ExpectMaterialEqual(*SceneBuilder::Glass(kr, kt, 1.1f, 1.33f), GlassMaterial(krTexture, ktTexture, 1.1f, 1.33f));
	ExpectMaterialEqual(*SceneBuilder::Plastic(kr, kt, 0.06f), PlasticMaterial(krTexture, ktTexture, roughnessX));
	ExpectMaterialEqual(*SceneBuilder::RoughConductor(kr, 0.06f, 0.02f, eta, k),
		RoughnessMaterial(krTexture, roughnessX, roughnessY,
			std::make_shared<ConstantTexture<Vec3>>(eta), std::make_shared<ConstantTexture<Vec3>>(k)));
	ExpectMaterialEqual(*SceneBuilder::RoughDielectric(kr, kt, 0.06f, 0.02f, 1.33f),
		RoughDielectricMaterial(krTexture, ktTexture, roughnessX, roughnessY,
			std::make_shared<ConstantTexture<real>>(1.33f)));
}

TEST(SceneBuilder, TriangleAndQuadPreserveCallAndVertexOrder) {
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	auto material = builder.Diffuse(Vec3(0.75f));
	const Vec3 a(-1, -1, 0), b(1, -1, 0), c(1, 1, 0), d(-1, 1, 0), normal(0, 0, 1);
	builder.AddTriangle(c, d, a, normal, material);
	builder.AddQuad(a, b, c, d, normal, material);
	const auto& primitives = scene->GetPrimitives();
	ASSERT_EQ(primitives.size(), 3u);
	ExpectShapeEqual(*primitives[0]->GetShape(), Triangle(Transform(), Transform(), c, d, a, normal));
	ExpectShapeEqual(*primitives[1]->GetShape(), Triangle(Transform(), Transform(), a, b, c, normal));
	ExpectShapeEqual(*primitives[2]->GetShape(), Triangle(Transform(), Transform(), a, c, d, normal));
	for (size_t i = 0; i < primitives.size(); ++i) {
		EXPECT_EQ(primitives[i]->GetMaterial(), material.get());
		EXPECT_EQ(primitives[i]->mPrimId, i);
		EXPECT_EQ(primitives[i]->GetShape()->mShapeId, i);
		EXPECT_FALSE(primitives[i]->IsLight());
	}
	EXPECT_TRUE(scene->GetLights().empty());
}

TEST(SceneBuilder, SpheresPreserveInsideNormalsAndParameters) {
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	auto material = builder.Glass(Vec3(1), Vec3(1));
	const Vec3 center(3, 4, 5);
	builder.AddSphere(center, 2, material);
	builder.AddSphere(center, 2, material, true);
	ASSERT_EQ(scene->GetPrimitives().size(), 2u);
	for (size_t i = 0; i < 2; ++i) {
		const auto& primitive = scene->GetPrimitives()[i];
		ExpectShapeEqual(*primitive->GetShape(), Sphere(Transform(), Transform(), 2, center, i == 1));
		ExpectVectorEqual(primitive->GetShape()->GetNorm(center + Vec3(2, 0, 0)), Vec3(i == 0 ? 1 : -1, 0, 0));
		EXPECT_EQ(primitive->GetMaterial(), material.get());
	}
}

TEST(SceneBuilder, LightsShareTheirEmittingShapesInInsertionOrder) {
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	const Vec3 a(0.25f, 0.965, 0.25f), b(0.25f, 0.965, -0.25f);
	const Vec3 c(-0.25f, 0.965, -0.25f), d(-0.25f, 0.965, 0.25f), normal(0, -1, 0);
	const Vec3 emission = Vec3(0.3f, 0.3f, 0.3f) * 85;
	builder.AddQuadLight(a, b, c, d, normal, emission);
	builder.AddSphereLight(Vec3(0, 2, 0), 0.5f, emission);
	const auto& primitives = scene->GetPrimitives();
	const auto& lights = scene->GetLights();
	ASSERT_EQ(primitives.size(), 3u);
	ASSERT_EQ(lights.size(), 3u);
	ExpectShapeEqual(*primitives[0]->GetShape(), Triangle(Transform(), Transform(), a, b, c, normal));
	ExpectShapeEqual(*primitives[1]->GetShape(), Triangle(Transform(), Transform(), a, c, d, normal));
	ExpectShapeEqual(*primitives[2]->GetShape(), Sphere(Transform(), Transform(), 0.5f, Vec3(0, 2, 0)));
	for (size_t i = 0; i < primitives.size(); ++i) {
		EXPECT_EQ(primitives[i]->GetLight(), lights[i].get());
		EXPECT_EQ(primitives[i]->GetShape(), lights[i]->GetShapePtr().get());
		EXPECT_TRUE(primitives[i]->IsLight());
		ExpectVectorEqual(lights[i]->Emission(), emission);
		ExpectMaterialEqual(*primitives[i]->GetMaterial(), *builder.Diffuse(Vec3(0)));
	}
}

TEST(SceneBuilder, DefaultCornellBoxMatchesOriginalEmptyScene) {
	auto original = std::make_shared<Scene>();
	CornellBoxMeshEmpty::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	AddCornellBox(builder);
	const auto& actual = scene->GetPrimitives();
	const auto& expected = original->GetPrimitives();
	ASSERT_EQ(actual.size(), expected.size());
	ASSERT_EQ(actual.size(), 12u);
	for (size_t i = 0; i < actual.size(); ++i) {
		SCOPED_TRACE(i);
		ExpectShapeEqual(*actual[i]->GetShape(), *expected[i]->GetShape());
		ExpectMaterialEqual(*actual[i]->GetMaterial(), *expected[i]->GetMaterial());
		EXPECT_EQ(actual[i]->mPrimId, expected[i]->mPrimId);
		EXPECT_EQ(actual[i]->GetShape()->mShapeId, expected[i]->GetShape()->mShapeId);
		EXPECT_EQ(actual[i]->IsLight(), expected[i]->IsLight());
	}
	ASSERT_EQ(scene->GetLights().size(), original->GetLights().size());
	for (size_t i = 0; i < scene->GetLights().size(); ++i) {
		ExpectVectorEqual(scene->GetLights()[i]->Emission(), original->GetLights()[i]->Emission());
		ExpectVectorEqual(scene->GetLights()[i]->Power(), original->GetLights()[i]->Power());
	}
}

TEST(SceneBuilder, CornellBoxCanOmitBackWallAndLightWithoutReorderingOtherWalls) {
	auto original = std::make_shared<Scene>();
	CornellBoxMeshEmpty::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxOptions options;
	options.includeBackWall = false;
	options.includeLight = false;
	AddCornellBox(builder, options);
	ASSERT_EQ(scene->GetPrimitives().size(), 8u);
	EXPECT_TRUE(scene->GetLights().empty());
	for (size_t i = 0; i < 8; ++i) {
		const size_t originalIndex = i < 4 ? i : i + 2;
		ExpectShapeEqual(*scene->GetPrimitives()[i]->GetShape(), *original->GetPrimitives()[originalIndex]->GetShape());
		ExpectMaterialEqual(*scene->GetPrimitives()[i]->GetMaterial(), *original->GetPrimitives()[originalIndex]->GetMaterial());
	}
}

TEST(SceneBuilder, CornellBoxSupportsExistingSmallLightAndPower) {
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxOptions options;
	options.lightHalfSize = 0.15f;
	options.lightPower = 100;
	AddCornellBox(builder, options);
	ASSERT_EQ(scene->GetPrimitives().size(), 12u);
	ASSERT_EQ(scene->GetLights().size(), 2u);
	for (const auto& light : scene->GetLights()) {
		ExpectVectorEqual(light->Emission(), Vec3(0.3f, 0.3f, 0.3f) * 100);
		ExpectVectorEqual(light->GetShapePtr()->WorldBould().minPoint, Vec3(-0.15f, 0.965, -0.15f));
		ExpectVectorEqual(light->GetShapePtr()->WorldBould().maxPoint, Vec3(0.15f, 0.965, 0.15f));
	}
}

TEST(SceneBuilder, SplitCornellHelpersAllowObjectsBeforeLights) {
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	AddCornellBoxWalls(builder);
	auto glass = builder.Glass(Vec3(1), Vec3(1));
	builder.AddSphere(Vec3(0, -0.5f, 0), 0.25f, glass);
	AddCornellBoxLight(builder, 100);
	const auto& primitives = scene->GetPrimitives();
	ASSERT_EQ(primitives.size(), 13u);
	EXPECT_EQ(primitives[10]->GetMaterial(), glass.get());
	EXPECT_FALSE(primitives[10]->IsLight());
	EXPECT_TRUE(primitives[11]->IsLight());
	EXPECT_TRUE(primitives[12]->IsLight());
	ExpectVectorEqual(primitives[11]->GetLight()->Emission(), Vec3(0.3f, 0.3f, 0.3f) * 100);
}

}
