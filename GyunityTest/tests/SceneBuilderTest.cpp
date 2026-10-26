#include "../../GyunityApp/scenes/SceneBuilder.h"
#include "../../GyunityApp/legacy/CornellBox.h"
#include "../../GyunityApp/scenes/CornellBoxScenes.h"
#include "accelerator/BVHAccel.h"
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

void ExpectMaterialEqual(const Material& actual, const Material& expected, const Vec2& uv = Vec2(0.2f, 0.4f)) {
	for (TransportMode mode : { TransportMode::Radiance, TransportMode::Importance }) {
		MemoryPool actualPool, expectedPool;
		Intersection actualPoint;
		actualPoint.mNormal = actualPoint.mGeometryNormal = Vec3(0, 0, 1);
		actualPoint.mShadingDpDu = Vec3(1, 0, 0);
		actualPoint.mShadingDpDv = Vec3(0, 1, 0);
		actualPoint.mUv = uv;
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

void ExpectMediumEqual(const Medium* actual, const Medium* expected) {
	ASSERT_EQ(actual == nullptr, expected == nullptr);
	if (!actual) return;
	for (int seed : { 123, 456 }) {
		RandomStateSequence actualSamples(std::make_shared<RandomSampler>(seed), 0);
		RandomStateSequence expectedSamples(std::make_shared<RandomSampler>(seed), 0);
		const Ray ray(Vec3(0, 0, 0), Vec3(0, 0, -1), 10);
		ExpectVectorEqual(actual->Tr(ray, actualSamples), expected->Tr(ray, expectedSamples));
		MemoryPool actualPool, expectedPool;
		MediumIntersection actualPoint, expectedPoint;
		ExpectVectorEqual(actual->Sample(ray, actualSamples, actualPool, &actualPoint),
			expected->Sample(ray, expectedSamples, expectedPool, &expectedPoint));
		ASSERT_EQ(actualPoint.IsValid(), expectedPoint.IsValid());
		if (actualPoint.IsValid()) {
			ExpectVectorEqual(actualPoint.mPos, expectedPoint.mPos);
			for (const Vec3& dir : { Vec3(0, 0, 1), Vec3(1, 0, 0), Vec3(0, 0, -1) }) {
				EXPECT_EQ(actualPoint.mPhase->Evaluate(Vec3(0, 0, 1), dir),
					expectedPoint.mPhase->Evaluate(Vec3(0, 0, 1), dir));
			}
		}
	}
}

void ExpectSceneEqual(Scene& actual, Scene& expected) {
	const auto& actualPrimitives = actual.GetPrimitives();
	const auto& expectedPrimitives = expected.GetPrimitives();
	ASSERT_EQ(actualPrimitives.size(), expectedPrimitives.size());
	for (size_t i = 0; i < actualPrimitives.size(); ++i) {
		SCOPED_TRACE(i);
		const auto& actualPrimitive = actualPrimitives[i];
		const auto& expectedPrimitive = expectedPrimitives[i];
		ExpectShapeEqual(*actualPrimitive->GetShape(), *expectedPrimitive->GetShape());
		EXPECT_EQ(actualPrimitive->mPrimId, expectedPrimitive->mPrimId);
		EXPECT_EQ(actualPrimitive->GetShape()->mShapeId, expectedPrimitive->GetShape()->mShapeId);
		EXPECT_EQ(actualPrimitive->IsLight(), expectedPrimitive->IsLight());
		ASSERT_EQ(actualPrimitive->GetMaterial() == nullptr, expectedPrimitive->GetMaterial() == nullptr);
		ExpectMediumEqual(actualPrimitive->mMediumInterface.inside, expectedPrimitive->mMediumInterface.inside);
		ExpectMediumEqual(actualPrimitive->mMediumInterface.outside, expectedPrimitive->mMediumInterface.outside);
		// Mesh triangles share one material; compare it once per contiguous group.
		if (actualPrimitive->GetMaterial() &&
			(i == 0 || actualPrimitive->GetMaterial() != actualPrimitives[i - 1]->GetMaterial() ||
				expectedPrimitive->GetMaterial() != expectedPrimitives[i - 1]->GetMaterial())) {
			ExpectMaterialEqual(*actualPrimitive->GetMaterial(), *expectedPrimitive->GetMaterial());
		}
	}

	// Environment light power depends on the initialized scene bounds.
	actual.SetAccelerator(std::make_shared<BVHAccel>(actualPrimitives));
	expected.SetAccelerator(std::make_shared<BVHAccel>(expectedPrimitives));
	actual.Initialize();
	expected.Initialize();

	const auto& actualLights = actual.GetLights();
	const auto& expectedLights = expected.GetLights();
	ASSERT_EQ(actualLights.size(), expectedLights.size());
	for (size_t i = 0; i < actualLights.size(); ++i) {
		const auto& actualLight = actualLights[i];
		const auto& expectedLight = expectedLights[i];
		EXPECT_EQ(actualLight->IsAreaLight(), expectedLight->IsAreaLight());
		EXPECT_EQ(actualLight->IsEnvironmentLight(), expectedLight->IsEnvironmentLight());
		ExpectVectorEqual(actualLight->Emission(), expectedLight->Emission());
		ExpectVectorEqual(actualLight->Power(), expectedLight->Power());
		for (size_t j = 0; j < actualPrimitives.size(); ++j) {
			EXPECT_EQ(actualPrimitives[j]->GetLight() == actualLight.get(),
				expectedPrimitives[j]->GetLight() == expectedLight.get());
		}
		if (actualLight->IsEnvironmentLight() && expectedLight->IsEnvironmentLight()) {
			EXPECT_EQ(actual.GetEnvironmentLight(), actualLight.get());
			for (const Vec3& direction : { Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, -1), Vec3(1, 2, 3).Norm() }) {
				ExpectVectorEqual(actualLight->Emission(direction), expectedLight->Emission(direction));
			}
		}
	}
}

TEST(CornellBoxScenes, GlassBunnyMatchesOriginal) {
	ASSERT_TRUE(std::ifstream(SceneBuilder::Resource("meshs\\bunny3.obj")).good())
		<< "Run GyunityTest from the GyunityApp directory so scene resources resolve.";
	auto original = std::make_shared<Scene>();
	CornellBoxMesh::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxScenes::GlassBunny(builder);
	ExpectSceneEqual(*scene, *original);
}

TEST(CornellBoxScenes, SlopeMatchesOriginal) {
	auto original = std::make_shared<Scene>();
	CornellBoxMeshWithSlope::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxScenes::Slope(builder);
	ExpectSceneEqual(*scene, *original);
}

TEST(CornellBoxScenes, SphereBoxMatchesOriginal) {
	auto original = std::make_shared<Scene>();
	CornellBoxSphere::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxScenes::SphereBox(builder);
	ExpectSceneEqual(*scene, *original);
}

TEST(CornellBoxScenes, EnvironmentMatchesOriginal) {
	for (const auto* path : { "meshs\\bunny3.obj", "meshs\\Alucy.obj", "media\\pictures\\uffizi-large.hdr" }) {
		ASSERT_TRUE(std::ifstream(SceneBuilder::Resource(path)).good())
			<< "Run GyunityTest from GyunityApp; missing resource: " << path;
	}
	auto original = std::make_shared<Scene>();
	EnvironmentMapScene::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxScenes::Environment(builder);
	ExpectSceneEqual(*scene, *original);
}

TEST(CornellBoxScenes, TriangleBoxMatchesOriginal) {
	auto original = std::make_shared<Scene>();
	CornellBoxTriangle::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxScenes::TriangleBox(builder);
	ExpectSceneEqual(*scene, *original);
}

TEST(CornellBoxScenes, TexturedSpheresMatchOriginal) {
	ASSERT_TRUE(std::ifstream(SceneBuilder::Resource("texture_images\\sphere_mapping_8k.jpg")).good())
		<< "Run GyunityTest from GyunityApp so scene resources resolve.";
	auto original = std::make_shared<Scene>();
	CornellBoxTriangle2::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxScenes::TexturedSpheres(builder);
	ExpectSceneEqual(*scene, *original);
	ASSERT_EQ(scene->GetPrimitives().size(), 14u);
	const auto& actualSphere = scene->GetPrimitives()[11];
	const auto& expectedSphere = original->GetPrimitives()[11];
	for (const Vec2& uv : { Vec2(0, 0), Vec2(1, 1), Vec2(0.3f, 0.7f), Vec2(0.8f, 0.2f) }) {
		ExpectMaterialEqual(*actualSphere->GetMaterial(), *expectedSphere->GetMaterial(), uv);
	}
	const Ray ray(Vec3(-0.5f, -0.6f, 3), Vec3(0, 0, -1));
	Intersection actualHit, expectedHit;
	real actualDistance, expectedDistance;
	ASSERT_TRUE(actualSphere->GetShape()->Intersect(ray, &actualHit, &actualDistance));
	ASSERT_TRUE(expectedSphere->GetShape()->Intersect(ray, &expectedHit, &expectedDistance));
	EXPECT_EQ(actualDistance, expectedDistance);
	ExpectVectorEqual(actualHit.mPos, expectedHit.mPos);
	EXPECT_EQ(actualHit.mUv.x, expectedHit.mUv.x);
	EXPECT_EQ(actualHit.mUv.y, expectedHit.mUv.y);
}

TEST(CornellBoxScenes, EmptyMatchesOriginal) {
	auto original = std::make_shared<Scene>();
	CornellBoxMeshEmpty::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxScenes::Empty(builder);
	ExpectSceneEqual(*scene, *original);
}

TEST(CornellBoxScenes, DiffuseBunnyMatchesOriginal) {
	ASSERT_TRUE(std::ifstream(SceneBuilder::Resource("meshs\\bunny3.obj")).good())
		<< "Run GyunityTest from GyunityApp so scene resources resolve.";
	auto original = std::make_shared<Scene>();
	CornellBoxDiffuseBunny::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxScenes::DiffuseBunny(builder);
	ExpectSceneEqual(*scene, *original);
}

TEST(CornellBoxScenes, WaterMatchesOriginal) {
	for (const auto* path : { "meshs\\water.obj", "meshs\\Alucy.obj" }) {
		ASSERT_TRUE(std::ifstream(SceneBuilder::Resource(path)).good())
			<< "Run GyunityTest from GyunityApp; missing resource: " << path;
	}
	auto original = std::make_shared<Scene>();
	CornellBoxWater::SetScene(original);
	auto scene = std::make_shared<Scene>();
	SceneBuilder builder(scene);
	CornellBoxScenes::Water(builder);
	ExpectSceneEqual(*scene, *original);
}

TEST(CornellBoxScenes, FogBunnyMatchesOriginalAfterBuilderDestruction) {
	ASSERT_TRUE(std::ifstream(SceneBuilder::Resource("meshs\\bunny3.obj")).good())
		<< "Run GyunityTest from GyunityApp so scene resources resolve.";
	auto original = std::make_shared<Scene>();
	CornellBoxMedium::SetScene(original);
	std::unique_ptr<const Medium> originalMedium(original->GetPrimitives()[10]->mMediumInterface.inside);
	auto scene = std::make_shared<Scene>();
	{
		SceneBuilder builder(scene);
		CornellBoxScenes::FogBunny(builder);
	}
	ExpectSceneEqual(*scene, *original);
	const auto& front = scene->GetPrimitives()[10];
	EXPECT_EQ(front->GetMaterial(), nullptr);
	EXPECT_EQ(front->mMediumInterface.outside, nullptr);
	ASSERT_NE(front->mMediumInterface.inside, nullptr);
	EXPECT_EQ(front->mMediumInterface.inside, scene->GetPrimitives()[11]->mMediumInterface.inside);
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
	auto plastic = PlasticMaterial::Create(krTexture, ktTexture, 1.000277f, 1.9f, false);
	ASSERT_NE(plastic, nullptr);
	ExpectMaterialEqual(*SceneBuilder::Plastic(kr, kt, 1.000277f, 1.9f, false), *plastic);
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
