#include "../../GyunityApp/SceneRenderer.h"
#include "json/single_include/nlohmann/json.hpp"
#include "gtest/gtest.h"
#include <sstream>

TEST(VeachBidirScene, OriginalDataRetainsObjNormalPoliciesAndEmitterBsdfs) {
	const auto directory = SceneBuilder::Resource("scenes\\mitsuba\\veach-bidir\\");
	std::ifstream input(directory + "scene.json");
	const auto data = nlohmann::json::parse(input);
	const auto native = LoadNativeScene(directory + "scene.json");
	ASSERT_NE(native, nullptr);
	EXPECT_EQ(native->name, "veach-bidir");
	EXPECT_EQ(native->width, 1024);
	EXPECT_EQ(native->height, 1024);
	EXPECT_FLOAT_EQ(native->fovY, 35);
	const auto& sourceCamera = data.at("metadata").at("source_camera_to_world");
	for (int i = 0; i < 16; ++i) {
		real expected = sourceCamera[i].get<real>();
		if (i < 12 && i % 4 == 0) expected = -expected;
		EXPECT_FLOAT_EQ(native->cameraToWorld[i], expected);
	}
	ASSERT_EQ(native->scene->GetLights().size(), 4u);
	size_t primitiveIndex = 0;
	int meshCount = 0, emittingTriangles = 0;
	bool smoothNormals = false;
	for (const auto& description : data.at("shapes")) {
		size_t count = 2;
		if (description.at("type") == "mesh") {
			++meshCount;
			count = 0;
			std::ifstream mesh(directory + description.at("file").get<std::string>());
			ASSERT_TRUE(mesh.good());
			for (std::string line; std::getline(mesh, line);) {
				std::istringstream tokens(line);
				std::string token;
				if (!(tokens >> token) || token != "f") continue;
				size_t vertices = 0;
				while (tokens >> token && token[0] != '#') ++vertices;
				ASSERT_GE(vertices, 3u);
				count += vertices - 2;
			}
		}
		for (size_t i = 0; i < count; ++i) {
			ASSERT_LT(primitiveIndex, native->scene->GetPrimitives().size());
			const auto& primitive = native->scene->GetPrimitives()[primitiveIndex++];
			real pdf;
			auto hit = primitive->GetShape()->Sample(&pdf, Vec2(0.3f, 0.4f));
			const real alignment = Dot(hit.mNormal.Norm(), hit.mGeometryNormal.Norm());
			if (description.value("face_normals", false)) EXPECT_GT(alignment, 0.9999f);
			else if (alignment < 0.999f) smoothNormals = true;
			if (primitive->IsLight()) {
				++emittingTriangles;
				CoordinateSystem(hit.mNormal, &hit.mShadingDpDu, &hit.mShadingDpDv);
				MemoryPool arena;
				primitive->GetMaterial()->ComputeScatteringFunction(&hit, arena);
				EXPECT_NEAR(hit.mBsdf->Evaluate(hit.mNormal, hit.mNormal).x, 0.5f * INV_PI, 1e-6f);
			}
		}
	}
	EXPECT_EQ(meshCount, 12);
	EXPECT_EQ(emittingTriangles, 4);
	EXPECT_TRUE(smoothNormals);
	EXPECT_EQ(primitiveIndex, native->scene->GetPrimitives().size());
}
