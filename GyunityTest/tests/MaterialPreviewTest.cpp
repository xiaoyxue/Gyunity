#include "stb/stb_image.h"
#include "../../GyunityApp/SceneCommandLine.h"
#include "accelerator/BVHAccel.h"
#include "image/ImageIO.h"
#include "json/single_include/nlohmann/json.hpp"
#include <openexr.h>
#include "gtest/gtest.h"
#include <cstdlib>

namespace {
using Json = nlohmann::json;

class NativeImageFeatures : public testing::Test {
protected:
	std::string output;
	void SetUp() override {
		const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
		if (!directory || !*directory) GTEST_SKIP() << "Set output under the main checkout Test directory.";
		output = directory;
	}
};

TEST_F(NativeImageFeatures, FloatExrRoundTripPreservesHdrValuesAndTextureOrientation) {
	const std::vector<Vec3> pixels = { Vec3(0.125f, 2, -0.25f), Vec3(100, 0.5f, 0),
		Vec3(3.25f, 0.75f, 8), Vec3(0.01f, 10, 1000) };
	const std::string file = output + "\\float-roundtrip.exr";
	ASSERT_TRUE(ImageIO::WriteImage(file, pixels, 2, 2));
	Array2D<Vec3> image, texture;
	ASSERT_TRUE(ImageIO::LoadImage(file, image));
	EXPECT_EQ(image.mRes, Vec2i(2, 2));
	ASSERT_TRUE(ImageIO::LoadTexture(file, texture));
	for (int y = 0; y < 2; ++y) {
		for (int x = 0; x < 2; ++x) {
			EXPECT_EQ(image[y][x], pixels[2 * y + x]);
			EXPECT_EQ(texture[1 - y][x], pixels[2 * y + x]);
		}
	}
	std::vector<Vec3> invalid(1, Vec3(std::numeric_limits<real>::quiet_NaN()));
	EXPECT_FALSE(ImageIO::WriteImage(output + "\\invalid-exr.exr", invalid, 1, 1));
	EXPECT_FALSE(ImageIO::LoadImage(output + "\\missing-image.exr", image));
	EXPECT_FALSE(ImageIO::LoadImage(output + "\\missing-image.png", image));
}

TEST_F(NativeImageFeatures, NonzeroExrDataWindowUsesTheCorrectBufferOrigin) {
	const auto file = output + "\\offset-window.exr";
	struct Context {
		exr_context_t value = nullptr;
		~Context() { if (value) EXPECT_EQ(exr_finish(&value), EXR_ERR_SUCCESS); }
	} context;
	ASSERT_EQ(exr_start_write(&context.value, file.c_str(), EXR_WRITE_FILE_DIRECTLY, nullptr), EXR_ERR_SUCCESS);
	int part = 0;
	ASSERT_EQ(exr_add_part(context.value, nullptr, EXR_STORAGE_SCANLINE, &part), EXR_ERR_SUCCESS);
	exr_attr_box2i_t window;
	window.min.x = 7; window.min.y = -4;
	window.max.x = 8; window.max.y = -3;
	exr_attr_v2f_t center = {};
	ASSERT_EQ(exr_initialize_required_attr(context.value, part, &window, &window, 1, &center, 1,
		EXR_LINEORDER_INCREASING_Y, EXR_COMPRESSION_NONE), EXR_ERR_SUCCESS);
	for (const char* channel : {"B","G","R"})
		ASSERT_EQ(exr_add_channel(context.value, part, channel, EXR_PIXEL_FLOAT,
			EXR_PERCEPTUALLY_LOGARITHMIC, 1, 1), EXR_ERR_SUCCESS);
	ASSERT_EQ(exr_write_header(context.value), EXR_ERR_SUCCESS);
	for (int y = 0; y < 2; ++y) {
		std::array<uint8_t, 24> packed;
		size_t offset = 0;
		for (int c : {2,1,0}) {
			for (int x = 0; x < 2; ++x) {
				const float value = float(y * 6 + x * 3 + c + 1);
				uint32 bits;
				std::memcpy(&bits, &value, sizeof(bits));
				for (int byte = 0; byte < 4; ++byte) packed[offset++] = uint8_t(bits >> (byte * 8));
			}
		}
		ASSERT_EQ(exr_write_scanline_chunk(context.value, part, y - 4, packed.data(), packed.size()), EXR_ERR_SUCCESS);
	}
	ASSERT_EQ(exr_finish(&context.value), EXR_ERR_SUCCESS);
	Array2D<Vec3> image;
	ASSERT_TRUE(ImageIO::LoadImage(file, image));
	EXPECT_EQ(image[0][0], Vec3(1,2,3));
	EXPECT_EQ(image[1][1], Vec3(10,11,12));
}

TEST_F(NativeImageFeatures, CheckerboardAndOneSidedDiffuseAreNativeParameters) {
	std::ifstream source(SceneBuilder::Resource("scenes\\native-demo\\scene.json"));
	Json data = Json::parse(source);
	data["materials"]["white"]["reflectance"] = {
		{ "type", "checkerboard" }, { "color0", { 0.4, 0.4, 0.4 } },
		{ "color1", { 0.2, 0.2, 0.2 } }, { "scale", { 8, 8 } }
	};
	data["materials"]["white"]["two_sided"] = false;
	data["render"]["filter"] = "gaussian";
	const auto file = output + "\\checker-scene.json";
	{ std::ofstream stream(file); stream << data.dump(2); }
	const auto scene = LoadNativeScene(file);
	ASSERT_NE(scene, nullptr);
	EXPECT_EQ(scene->filter, "gaussian");
	for (const auto& sample : { std::pair<Vec2, real>{ Vec2(0.03f, 0.03f), 0.4f },
		{ Vec2(0.09f, 0.03f), 0.2f }, { Vec2(-0.01f, -0.01f), 0.4f } }) {
		Intersection hit;
		hit.mNormal = hit.mGeometryNormal = Vec3(0, 0, 1);
		hit.mShadingDpDu = Vec3(1, 0, 0);
		hit.mShadingDpDv = Vec3(0, 1, 0);
		hit.mUv = sample.first;
		MemoryPool arena;
		scene->scene->GetPrimitives()[0]->GetMaterial()->ComputeScatteringFunction(&hit, arena);
		EXPECT_NEAR(hit.mBsdf->Evaluate(Vec3(0,0,1), Vec3(0,0,1)).x, sample.second * INV_PI, 1e-6f);
		EXPECT_EQ(hit.mBsdf->Evaluate(Vec3(0,0,-1), Vec3(0,0,-1)), Vec3());
	}
}

TEST(MaterialPreview, OriginalEnvironmentMappingMatchesMitsubaDirectionalRadiance) {
	const char* directory = std::getenv("GYUNITY_MATPREVIEW_SCENES");
	if (!directory || !*directory) GTEST_SKIP() << "Import local matpreview profiles and set GYUNITY_MATPREVIEW_SCENES.";
	const std::string file = std::string(directory) + "\\matpreview-diffuse.json";
	std::ifstream stream(file);
	const Json data = Json::parse(stream);
	const auto native = LoadNativeScene(file);
	ASSERT_NE(native, nullptr);
	EXPECT_EQ(native->scene->GetPrimitives().size(), 61600u);
	ASSERT_EQ(native->scene->GetLights().size(), 1u);
	const auto* environment = native->scene->GetEnvironmentLight();
	ASSERT_NE(environment, nullptr);
	for (const auto& sample : data.at("metadata").at("expected_environment_samples")) {
		const auto& d = sample.at("direction");
		const Vec3 direction(d[0].get<real>(), d[1].get<real>(), d[2].get<real>());
		const Vec3 radiance = environment->Emission(direction);
		for (int c = 0; c < 3; ++c) {
			const real expected = sample.at("radiance")[c].get<real>();
			EXPECT_NEAR(radiance[c], expected, 0.002f * (1 + std::abs(expected)));
		}
	}
	native->scene->SetAccelerator(std::make_shared<BVHAccel>(native->scene->GetPrimitives()));
	native->scene->Initialize();
	Intersection reference;
	reference.mPos = Vec3();
	for (const Vec2& sample : { Vec2(0.173f, 0.427f), Vec2(0.581f, 0.719f) }) {
		Vec3 direction;
		real pdf;
		Intersection point;
		const Vec3 radiance = environment->SampleLi(reference, &direction, &pdf, &point, sample);
		ASSERT_TRUE(std::isfinite(pdf));
		EXPECT_GT(pdf, 0);
		EXPECT_NEAR(pdf, environment->PdfLi(reference, direction), pdf * 0.001f);
		EXPECT_TRUE(std::isfinite(radiance.x));
	}
}
}
