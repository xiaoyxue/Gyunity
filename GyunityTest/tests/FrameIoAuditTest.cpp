#include "stb/stb_image_write.h"
#include "gtest/gtest.h"
#include "math/Frame.h"
#include "math/Transform.h"
#include "shape/Sphere.h"
#include "camera/PinholeCamera.h"
#include "image/ImageIO.h"
#include "../../GyunityApp/SceneRenderer.h"
#include <cstdlib>
#include <fstream>

using namespace Gyunity;

namespace {
Transform Shear() {
	return Transform(1, .7f, .2f, 0, 0, 1, .4f, 0, 0, 0, 1, 0, 0, 0, 0, 1);
}

void ExpectFrame(const Intersection& hit) {
	const Vec3 axes[] = { hit.mShadingDpDu, hit.mShadingDpDv, hit.mNormal };
	for (int i = 0; i < 3; ++i) {
		EXPECT_NEAR(axes[i].Length(), 1, 2e-6f);
		for (int j = 0; j < i; ++j) EXPECT_NEAR(Dot(axes[i], axes[j]), 0, 2e-6f);
	}
	const Frame frame(axes[0], axes[1], axes[2]);
	for (const Vec3& w : { Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), Vec3(1, 2, 3).Norm() }) {
		EXPECT_NEAR(frame.WorldToLocal(w).Length(), 1, 2e-6f);
		EXPECT_LT((frame.LocalToWorld(frame.WorldToLocal(w)) - w).Length(), 3e-6f);
	}
}

Intersection Surface() {
	const Vec3 n = Vec3(1, 1, 1).Norm();
	Vec3 u, v;
	CoordinateSystem(n, &u, &v);
	Intersection hit(Vec3(.2f, .3f, .4f), n, n, u, v, n, Vec3(1e-6f));
	hit.SetShading(n, u, v);
	return hit;
}

class FrameIoFiles : public testing::Test {
protected:
	std::string output;
	void SetUp() override {
		const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
		ASSERT_NE(directory, nullptr) << "Set output under the main checkout Test directory.";
		output = directory;
	}
};

void FrameScene(SceneBuilder& b, const Transform& transform) {
	b.AddSphere(Vec3(), .65f, b.Diffuse(Vec3(.6f)), transform);
	b.AddQuad(Vec3(-4, -.9f, 3), Vec3(4, -.9f, 3), Vec3(4, -.9f, -4), Vec3(-4, -.9f, -4),
		Vec3(0, 1, 0), b.Diffuse(Vec3(.5f)));
	b.AddQuadLight(Vec3(-1, 2, 1), Vec3(1, 2, 1), Vec3(1, 2, -1), Vec3(-1, 2, -1),
		Vec3(0, -1, 0), Vec3(5));
}
void NormalScene(SceneBuilder& b) { FrameScene(b, Transform()); }
void ScaledScene(SceneBuilder& b) { FrameScene(b, Transform::Scale(1.7f, 1, .7f)); }
void ShearedScene(SceneBuilder& b) { FrameScene(b, Shear()); }
void MirroredScene(SceneBuilder& b) { FrameScene(b, Transform::Scale(-1.7f, 1, .7f)); }
void ParallelScene(SceneBuilder& b) {
	b.AddSphere(Vec3(), .65f, b.Diffuse(Vec3(.6f)));
	b.AddQuadLight(Vec3(1.5f, 2, 1), Vec3(2.5f, 2, 1), Vec3(2.5f, 2, -1), Vec3(1.5f, 2, -1),
		Vec3(0, -1, 0), Vec3(5));
}

void TextureScene(SceneBuilder& b) {
	const std::string directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	for (int c = 1; c <= 2; ++c) {
		auto texture = std::make_shared<ImageTexture<Vec3>>(directory + "\\gray" + std::to_string(c) + ".png");
		const real x = c == 1 ? -1.f : 0.f;
		b.AddQuad(Vec3(x, -1, 0), Vec3(x + 1, -1, 0), Vec3(x + 1, 1, 0), Vec3(x, 1, 0),
			Vec3(0, 0, 1), b.Diffuse(texture));
	}
	b.AddQuadLight(Vec3(-1, 2, 3), Vec3(1, 2, 3), Vec3(1, 2, 1), Vec3(-1, 2, 1),
		Vec3(0, -1, 0), Vec3(3));
}
}

TEST(FrameIoAudit, TransformedIntersectionHasUnitInvertibleShadingFrame) {
	const auto hit = Surface();
	for (const Transform& transform : { Transform(), Transform::RotateY(35), Transform::Scale(2, 1, .5f), Shear() }) {
		const Intersection world = transform(hit);
		ExpectFrame(world);
		EXPECT_LT((world.mNormal - transform.TransformNormal(hit.mNormal).Norm()).Length(), 1e-6f);
		EXPECT_GT(Dot(world.mShadingDpDu, transform.TransformVector(hit.mShadingDpDu)), 0);
	}
}

TEST(FrameIoAudit, MirrorPreservesTransformedTangentHandedness) {
	const auto hit = Surface();
	for (real sign : { -1.f, 1.f }) {
		auto local = hit;
		local.mShadingDpDv *= sign;
		const auto transform = Transform::Scale(-2, 1, .5f);
		const auto world = transform(local);
		ExpectFrame(world);
		EXPECT_NEAR(Dot(Cross(world.mShadingDpDu, world.mShadingDpDv), world.mNormal), -sign, 2e-6f);
		EXPECT_GT(Dot(world.mShadingDpDv, transform.TransformVector(local.mShadingDpDv)), 0);
	}
}

TEST(FrameIoAudit, DegenerateTangentProjectionUsesDeterministicBasis) {
	auto hit = Surface();
	for (const Vec3& tangent : { Vec3(), hit.mNormal, hit.mNormal + hit.mShadingDpDu * 1e-8f }) {
		hit.mShadingDpDu = tangent;
		const auto world = Transform()(hit);
		ExpectFrame(world);
		EXPECT_EQ(world.mShadingDpDu, Transform()(hit).mShadingDpDu);
	}
	for (const Vec3& tangent : { Vec3(), hit.mNormal }) {
		hit.mShadingDpDu = hit.mShadingDpDv = tangent;
		ExpectFrame(Transform()(hit));
	}
	for (real magnitude : { 1e-30f, 1e30f }) {
		hit = Surface();
		hit.mShadingDpDu *= magnitude;
		hit.mShadingDpDv *= magnitude;
		ExpectFrame(Shear()(hit));
	}
}

TEST(FrameIoAudit, SphereIntersectionUsesCorrectTransformedFrame) {
	const Vec3 target = Vec3(.4f, .3f, .8f).Norm();
	for (const Transform& transform : { Transform(), Transform::Scale(2, 1, .5f), Shear(), Transform::Scale(-2, 1, .5f) }) {
		const Sphere sphere(transform, Inverse(transform), 1, Vec3());
		const Vec3 world = transform(target);
		const Vec3 normal = transform.TransformNormal(target).Norm();
		Intersection hit;
		real distance;
		ASSERT_TRUE(sphere.Intersect(Ray(world + 2 * normal, -normal), &hit, &distance));
		ExpectFrame(hit);
		EXPECT_LT((hit.mNormal - normal).Length(), 2e-5f);
	}
}

TEST(FrameIoAudit, ParallelAndAntiparallelCameraUpProducesFiniteRays) {
	for (const Vec3& position : { Vec3(0, 3, 0), Vec3(0, -3, 0) }) {
		std::shared_ptr<Camera> camera = std::make_shared<PinHoleCamera>(
			std::make_shared<Film>(32, 24), position, Vec3(), Vec3(0, 1, 0), 45, 1);
		ASSERT_TRUE(camera->IsValid());
		for (int x : { 0, 16, 31 }) {
			const Ray ray = camera->GenerateRay(x, 12, Vec2(.5f));
			for (int c = 0; c < 3; ++c) EXPECT_TRUE(std::isfinite(ray.mDir[c]));
			EXPECT_NEAR(ray.mDir.Length(), 1, 1e-6f);
		}
		EXPECT_NEAR(Dot(camera->mCx, camera->mCy), 0, 1e-6f);
		EXPECT_NEAR(Dot(Cross(camera->mCx, camera->mCy), camera->mCz), -1, 1e-6f);
	}
}

TEST(FrameIoAudit, CameraCheckedCreationRejectsInvalidInputsWithoutExceptions) {
	const auto film = std::make_shared<Film>(32, 24);
	const real nan = std::numeric_limits<real>::quiet_NaN();
	const real inf = std::numeric_limits<real>::infinity();
	EXPECT_EQ(PinHoleCamera::Create(nullptr, Vec3(0, 0, 3), Vec3(), Vec3(0, 1, 0), 45, 1), nullptr);
	EXPECT_EQ(PinHoleCamera::Create(film, Vec3(), Vec3(), Vec3(0, 1, 0), 45, 1), nullptr);
	for (const Vec3& up : { Vec3(), Vec3(nan, 1, 0), Vec3(0, inf, 0) })
		EXPECT_EQ(PinHoleCamera::Create(film, Vec3(0, 0, 3), Vec3(), up, 45, 1), nullptr);
	for (real fov : { 0.f, 179.f, -1.f, nan })
		EXPECT_EQ(PinHoleCamera::Create(film, Vec3(0, 0, 3), Vec3(), Vec3(0, 1, 0), fov, 1), nullptr);
	for (real distance : { 0.f, -1.f, inf })
		EXPECT_EQ(PinHoleCamera::Create(film, Vec3(0, 0, 3), Vec3(), Vec3(0, 1, 0), 45, distance), nullptr);
	EXPECT_EQ(PinHoleCamera::Create(film, Vec3(nan), Vec3(), Vec3(0, 1, 0), 45, 1), nullptr);
	EXPECT_EQ(PinHoleCamera::Create(film, Vec3(0, 0, 3), Vec3(inf), Vec3(0, 1, 0), 45, 1), nullptr);
	ASSERT_NE(PinHoleCamera::Create(film, Vec3(0, 3, 0), Vec3(), Vec3(0, 1, 0), 45, 1), nullptr);

	PinHoleCamera invalid(film, Vec3(), Vec3(), Vec3(0, 1, 0), 45, 1);
	EXPECT_FALSE(invalid.IsValid());
	const Ray ray = invalid.GenerateRay(0, 0, Vec2(.5f), 0);
	EXPECT_EQ(ray.mTmax, 0);
	EXPECT_EQ(ray.mDir, Vec3());
	EXPECT_EQ(invalid.We(ray), Vec3());
	EXPECT_EQ(invalid.PdfPos(), 0);
	EXPECT_EQ(invalid.PdfDir(ray), 0);
	Intersection hit;
	real pdf = -1;
	Vec3 wi(1);
	EXPECT_EQ(invalid.SampleWi(hit, &pdf, &wi, Vec3(.5f)), Vec3());
	EXPECT_EQ(pdf, 0);
	EXPECT_EQ(wi, Vec3());
}

TEST(FrameIoAudit, NearParallelUpFallbackIsDeterministicForEveryPrincipalAxis) {
	for (const Vec3& axis : { Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), Vec3(1, 2, 3).Norm() }) {
		Vec3 perpendicular, unused;
		CoordinateSystem(-axis, &perpendicular, &unused);
		auto reference = PinHoleCamera::Create(std::make_shared<Film>(32, 24), 3 * axis, Vec3(), perpendicular, 45, 1);
		ASSERT_NE(reference, nullptr);
		for (const Vec3& up : { axis, -axis, axis + perpendicular * 1e-8f }) {
			auto camera = PinHoleCamera::Create(std::make_shared<Film>(32, 24), 3 * axis, Vec3(), up, 45, 1);
			ASSERT_NE(camera, nullptr);
			const Ray a = reference->GenerateRay(7, 9, Vec2(.5f), 0);
			const Ray b = camera->GenerateRay(7, 9, Vec2(.5f), 0);
			EXPECT_LT((a.mDir - b.mDir).Length(), 1e-6f);
		}
	}
}

TEST(FrameIoAudit, ExplicitCameraAxesKeepEitherHandednessAndRejectDegeneracy) {
	for (real forward : { -1.f, 1.f }) {
		PinHoleCamera camera(std::make_shared<Film>(32, 24), Vec3(), Vec3(0, 0, forward),
			Vec3(1, 0, 0), Vec3(0, 1, 0), 45, 1);
		ASSERT_TRUE(camera.IsValid());
		EXPECT_EQ(camera.mCz, Vec3(0, 0, forward));
		EXPECT_EQ(camera.mCameraToWorld.TransformVector(Vec3(1, 0, 0)), Vec3(1, 0, 0));
		EXPECT_EQ(camera.mCameraToWorld.TransformVector(Vec3(0, 0, 1)), Vec3(0, 0, forward));
	}
	PinHoleCamera invalid(std::make_shared<Film>(32, 24), Vec3(), Vec3(0, 0, 1),
		Vec3(0, 0, 1), Vec3(0, 1, 0), 45, 1);
	EXPECT_FALSE(invalid.IsValid());
}

TEST(FrameIoAudit, FailedCameraReinitializationClearsValidity) {
	class ReinitializableCamera : public PinHoleCamera {
	public:
		using PinHoleCamera::PinHoleCamera;
		using ProjectiveCamera::Initialize;
	};
	ReinitializableCamera camera(std::make_shared<Film>(4, 4), Vec3(0, 0, 3),
		Vec3(), Vec3(0, 1, 0), 45, 1);
	ASSERT_TRUE(camera.IsValid());
	camera.mFovy = 0;
	camera.Initialize();
	EXPECT_FALSE(camera.IsValid());
	EXPECT_EQ(camera.GenerateRay(0, 0, Vec2(.5f), 0).mTmax, 0);
	camera.mFovy = 45;
	camera.Initialize();
	EXPECT_TRUE(camera.IsValid());
}

TEST(FrameIoAudit, RegisteredCameraFactoryPropagatesCheckedFailure) {
	SceneCamera description;
	description.lookAt = description.pos;
	EXPECT_EQ(description.Create(std::make_shared<Film>(4, 4)), nullptr);
	description.lookAt = Vec3();
	description.fovy = 0;
	EXPECT_EQ(description.Create(std::make_shared<Film>(4, 4)), nullptr);
}

TEST_F(FrameIoFiles, InvalidCameraFailsBothRenderEntryPointsWithoutWritingAnImage) {
	class RecordingIntegrator : public Integrator {
	public:
		bool called = false;
		void Render(const Scene&, const Camera&) override { called = true; }
	};
	for (int scenario = 0; scenario < 4; ++scenario) {
		SceneDescription description = { "invalid-camera", NormalScene, {}, false, {} };
		SceneRenderOptions options;
		options.scene = &description;
		options.width = options.height = 4;
		options.spp = options.maxDepth = 1;
		options.integrator = "path";
		if (scenario == 0) description.camera.lookAt = description.camera.pos;
		if (scenario == 1) description.camera.fovy = 0;
		if (scenario >= 2) {
			auto native = std::make_shared<NativeScene>();
			native->scene = std::make_shared<Scene>();
			SceneBuilder builder(native->scene);
			NormalScene(builder);
			native->seed = 123;
			native->fovY = scenario == 2 ? 0.f : 45.f;
			native->cameraToWorld = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 3, 0, 0, 0, 1 };
			if (scenario == 3) {
				native->cameraToWorld[0] = 0;
				native->cameraToWorld[8] = -1;
			}
			options.scene = nullptr;
			options.nativeScene = native;
		}
		options.output = output + "\\invalid-camera-default-" + std::to_string(scenario) + ".exr";
		EXPECT_FALSE(RenderScene(options));
		EXPECT_FALSE(std::ifstream(options.output, std::ios::binary).good());
		options.output = output + "\\invalid-camera-custom-" + std::to_string(scenario) + ".exr";
		RecordingIntegrator integrator;
		EXPECT_FALSE(RenderScene(options, integrator));
		EXPECT_FALSE(integrator.called);
		EXPECT_FALSE(std::ifstream(options.output, std::ios::binary).good());
	}
}

TEST_F(FrameIoFiles, NativeChannelsConvertToRgbAndFailuresLeaveOutputsUnchanged) {
	for (int channels = 1; channels <= 4; ++channels) {
		const auto file = output + "\\channels" + std::to_string(channels) + ".png";
		std::vector<unsigned char> bytes(2 * channels);
		for (int pixel = 0; pixel < 2; ++pixel)
			for (int c = 0; c < channels; ++c)
				bytes[pixel * channels + c] = ((channels == 2 && c == 1) || (channels == 4 && c == 3)) ?
					17 : (pixel == 0 ? 128 : 255);
		ASSERT_NE(stbi_write_png(file.c_str(), 1, 2, channels, bytes.data(), channels), 0);
		std::vector<Vec3> pixels;
		int width = 0, height = 0;
		Array2D<Vec3> image, texture;
		ASSERT_TRUE(ImageIO::LoadImage(file, pixels, width, height));
		ASSERT_TRUE(ImageIO::LoadImage(file, image));
		ASSERT_TRUE(ImageIO::LoadTexture(file, texture));
		EXPECT_EQ(width, 1); EXPECT_EQ(height, 2);
		const real expected = std::pow(128.f / 255, 2.2f);
		for (int c = 0; c < 3; ++c) {
			EXPECT_NEAR(pixels[0][c], expected, 1e-6f);
			EXPECT_FLOAT_EQ(pixels[1][c], 1);
		}
		EXPECT_EQ(image[0][0], pixels[0]);
		EXPECT_EQ(texture[1][0], pixels[0]);
		const auto corrupt = output + "\\corrupt.png";
		{ std::ofstream stream(corrupt); stream << "not a PNG"; }
		for (const auto& invalid : { corrupt, output + "\\absent.png" }) {
			EXPECT_FALSE(ImageIO::LoadImage(invalid, pixels, width, height));
			EXPECT_FALSE(ImageIO::LoadImage(invalid, image));
			EXPECT_FALSE(ImageIO::LoadTexture(invalid, texture));
			EXPECT_EQ(width, 1); EXPECT_EQ(height, 2);
			EXPECT_EQ(pixels.size(), 2u);
			EXPECT_EQ(image[0][0], pixels[0]);
			EXPECT_EQ(texture[1][0], pixels[0]);
		}
	}
}

TEST_F(FrameIoFiles, RgbAndRgbaRetainDistinctColorChannelsIgnoringAlpha) {
	for (int channels : { 3, 4 }) {
		const unsigned char bytes[] = { 64, 128, 255, 0 };
		const auto file = output + "\\color" + std::to_string(channels) + ".png";
		ASSERT_NE(stbi_write_png(file.c_str(), 1, 1, channels, bytes, channels), 0);
		std::vector<Vec3> pixels;
		int width, height;
		ASSERT_TRUE(ImageIO::LoadImage(file, pixels, width, height));
		for (int c = 0; c < 3; ++c)
			EXPECT_NEAR(pixels[0][c], std::pow(real(bytes[c]) / 255, 2.2f), 1e-6f);
	}
}

TEST_F(FrameIoFiles, RenderFrameCameraAndTextureRegressions) {
	for (int channels : { 1, 2 }) {
		const unsigned char bytes[] = { 128, 17 };
		const auto file = output + "\\gray" + std::to_string(channels) + ".png";
		ASSERT_NE(stbi_write_png(file.c_str(), 1, 1, channels, bytes, channels), 0);
	}
	const SceneCamera standard = { Vec3(0, .4f, 4), Vec3(), Vec3(0, 1, 0), 45, 1 };
	const SceneCamera parallel = { Vec3(0, 3, 0), Vec3(), Vec3(0, 1, 0), 45, 1 };
	const SceneDescription scenes[] = {
		{ "frame-normal", NormalScene, standard, false, {} },
		{ "frame-scale", ScaledScene, standard, false, {} },
		{ "frame-shear", ShearedScene, standard, false, {} },
		{ "frame-mirror", MirroredScene, standard, false, {} },
		{ "camera-parallel", ParallelScene, parallel, false, {} },
		{ "texture-neutral", TextureScene, {}, false, {} }
	};
	for (const auto& scene : scenes) {
		SceneRenderOptions options;
		options.scene = &scene;
		options.width = 64; options.height = 48; options.spp = 32; options.maxDepth = 4;
		options.integrator = "path";
		options.output = output + "\\" + scene.name + ".exr";
		ASSERT_TRUE(RenderScene(options));
		std::vector<Vec3> pixels;
		int width, height;
		ASSERT_TRUE(ImageIO::LoadImage(options.output, pixels, width, height));
		double energy = 0;
		for (const auto& pixel : pixels) {
			for (int c = 0; c < 3; ++c) EXPECT_TRUE(std::isfinite(pixel[c]));
			energy += pixel.x + pixel.y + pixel.z;
			if (std::string(scene.name) == "texture-neutral") {
				EXPECT_FLOAT_EQ(pixel.x, pixel.y);
				EXPECT_FLOAT_EQ(pixel.x, pixel.z);
			}
		}
		EXPECT_GT(energy, 0) << scene.name;
		ASSERT_TRUE(ImageIO::WriteSrgbLdrImage(output + "\\" + scene.name + ".png", pixels, width, height));
	}
	SceneRenderOptions options;
	options.scene = FindScene("empty");
	options.width = 64; options.height = 48; options.spp = 32; options.maxDepth = 4;
	options.integrator = "path"; options.output = output + "\\registered-empty.exr";
	ASSERT_TRUE(RenderScene(options));
}
