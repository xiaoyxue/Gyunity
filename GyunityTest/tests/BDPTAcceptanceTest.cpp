#include "gtest/gtest.h"
#include "integrator/BDPT.h"
#include "accelerator/BruteForceAccel.h"
#include "camera/PinholeCamera.h"
#include "light/AreaLight.h"
#include "light/EnvLight.h"
#include "material/Diffuse.h"
#include "texture/ConstantTexture.h"
#include "image/ImageIO.h"
#include <array>

using namespace Gyunity;

namespace {

std::shared_ptr<Material> WhiteDiffuse() {
	return std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
}

std::shared_ptr<Triangle> Plane(real z, real halfSize = 10, real centerX = 0) {
	return std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(centerX - halfSize, -halfSize, z), Vec3(centerX + halfSize, -halfSize, z),
		Vec3(centerX, halfSize, z), Vec3(0, 0, 1));
}

class FixedSampler : public Sampler {
public:
	real Sample(unsigned int, unsigned long long) override { return 0.5f; }
	std::shared_ptr<Sampler> Clone(uint32) override { return std::make_shared<FixedSampler>(); }
};

class ExpandedBounds : public BruteForce {
public:
	ExpandedBounds(const std::vector<std::shared_ptr<Primitive>>& primitives, real scale)
		: BruteForce(primitives), mScale(scale) {}
	AABB WorldBound() const override {
		const AABB bound = BruteForce::WorldBound();
		const Vec3 center = (bound.minPoint + bound.maxPoint) * 0.5f;
		const Vec3 extent = (bound.maxPoint - bound.minPoint) * (0.5f * mScale);
		return AABB(center - extent, center + extent);
	}
private:
	real mScale;
};

class BDPTAcceptance : public testing::Test {
protected:
	std::string output;
	void SetUp() override {
		const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
		if (!directory) GTEST_SKIP() << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test folder.";
		output = directory;
	}
};

}

TEST_F(BDPTAcceptance, EverySupportedFiniteStrategyProducesAContribution) {
	auto scene = std::make_shared<Scene>();
	auto shape = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-2, -1, 2), Vec3(0, -1, 2), Vec3(-1, 1, 2), Vec3(0, 0, -1));
	auto emitter = std::make_shared<AreaLight>(shape, Vec3(1));
	scene->AddLight(emitter);
	scene->SetAccelerator(std::make_shared<BruteForce>());
	scene->Initialize();
	auto camera = std::make_shared<PinHoleCamera>(std::make_shared<Film>(16, 16),
		Vec3(1, 0, -1), Vec3(1, 0, 2), Vec3(0, 1, 0), 53.13010235415597f, 1);
	MemoryPool arena;
	std::array<PathVertex, 4> path;
	path[0].mType = PathVertexType::Light;
	path[0].mLight = emitter.get();
	path[0].mIsect.mPos = Vec3(-1, 0, 2);
	path[0].mIsect.mNormal = path[0].mIsect.mGeometryNormal = Vec3(0, 0, -1);
	path[1].mIsect.mPos = Vec3(0);
	path[2].mIsect.mPos = Vec3(1, 0, 2);
	for (int i : { 1, 2 }) {
		path[i].mIsect.mNormal = path[i].mIsect.mGeometryNormal = Vec3(0, 0, i == 1 ? 1 : -1);
		CoordinateSystem(path[i].mIsect.mNormal, &path[i].mIsect.mShadingDpDu, &path[i].mIsect.mShadingDpDv);
		WhiteDiffuse()->ComputeScatteringFunction(&path[i].mIsect, arena);
	}
	path[3].mType = PathVertexType::Camera;
	path[3].mCamera = camera.get();
	path[3].mIsect.mPos = camera->mPos;
	for (auto& vertex : path) vertex.mThroughput = Vec3(1);
	const real cameraPdf = camera->PdfDir(Ray(camera->mPos, Vec3(0, 0, 1)));
	const std::array<real, 4> f = { 0.5f, 4 / (25 * PI), 4 / (25 * PI), 1 / (9 * PI) };
	const std::array<real, 4> r = { 4 / (25 * PI), 4 / (25 * PI), cameraPdf / 9, 1 };
	double sum = 0;
	for (int s = 0; s < 4; ++s) {
		std::vector<PathVertex> light, eye;
		for (int i = 0; i < s; ++i) {
			auto vertex = path[i];
			vertex.mPdfFwd = f[i];
			vertex.mPdfPrev = r[i];
			if (i > 0) vertex.mIsect.mOutDir = (path[i - 1].mIsect.mPos - vertex.mIsect.mPos).Norm();
			light.push_back(vertex);
		}
		for (int i = 3; i >= s; --i) {
			auto vertex = path[i];
			vertex.mPdfFwd = r[i];
			vertex.mPdfPrev = f[i];
			if (i < 3) vertex.mIsect.mOutDir = (path[i + 1].mIsect.mPos - vertex.mIsect.mPos).Norm();
			eye.push_back(vertex);
		}
		Vec3 contribution;
		Vec2 raster;
		real weight;
		ASSERT_TRUE(MisWeight(*scene, light, eye, s, 4 - s, &weight));
		ASSERT_TRUE(ConnectPaths(*scene, *camera, light, eye, s, 4 - s, &contribution, &raster));
		ASSERT_TRUE(std::isfinite(contribution.x));
		EXPECT_GT(contribution.x, 0) << "s=" << s << " t=" << 4 - s;
		if (s == 2) EXPECT_NEAR(contribution.x, weight * INV_PI * INV_PI * 4 / 25, 1e-6f);
		RecordProperty("strategy_" + std::to_string(s) + "_contribution", std::to_string(contribution.x));
		sum += weight;
	}
	EXPECT_NEAR(sum, 1, 1e-6);
}

TEST_F(BDPTAcceptance, CameraOnlySplatsDoNotScaleWithSamplesPerPixel) {
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0, 0.01f, 0.04f), WhiteDiffuse()));
	auto source = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(0.03f, -0.01f, 2), Vec3(0.05f, -0.01f, 2), Vec3(0.04f, 0.01f, 2), Vec3(0, 0, -1));
	source->mShapeId = 0;
	auto emitter = std::make_shared<AreaLight>(source, Vec3(1000));
	scene->AddLight(emitter);
	scene->AddPrimitive(std::make_shared<Primitive>(source,
		std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(0))), emitter));
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
	double reference = 0;
	for (int spp : { 1, 4, 16 }) {
		auto film = std::make_shared<Film>(16, 16);
		PinHoleCamera camera(film, Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
		for (int y = 0; y < 16; ++y) {
			for (int x = 0; x < 16; ++x) {
				ASSERT_FALSE(scene->Intersect(camera.GenerateRay(x, y, Vec2(0.5f), 0)));
			}
		}
		// Every primary ray misses; only t=1 light-path camera splats can light the film.
		BidirectionalPathTracing integrator(std::make_shared<FixedSampler>(), 1, spp);
		ASSERT_TRUE(integrator.RenderChecked(*scene, camera));
		const std::string filename = output + "\\acceptance-splats-" + std::to_string(spp) + ".hdr";
		film->SetFileName(filename);
		ASSERT_TRUE(film->SaveImage());
		double sum = 0;
		std::vector<Vec3> image;
		int width = 0, height = 0;
		ASSERT_TRUE(ImageIO::LoadImage(filename, image, width, height));
		for (const auto& pixel : image) {
			ASSERT_TRUE(std::isfinite(pixel.x));
			sum += pixel.x;
		}
		ASSERT_GT(sum, 0);
		RecordProperty("splat_sum_spp_" + std::to_string(spp), std::to_string(sum));
		if (spp == 1) reference = sum;
		else EXPECT_NEAR(sum, reference, reference * 0.005);
	}
}

TEST_F(BDPTAcceptance, ExpandedLaunchDiskDoesNotDimConstantEnvironment) {
	const std::string texture = output + "\\acceptance-constant.hdr";
	ASSERT_TRUE(ImageIO::WriteImage(texture, std::vector<Vec3>(16, Vec3(2)), 4, 4));
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0), WhiteDiffuse()));
	auto environment = std::make_shared<EnvironmentLight>(texture, 0);
	scene->AddLight(environment);
	double smallest = 10, largest = 0;
	for (real scale : { 1.f, 2.f, 4.f }) {
		scene->SetAccelerator(std::make_shared<ExpandedBounds>(scene->GetPrimitives(), scale));
		scene->Initialize();
		real positionPdf, directionPdf;
		ASSERT_TRUE(environment->PdfLight(Intersection(), Vec3(0, 0, -1), &positionPdf, &directionPdf));
		EXPECT_NEAR(positionPdf, 1 / (200 * PI * scale * scale), 1e-8f);
		auto film = std::make_shared<Film>(16, 16);
		PinHoleCamera camera(film, Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
		BidirectionalPathTracing integrator(std::make_shared<RandomSampler>(123), 1, 1024);
		ASSERT_TRUE(integrator.RenderChecked(*scene, camera));
		const std::string filename = output + "\\acceptance-disk-" + std::to_string(int(scale)) + ".hdr";
		film->SetFileName(filename);
		ASSERT_TRUE(film->SaveImage());
		double sum = 0;
		std::vector<Vec3> image;
		int width = 0, height = 0;
		ASSERT_TRUE(ImageIO::LoadImage(filename, image, width, height));
		for (const auto& pixel : image) {
			ASSERT_TRUE(std::isfinite(pixel.x));
			sum += pixel.x;
		}
		const double mean = sum / 256;
		RecordProperty("radius_scale_" + std::to_string(int(scale)) + "_mean", std::to_string(mean));
		EXPECT_NEAR(mean, 2, 0.06);
		smallest = std::min(smallest, mean);
		largest = std::max(largest, mean);
	}
	EXPECT_LT(largest - smallest, 0.06);
}

TEST_F(BDPTAcceptance, MarkerEnvironmentPreservesLatitudeLongitudeAndEmissionSign) {
	const int width = 8, height = 4;
	std::vector<Vec3> markers;
	for (int row = 0; row < height; ++row) {
		for (int col = 0; col < width; ++col) markers.emplace_back(1 + col, 2 + row, 1 + col + row * width);
	}
	const std::string texture = output + "\\acceptance-markers.hdr";
	ASSERT_TRUE(ImageIO::WriteImage(texture, markers, width, height));
	auto environment = std::make_shared<EnvironmentLight>(texture, 0);
	for (int row = 0; row < height; ++row) {
		for (int col = 0; col < width; ++col) {
			const real theta = PI * (row + 0.5f) / height, phi = 2 * PI * (col + 0.5f) / width;
			const Vec3 dir(std::sin(theta) * std::cos(phi), std::cos(theta), std::sin(theta) * std::sin(phi));
			const Vec3 actual = environment->Emission(dir);
			for (int channel = 0; channel < 3; ++channel) {
				EXPECT_NEAR(actual[channel], markers[row * width + col][channel], 1e-4f);
			}
		}
	}
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0), WhiteDiffuse()));
	scene->AddLight(environment);
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
	for (const Vec2& sample : { Vec2(0.1f, 0.2f), Vec2(0.3f, 0.7f), Vec2(0.8f, 0.4f) }) {
		Intersection point;
		Vec3 emittedDir;
		real pdfPos, pdfDir;
		const Vec3 actual = environment->SampleLight(&point, &emittedDir, &pdfPos, &pdfDir, sample, Vec2(0.5f));
		const Vec3 expected = environment->Emission(-emittedDir);
		for (int channel = 0; channel < 3; ++channel) EXPECT_NEAR(actual[channel], expected[channel], 1e-4f);
	}
}

TEST_F(BDPTAcceptance, NonzeroEnvironmentRotationIsExplicitlyUnsupported) {
	const std::string texture = output + "\\acceptance-rotation.hdr";
	ASSERT_TRUE(ImageIO::WriteImage(texture, std::vector<Vec3>(16, Vec3(2)), 4, 4));
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(Plane(0), WhiteDiffuse()));
	auto environment = std::make_shared<EnvironmentLight>(texture, 90);
	scene->AddLight(environment);
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
	PinHoleCamera camera(std::make_shared<Film>(16, 16), Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), 53.13010235415597f, 1);
	BidirectionalPathTracing integrator(std::make_shared<FixedSampler>(), 1, 1);
	EXPECT_EQ(environment->GetRotation(), 90);
	EXPECT_FALSE(integrator.RenderChecked(*scene, camera));
}
