#include "gtest/gtest.h"
#include "light/EnvLight.h"
#include "light/PointLight.h"
#include "visual/Scene.h"
#include "accelerator/BruteForceAccel.h"
#include "shape/Sphere.h"
#include "integrator/Integrator.h"
#include "bsdf/BSDF.h"
#include "volume/Homogeneous.h"
#include <cstdlib>
#include <filesystem>

using namespace Gyunity;

namespace {

Vec3 Direction(double u, double v) {
	const double theta = 3.14159265358979323846 * v, phi = 6.28318530717958647692 * u;
	return Vec3(real(std::sin(theta) * std::cos(phi)), real(std::cos(theta)),
		real(std::sin(theta) * std::sin(phi)));
}

class EnvironmentImportanceAudit : public testing::Test {
protected:
	std::string path;
	void SetUp() override {
		const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
		ASSERT_NE(output, nullptr) << "Set GYUNITY_TEST_OUTPUT_DIRECTORY under the main checkout Test.";
		path = (std::filesystem::path(output) / "environment-importance.exr").string();
	}
	bool Write(const std::vector<Vec3>& pixels, int width = 8, int height = 4) {
		return ImageIO::WriteImage(path, pixels, width, height);
	}
	void Initialize(Scene& scene, const std::shared_ptr<Light>& light) {
		scene.AddPrimitive(std::make_shared<Primitive>(
			std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3(0, -10, 0))));
		scene.AddLight(light);
		scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
		scene.Initialize();
	}
};

class AuditSequence : public StateSequence {
public:
	real Sample() override { return 0.3f; }
};

// Deliberately disjoint proposal, retaining the real environment emission and scene traversal.
class ZeroProposalEnvironment : public EnvironmentLight {
public:
	explicit ZeroProposalEnvironment(const std::string& path) : EnvironmentLight(path, 0) {}
	Vec3 SampleLi(const Intersection&, Vec3*, real* pdf, Intersection*, const Vec2&) const override {
		*pdf = 0;
		return Vec3(0);
	}
	real PdfLi(const Intersection&, const Vec3&) const override { return 0; }
};

}

TEST_F(EnvironmentImportanceAudit, ConstantPowerHasRgbSolidAngleUnitsAndScale) {
	ASSERT_TRUE(Write(std::vector<Vec3>(32, Vec3(1, 2, 4))));
	for (real scale : { 0.25f, 3.f }) {
		Scene scene{};
		auto light = std::make_shared<EnvironmentLight>(path, 0, scale, Transform::RotateY(37), true);
		Initialize(scene, light);
		for (int c = 0; c < 3; ++c) {
			const double expected = Vec3(1, 2, 4)[c] * scale * 12 * double(PI) * PI;
			EXPECT_NEAR(light->Power()[c], expected, expected * 2e-6);
		}
	}
}

TEST_F(EnvironmentImportanceAudit, BlackCenterRetainsPhysicalPowerAndSelectionProbability) {
	std::vector<Vec3> pixels(32);
	pixels[0] = Vec3(8, 2, 1);
	ASSERT_TRUE(Write(pixels));
	for (bool wrap : { false, true }) {
		EnvironmentMap map(path, 0, 2, Transform(), wrap);
		ASSERT_TRUE(map.IsValid());
		ASSERT_EQ(map.LookupRadiance(0.5f, 0.5f), Vec3(0));
		double integral[3] = {};
		const int width = 512, height = 256;
		for (int y = 0; y < height; ++y) {
			const double v = (y + 0.5) / height;
			for (int x = 0; x < width; ++x) {
				const Vec3 radiance = map.LookupRadiance(real((x + 0.5) / width), real(v));
				for (int c = 0; c < 3; ++c)
					integral[c] += radiance[c] * std::sin(PI * v) * 2 * double(PI) * PI / (width * height);
			}
		}
		Scene scene{};
		auto light = std::make_shared<EnvironmentLight>(path, 0, 2, Transform(), wrap);
		scene.AddLight(std::make_shared<PointLight>(Vec3(0, 10, 0), Vec3(1)));
		Initialize(scene, light);
		for (int c = 0; c < 3; ++c)
			EXPECT_NEAR(light->Power()[c], integral[c] * 3 * PI, integral[c] * 3 * PI * 0.0003);
		EnvironmentLight rotated(path, 0, 2, Transform::RotateX(57) * Transform::RotateY(31), wrap);
		rotated.Initialize(scene);
		EXPECT_EQ(rotated.Power(), light->Power());
		real pmf;
		ASSERT_TRUE(scene.LightSelectionPdf(1, &pmf));
		EXPECT_GT(pmf, 0);
		const real expected = light->Power().Y() / (light->Power().Y() + 4 * PI);
		EXPECT_NEAR(pmf, expected, 1e-6);
		real sampledPmf;
		EXPECT_EQ(scene.SampleOneLight(&sampledPmf, 1 - pmf / 2), light.get());
		EXPECT_FLOAT_EQ(sampledPmf, pmf);
	}
}

TEST_F(EnvironmentImportanceAudit, BilinearSupportSeamsPolesAndNormalization) {
	for (bool wrap : { false, true }) {
		for (int texel : { 0, 7, 10, 31 }) {
			std::vector<Vec3> pixels(32);
			pixels[texel] = Vec3(4, 2, 1);
			ASSERT_TRUE(Write(pixels));
			EnvironmentMap map(path, 0, 1, wrap ? Transform::RotateY(37) : Transform(), wrap);
			double normalization = 0;
			int unsupported = 0, lit = 0;
			for (int y = 0; y < 128; ++y) {
				const double v = (y + 0.5) / 128;
				for (int x = 0; x < 256; ++x) {
					const Vec3 direction = Direction((x + 0.5) / 256, v);
					real pdf;
					const Vec3 radiance = map.Lookup(direction, &pdf);
					EXPECT_TRUE(std::isfinite(pdf));
					EXPECT_GE(pdf, 0);
					EXPECT_FLOAT_EQ(pdf, map.PdfDir(direction));
					if (radiance.Y() > 0) { ++lit; if (pdf == 0) ++unsupported; }
					normalization += pdf * std::sin(PI * v) * 2 * double(PI) * PI / (128 * 256);
				}
			}
			EXPECT_GT(lit, 0);
			EXPECT_EQ(unsupported, 0) << "wrap " << wrap << ", texel " << texel;
			EXPECT_NEAR(normalization, 1, 0.0001);
			for (int i = 0; i < 2048; ++i) {
				Vec3 direction;
				real pdf;
				map.Sample(&direction, &pdf, Vec2((i + 0.5f) / 2048, ((i * 569) % 2048 + 0.5f) / 2048));
				ASSERT_GT(pdf, 0);
				EXPECT_NEAR(pdf, map.PdfDir(direction), pdf * 0.002 + 1e-6);
			}
		}
	}
}

TEST_F(EnvironmentImportanceAudit, LookupRetainsBilinearFiltering) {
	std::vector<Vec3> pixels(32);
	pixels[9] = Vec3(4, 2, 1);
	ASSERT_TRUE(Write(pixels));
	EnvironmentMap map(path, 0, 2, Transform(), true);
	EXPECT_EQ(map.LookupRadiance(1.5f / 8, 1.5f / 4), Vec3(8, 4, 2));
	EXPECT_EQ(map.LookupRadiance(2.f / 8, 1.5f / 4), Vec3(4, 2, 1));
	EXPECT_EQ(map.LookupRadiance(2.f / 8, 2.f / 4), Vec3(2, 1, 0.5f));
}

TEST_F(EnvironmentImportanceAudit, BsdfSampleWithZeroCompetingPdfHasUnitMisWeight) {
	ASSERT_TRUE(Write(std::vector<Vec3>(32, Vec3(2))));
	Scene scene{};
	auto light = std::make_shared<ZeroProposalEnvironment>(path);
	scene.AddLight(std::make_shared<PointLight>(Vec3(0, 10, 0), Vec3(10)));
	Initialize(scene, light);
	Intersection hit;
	hit.mNormal = hit.mGeometryNormal = hit.mAbsNormal = hit.mOutDir = Vec3(0, 1, 0);
	hit.mShadingDpDu = Vec3(1, 0, 0);
	hit.mShadingDpDv = Vec3(0, 0, 1);
	BSDF bsdf(hit);
	DiffuseBSDF diffuse(Vec3(0.5f));
	SpecularBSDF mirror(Vec3(100));
	bsdf.Add(&diffuse);
	bsdf.Add(&mirror);
	hit.mBsdf = &bsdf;
	real pmf;
	ASSERT_TRUE(scene.LightSelectionPdf(1, &pmf));
	for (bool media : { false, true }) {
		AuditSequence sequence;
		const Vec3 estimate = Integrator::DirectIllumination(scene, hit, 1 - pmf / 2,
			Vec2(0.3f, 0.4f), Vec3(0.2f, 0.3f, 0.4f), sequence, media);
		for (int c = 0; c < 3; ++c) EXPECT_NEAR(estimate[c], 1 / pmf, 1e-5);
	}
	HomogeneousMedium medium(Vec3(0.2f), Vec3(0), 0);
	scene.AddPrimitive(std::make_shared<Primitive>(
		std::make_shared<Sphere>(Transform(), Transform(), 2, Vec3(0)),
		nullptr, nullptr, MediumInterface(&medium, nullptr)));
	scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
	scene.Initialize();
	hit.mMediumInterface = MediumInterface(&medium);
	ASSERT_TRUE(scene.LightSelectionPdf(1, &pmf));
	AuditSequence sequence;
	const Vec3 attenuated = Integrator::DirectIllumination(scene, hit, 1 - pmf / 2,
		Vec2(0.3f, 0.4f), Vec3(0.2f, 0.3f, 0.4f), sequence, true);
	for (int c = 0; c < 3; ++c)
		EXPECT_NEAR(attenuated[c], std::exp(-0.4f) / pmf, 1e-5);
}

TEST_F(EnvironmentImportanceAudit, BlackAndSinglePixelMapsHaveFiniteConsistentSamples) {
	for (int width : { 1, 8 }) {
		for (real radiance : { 0.f, 2.f }) {
			ASSERT_TRUE(Write(std::vector<Vec3>(width, Vec3(radiance)), width, 1));
			EnvironmentMap map(path, 0, 3, Transform::RotateY(37), true);
			ASSERT_TRUE(map.IsValid());
			for (int i = 0; i < 128; ++i) {
				real pdf;
				Vec3 direction;
				const Vec3 value = map.Sample(&direction, &pdf,
					Vec2((i + 0.5f) / 128, ((i * 43) % 128 + 0.5f) / 128));
				EXPECT_EQ(value, Vec3(3 * radiance));
				EXPECT_TRUE(std::isfinite(pdf));
				ASSERT_GT(pdf, 0);
				EXPECT_NEAR(pdf, map.PdfDir(direction), pdf * 0.0001);
			}
		}
	}
}
