#include "gtest/gtest.h"
#include "bsdf/BSDF.h"
#include "material/Plastic.h"
#include "material/RoughDielectric.h"
#include "texture/ConstantTexture.h"
#include "visual/Intersection.h"
#include "visual/Microfacet.h"
#include <cmath>
#include <random>

GYT_NAMESPACE_BEGIN

namespace {

struct Estimate {
	double mean;
	double stdError;
};

void ExpectCanonicalNormalSamples(bool visible) {
	for (const Vec2& alpha : { Vec2(0.25f, 0.25f), Vec2(0.1f, 0.4f) }) {
		GGXDistribution distribution(alpha.x, alpha.y, visible);
		for (const Vec3& front : { Vec3(0, 0, 1), Vec3(0.6f, 0, 0.8f) }) {
			for (int i = 0; i < 16; ++i) {
				for (int j = 0; j < 16; ++j) {
					const Vec2 sample((i + 0.5f) / 16, (j + 0.5f) / 16);
					const Vec3 expected = distribution.SampleWh(front, sample);
					const Vec3 actual = distribution.SampleWh(-front, sample);
					ASSERT_TRUE(std::isfinite(actual.x) && std::isfinite(actual.y) && std::isfinite(actual.z));
					ASSERT_NEAR(actual.Length2(), 1, 2e-6f);
					ASSERT_GT(actual.z, 0);
					EXPECT_EQ(actual, expected);
					EXPECT_GT(distribution.Pdf(-front, actual), 0);
					EXPECT_NEAR(distribution.Pdf(-front, actual), distribution.Pdf(front, expected), 1e-6f);
				}
			}
		}
	}
}

double ExteriorFresnel(double cosine, double eta) {
	const double transmittedCosine = std::sqrt(1 - (1 - cosine * cosine) / (eta * eta));
	const double parallel = (eta * cosine - transmittedCosine) / (eta * cosine + transmittedCosine);
	const double perpendicular = (cosine - eta * transmittedCosine) / (cosine + eta * transmittedCosine);
	return (parallel * parallel + perpendicular * perpendicular) / 2;
}

// Monte Carlo estimate of the albedo (red channel) of `bsdf` for direction `wo`,
// importance sampled with BSDF::Sample: f * cos / pdf.
Estimate AlbedoBySampling(const BSDF& bsdf, const Vec3& wo, int n, std::mt19937& rng) {
	std::uniform_real_distribution<real> u(0, 1);
	double sum = 0, sum2 = 0;
	for (int i = 0; i < n; ++i) {
		Vec3 wi;
		real pdf = 0;
		Vec3 f = bsdf.Sample(wo, &wi, &pdf, Vec3(u(rng), u(rng), u(rng)));
		double x = pdf > 0 ? f.x * std::abs(wi.z) / pdf : 0.0;
		sum += x;
		sum2 += x * x;
	}
	double mean = sum / n;
	return { mean, std::sqrt((sum2 / n - mean * mean) / n) };
}

// Same albedo, estimated with uniform hemisphere sampling and BSDF::Evaluate only.
Estimate AlbedoByUniformSampling(const BSDF& bsdf, const Vec3& wo, int n, std::mt19937& rng) {
	std::uniform_real_distribution<real> u(0, 1);
	double sum = 0, sum2 = 0;
	for (int i = 0; i < n; ++i) {
		real z = u(rng);
		real r = std::sqrt(std::max((real)0, 1 - z * z));
		real phi = 2 * PI * u(rng);
		Vec3 wi(r * std::cos(phi), r * std::sin(phi), z);
		double x = bsdf.Evaluate(wo, wi).x * z * 2 * PI;
		sum += x;
		sum2 += x * x;
	}
	double mean = sum / n;
	return { mean, std::sqrt((sum2 / n - mean * mean) / n) };
}

}

// A plastic-like BSDF (diffuse + GGX reflection): importance sampling through
// BSDF::Sample must agree with a uniform-sampling estimate that doesn't use it.
TEST(TestBsdfSample, MultiLobeSampleMatchesPdf) {
	Intersection isect;
	isect.mNormal = isect.mGeometryNormal = Vec3(0, 0, 1);
	isect.mShadingDpDu = Vec3(1, 0, 0);
	isect.mShadingDpDv = Vec3(0, 1, 0);
	BSDF bsdf(isect);

	DiffuseBSDF diffuse(Vec3(0.5f));
	MicrofacetReflectionBSDF glossy(new GGXDistribution(0.3f, true), new FresnelDielectric(1.5f, 1.f), Vec3(0.5f));
	bsdf.Add(&diffuse);
	bsdf.Add(&glossy);

	Vec3 wo = Vec3(0.3f, 0.2f, 0.9f).Norm();
	std::mt19937 rng(7);
	const int n = 1 << 20;
	Estimate sampled = AlbedoBySampling(bsdf, wo, n, rng);
	Estimate uniform = AlbedoByUniformSampling(bsdf, wo, n, rng);
	double tolerance = 5 * std::sqrt(sampled.stdError * sampled.stdError + uniform.stdError * uniform.stdError);
	std::cout << "albedo: BSDF::Sample " << sampled.mean << " +- " << sampled.stdError
		<< ", uniform " << uniform.mean << " +- " << uniform.stdError << std::endl;
	EXPECT_NEAR(sampled.mean, uniform.mean, tolerance);
}

// A single-lobe BSDF must be unaffected by how lobes are selected.
TEST(TestBsdfSample, SingleLobeSampleMatchesPdf) {
	Intersection isect;
	isect.mNormal = isect.mGeometryNormal = Vec3(0, 0, 1);
	isect.mShadingDpDu = Vec3(1, 0, 0);
	isect.mShadingDpDv = Vec3(0, 1, 0);
	BSDF bsdf(isect);

	DiffuseBSDF diffuse(Vec3(0.5f));
	bsdf.Add(&diffuse);

	Vec3 wo = Vec3(0.3f, 0.2f, 0.9f).Norm();
	std::mt19937 rng(11);
	const int n = 1 << 18;
	Estimate sampled = AlbedoBySampling(bsdf, wo, n, rng);
	EXPECT_NEAR(sampled.mean, 0.5, 1e-3);
}

TEST(TestMicrofacetDistribution, FullAnisotropicNormalsMatchIndependentSlopeSampling) {
	GGXDistribution distribution(0.1f, 0.4f, false);
	const double pi = std::acos(-1.0);
	for (real u : { 0.f, 0.1f, 0.5f, 0.9f, ONE_MINUS_EPSILON }) {
		for (real v : { 0.f, 0.125f, 0.25f, 0.375f, 0.5f, 0.625f, 0.75f, 0.875f, ONE_MINUS_EPSILON }) {
			SCOPED_TRACE(testing::Message() << "u=" << u << " v=" << v);
			const Vec3 wh = distribution.SampleWh(Vec3(0, 0, 1), Vec2(u, v));
			ASSERT_TRUE(std::isfinite(wh.x) && std::isfinite(wh.y) && std::isfinite(wh.z));
			ASSERT_NEAR(wh.Length2(), 1, 2e-6f);
			const double radius = std::sqrt(double(u) / (1 - double(u)));
			const double x = double(0.1f) * radius * std::cos(2 * pi * v);
			const double y = double(0.4f) * radius * std::sin(2 * pi * v);
			const double length = std::sqrt(x * x + y * y + 1);
			EXPECT_NEAR(wh.x, x / length, 3e-6);
			EXPECT_NEAR(wh.y, y / length, 3e-6);
			EXPECT_NEAR(wh.z, 1 / length, 3e-6);
		}
	}
}

TEST(TestMicrofacetDistribution, FullAndVisibleSamplersStayNormalized) {
	for (bool visible : { false, true }) {
		for (real alphaY : { 0.25f, 0.4f }) {
			GGXDistribution distribution(0.25f, alphaY, visible);
			for (const Vec3& wo : { Vec3(0, 0, 1), Vec3(0.6f, 0, 0.8f) }) {
				for (int i = 0; i < 32; ++i) {
					for (int j = 0; j < 32; ++j) {
						const Vec3 wh = distribution.SampleWh(wo, Vec2((i + 0.5f) / 32, (j + 0.5f) / 32));
						ASSERT_TRUE(std::isfinite(wh.x) && std::isfinite(wh.y) && std::isfinite(wh.z));
						ASSERT_NEAR(wh.Length2(), 1, 2e-6f);
						EXPECT_GT(wh.z, 0);
					}
				}
			}
		}
	}
}

TEST(TestMicrofacetDistribution, FullNormalSamplesUseCanonicalHemisphereOnBothSides) {
	ExpectCanonicalNormalSamples(false);
}

TEST(TestMicrofacetDistribution, VisibleNormalSamplesUseCanonicalHemisphereOnBothSides) {
	ExpectCanonicalNormalSamples(true);
}

TEST(TestMicrofacetDistribution, FullNormalPdfUsesTheSelectedSamplingMode) {
	for (real alphaY : { 0.25f, 0.4f }) {
		GGXDistribution full(0.25f, alphaY, false);
		GGXDistribution visible(0.25f, alphaY, true);
		const Vec3 wh(0.6f, 0, 0.8f);
		const real expectedFull = full.D(wh) * wh.z;
		for (const Vec3& wo : { Vec3(0, 0, 1), wh }) {
			EXPECT_NEAR(full.Pdf(wo, wh), expectedFull, expectedFull * 2e-6f);
			const real expectedVisible = visible.D(wh) * visible.SmithG1(wo, wh) *
				std::abs(Dot(wo, wh)) / std::abs(wo.z);
			EXPECT_NEAR(visible.Pdf(wo, wh), expectedVisible, expectedVisible * 2e-6f);
		}
	}
}

TEST(TestMicrofacetDistribution, VisiblePdfRetainsBackfacetRejection) {
	GGXDistribution distribution(0.25f, true);
	EXPECT_EQ(distribution.Pdf(Vec3(0.8f, 0, 0.6f), Vec3(-0.8f, 0, 0.6f)), 0);
	EXPECT_EQ(distribution.Pdf(Vec3(1, 0, 0), Vec3(0, 0, 1)), 0);
}

TEST(TestMicrofacetDistribution, NormalPdfsIntegrateToOne) {
	const int polarSteps = 2048, azimuthSteps = 128;
	const double pi = std::acos(-1.0);
	const Vec3 wo(0.6f, 0, 0.8f);
	for (const Vec2& alpha : { Vec2(0.25f, 0.25f), Vec2(0.1f, 0.4f) }) {
		for (bool visible : { false, true }) {
			GGXDistribution distribution(alpha.x, alpha.y, visible);
			double sum = 0;
			for (int i = 0; i < polarSteps; ++i) {
				const double z = (i + 0.5) / polarSteps;
				const double radius = std::sqrt(1 - z * z);
				for (int j = 0; j < azimuthSteps; ++j) {
					const double phi = 2 * pi * (j + 0.5) / azimuthSteps;
					const Vec3 wh(real(radius * std::cos(phi)), real(radius * std::sin(phi)), real(z));
					sum += distribution.Pdf(wo, wh);
				}
			}
			EXPECT_NEAR(sum * 2 * pi / (polarSteps * azimuthSteps), 1, 2e-3)
				<< "alpha=" << alpha << " visible=" << visible;
		}
	}
}

TEST(TestBsdfSample, ReflectionPdfRejectsUnsupportedAndDegenerateDirections) {
	Intersection isect;
	isect.mNormal = isect.mGeometryNormal = Vec3(0, 0, 1);
	isect.mShadingDpDu = Vec3(1, 0, 0);
	isect.mShadingDpDv = Vec3(0, 1, 0);
	for (bool visible : { false, true }) {
		MicrofacetReflectionBSDF reflection(new GGXDistribution(0.25f, visible),
			new FresnelDielectric(1.f, 1.5f), Vec3(1));
		BSDF bsdf(isect);
		bsdf.Add(&reflection);
		for (real side : { -1.f, 1.f }) {
			const Vec3 wo(0, 0, side);
			for (const Vec3& wi : { Vec3(0.6f, 0, -0.8f * side), -wo, Vec3(1, 0, 0) }) {
				EXPECT_EQ(reflection.Pdf(wo, wi), 0);
				EXPECT_EQ(reflection.Pdf(wi, wo), 0);
				EXPECT_EQ(bsdf.Pdf(wo, wi), 0);
				EXPECT_EQ(bsdf.Pdf(wi, wo), 0);
				EXPECT_EQ(bsdf.Evaluate(wo, wi), Vec3(0));
			}
		}
	}
}

TEST(TestBsdfSample, ReflectionSampleAndPdfAgreeOnBothSurfaceSides) {
	for (bool visible : { false, true }) {
		MicrofacetReflectionBSDF reflection(new GGXDistribution(0.1f, 0.4f, visible),
			new FresnelDielectric(1.f, 1.5f), Vec3(1));
		for (real side : { -1.f, 1.f }) {
			const Vec3 wo = Vec3(0.3f, 0.2f, 0.9f * side).Norm();
			Vec3 wi;
			real pdf = 0;
			const Vec3 f = reflection.Sample(wo, &wi, &pdf, Vec3(0.5f, 0.125f, 0));
			ASSERT_GT(pdf, 0);
			ASSERT_GT(f.x, 0);
			EXPECT_NEAR(wi.Length2(), 1, 2e-6f);
			EXPECT_GT(wo.z * wi.z, 0);
			EXPECT_NEAR(reflection.Pdf(wo, wi), pdf, pdf * 5e-5f);
			EXPECT_EQ(reflection.Evaluate(wo, wi), f);
		}
	}
}

TEST(TestBsdfSample, FullAnisotropicReflectionMatchesUniformIntegral) {
	Intersection isect;
	isect.mNormal = isect.mGeometryNormal = Vec3(0, 0, 1);
	isect.mShadingDpDu = Vec3(1, 0, 0);
	isect.mShadingDpDv = Vec3(0, 1, 0);
	BSDF bsdf(isect);
	MicrofacetReflectionBSDF reflection(new GGXDistribution(0.1f, 0.4f, false),
		new FresnelDielectric(1.f, 1.5f), Vec3(1));
	bsdf.Add(&reflection);
	const Vec3 wo = Vec3(0.3f, 0.2f, 0.9f).Norm();
	std::mt19937 rng(123);
	const Estimate sampled = AlbedoBySampling(bsdf, wo, 1 << 19, rng);
	const Estimate uniform = AlbedoByUniformSampling(bsdf, wo, 1 << 19, rng);
	const double tolerance = 5 * std::sqrt(sampled.stdError * sampled.stdError +
		uniform.stdError * uniform.stdError);
	EXPECT_NEAR(sampled.mean, uniform.mean, tolerance);
}

TEST(TestBsdfSample, EvaluateHonorsTheSameLobeFlagsAsSamplingAndPdf) {
	Intersection isect;
	isect.mNormal = isect.mGeometryNormal = Vec3(0, 0, 1);
	isect.mShadingDpDu = Vec3(1, 0, 0);
	isect.mShadingDpDv = Vec3(0, 1, 0);
	BSDF bsdf(isect);
	DiffuseBSDF diffuse(Vec3(0.5f));
	MicrofacetReflectionBSDF glossy(new GGXDistribution(0.25f, true),
		new FresnelDielectric(1.f, 1.5f), Vec3(1));
	bsdf.Add(&diffuse);
	bsdf.Add(&glossy);
	const Vec3 wo(0, 0, 1), wi(0.6f, 0, 0.8f);
	const auto diffuseFlags = ScatterEventType(BSDF_REFLECTION | BSDF_DIFFUSE);
	const auto glossyFlags = ScatterEventType(BSDF_REFLECTION | BSDF_GLOSSY);
	const auto excludedFlags = ScatterEventType(BSDF_TRANSMISSION | BSDF_DIFFUSE);
	EXPECT_EQ(bsdf.Evaluate(wo, wi, diffuseFlags), diffuse.Evaluate(wo, wi));
	EXPECT_EQ(bsdf.Evaluate(wo, wi, glossyFlags), glossy.Evaluate(wo, wi));
	EXPECT_EQ(bsdf.Evaluate(wo, wi), diffuse.Evaluate(wo, wi) + glossy.Evaluate(wo, wi));
	EXPECT_EQ(bsdf.Evaluate(wo, wi, excludedFlags), Vec3(0));
	EXPECT_EQ(bsdf.Pdf(wo, wi, excludedFlags), 0);
	Vec3 sampledWi;
	real pdf = -1;
	EXPECT_EQ(bsdf.Sample(wo, &sampledWi, &pdf, Vec3(0.5f), excludedFlags), Vec3(0));
	EXPECT_EQ(pdf, 0);
}

TEST(TestBsdfSample, TransmissionRejectsDirectionsOnTheSameSideOfTheMicrofacet) {
	const Vec3 wo(0.6f, 0, 0.8f), wi(0.96f, 0, -0.28f);
	const Vec3 wh = (wo + 1.9f * wi).Norm();
	ASSERT_GT(Dot(wo, wh) * Dot(wi, wh), 0);
	for (bool visible : { false, true }) {
		MicrofacetTransmissionBSDF transmission(new GGXDistribution(0.46176004f, visible),
			Vec3(1), 1.f, 1.9f, TransportMode::Radiance);
		RoughDielectricBSDF rough(new GGXDistribution(0.46176004f, visible),
			Vec3(1), Vec3(1), 1.f, 1.9f, TransportMode::Radiance);
		for (const BxDF* bxdf : { static_cast<const BxDF*>(&transmission), static_cast<const BxDF*>(&rough) }) {
			for (real side : { -1.f, 1.f }) {
				EXPECT_EQ(bxdf->Evaluate(side * wo, side * wi), Vec3(0));
				EXPECT_EQ(bxdf->Pdf(side * wo, side * wi), 0);
				EXPECT_EQ(bxdf->Evaluate(side * wi, side * wo), Vec3(0));
				EXPECT_EQ(bxdf->Pdf(side * wi, side * wo), 0);
			}
		}
	}
}

TEST(TestBsdfSample, TransmissionContinuousQueriesRejectTangentAndZeroHalfVectors) {
	for (real eta : { 1.f, 1.9f }) {
		MicrofacetTransmissionBSDF transmission(new GGXDistribution(0.25f, true),
			Vec3(1), 1.f, eta, TransportMode::Radiance);
		RoughDielectricBSDF rough(new GGXDistribution(0.25f, true),
			Vec3(1), Vec3(1), 1.f, eta, TransportMode::Radiance);
		for (const BxDF* bxdf : { static_cast<const BxDF*>(&transmission), static_cast<const BxDF*>(&rough) }) {
			for (const Vec3& wo : { Vec3(0, 0, 1), Vec3(1, 0, 0) }) {
				const Vec3 wi = eta == 1.f ? -wo : Vec3(0, 1, 0);
				EXPECT_EQ(bxdf->Evaluate(wo, wi), Vec3(0));
				EXPECT_EQ(bxdf->Pdf(wo, wi), 0);
			}
		}
	}
}

TEST(TestBsdfSample, TransmissionRejectsOpposedMicroAndMacroOrientation) {
	const Vec3 wo(0.6f, 0, 0.8f), wi(-0.96f, 0, -0.28f);
	const Vec3 wh = (wo + 1.9f * wi).Norm();
	ASSERT_LT(Dot(wo, wh) * Dot(wi, wh), 0);
	ASSERT_LT(Dot(wo, wh) * wo.z, 0);
	ASSERT_LT(Dot(wi, wh) * wi.z, 0);
	for (bool visible : { false, true }) {
		MicrofacetTransmissionBSDF transmission(new GGXDistribution(0.46176004f, visible),
			Vec3(1), 1.f, 1.9f, TransportMode::Radiance);
		RoughDielectricBSDF rough(new GGXDistribution(0.46176004f, visible),
			Vec3(1), Vec3(1), 1.f, 1.9f, TransportMode::Radiance);
		for (const BxDF* bxdf : { static_cast<const BxDF*>(&transmission), static_cast<const BxDF*>(&rough) }) {
			EXPECT_EQ(bxdf->Evaluate(wo, wi), Vec3(0));
			EXPECT_EQ(bxdf->Pdf(wo, wi), 0);
			EXPECT_EQ(bxdf->Evaluate(wi, wo), Vec3(0));
			EXPECT_EQ(bxdf->Pdf(wi, wo), 0);
		}
	}
}

TEST(TestBsdfSample, PlasticUsesExteriorAirToCoatingFresnel) {
	const auto color = std::make_shared<ConstantTexture<Vec3>>(Vec3(1));
	const auto roughness = std::make_shared<ConstantTexture<real>>(0.1f);
	PlasticMaterial material(color, color, roughness);
	Intersection hit;
	hit.mNormal = hit.mGeometryNormal = Vec3(0, 0, 1);
	hit.mShadingDpDu = Vec3(1, 0, 0);
	hit.mShadingDpDv = Vec3(0, 1, 0);
	MemoryPool arena;
	material.ComputeScatteringFunction(&hit, arena);
	ASSERT_NE(hit.mBsdf, nullptr);
	GGXDistribution distribution(GGXDistribution::RoughnessToAlpha(0.1f), true);
	for (real cosine : { 1.f, 0.8f, 0.6f, 0.5f }) {
		const real sine = std::sqrt(1 - cosine * cosine);
		const Vec3 wo(sine, 0, cosine), wi(-sine, 0, cosine), wh(0, 0, 1);
		const auto flags = ScatterEventType(BSDF_REFLECTION | BSDF_GLOSSY);
		const real value = hit.mBsdf->Evaluate(wo, wi, flags).x;
		const double measuredFresnel = value * (4 * cosine * cosine) /
			(distribution.D(wh) * distribution.G(wo, wi, wh));
		EXPECT_NEAR(measuredFresnel, ExteriorFresnel(cosine, 1.5), 2e-6);
	}
}

TEST(TestBsdfSample, RoughGlassUsesExteriorAirToGlassFresnel) {
	const auto color = std::make_shared<ConstantTexture<Vec3>>(Vec3(1));
	const auto roughness = std::make_shared<ConstantTexture<real>>(0.1f);
	const auto index = std::make_shared<ConstantTexture<real>>(1.9f);
	RoughDielectricMaterial material(color, color, roughness, roughness, index);
	Intersection hit;
	hit.mNormal = hit.mGeometryNormal = Vec3(0, 0, 1);
	hit.mShadingDpDu = Vec3(1, 0, 0);
	hit.mShadingDpDv = Vec3(0, 1, 0);
	MemoryPool arena;
	material.ComputeScatteringFunction(&hit, arena);
	ASSERT_NE(hit.mBsdf, nullptr);
	GGXDistribution distribution(GGXDistribution::RoughnessToAlpha(0.1f), true);
	for (real cosine : { 1.f, 0.8f, 0.6f, 0.5f }) {
		const real sine = std::sqrt(1 - cosine * cosine);
		const Vec3 wo(sine, 0, cosine), wi(-sine, 0, cosine), wh(0, 0, 1);
		const real value = hit.mBsdf->Evaluate(wo, wi).x;
		const double measuredFresnel = value * (4 * cosine * cosine) /
			(distribution.D(wh) * distribution.G(wo, wi, wh));
		EXPECT_NEAR(measuredFresnel, ExteriorFresnel(cosine, double(1.9f)), 2e-6);
	}
	const Vec3 wo(0.6f, 0, 0.8f);
	for (const Vec3& wi : { Vec3(0.96f, 0, -0.28f), Vec3(-0.96f, 0, -0.28f) }) {
		EXPECT_EQ(hit.mBsdf->Evaluate(wo, wi), Vec3(0));
		EXPECT_EQ(hit.mBsdf->Pdf(wo, wi), 0);
	}
}

GYT_NAMESPACE_END
