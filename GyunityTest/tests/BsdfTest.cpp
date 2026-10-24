#include "gtest/gtest.h"
#include "bsdf/BSDF.h"
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

GYT_NAMESPACE_END
