#pragma once
#include "common/Core.h"
#include "Material.h"
#include "texture/Texture.h"
#include "bsdf/BSDF.h"
#include "math/GeometryUtils.h"
#include <cmath>

GYT_NAMESPACE_BEGIN

// SmallVCM's normalized Phong lobe around the mirror direction R of wo:
// f = ks (n + 2) / (2 pi) cos^n(R, wi), sampled with pdf (n + 1) / (2 pi) cos^n(R, wi).
class PhongBSDF : public BxDF {
public:
	PhongBSDF(const Vec3& ks, real exponent, real samplingWeight)
		: BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_GLOSSY)), mKs(ks), mExponent(exponent),
		mSamplingWeight(samplingWeight) {}

	real SamplingWeight(const Vec3& wo) const override { return wo.z > 0 ? mSamplingWeight : 0; }

	real Pdf(const Vec3& wo, const Vec3& wi) const override {
		const real cosine = MirrorCosine(wo, wi);
		return cosine > 0 ? (mExponent + 1) * std::pow(cosine, mExponent) * INV_2PI : 0;
	}

	Vec3 Evaluate(const Vec3& wo, const Vec3& wi) const override {
		const real cosine = MirrorCosine(wo, wi);
		return cosine > 0 ? mKs * ((mExponent + 2) * INV_2PI * std::pow(cosine, mExponent)) : Vec3(0);
	}

	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand) const override {
		const real phi = 2 * PI * rand[0];
		const real cosTheta = std::pow(rand[1], 1 / (mExponent + 1));
		const real sinTheta = std::sqrt((std::max)(real(0), 1 - cosTheta * cosTheta));
		const Vec3 mirror(-wo.x, -wo.y, wo.z);
		Vec3 tangent, bitangent;
		CoordinateSystem(mirror, &tangent, &bitangent);
		*wi = tangent * (std::cos(phi) * sinTheta) + bitangent * (std::sin(phi) * sinTheta) + mirror * cosTheta;
		*pdf = Pdf(wo, *wi);
		return Evaluate(wo, *wi);
	}

private:
	// Reflection only: both directions above the surface and inside the lobe.
	static real MirrorCosine(const Vec3& wo, const Vec3& wi) {
		if (wo.z <= 0 || wi.z <= 0) return 0;
		return -wo.x * wi.x - wo.y * wi.y + wo.z * wi.z;
	}

	Vec3 mKs;
	real mExponent, mSamplingWeight;
};

// Diffuse plus Phong, as SmallVCM's glossy floor. Lobes are chosen by albedo luminance.
class PhongMaterial : public Material {
public:
	PhongMaterial(const std::shared_ptr<Texture<Vec3>>& kd, const std::shared_ptr<Texture<Vec3>>& ks, real exponent)
		: mKd(kd), mKs(ks), mExponent(exponent) {}

	void ComputeScatteringFunction(Intersection* isect, MemoryPool& arena,
		TransportMode mode = TransportMode::Radiance) const {
		const Vec3 kd = mKd->Sample(*isect), ks = mKs->Sample(*isect);
		isect->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*isect);
		// DiffuseBSDF has unit sampling weight; the Phong weight is relative to it.
		if (Luminance(kd) > 0) isect->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(kd));
		const real phongWeight = Luminance(kd) > 0 ? Luminance(ks) / Luminance(kd) : 1;
		if (Luminance(ks) > 0) isect->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, PhongBSDF)(ks, mExponent, phongWeight));
	}

private:
	static real Luminance(const Vec3& value) {
		return real(0.212671) * value.x + real(0.715160) * value.y + real(0.072169) * value.z;
	}

	std::shared_ptr<Texture<Vec3>> mKd, mKs;
	real mExponent;
};

GYT_NAMESPACE_END
