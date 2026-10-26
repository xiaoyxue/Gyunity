#pragma once

#include "Material.h"
#include "texture/Texture.h"
#include "bsdf/BSDF.h"
#include <cmath>

GYT_NAMESPACE_BEGIN

class SmoothPlasticReflectionBSDF : public BxDF {
public:
	SmoothPlasticReflectionBSDF(const Vec3& ks, real eta)
		: BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_SPECULAR)), mKs(ks), mEta(eta) {}

	real SamplingWeight(const Vec3& wo) const override {
		return wo.z > 0 ? (mKs.x + mKs.y + mKs.z) / 3 * Fresnel(wo.z) : 0;
	}

	real Pdf(const Vec3& wo, const Vec3& wi) const override { return 0; }
	Vec3 Evaluate(const Vec3& wo, const Vec3& wi) const override { return Vec3(0); }

	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand) const override {
		*wi = Vec3(0);
		*pdf = 0;
		if (wo.z <= 0) return Vec3(0);
		const real fresnel = Fresnel(wo.z);
		if (fresnel == 0) return Vec3(0);
		*wi = Vec3(-wo.x, -wo.y, wo.z);
		*pdf = 1;
		return mKs * (fresnel / wo.z);
	}

private:
	real Fresnel(real cosine) const { return mEta == 1 ? 0 : FrDielectric(cosine, 1, mEta); }
	Vec3 mKs;
	real mEta;
};

class SmoothPlasticDiffuseBSDF : public BxDF {
public:
	SmoothPlasticDiffuseBSDF(const Vec3& kd, real eta, real invEtaSquared, real fdrInt, bool nonlinear)
		: BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_DIFFUSE)), mKd(kd),
		mDiffuse(kd / (nonlinear ? Vec3(1) - kd * fdrInt : Vec3(1 - fdrInt)) * invEtaSquared), mEta(eta) {}

	real SamplingWeight(const Vec3& wo) const override {
		return wo.z > 0 ? (mKd.x + mKd.y + mKd.z) / 3 * (1 - Fresnel(wo.z)) : 0;
	}

	real Pdf(const Vec3& wo, const Vec3& wi) const override {
		return wo.z > 0 && wi.z > 0 ? wi.z * INV_PI : 0;
	}

	Vec3 Evaluate(const Vec3& wo, const Vec3& wi) const override {
		if (wo.z <= 0 || wi.z <= 0) return Vec3(0);
		return mDiffuse * ((1 - Fresnel(wo.z)) * (1 - Fresnel(wi.z)) * INV_PI);
	}

	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand) const override {
		*wi = Vec3(0);
		*pdf = 0;
		if (wo.z <= 0) return Vec3(0);
		*wi = CosineSampleHemisphere(Vec2(rand[0], rand[1]));
		*pdf = Pdf(wo, *wi);
		return Evaluate(wo, *wi);
	}

private:
	real Fresnel(real cosine) const { return mEta == 1 ? 0 : FrDielectric(cosine, 1, mEta); }
	Vec3 mKd, mDiffuse;
	real mEta;
};

class PlasticMaterial : public Material {
public:
	static std::shared_ptr<PlasticMaterial> Create(const std::shared_ptr<Texture<Vec3>>& kd,
		const std::shared_ptr<Texture<Vec3>>& ks, real etaOutside, real etaInside, bool nonlinear) {
		if (!kd || !ks || !std::isfinite(etaOutside) || !std::isfinite(etaInside) ||
			etaOutside <= 0 || etaInside <= 0) {
			GetDefaultLogger()->Error("Plastic requires reflectance textures and finite positive IORs.", false);
			return nullptr;
		}
		const real eta = etaInside / etaOutside;
		const real invEta = 1 / eta;
		const real invEtaSquared = invEta * invEta;
		const real fdrInt = FrDiffuseDielectric(invEta);
		if (!std::isfinite(eta) || eta <= 0 || !std::isfinite(invEta) || invEta <= 0 ||
			!std::isfinite(invEtaSquared) || invEtaSquared <= 0 ||
			!std::isfinite(fdrInt) || fdrInt < 0 || fdrInt >= 1 ||
			!std::isfinite(1 / (1 - fdrInt)) || !std::isfinite(invEtaSquared / (1 - fdrInt))) {
			GetDefaultLogger()->Error("Plastic IOR ratio produces an invalid diffuse Fresnel fit or scale.", false);
			return nullptr;
		}
		return std::shared_ptr<PlasticMaterial>(new PlasticMaterial(kd, ks, eta, invEtaSquared, fdrInt, nonlinear));
	}

	void ComputeScatteringFunction(Intersection* isect, MemoryPool& arena,
		TransportMode mode = TransportMode::Radiance) const override {
		isect->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*isect);
		const Vec3 kd = mKd->Sample(*isect), ks = mKs->Sample(*isect);
		if (ks != Vec3(0))
			isect->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, SmoothPlasticReflectionBSDF)(ks, mEta));
		if (kd != Vec3(0))
			isect->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, SmoothPlasticDiffuseBSDF)(
				kd, mEta, mInvEtaSquared, mFdrInt, mNonlinear));
	}

private:
	PlasticMaterial(const std::shared_ptr<Texture<Vec3>>& kd, const std::shared_ptr<Texture<Vec3>>& ks,
		real eta, real invEtaSquared, real fdrInt, bool nonlinear)
		: mKd(kd), mKs(ks), mEta(eta), mInvEtaSquared(invEtaSquared), mFdrInt(fdrInt), mNonlinear(nonlinear) {}

	std::shared_ptr<Texture<Vec3>> mKd, mKs;
	real mEta, mInvEtaSquared, mFdrInt;
	bool mNonlinear;
};

GYT_NAMESPACE_END