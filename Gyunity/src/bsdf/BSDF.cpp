#include "BSDF.h"
#include "visual/Rng.h"
#include <cmath>
#include <limits>

GYT_NAMESPACE_BEGIN

real FrDielectric(real cosThetaI, real etaI, real etaT) {
	cosThetaI = Clamp(cosThetaI, -1.f, 1.f);
	// Potentially swap indices of refraction
	bool entering = cosThetaI > 0.f;
	if (!entering) {
		std::swap(etaI, etaT);
		cosThetaI = std::abs(cosThetaI);
	}

	// Compute _cosThetaT_ using Snell's law
	real sinThetaI = std::sqrt(std::max((real)0, 1 - cosThetaI * cosThetaI));
	real sinThetaT = etaI / etaT * sinThetaI;

	// Handle total internal reflection
	if (sinThetaT >= 1) return 1;
	real cosThetaT = std::sqrt(std::max((real)0, 1 - sinThetaT * sinThetaT));
	real Rparl = ((etaT * cosThetaI) - (etaI * cosThetaT)) /
		((etaT * cosThetaI) + (etaI * cosThetaT));
	real Rperp = ((etaI * cosThetaI) - (etaT * cosThetaT)) /
		((etaI * cosThetaI) + (etaT * cosThetaT));
	return (Rparl * Rparl + Rperp * Rperp) / 2;
}

real FrDiffuseDielectric(real eta) {
	// Egan/Hilgeman (1973) and d'Eon/Irving (2011) fits, as used by Mitsuba.
	if (eta < 1)
		return real(0.0636) / eta + real(0.6681) + eta * (real(0.7099) - real(1.4399) * eta);
	const real inv = 1 / eta;
	return real(0.919317) + inv * (real(-3.4793) + inv * (real(6.75335) +
		inv * (real(-7.80989) + inv * (real(4.98554) - real(1.36881) * inv))));
}

// https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
Vec3 FrConductor(real cosThetaI, const Vec3& etai, const Vec3& etat, const Vec3& k) {
	cosThetaI = Clamp(cosThetaI, -1, 1);
	Vec3 eta = etat / etai;
	Vec3 etak = k / etai;

	real cosThetaI2 = cosThetaI * cosThetaI;
	real sinThetaI2 = 1. - cosThetaI2;
	Vec3 eta2 = eta * eta;
	Vec3 etak2 = etak * etak;

	Vec3 t0 = eta2 - etak2 - sinThetaI2;
	Vec3 a2plusb2 = Sqrt(t0 * t0 + 4 * eta2 * etak2);
	Vec3 t1 = a2plusb2 + cosThetaI2;
	Vec3 a = Sqrt(0.5f * (a2plusb2 + t0));
	Vec3 t2 = (real)2 * cosThetaI * a;
	Vec3 Rs = (t1 - t2) / (t1 + t2);

	Vec3 t3 = cosThetaI2 * a2plusb2 + sinThetaI2 * sinThetaI2;
	Vec3 t4 = t2 * sinThetaI2;
	Vec3 Rp = Rs * (t3 - t4) / (t3 + t4);

	return 0.5 * (Rp + Rs);
}

void BSDF::Add(BxDF* bxdf) {
	mIsDelta = (mBsdfCnt == 0 || mIsDelta) && bxdf->IsDelta();
	mBxdfs[mBsdfCnt++] = bxdf;
}

bool BSDF::SamplingWeights(const Vec3& woLocal, ScatterEventType flags, real* weights, real* total) const {
	*total = 0;
	for (int i = 0; i < mBsdfCnt; ++i) {
		weights[i] = mBxdfs[i]->MatchesTypes(flags) ? mBxdfs[i]->SamplingWeight(woLocal) : 0;
		if (!std::isfinite(weights[i]) || weights[i] < 0) {
			GetDefaultLogger()->Error("BSDF sampling weights must be finite and nonnegative.", false);
			return false;
		}
		*total += weights[i];
	}
	if (!std::isfinite(*total)) {
		GetDefaultLogger()->Error("BSDF sampling weight sum is not finite.", false);
		return false;
	}
	return true;
}

real BSDF::Pdf(const Vec3& wo, const Vec3& wi, ScatterEventType flags) const {
	const Vec3 woLocal = WorldToLocal(wo), wiLocal = WorldToLocal(wi);
	real weights[MAX_BSDFS], total;
	if (!SamplingWeights(woLocal, flags, weights, &total))
		return std::numeric_limits<real>::quiet_NaN();
	if (total == 0) return 0;
	real pdf = 0;
	for (int i = 0; i < mBsdfCnt; ++i)
		if (weights[i] > 0 && !mBxdfs[i]->IsDelta())
			pdf += weights[i] * mBxdfs[i]->Pdf(woLocal, wiLocal);
	return pdf / total;
}

Vec3 BSDF::Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand, ScatterEventType flags,
	ScatterEventType* sampledType, real* eventProbability) const {
	*wi = Vec3(0);
	*pdf = 0;
	if (sampledType) *sampledType = ScatterEventType(0);
	if (eventProbability) *eventProbability = 0;
	const Vec3 woLocal = WorldToLocal(wo);
	real weights[MAX_BSDFS], total;
	if (!SamplingWeights(woLocal, flags, weights, &total)) {
		*pdf = std::numeric_limits<real>::quiet_NaN();
		if (eventProbability) *eventProbability = *pdf;
		return Vec3(0);
	}
	if (total == 0) return Vec3(0);

	const real target = rand[0] * total;
	real cumulative = 0, offset = 0;
	int selected = -1;
	for (int i = 0; i < mBsdfCnt; ++i) {
		if (weights[i] == 0) continue;
		selected = i;
		offset = cumulative;
		cumulative += weights[i];
		if (target < cumulative) break;
	}
	BxDF* bxdf = mBxdfs[selected];
	if (sampledType) *sampledType = bxdf->mScatterEventType;
	if (eventProbability) {
		real groupWeight = 0;
		for (int i = 0; i < mBsdfCnt; ++i)
			if (mBxdfs[i]->IsDelta() == bxdf->IsDelta()) groupWeight += weights[i];
		*eventProbability = groupWeight / total;
	}
	const real remapped = std::min((target - offset) / weights[selected], ONE_MINUS_EPSILON);
	Vec3 wiLocal(0);
	Vec3 f = bxdf->Sample(woLocal, &wiLocal, pdf, Vec3(remapped, rand[1], rand[2]));
	*wi = LocalToWorld(wiLocal);

	// Preserve malformed lobe outputs for checked integrators to diagnose.
	if (!std::isfinite(*pdf) || *pdf < 0) return f;
	for (int c = 0; c < 3; ++c)
		if (!std::isfinite(f[c]) || f[c] < 0 || !std::isfinite((*wi)[c])) return f;
	if (*pdf == 0) {
		*wi = Vec3(0);
		return f;
	}
	if (sampledType) {
		ScatterEventType type = bxdf->mScatterEventType;
		if (SurfaceEventClassifier::IsReflection(type) && SurfaceEventClassifier::IsTransmission(type)) {
			type = ScatterEventType(type & ~(SameHemisphere(woLocal, wiLocal) ? BSDF_TRANSMISSION : BSDF_REFLECTION));
		}
		*sampledType = type;
	}
	if (bxdf->IsDelta()) {
		*pdf *= weights[selected] / total;
		return f;
	}
	*pdf *= weights[selected];
	for (int i = 0; i < mBsdfCnt; ++i)
		if (i != selected && weights[i] > 0 && !mBxdfs[i]->IsDelta())
			*pdf += weights[i] * mBxdfs[i]->Pdf(woLocal, wiLocal);
	*pdf /= total;
	f = Evaluate(wo, *wi, flags);
	return f;
}

Vec3 BSDF::Evaluate(const Vec3& wo, const Vec3& wi, ScatterEventType flags) const{
	Vec3 f(0, 0, 0);
	Vec3 woLocal = WorldToLocal(wo);
	Vec3 wiLocal = WorldToLocal(wi);
	bool reflect = Dot(wi, mGeometryNormal) * Dot(wo, mGeometryNormal) > 0;

	for (int i = 0; i < mBsdfCnt; ++i) {
		if (!mBxdfs[i]->MatchesTypes(flags)) continue;
		if ((reflect && SurfaceEventClassifier::IsReflection(mBxdfs[i]->mScatterEventType)) ||
			(!reflect && SurfaceEventClassifier::IsTransmission(mBxdfs[i]->mScatterEventType)))
			f += mBxdfs[i]->Evaluate(woLocal, wiLocal);
	}

	return f;
}


bool BSDF::MatchScatterType(ScatterEventType flag) const {
	int ret = 0;
	for (int i = 0; i < mBsdfCnt; ++i) {
		ret |= (mBxdfs[i]->mScatterEventType & flag);
	}
	return ret > 0;
}

bool BSDF::HasMixedDeltaLobes() const {
	bool delta = false, continuous = false;
	for (int i = 0; i < mBsdfCnt; ++i) {
		if (mBxdfs[i]->IsDelta()) delta = true;
		else continuous = true;
	}
	return delta && continuous;
}

bool BSDF::HasNonDeltaLobes() const {
	for (int i = 0; i < mBsdfCnt; ++i)
		if (!mBxdfs[i]->IsDelta()) return true;
	return false;
}
GYT_NAMESPACE_END