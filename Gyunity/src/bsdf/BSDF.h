#pragma once
#include "common/DebugUtils.h"

#include "math/Linalg.h"
#include "common/Core.h"
#include "visual/Intersection.h"
#include "math/Transform.h"
#include "visual/Sampling.h"
#include "visual/Microfacet.h"
#include "system/Threading.h"
#include "math/MathUtils.h"
#include "math/Frame.h"
#include "common/Core.h"

GYT_NAMESPACE_BEGIN

namespace BSDFCoordinate {
	GYT_FORCE_INLINE real CosTheta(const Vec3& w) { return w.z; }

	GYT_FORCE_INLINE real Cos2Theta(const Vec3& w) { return w.z * w.z; }

	GYT_FORCE_INLINE real AbsCosTheta(const Vec3& w) { return std::abs(w.z); }

	GYT_FORCE_INLINE real Sin2Theta(const Vec3& w) {
		return std::max((real)0, (real)1 - Cos2Theta(w));
	}

	GYT_FORCE_INLINE real SinTheta(const Vec3& w) { return std::sqrt(Sin2Theta(w)); }

	GYT_FORCE_INLINE real TanTheta(const Vec3& w) { return SinTheta(w) / CosTheta(w); }

	GYT_FORCE_INLINE real Tan2Theta(const Vec3& w) {
		return Sin2Theta(w) / Cos2Theta(w);
	}

	GYT_FORCE_INLINE real CosPhi(const Vec3& w) {
		real sinTheta = SinTheta(w);
		return (sinTheta == 0) ? 1 : Clamp(w.x / sinTheta, -1, 1);
	}

	GYT_FORCE_INLINE real SinPhi(const Vec3& w) {
		real sinTheta = SinTheta(w);
		return (sinTheta == 0) ? 0 : Clamp(w.y / sinTheta, -1, 1);
	}

	GYT_FORCE_INLINE real Cos2Phi(const Vec3& w) { return CosPhi(w) * CosPhi(w); }

	GYT_FORCE_INLINE real Sin2Phi(const Vec3& w) { return SinPhi(w) * SinPhi(w); }

	GYT_FORCE_INLINE real CosDPhi(const Vec3& wa, const Vec3& wb) {
		return Clamp(
			(wa.x * wb.x + wa.y * wb.y) / std::sqrt((wa.x * wa.x + wa.y * wa.y) *
			(wb.x * wb.x + wb.y * wb.y)),
			-1, 1);
	}
}

enum ScatterEventType
{
	BSDF_REFLECTION = 1 << 0,
	BSDF_TRANSMISSION = 1 << 1,
	BSDF_DIFFUSE = 1 << 2,
	BSDF_GLOSSY = 1 << 3,
	BSDF_SPECULAR = 1 << 4,
	BSDF_ALL_TYPES = BSDF_DIFFUSE | BSDF_GLOSSY | BSDF_SPECULAR,
	BSDF_ALL_REFLECTION = BSDF_REFLECTION | BSDF_ALL_TYPES,
	BSDF_ALL_TRANSMISSION = BSDF_TRANSMISSION | BSDF_ALL_TYPES,
	BSDF_ALL = BSDF_ALL_REFLECTION | BSDF_ALL_TRANSMISSION
};

real FrDielectric(real cosThetaI, real etaI, real etaT);

// https://seblagarde.wordpress.com/2013/04/29/memo-on-fresnel-equations/
Vec3 FrConductor(real cosThetaI, const Vec3& etai, const Vec3& etat, const Vec3& k);


class Fresnel {
public:
	// Fresnel Interface
	virtual ~Fresnel(){}
	virtual Vec3 Evaluate(real cosI) const = 0;
	virtual real EvaluateScalar(real cosI) const = 0;
};

class FresnelConductor : public Fresnel {
private:
	Vec3 mEtaI, mEtaT, mK;
public:
	FresnelConductor(const Vec3& etaI, const Vec3& etaT, const Vec3& k)
		: mEtaI(etaI), mEtaT(etaT), mK(k) {}

	Vec3 Evaluate(real cosThetaI) const override{
		return FrConductor(std::abs(cosThetaI), mEtaI, mEtaT, mK);
	}

	real EvaluateScalar(real cosI) const override {
		return 0;
	}
};

class FresnelDielectric : public Fresnel {
private:
	real mEtaI, mEtaT;
public:
	FresnelDielectric(real etaI, real etaT) : mEtaI(etaI), mEtaT(etaT) {}

	Vec3 Evaluate(real cosThetaI) const {
		return FrDielectric(cosThetaI, mEtaI, mEtaT);
	}

	real EvaluateScalar(real cosThetaI) const override {
		return FrDielectric(cosThetaI, mEtaI, mEtaT);
	}
};


class BxDF;
class BSDF {
protected:
	Vec3 mGeometryNormal, mNormal;
	Frame mFrame;
	static constexpr int MAX_BSDFS = 8;
	BxDF* mBxdfs[MAX_BSDFS];
	int mBsdfCnt = 0;
	bool mIsDelta = false;
public:
	BSDF(const Intersection &isect)
		: mNormal(isect.mNormal), mGeometryNormal(isect.mGeometryNormal), mFrame(isect.mShadingDpDu, isect.mShadingDpDv, isect.mNormal)
	{
	}
	GYT_FORCE_INLINE Vec3 WorldToLocal(const Vec3& v) const 
	{
		return mFrame.WorldToLocal(v);
	}
	GYT_FORCE_INLINE Vec3 LocalToWorld(const Vec3& v) const
	{
		return mFrame.LocalToWorld(v);
	}
	~BSDF(){}
	void Add(BxDF* bxdf);
	real Pdf(const Vec3 &wo, const Vec3 &wi, ScatterEventType flags = BSDF_ALL) const;
	Vec3 Sample(const Vec3 &wo, Vec3 *wi, real *pdf, const Vec3 &rand = Vec3(0, 0, 0), ScatterEventType flags = BSDF_ALL) const;
	Vec3 Evaluate(const Vec3 &wo, const Vec3 &wi, ScatterEventType flags = BSDF_ALL) const;
	bool IsDelta() const { 
		return mIsDelta;
	}
	bool MatchScatterType(ScatterEventType flag) const;
	bool HasMixedDeltaLobes() const;

};

class BxDF {
public:
	BxDF(ScatterEventType scatterType) : mScatterEventType(scatterType){}
	virtual ~BxDF(){}
	virtual real Pdf(const Vec3& wo, const Vec3& wi) const = 0;
	virtual Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand = Vec3(0, 0, 0)) const = 0;
	virtual Vec3 Evaluate(const Vec3& wo, const Vec3& wi) const = 0;
	virtual bool IsDelta() const { return mScatterEventType & BSDF_SPECULAR; }
	virtual bool MatchesTypes(ScatterEventType flags) const { return (mScatterEventType & flags) == mScatterEventType; }
	const ScatterEventType mScatterEventType;
};

class DiffuseBSDF : public BxDF {
private:
	Vec3 mR;
public:
	DiffuseBSDF(Vec3 r) 
		: BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_DIFFUSE)), mR(r) 
	{ 
	}

	real Pdf(const Vec3& wo, const Vec3& wi) const override 
	{
		return SameHemisphere(wo, wi) ? BSDFCoordinate::AbsCosTheta(wi) * INV_PI : 0.f;
	}

	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand) const override 
	{
		*wi = CosineSampleHemisphere(Vec2(rand[0], rand[1]));
		if (wo.z < 0) wi->z *= -1;
		*pdf = Pdf(wo, *wi);
		return Evaluate(wo, *wi);
	}

	Vec3 Evaluate(const Vec3 &wo, const Vec3 &wi) const override {
		return mR * INV_PI;
	}
};

class SpecularBSDF : public BxDF {
public:
	SpecularBSDF(Vec3 r = Vec3(1.0, 1.0, 1.0)) 
		: BxDF(ScatterEventType(BSDF_ALL_REFLECTION | BSDF_SPECULAR)), mR(r) 
	{
	}

	real Pdf(const Vec3 &wo, const Vec3 &wi) const override 
	{
		return 0.0;
	}

	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand) const override 
	{
		*wi = Vec3(-wo.x, -wo.y, wo.z);
		*pdf = 1.0;
		real cosTheta = BSDFCoordinate::AbsCosTheta(*wi);
		return mR / cosTheta;
	}

	Vec3 Evaluate(const Vec3 &wo, const Vec3 &wi) const override 
	{
		return Vec3();
	}

	inline bool IsDelta() const override { return true; }
private:
	Vec3 mR;
};

//TODO

class TransmissionBSDF : public BxDF 
{
private:
	real mNc, mNt;
	Vec3 mR, mT;
	TransportMode mTransportMode;
public:
	TransmissionBSDF(const Vec3 kt = Vec3(1.0, 1.0, 1.0), const Vec3 kr = Vec3(1.0, 1.0, 1.0), 
		TransportMode mode = TransportMode::Radiance, real eta1 = 1.0, real eta2 = 1.5) 
		:BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_SPECULAR)),
		mT(kt), mR(kr), mNc(eta1), mNt(eta2), mTransportMode(mode) 
	{
	}

	real Pdf(const Vec3 &wo, const Vec3 &wi) const override 
	{
		return 0.0;
	}

	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand) const override 
	{

		real Re = FrDielectric(BSDFCoordinate::CosTheta(wo), mNc, mNt);

		bool entering = BSDFCoordinate::CosTheta(wo) > 0;
		real etaI = entering ? mNc : mNt;
		real etaT = entering ? mNt : mNc;
		Vec3 nForward = BSDFCoordinate::CosTheta(wo) > 0 ? Vec3(0, 0, 1) : Vec3(0, 0, -1);
		real P = Re * 0.5f + 0.25f;
		P = Re;
		//P = 1.0;
		if (rand[1] < P) {
			//*wi = (2 * wo.Dot(nForward) * nForward - wo).Norm();
			*wi = Vec3(-wo.x, -wo.y, wo.z);
			*pdf = P;
			real cosTheta = BSDFCoordinate::AbsCosTheta(*wi);
			return mR * Re / cosTheta;
		}
		else {
			if (!Refract(wo, nForward, etaI / etaT, wi)) {
				return Vec3();
			}
			*pdf = 1 - P;
			real cosTheta = BSDFCoordinate::AbsCosTheta(*wi);
			real eta = etaI / etaT;
			if (mTransportMode == TransportMode::Radiance) {
				return eta * eta * mT * (1.f - Re) / cosTheta;
			}
			else {
				return mT * (1.f - Re) / cosTheta;
			}
		}
	}

	Vec3 Evaluate(const Vec3 &wo, const Vec3 &wi) const override 
	{
		return Vec3();
	}

	real FresnelSchlick(const Vec3 &wo, const Vec3 &td, const Vec3 &normal, const Vec3 &absNormal) const 
	{
		bool into = (normal.Dot(absNormal) > 0.0);
		real nnt = into ? mNc / mNt : mNt / mNc, ddn = (-1 * wo).Dot(absNormal);//, cos2t;
		//cos2t = 1 - nnt * nnt*(1 - ddn * ddn);
		real a = mNt - mNc, b = mNt + mNc, R0 = a * a / (b * b), c = 1 - (into ? -ddn : td.Dot(normal));
		real Re = R0 + (1 - R0) * c * c* c * c * c;
		return Re;
	}

	inline bool IsDelta() const override { return true; }

	static bool Refract(const Vec3 &wi, const Vec3 &normal, real eta, Vec3 *wt) 
	{
		// Compute $\cos \theta_\roman{t}$ using Snell's law
		real cosThetaI = normal.Dot(wi);
		real sin2ThetaI = std::max(real(0), real(1 - cosThetaI * cosThetaI));
		real sin2ThetaT = eta * eta * sin2ThetaI;

		// Handle total internal reflection for transmission
		if (sin2ThetaT >= 1) return false;
		real cosThetaT = std::sqrt(1 - sin2ThetaT);
		*wt = eta * (-1 * wi) + (eta * cosThetaI - cosThetaT) * Vec3(normal);
		return true;
	}
};


class MicrofacetReflectionBSDF : public BxDF {
private:
	std::unique_ptr<MicrofacetDistribution> mDistribution;
	std::unique_ptr<Fresnel> mFresnel;
	Vec3 mR;

public:

	MicrofacetReflectionBSDF(MicrofacetDistribution* distribution, Fresnel *fresnel, const Vec3& R) 
		:BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_GLOSSY)), mDistribution(distribution), mFresnel(fresnel), mR(R) 
	{
	}

	real Pdf(const Vec3& wo, const Vec3& wi) const override 
	{
		if (!SameHemisphere(wo, wi)) return 0;
		real pdf = 0;
		Vec3 wh = wo + wi;
		if (wh == Vec3(0)) return 0;
		real dwhDwi;
		wh = wh.Norm();
		dwhDwi = 1.f / (4 * Dot(wi, wh));
		wh = wh * Sgn(BSDFCoordinate::CosTheta(wh));
		pdf = std::abs(mDistribution->Pdf(wo, wh) * dwhDwi);
		return pdf;
	}

	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand) const override 
	{
		if (wo.z == 0) return 0.;
		//Vec3 wh = distribution->SampleWh(wo, Vec2(rand[0], rand[1]));
		Vec3 wh = mDistribution->SampleWh(Sgn(BSDFCoordinate::CosTheta(wo)) * wo, Vec2(rand[0], rand[1]));
		Vec3 microfacetPdf = mDistribution->Pdf(wo, wh);;
		if (microfacetPdf == 0) return Vec3();

		//if (Dot(wo, wh) < 0) return 0.;   // Should be rare
		*wi = Reflect(wo, wh);
		if (!SameHemisphere(wo, *wi)) return Vec3(0, 0, 0);

		wh = Sgn(BSDFCoordinate::CosTheta(wh)) * wh;

		// Compute PDF of _wi_ for microfacet reflection
		*pdf = std::abs(mDistribution->Pdf(wo, wh) / (4 * Dot(*wi, wh)));
		return Evaluate(wo, *wi);
	}

	Vec3 Evaluate(const Vec3& wo, const Vec3& wi) const override {
		if (!SameHemisphere(wo, wi)) return Vec3();

		real cosThetaO = BSDFCoordinate::AbsCosTheta(wo), cosThetaI = BSDFCoordinate::AbsCosTheta(wi);
		Vec3 wh = wi + wo;
		// Handle degenerate cases for microfacet reflection
		if (cosThetaI == 0 || cosThetaO == 0) return Vec3(0, 0, 0);
		if (wh.x == 0 && wh.y == 0 && wh.z == 0) return Vec3(0, 0, 0);
		wh = wh.Norm();
		// For the Fresnel call, make sure that wh is in the same hemisphere
		// as the surface normal, so that TIR is handled correctly.
		wh = Sgn(BSDFCoordinate::CosTheta(wh)) * wh;
		Vec3 F = mFresnel->Evaluate(Dot(wo, wh));
		Vec3 D = mDistribution->D(wh);
		Vec3 G = mDistribution->G(wo, wi, wh);
		return mR * F * D * G / (4 * cosThetaI * cosThetaO);

	}
};


class MicrofacetTransmissionBSDF : public BxDF {
private:
	const Vec3 mT;
	const real mEtaA, mEtaB;
	const FresnelDielectric mFresnel;
	const TransportMode mTransportMode;
	std::unique_ptr<MicrofacetDistribution> mDistribution;
public:
	// MicrofacetTransmission Public Methods
	MicrofacetTransmissionBSDF(MicrofacetDistribution* distribution, const Vec3& T, real etaA, real etaB, TransportMode mode)
		: BxDF(ScatterEventType(BSDF_TRANSMISSION | BSDF_GLOSSY)),
		mT(T),
		mDistribution(distribution),
		mEtaA(etaA),
		mEtaB(etaB),
		mFresnel(etaA, etaB),
		mTransportMode(mode)
	{
	}

	Vec3 Evaluate(const Vec3& wo, const Vec3& wi) const override 
	{
		if (SameHemisphere(wo, wi)) return Vec3();  // transmission only

		real cosThetaO = BSDFCoordinate::CosTheta(wo);
		real cosThetaI = BSDFCoordinate::CosTheta(wi);
		if (cosThetaI == 0 || cosThetaO == 0) return Vec3();

		real eta = BSDFCoordinate::CosTheta(wo) > 0 ? (mEtaB / mEtaA) : (mEtaA / mEtaB);
		Vec3 wh = (wo + wi * eta).Norm();
		if (wh.z < 0) wh = -wh;

		Vec3 F = mFresnel.Evaluate(Dot(wo, wh));

		real sqrtDenom = Dot(wo, wh) + eta * Dot(wi, wh);
		real factor = (mTransportMode == TransportMode::Radiance) ? (1 / eta) : 1;

		return (Vec3(1.f) - F) * mT *
			std::abs(mDistribution->D(wh) * mDistribution->G(wo, wi, wh) * eta * eta *
				std::abs(Dot(wi, wh)) * std::abs(Dot(wo, wh)) * factor * factor /
				(cosThetaI * cosThetaO * sqrtDenom * sqrtDenom));
	}

	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand) const override 
	{
		if (wo.z == 0) return 0.;
		//Vec3 wh = distribution->SampleWh(wo, Vec2(rand[0], rand[1]));
		Vec3 wh = mDistribution->SampleWh(Sgn(BSDFCoordinate::CosTheta(wo)) * wo, Vec2(rand[0], rand[1]));
		real eta = BSDFCoordinate::CosTheta(wo) > 0 ? (mEtaA / mEtaB) : (mEtaB / mEtaA);
		if (!TransmissionBSDF::Refract(wo, Faceforward(wh, wo), eta, wi)) {
			*pdf = 0;
			return Vec3();
		}
		*pdf = Pdf(wo, *wi);

		return Evaluate(wo, *wi);
	}

	real Pdf(const Vec3& wo, const Vec3& wi) const override
	{
		if (SameHemisphere(wo, wi)) return 0;

		real eta = BSDFCoordinate::CosTheta(wo) > 0 ? (mEtaB / mEtaA) : (mEtaA / mEtaB);
		Vec3 wh = -((wo + wi * eta).Norm());

		wh = Sgn(BSDFCoordinate::CosTheta(wh)) * wh;
		// Compute change of variables _dwh\_dwi_ for microfacet transmission
		real sqrtDenom = Dot(wo, wh) + eta * Dot(wi, wh);
		real dwhDwi = std::abs((eta * eta * Dot(wi, wh)) / (sqrtDenom * sqrtDenom));

		return mDistribution->Pdf(wo, wh) * dwhDwi;
	}
};


class RoughDielectricBSDF : public BxDF{
private:
	const Vec3 mR, mT;
	const real mEtaA, mEtaB;
	const FresnelDielectric mFresnel;
	const TransportMode mTransportMode;
	std::unique_ptr<MicrofacetDistribution> mDistribution;

public:
	RoughDielectricBSDF(MicrofacetDistribution* distribution, const Vec3 &R, const Vec3& T, real etaA, real etaB, TransportMode mode)
		: BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_GLOSSY)),
		mR(R), mT(T),
		mDistribution(distribution),
		mEtaA(etaA),
		mEtaB(etaB),
		mFresnel(etaA, etaB),
		mTransportMode(mode) 
	{
	}

	Vec3 Evaluate(const Vec3& wo, const Vec3& wi) const override 
	{
		bool reflect = BSDFCoordinate::CosTheta(wo) * BSDFCoordinate::CosTheta(wi) > 0;
		Vec3 wh;
		if (reflect) 
		{
			real cosThetaO = BSDFCoordinate::AbsCosTheta(wo), cosThetaI = BSDFCoordinate::AbsCosTheta(wi);
			wh = wi + wo;
			// Handle degenerate cases for microfacet reflection
			if (cosThetaI == 0 || cosThetaO == 0) return Vec3(0, 0, 0);
			if (wh.x == 0 && wh.y == 0 && wh.z == 0) return Vec3(0, 0, 0);
			wh = wh.Norm();
			// For the Fresnel call, make sure that wh is in the same hemisphere
			// as the surface normal, so that TIR is handled correctly.
			wh = Sgn(BSDFCoordinate::CosTheta(wh)) * wh;
			real F = mFresnel.EvaluateScalar(Dot(wo, wh));
			Vec3 D = mDistribution->D(wh);
			Vec3 G = mDistribution->G(wo, wi, wh);
			return mR * F * D * G / (4 * cosThetaI * cosThetaO);
		}
		else 
		{

			real cosThetaO = BSDFCoordinate::CosTheta(wo);
			real cosThetaI = BSDFCoordinate::CosTheta(wi);
			if (cosThetaI == 0 || cosThetaO == 0) return Vec3();

			real eta = BSDFCoordinate::CosTheta(wo) > 0 ? (mEtaB / mEtaA) : (mEtaA / mEtaB);
			Vec3 wh = (-(wo + wi * eta).Norm());
			wh = Sgn(BSDFCoordinate::CosTheta(wh)) * wh;

			real F = mFresnel.EvaluateScalar(Dot(wo, wh));
			real D = mDistribution->D(wh);
			real G = mDistribution->G(wo, wi, wh);


			real sqrtDenom = Dot(wo, wh) + eta * Dot(wi, wh);
			real factor = (mTransportMode == TransportMode::Radiance) ? (1 / eta) : 1;
			
			real value = (1 - F) * D * G * eta * eta * std::abs(Dot(wi, wh)) * std::abs(Dot(wo, wh)) / std::abs(cosThetaI * cosThetaO * sqrtDenom * sqrtDenom);

			return mT * std::abs(value * factor * factor);

		}
	}

	real Pdf(const Vec3& wo, const Vec3& wi) const override 
	{

		bool reflect = BSDFCoordinate::CosTheta(wo) * BSDFCoordinate::CosTheta(wi) > 0;
		real pdf = 0;
		Vec3 wh;
		real dwhDwi ;
		if (reflect) {
			wh = (wo + wi).Norm();
			dwhDwi = 1.f / (4 * Dot(wi, wh));
		}
		else {
			real eta = BSDFCoordinate::CosTheta(wo) > 0 ? (mEtaB / mEtaA) : (mEtaA / mEtaB);
			wh = -((wo + wi * eta).Norm());

			// Compute change of variables _dwh\_dwi_ for microfacet transmission
			real sqrtDenom = Dot(wo, wh) + eta * Dot(wi, wh);
			dwhDwi = std::abs((eta * eta * Dot(wi, wh)) / (sqrtDenom * sqrtDenom));
		}

		wh = wh * Sgn(BSDFCoordinate::CosTheta(wh));

		pdf = std::abs(mDistribution->Pdf(wo, wh) * dwhDwi);

		real F = mFresnel.EvaluateScalar(Dot(wo, wh));

		pdf *= reflect ? F : (1 - F);

		return pdf;
	}

	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& rand) const override 
	{
		Vec3 wh = mDistribution->SampleWh(Sgn(BSDFCoordinate::CosTheta(wo)) * wo, Vec2(rand[0], rand[1]));
		real microfacetPdf = mDistribution->Pdf(wo, wh);
		if (microfacetPdf == 0) return Vec3();

		real F = mFresnel.EvaluateScalar(Dot(wo, wh));
		real P = F;
		*pdf = 0;
		if (IsPixelDebugEnabled()) {
			std::cout << "SampleF fresnel : " << F << std::endl;
		}
		if (rand[2] < P) {
			if (wo.z == 0) return Vec3();
			//if (Dot(wo, wh) < 0) return 0.;   // Should be rare
			*wi = Reflect(wo, wh);

			if (!SameHemisphere(wo, *wi)) return Vec3(0, 0, 0);
			
			real dwhDwi = 1.f / (4 * Dot(*wi, wh));

			*pdf = P * std::abs(mDistribution->Pdf(wo, wh) * dwhDwi);

			real cosThetaO = BSDFCoordinate::AbsCosTheta(wo), cosThetaI = BSDFCoordinate::AbsCosTheta(*wi);

			Vec3 D = mDistribution->D(wh);

			Vec3 G = mDistribution->G(wo, *wi, wh);

			return mR * F * D * G / (4 * cosThetaI * cosThetaO);
		}
		else 
		{

			if (IsPixelDebugEnabled()) {
				std::cout << "SampleF wo : " << wo << " Dot(wo, wh): " << Dot(wo, wh) << std::endl;
			}

			if (wo.z == 0) return Vec3();
			//if (Dot(wo, wh) < 0) return 0.;  // Should be rare

			real etaRefract = BSDFCoordinate::CosTheta(wo) > 0 ? (mEtaA / mEtaB) : (mEtaB / mEtaA);
			if (!TransmissionBSDF::Refract(wo, Faceforward(wh, wo), etaRefract, wi)) {
				*pdf = 0;
				return Vec3();
			}

			if (BSDFCoordinate::CosTheta(wo) * BSDFCoordinate::CosTheta(*wi) >= 0) return Vec3();

			real eta = BSDFCoordinate::CosTheta(wo) > 0 ? (mEtaB / mEtaA) : (mEtaA / mEtaB);
			real sqrtDenom = Dot(wo, wh) + eta * Dot(*wi, wh);
			real dwhDwi = std::abs((eta * eta * Dot(*wi, wh)) / (sqrtDenom * sqrtDenom));
			*pdf = std::abs((1 - P) * mDistribution->Pdf(wo, wh) * dwhDwi);


			real cosThetaO = BSDFCoordinate::CosTheta(wo);
			real cosThetaI = BSDFCoordinate::CosTheta(*wi);
			if (cosThetaI == 0 || cosThetaO == 0) return Vec3();

			real D = mDistribution->D(wh);

			real G = mDistribution->G(wo, *wi, wh);

			real factor = (mTransportMode == TransportMode::Radiance) ? (1 / eta) : 1;

			real value = (1 - F) * D * G * eta * eta * std::abs(Dot(*wi, wh)) * std::abs(Dot(wo, wh)) / std::abs(cosThetaI * cosThetaO * sqrtDenom * sqrtDenom);

			return mT * std::abs(value * factor * factor);
		}

	}
};

GYT_NAMESPACE_END