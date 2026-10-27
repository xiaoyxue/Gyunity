#pragma once

#include "common/Core.h"
#include "math/Linalg.h"

GYT_NAMESPACE_BEGIN

class MicrofacetDistribution {
public:
	virtual ~MicrofacetDistribution() { }
	virtual real D(const Vec3 &wh) const = 0;
	virtual real Lambda(const Vec3& w) const = 0;
	virtual real G1(const Vec3& w, const Vec3 &wh) const {
		return 1 / (1 + Lambda(w));
	}
	virtual real G(const Vec3& wo, const Vec3& wi, const Vec3 &wh) const {
		return 1 / (1 + Lambda(wo) + Lambda(wi));
	}
	// Microfacet normals stay in the canonical +z hemisphere for either view side.
	virtual Vec3 SampleWh(const Vec3& wo, const Vec2& u) const = 0;
	virtual real Pdf(const Vec3& wo, const Vec3& wh) const;

protected:
	MicrofacetDistribution(bool sampleVisibleArea) : mSampleVisibleArea(sampleVisibleArea) {}

	const bool mSampleVisibleArea;

};

class BeckmannDistribution : public MicrofacetDistribution {
public:
	BeckmannDistribution(real alphaX, real alphaY)
		: MicrofacetDistribution(false), mAlphaX(alphaX), mAlphaY(alphaY) {}
	real D(const Vec3& normal) const override;
	real Lambda(const Vec3& direction) const override;
	real G1(const Vec3& direction, const Vec3& normal) const override;
	real G(const Vec3& wo, const Vec3& wi, const Vec3& normal) const override;
	Vec3 SampleWh(const Vec3& wo, const Vec2& sample) const override;
private:
	real mAlphaX, mAlphaY;
};

class GGXDistribution : public MicrofacetDistribution 
{
private:
	real mAlphax, mAlphay;
public:
	GGXDistribution(real alpha, bool samplevis = false)
		: MicrofacetDistribution(samplevis), mAlphax(alpha), mAlphay(alpha) {

	}

	GGXDistribution(real alphax, real alphay, bool samplevis = false) 
		: MicrofacetDistribution(samplevis), mAlphax(alphax), mAlphay(alphay) {
	}

	real D(const Vec3& wh) const;
	real G1(const Vec3& v, const Vec3 &wh) const;
	real G(const Vec3& wo, const Vec3& wi, const Vec3 &wh) const override;
	Vec3 SampleWh(const Vec3& wo, const Vec2& u) const;
	real Lambda(const Vec3& w) const;

	static inline real RoughnessToAlpha(real roughness) {
		roughness = std::max(roughness, (real)1e-3);
		real x = std::log(roughness);
		return 1.62142f + 0.819955f * x + 0.1734f * x * x + 0.0171201f * x * x * x +
			0.000640711f * x * x * x * x;
	}

	Vec3 SampleGGXVNDF(const Vec3& v, const Vec2& u) const;

	real ProjectRoughness(const Vec3& v) const;

	real SmithG1(const Vec3& v, const Vec3& m) const;

	real Pdf(const Vec3& wo, const Vec3& wh) const override;
};

GYT_NAMESPACE_END