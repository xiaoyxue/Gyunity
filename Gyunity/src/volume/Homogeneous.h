#pragma once

#include "visual/Medium.h"

GYT_NAMESPACE_BEGIN

//class MediumIntersection;

class HomogeneousMedium : public Medium {
public:
	HomogeneousMedium(const Vec3 &sigmaA, const Vec3 &sigmaS, real g):
		mSigmaA(sigmaA), mSigmaS(sigmaS), mSigmaT(mSigmaS + mSigmaA), mG(g) {}

	Vec3 Tr(const Ray& ray, StateSequence& rand) const;

	Vec3 Sample(const Ray& ray, StateSequence& rand, MemoryPool& arena, MediumIntersection* mi) const;

	Vec3 EquiAngularSampling(const Ray& ray, StateSequence& rand, MemoryPool& arena,
		const Intersection& lightPoint, MediumIntersection* mi) const override;

private:
	Vec3 mSigmaA, mSigmaS, mSigmaT;
	real mG;
};

GYT_NAMESPACE_END
