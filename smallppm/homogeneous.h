#pragma once

#include "medium.h"

NAMESPACE_BEGIN

//class MediumIntersection;

class HomogeneousMedium : public Medium {
public:
	HomogeneousMedium(const Vec3 &sigmaA, const Vec3 &sigmaS, real g):
		sigma_a(sigmaA), sigma_s(sigmaS), sigma_t(sigma_s + sigma_a), g(g) {}

	Vec3 Tr(const Ray& ray, StateSequence& rand) const;

	Vec3 Sample(const Ray& ray, StateSequence& rand, MemoryArena& arena, MediumIntersection* mi) const;
private:
	Vec3 sigma_a, sigma_s, sigma_t;
	real g;
};

NAMESPACE_END
