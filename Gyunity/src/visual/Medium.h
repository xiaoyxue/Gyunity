#pragma once

#include "common/Core.h"
#include "math/Linalg.h"
#include "math/Ray.h"
#include "sampler/Sampler.h"
#include "system/Memory.h"

GYT_NAMESPACE_BEGIN

class PhaseFunction {
public:
    // PhaseFunction Interface
    virtual ~PhaseFunction() {};
    virtual real Evaluate(const Vec3& wo, const Vec3& wi) const = 0;
    virtual real SampleP(const Vec3& wo, Vec3* wi, const Vec2& u) const = 0;
};


// Media Inline Functions
GYT_FORCE_INLINE real PhaseHG(real cosTheta, real g) {
    real denom = 1 + g * g + 2 * g * cosTheta;
    return INV_4PI * (1 - g * g) / (denom * std::sqrt(denom));
}

class Intersection;
class MediumIntersection;
class Medium {
public:
    // Medium Interface
    virtual ~Medium() {}
    virtual Vec3 Tr(const Ray& ray, StateSequence& rand) const = 0;
    virtual Vec3 Sample(const Ray& ray, StateSequence& rand, MemoryPool& arena, MediumIntersection* mi) const = 0;
    virtual Vec3 EquiAngularSampling(const Ray& ray, StateSequence& rand, MemoryPool& arena,
        const Intersection& lightPoint, MediumIntersection* mi) const = 0;
};


// HenyeyGreenstein Declarations
class HenyeyGreenstein : public PhaseFunction {
public:
    // HenyeyGreenstein Public Methods
    HenyeyGreenstein(real g) : mG(g) {}
    real Evaluate(const Vec3& wo, const Vec3& wi) const;
    real SampleP(const Vec3& wo, Vec3* wi, const Vec2& sample) const;
private:
    const real mG;
};

// MediumInterface Declarations
struct MediumInterface {
    MediumInterface() : inside(nullptr), outside(nullptr) {}
    // MediumInterface Public Methods
    MediumInterface(const Medium* medium) : inside(medium), outside(medium) {}
    MediumInterface(const Medium* inside, const Medium* outside)
        : inside(inside), outside(outside) {}
    bool IsMediumTransition() const { return inside != outside; }
    const Medium* inside, * outside;
};

GYT_NAMESPACE_END