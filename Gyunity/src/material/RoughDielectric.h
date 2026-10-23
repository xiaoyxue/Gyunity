#pragma once

#include "Material.h"
#include "visual/Microfacet.h"
#include "texture/Texture.h"
#include "bsdf/BSDF.h"

GYT_NAMESPACE_BEGIN

class RoughDielectricMaterial : public Material {
public:
    RoughDielectricMaterial(const std::shared_ptr<Texture<Vec3>>& Kr,
        const std::shared_ptr<Texture<Vec3>>& Kt,
        const std::shared_ptr<Texture<real>>& xRoughness,
        const std::shared_ptr<Texture<real>>& yRoughness,
        const std::shared_ptr<Texture<real>>& index) :
        Kr(Kr), Kt(Kt), xRoughness(xRoughness), yRoughness(yRoughness), index(index) {

    }

	void ComputeScatteringFunction(Intersection* isect, MemoryPool& arena, TransportMode mode = TransportMode::Radiance) const {
		real alphax = GGXDistribution::RoughnessToAlpha(xRoughness->Sample(*isect));
		real alphay = GGXDistribution::RoughnessToAlpha(yRoughness->Sample(*isect));
		//real alphax = xRoughness->Sample(*isect);
		//real alphay = yRoughness->Sample(*isect);
		isect->mBsdf = MEMORY_POOL_ALLOC(arena, BSDF)(*isect);

		Vec3 kr = Kr->Sample(*isect);
		Vec3 kt = Kt->Sample(*isect);
		real eta = index->Sample(*isect);
		Fresnel* fresnel = MEMORY_POOL_ALLOC(arena, FresnelDielectric)(1.f, eta);
		MicrofacetDistribution* distribution =
			MEMORY_POOL_ALLOC(arena, GGXDistribution)(alphax, alphay, true);

		BxDF* microfacetReflectionBSDF = MEMORY_POOL_ALLOC(arena, MicrofacetReflectionBSDF)(distribution, fresnel, kr);
		BxDF* microfacetTransmissionBSDF = MEMORY_POOL_ALLOC(arena, MicrofacetTransmissionBSDF)(distribution, kt, 1.0, eta, mode);
		//isect->bsdf->Add(microfacetReflectionBSDF);
		//isect->bsdf->Add(microfacetTransmissionBSDF);
		BxDF* roughDielectricBSDF = MEMORY_POOL_ALLOC(arena, RoughDielectricBSDF)(distribution, kr, kt, 1.f, eta, mode);
		isect->mBsdf->Add(roughDielectricBSDF);
	}

private:
    std::shared_ptr<Texture<Vec3>> Kr, Kt;
    std::shared_ptr<Texture<real>> xRoughness, yRoughness;
    std::shared_ptr<Texture<real>> index;
};

GYT_NAMESPACE_END