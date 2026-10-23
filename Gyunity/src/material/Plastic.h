#pragma once

#include "Material.h"
#include "visual/Microfacet.h"
#include "texture/Texture.h"
#include "bsdf/BSDF.h"

GYT_NAMESPACE_BEGIN

class PlasticMaterial : public Material {
public:
	PlasticMaterial(const std::shared_ptr<Texture<Vec3>>& Kd, const std::shared_ptr<Texture<Vec3>>& Ks, const std::shared_ptr<Texture<real>>& roughness):
		kd(Kd), ks(Ks), roughness(roughness){}

	void ComputeScatteringFunction(Intersection* isect, MemoryPool& arena, TransportMode mode = TransportMode::Radiance) const {
		real alpha = GGXDistribution::RoughnessToAlpha(roughness->Sample(*isect));
		isect->mBsdf = MEMORY_POOL_ALLOC(arena, BSDF)(*isect);

		Vec3 Kd = kd->Sample(*isect);
		isect->mBsdf->Add(MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(Kd));
		Fresnel* fresnel = MEMORY_POOL_ALLOC(arena, FresnelDielectric)(1.5f, 1.f);
		MicrofacetDistribution* distribution =
			MEMORY_POOL_ALLOC(arena, GGXDistribution)(alpha, alpha, true);
		Vec3 Ks = ks->Sample(*isect);
		BxDF* microfacetReflectionBSDF = MEMORY_POOL_ALLOC(arena, MicrofacetReflectionBSDF)(distribution, fresnel, Ks);
		isect->mBsdf->Add(microfacetReflectionBSDF);
	}
private:
	std::shared_ptr<Texture<Vec3>> kd, ks;
	std::shared_ptr<Texture<real>> roughness;
};

GYT_NAMESPACE_END