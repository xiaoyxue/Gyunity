#pragma once

#include "Material.h"
#include "visual/Microfacet.h"
#include "texture/Texture.h"
#include "bsdf/BSDF.h"

GYT_NAMESPACE_BEGIN

class PlasticMaterial : public Material {
public:
	PlasticMaterial(const std::shared_ptr<Texture<Vec3>>& Kd, const std::shared_ptr<Texture<Vec3>>& Ks, const std::shared_ptr<Texture<real>>& roughness):
		mKd(Kd), mKs(Ks), mRoughness(roughness){}

	void ComputeScatteringFunction(Intersection* isect, MemoryPool& arena, TransportMode mode = TransportMode::Radiance) const {
		real alpha = GGXDistribution::RoughnessToAlpha(mRoughness->Sample(*isect));
		isect->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*isect);

		Vec3 Kd = mKd->Sample(*isect);
		isect->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(Kd));
		Fresnel* fresnel = GYT_MEMORY_POOL_ALLOC(arena, FresnelDielectric)(1.f, 1.5f);
		MicrofacetDistribution* distribution =
			GYT_MEMORY_POOL_ALLOC(arena, GGXDistribution)(alpha, alpha, true);
		Vec3 Ks = mKs->Sample(*isect);
		BxDF* microfacetReflectionBSDF = GYT_MEMORY_POOL_ALLOC(arena, MicrofacetReflectionBSDF)(distribution, fresnel, Ks);
		isect->mBsdf->Add(microfacetReflectionBSDF);
	}
private:
	std::shared_ptr<Texture<Vec3>> mKd, mKs;
	std::shared_ptr<Texture<real>> mRoughness;
};

GYT_NAMESPACE_END