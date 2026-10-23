#pragma once

#include "common/Core.h"
#include "Material.h"
#include "texture/Texture.h"
#include "bsdf/BSDF.h"
#include "visual/Microfacet.h"

GYT_NAMESPACE_BEGIN

class RoughnessMaterial : public Material {
public:
	RoughnessMaterial(const std::shared_ptr<Texture<Vec3>>& reflectence, 
		const std::shared_ptr<Texture<real>> &roughnessx, const std::shared_ptr<Texture<real>>& roughnessy,
		const std::shared_ptr<Texture<Vec3>>& eta, const std::shared_ptr<Texture<Vec3>>& k)
		: reflectence(reflectence), roughnessx(roughnessx), roughnessy(roughnessy), eta(eta), k(k) {

	}
	//void ComputeScatteringFunction(Intersection* isect,
	//	TransportMode mode = TransportMode::Radiance) const {
	//	MicrofacetDistribution *distribution = new GGXDistribution(alpha);
	//	isect->bsdf = std::shared_ptr<BSDF>(new MicrofacetReflectionBSDF(*isect, distribution, reflectence->Sample(*isect)));
	//}
	//void ComputeScatteringFunction(Intersection* isect,
	//	TransportMode mode = TransportMode::Radiance) const {
	//	real alphax = GGXDistribution::RoughnessToAlpha(roughnessx->Sample(*isect));
	//	real alphay = GGXDistribution::RoughnessToAlpha(roughnessy->Sample(*isect));
	//	MicrofacetDistribution* distribution = 
	//		new GGXDistribution(alphax, alphay, true);
	//	isect->bsdf = std::shared_ptr<BSDF>(new BSDF(*isect));
	//	Fresnel* fresnel = new FresnelConductor(Vec3(1.0, 1.0, 1.0), eta->Sample(*isect), k->Sample(*isect));
	//	std::shared_ptr<BxDF> microfacetReflectionBSDF(new MicrofacetReflectionBSDF(distribution, fresnel, reflectence->Sample(*isect)));
	//	isect->bsdf->Add(microfacetReflectionBSDF);
	//}

	void ComputeScatteringFunction(Intersection* isect, MemoryPool &arena,
		TransportMode mode = TransportMode::Radiance) const {
		real alphax = GGXDistribution::RoughnessToAlpha(roughnessx->Sample(*isect));
		real alphay = GGXDistribution::RoughnessToAlpha(roughnessy->Sample(*isect));
		MicrofacetDistribution* distribution =
			MEMORY_POOL_ALLOC(arena, GGXDistribution)(alphax, alphay, true);
		isect->mBsdf = MEMORY_POOL_ALLOC(arena, BSDF)(*isect);
		Fresnel* fresnel = MEMORY_POOL_ALLOC(arena, FresnelConductor)(Vec3(1.0, 1.0, 1.0), eta->Sample(*isect), k->Sample(*isect));
		BxDF* microfacetReflectionBSDF = MEMORY_POOL_ALLOC(arena, MicrofacetReflectionBSDF)(distribution, fresnel, reflectence->Sample(*isect));
		isect->mBsdf->Add(microfacetReflectionBSDF);
	}
private:
	std::shared_ptr<Texture<Vec3>> reflectence;
	std::shared_ptr<Texture<real>> roughnessx, roughnessy;
	std::shared_ptr<Texture<Vec3>> eta;
	std::shared_ptr<Texture<Vec3>> k;
};

GYT_NAMESPACE_END