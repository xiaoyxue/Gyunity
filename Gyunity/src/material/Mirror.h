#pragma once
#include "common/Core.h"
#include "Material.h"
#include "texture/Texture.h"
#include "bsdf/BSDF.h"
GYT_NAMESPACE_BEGIN

class MirrorMaterial : public Material {
public:
	MirrorMaterial(const std::shared_ptr<Texture<Vec3>>& kr) : mKr(kr) {

	}
	//void ComputeScatteringFunction(Intersection* isect,
	//	TransportMode mode = TransportMode::Radiance) const {

	//	isect->bsdf = std::shared_ptr<BSDF>(new SpecularBSDF(*isect, kr->Sample(*isect)));
	//}
	//void ComputeScatteringFunction(Intersection* isect,
	//	TransportMode mode = TransportMode::Radiance) const {

	//	isect->bsdf = std::shared_ptr<BSDF>(new BSDF(*isect));
	//	std::shared_ptr<BxDF> specularBSDF(new SpecularBSDF(kr->Sample(*isect)));
	//	isect->bsdf->Add(specularBSDF);
	//}

	void ComputeScatteringFunction(Intersection* isect, MemoryPool &arena,
		TransportMode mode = TransportMode::Radiance) const {

		isect->mBsdf = MEMORY_POOL_ALLOC(arena, BSDF)(*isect);

		isect->mBsdf->Add(MEMORY_POOL_ALLOC(arena, SpecularBSDF)(mKr->Sample(*isect)));
	}
private:
	std::shared_ptr<Texture<Vec3>> mKr;
};

GYT_NAMESPACE_END