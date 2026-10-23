#pragma once
#include "common/Core.h"
#include "Material.h"
#include "texture/Texture.h"
#include "bsdf/BSDF.h"
#include "visual/Intersection.h"

GYT_NAMESPACE_BEGIN


class DiffuseMaterial: public Material{
public:
	DiffuseMaterial(const std::shared_ptr<Texture<Vec3>> &kd) : mKd(kd) {

	}
	//void ComputeScatteringFunction(Intersection* isect,
	//	TransportMode mode = TransportMode::Radiance) const {
	//	isect->bsdf = std::shared_ptr<BSDF>(new DiffuseBSDF(*isect, kd->Sample(*isect)));
	//}

	//void ComputeScatteringFunction(Intersection* isect,
	//	TransportMode mode = TransportMode::Radiance) const {
	//	isect->bsdf = std::shared_ptr<BSDF>(new BSDF(*isect));
	//	std::shared_ptr<BxDF> diffuseBSDF(new DiffuseBSDF(kd->Sample(*isect)));
	//	isect->bsdf->Add(diffuseBSDF);
	//}
	void ComputeScatteringFunction(Intersection* isect, MemoryPool& arena,
		TransportMode mode = TransportMode::Radiance) const {
		isect->mBsdf = MEMORY_POOL_ALLOC(arena, BSDF)(*isect);
		isect->mBsdf->Add(MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(mKd->Sample(*isect)));
	}
private:
	std::shared_ptr<Texture<Vec3>> mKd;
};

GYT_NAMESPACE_END