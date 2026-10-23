#pragma once
#include "common/Core.h"
#include "Material.h"
#include "texture/Texture.h"
#include "bsdf/BSDF.h"

GYT_NAMESPACE_BEGIN

class GlassMaterial : public Material {
public:
	GlassMaterial(const std::shared_ptr<Texture<Vec3>>& kr, const std::shared_ptr<Texture<Vec3>> &kt, real n1 = 1.f, real n2 = 1.5f)
		: kr(kr), kt(kt), eta1(n1), eta2(n2)
	{

	}
	//void ComputeScatteringFunction(Intersection* isect,
	//	TransportMode mode = TransportMode::Radiance) const {

	//	isect->bsdf =
	//		std::shared_ptr<BSDF>(new TransmissionBSDF(*isect, kt->Sample(*isect), mode, 1.0, 1.5));
	//}
	//void ComputeScatteringFunction(Intersection* isect,
	//	TransportMode mode = TransportMode::Radiance) const {

	//	isect->bsdf = std::shared_ptr<BSDF>(new BSDF(*isect));
	//	std::shared_ptr<BxDF> transmissionBSDF(new TransmissionBSDF(kt->Sample(*isect), kt->Sample(*isect), mode, 1.0, 1.5));
	//	isect->bsdf->Add(transmissionBSDF);
	//}

	void ComputeScatteringFunction(Intersection* isect, MemoryPool &arena,
		TransportMode mode = TransportMode::Radiance) const {
		isect->mBsdf = MEMORY_POOL_ALLOC(arena, BSDF)(*isect);
		isect->mBsdf->Add(MEMORY_POOL_ALLOC(arena, TransmissionBSDF)(kt->Sample(*isect), kr->Sample(*isect), mode, eta1, eta2));
	}
private:
	std::shared_ptr<Texture<Vec3>> kr, kt;
	real eta1, eta2;
};

GYT_NAMESPACE_END