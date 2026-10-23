#include "Primitive.h"

GYT_NAMESPACE_BEGIN

bool Primitive::Intersect(const Ray& r, Intersection* isect) const {
	real t;
	if (mpShape->Intersect(r, isect, &t)) {
		r.mTmax = t;
		isect->mpPrimitive = this;
		isect->mPrimId = this->mPrimId;
		if (mMediumInterface.IsMediumTransition())
			isect->mMediumInterface = mMediumInterface;
		else
			isect->mMediumInterface = MediumInterface(r.mpMedium);
		return true;
	}
	return false;
}

bool Primitive::Intersect(const Ray& r) const {
	return mpShape->Intersect(r);
}

void Primitive::ComputeScatteringFunction(Intersection* isect, MemoryPool& arena,
	TransportMode mode) const {
	if (mpMaterial) {
		mpMaterial->ComputeScatteringFunction(isect, arena, mode);
	}
}

void Primitive::QueryIntersectionInfo(const Ray& ray, Intersection* isect) const {
	mpShape->QueryIntersectionInfo(ray, isect);
}

GYT_NAMESPACE_END