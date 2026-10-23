#include "Primitive.h"

GYT_NAMESPACE_BEGIN

bool Primitive::Intersect(const Ray& r, Intersection* isect) const {
	real t;
	if (mShape->Intersect(r, isect, &t)) {
		r.mTmax = t;
		isect->mPrimitive = this;
		isect->mPrimId = this->mPrimId;
		if (mMediumInterface.IsMediumTransition())
			isect->mMediumInterface = mMediumInterface;
		else
			isect->mMediumInterface = MediumInterface(r.mMedium);
		return true;
	}
	return false;
}

bool Primitive::Intersect(const Ray& r) const {
	return mShape->Intersect(r);
}

void Primitive::ComputeScatteringFunction(Intersection* isect, MemoryPool& arena,
	TransportMode mode) const {
	if (mMaterial) {
		mMaterial->ComputeScatteringFunction(isect, arena, mode);
	}
}

void Primitive::QueryIntersectionInfo(const Ray& ray, Intersection* isect) const {
	mShape->QueryIntersectionInfo(ray, isect);
}

GYT_NAMESPACE_END