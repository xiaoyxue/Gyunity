#pragma once

#include "Linalg.h"
#include "common/Core.h"
#include "ForwardDecl.h"

GYT_NAMESPACE_BEGIN

struct Ray {
	Ray() 
		: mMedium(nullptr)
	{
		mTmax = INF;
		mTmin = 0.f;
	}

	Ray(Vec3 orig, Vec3 dir, real tmax = INF, real tmin = 0.f, const Medium* medium = nullptr) 
		: mOrig(orig), mDir(dir), mTmax(tmax), mTmin(tmin), mMedium(medium) 
	{}

	Vec3 operator()(real t) const 
	{
		return mOrig + mDir * t;
	}

	Vec3 mOrig, mDir;
	// Deprecated: mTmin. Self-intersection is avoided by offsetting the ray origin
	// (OffsetRayOrigin), so new code should treat tmin as 0. Triangle, BVHAccel and Sphere
	// no longer read it; Transform still does.
	mutable real mTmin, mTmax;
	const Medium* mMedium;
};

std::ostream& operator<<(std::ostream &os, const Ray &ray);

GYT_NAMESPACE_END