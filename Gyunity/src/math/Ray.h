#pragma once

#include "Linagl.h"
#include "common/Core.h"
#include "ForwardDecl.h"

GYT_NAMESPACE_BEGIN

struct Ray {
	Ray() 
		: mpMedium(nullptr)
	{
		mTMax = Inf;
		mTMin = 0.f;
	}

	Ray(Vec3 orig, Vec3 dir, real tmax = Inf, real tmin = 0.f, const Medium* medium = nullptr) 
		: mOrig(orig + dir * RayEps), mDir(dir), mTMax(tmax), mTMin(tmin), mpMedium(medium) 
	{}

	Vec3 operator()(real t) const 
	{
		return mOrig + mDir * t;
	}

	Vec3 mOrig, mDir;
	mutable real mTMin, mTMax;
	const Medium* mpMedium;
};

std::ostream& operator<<(std::ostream &os, const Ray &ray);

GYT_NAMESPACE_END