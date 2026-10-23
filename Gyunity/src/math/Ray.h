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
	}

	Ray(Vec3 orig, Vec3 dir, real tmax = INF, const Medium* medium = nullptr) 
		: mOrig(orig), mDir(dir), mTmax(tmax), mMedium(medium) 
	{}

	Vec3 operator()(real t) const 
	{
		return mOrig + mDir * t;
	}

	Vec3 mOrig, mDir;
	mutable real mTmax;
	const Medium* mMedium;
};

std::ostream& operator<<(std::ostream &os, const Ray &ray);

GYT_NAMESPACE_END