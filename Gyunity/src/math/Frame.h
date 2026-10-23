#pragma once
#include "common/Core.h"
#include "math/Linalg.h"
#include "math/GeometryUtils.h"

GYT_NAMESPACE_BEGIN

class Frame
{
private:
	Vec3 mX, mY, mZ;
public:

	Frame()
	{
		mX = Vec3(1, 0, 0);
		mY = Vec3(0, 1, 0);
		mZ = Vec3(0, 0, 1);
	}

	Frame(const Vec3 &x, const Vec3 &y, const Vec3 &z)
		: mX(x), mY(y), mZ(z)
	{
	}

	Frame(const Vec3& normal)
	{
		mZ = normal;
		CoordinateSystem(mZ, &mX, &mY);
	}

	GYT_FORCE_INLINE void SetFromZ(const Vec3& normal) 
	{
		mZ = normal;
		CoordinateSystem(mZ, &mX, &mY);
	}

	Vec3 LocalToWorld(const Vec3& v) const
	{
		return mX * v.x + mY * v.y + mZ * v.z;
	}

	Vec3 WorldToLocal(const Vec3& v) const
	{
		return Vec3(Dot(v, mX), Dot(v, mY), Dot(v, mZ));
	}

	GYT_FORCE_INLINE const Vec3& Binormal() const { return mX; }

	GYT_FORCE_INLINE const Vec3& Tangent() const { return mY; }

	GYT_FORCE_INLINE const Vec3& Normal() const { return mZ; }
};

GYT_NAMESPACE_END