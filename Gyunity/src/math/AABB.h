#pragma once

#include <algorithm>
#include "math/Linalg.h"
#include "math/Ray.h"

GYT_NAMESPACE_BEGIN

struct AABB {
	Vec3 minPoint, maxPoint; // axis aligned bounding box
	AABB() {
		minPoint = Vec3(INF, INF, INF);
		maxPoint = Vec3(-INF, -INF, -INF);
	}
	AABB(const Vec3 &p0, const Vec3 &p1) {
		minPoint = Vec3(std::min(p0.x, p1.x), std::min(p0.y, p1.y), std::min(p0.z, p1.z));
		maxPoint = Vec3(std::max(p0.x, p1.x), std::max(p0.y, p1.y), std::max(p0.z, p1.z));
	}

	Vec3& operator[](int i) {
		return (i == 0) ? minPoint : maxPoint;
	}

	const Vec3& operator[](int i) const {
		return (i == 0) ? minPoint : maxPoint;
	}
	bool Intersect(const Ray& ray, real* hitt0, real* hitt1) const;

	GYT_FORCE_INLINE void Fit(const Vec3 &pos)
	{
		if (pos.x < minPoint.x) minPoint.x = pos.x; // min
		if (pos.y < minPoint.y) minPoint.y = pos.y; // min
		if (pos.z < minPoint.z) minPoint.z = pos.z; // min
		maxPoint.x = std::max(pos.x, maxPoint.x);
		maxPoint.y = std::max(pos.y, maxPoint.y);
		maxPoint.z = std::max(pos.z, maxPoint.z);
	}
	GYT_FORCE_INLINE void Reset() {
		minPoint = Vec3(INF, INF, INF);
		maxPoint = Vec3(-INF, -INF, -INF);
	}

	GYT_FORCE_INLINE Vec3 Diagonal() const { return maxPoint - minPoint; }

	GYT_FORCE_INLINE real SurfaceArea() const {
		Vec3 diagonal = Diagonal();
		return 2 * (diagonal.x * diagonal.y + diagonal.x * diagonal.z + diagonal.y * diagonal.z);
	}

	GYT_FORCE_INLINE real Volume() const {
		Vec3 diagonal = Diagonal();
		return diagonal.x * diagonal.y * diagonal.z;
	}

	GYT_FORCE_INLINE real MaximumExtent() const {
		Vec3 diagonal = Diagonal();
		if (diagonal.x > diagonal.y&& diagonal.x > diagonal.z)
			return 0;
		else if (diagonal.y > diagonal.z)
			return 1;
		else
			return 2;
	}

	GYT_FORCE_INLINE Vec3 Center() const {
		return (maxPoint + minPoint) * 0.5f;
	}

	GYT_FORCE_INLINE void GetBoundingSphere(Vec3* center, real* radius) const {
		*center = (minPoint + maxPoint) * 0.5;
		*radius = (maxPoint - minPoint).Length() * 0.5;
	}
};

GYT_FORCE_INLINE std::ostream& operator<<(std::ostream &os, const AABB &aabb) {
	os << "AABB: \n" << "minPoint: " << aabb.minPoint << std::endl << "maxPoint: " << aabb.maxPoint;
	return os;
}

AABB Union(const AABB& a, const AABB& b);
AABB Union(const AABB& a, const Vec3& pos);

using BBox = AABB;

GYT_NAMESPACE_END