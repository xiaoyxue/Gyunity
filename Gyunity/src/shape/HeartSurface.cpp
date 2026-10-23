#include "HeartSurface.h"

GYT_NAMESPACE_BEGIN

bool HeartSurface::BinarySearch(real left, real right, const Ray &ray, real *t) const {
	if (left > right) return false;
	real sgn = Sgn(F(ray(left)));
	while (std::abs(left - right) > 1e-6) {
		real mid = (left + right) / 2;
		real val = sgn * F(ray(mid));
		if (val == 0) {
			*t = mid;
			return true;
		}
		else if (val > 0) left = mid;
		else right = mid;
	}
	*t = left;
	return true;
}

bool HeartSurface::Intersect(const Ray& r, Intersection* isect, real* t) const {
	Ray ray = mWorldToObject(r);

	real t0, t1;
	if (!mBounding.Intersect(ray, &t0, &t1)) return false;


	real segmentStart = std::max(ray.mTmin, t0) - 1e-3;
	real segmentEnd = std::min(ray.mTmax, t1) + 1e-3;
	if (segmentStart > segmentEnd) return false;


	real step = (segmentEnd - segmentStart) / 10;
	real F0 = F(ray(segmentStart)), F1 = F(ray(segmentStart));
	real segmentPoint0 = segmentStart, segmentPoint1 = segmentStart;
	for (real tTest = segmentStart + step; tTest <= segmentEnd; tTest += step) {
		F1 = F(ray(tTest));
		segmentPoint1 = tTest;
		if (F0 * F1 < 0) break;
		F0 = F1;
		segmentPoint0 = segmentPoint1;
	}

	if (F0 * F1 > 0) return false;

	if (!BinarySearch(segmentPoint0, segmentPoint1, ray, t)) return false;

	isect->mPos = ray(*t);
	isect->mPos = mObjectToWorld.TransformPoint(ray(*t));

	isect->mRayEps = 5e-3;
	//std::cout << isect->rayEps << std::endl;
	isect->mShapeId = mShapeId;
	return true;
}

bool HeartSurface::Intersect(const Ray& r) const {
	Ray ray = mWorldToObject(r);

	real t0, t1;
	if (!mBounding.Intersect(ray, &t0, &t1)) return false;

	real segmentStart = std::max(ray.mTmin, t0) - 1e-3;
	real segmentEnd = std::min(ray.mTmax, t1) + 1e-3;
	if (segmentStart > segmentEnd) return false;

	real step = (segmentEnd - segmentStart) / 10;
	real F0 = F(ray(segmentStart)), F1 = F(ray(segmentStart));
	real segmentPoint0 = segmentStart, segmentPoint1 = segmentStart;
	for (real tTest = segmentStart + step; tTest <= segmentEnd; tTest += step) {
		F1 = F(ray(tTest));
		segmentPoint1 = tTest;
		if (F0 * F1 < 0) break;
		F0 = F1;
		segmentPoint0 = segmentPoint1;
	}

	if (F0 * F1 > 0) return false;

	real t;
	if (!BinarySearch(segmentPoint0, segmentPoint1, ray, &t)) return false;

	return true;
}

void HeartSurface::QueryIntersectionInfo(const Ray& ray, Intersection* isect) const {
	Vec3 p = mWorldToObject(isect->mPos);

	Vec3 normal = Gradient(p);

	normal = mObjectToWorld.TransformNormal(normal).Norm();

	Vec3 dpdu, dpdv;
	CoordinateSystem(normal, &dpdu, &dpdv);
	isect->mGeometryNormal = normal;
	isect->mAbsNormal = isect->mGeometryNormal.Dot(ray.mDir) < 0 ? isect->mGeometryNormal : isect->mGeometryNormal * -1;
	//std::cout << "nl: " << std::endl;
	//std::cout << isect->nl << std::endl;
	//std::cout << Sgn(Dot(normal, -ray.d)) * normal << std::endl;
	isect->mDpDu = dpdu;
	isect->mDpDv = dpdv;
	isect->mOutDir = (-ray.mDir).Norm();
	isect->SetShading(normal, dpdu, dpdv);
}

GYT_NAMESPACE_END