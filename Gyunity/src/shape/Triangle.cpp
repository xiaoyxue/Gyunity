#include "Triangle.h"
#include "visual/Sampling.h"

GYT_NAMESPACE_BEGIN

void Triangle::QueryIntersectionInfo(const Ray& ray, Intersection* isect) const {
	//Interpolate $(u,v)$ triangle parametric coordinates
	real b1 = isect->mB1, b2 = isect->mB2;
	real uvs[3][2];
	GetUVs(uvs);
	
	// Interpolate $(u,v)$ triangle parametric coordinates
	real b0 = 1 - b1 - b2;
	real u = b0 * uvs[0][0] + b1 * uvs[1][0] + b2 * uvs[2][0];
	real v = b0 * uvs[0][1] + b1 * uvs[1][1] + b2 * uvs[2][1];
	real xAbsSum =
		(std::abs(b0 * p0.x) + std::abs(b1 * p1.x) + std::abs(b2 * p2.x));
	real yAbsSum =
		(std::abs(b0 * p0.y) + std::abs(b1 * p1.y) + std::abs(b2 * p2.y));
	real zAbsSum =
		(std::abs(b0 * p0.z) + std::abs(b1 * p1.z) + std::abs(b2 * p2.z));
	
	// Compute deltas for triangle partial derivatives
	Vec3 dpdu, dpdv;

	real determinant = du1 * dv2 - dv1 * du2;
	if (std::abs(determinant) < 1e-8) {
		// Handle zero determinant for triangle partial derivative matrix
		CoordinateSystem(Cross(e2, e1).Norm(), &dpdu, &dpdv);
	}
	else {
		real invdet = 1.f / determinant;
		dpdu = (dv2 * dp1 - dv1 * dp2) * invdet;
		dpdv = (-du2 * dp1 + du1 * dp2) * invdet;
	}
	
	Vec3 ns = b0 * n0 + b1 * n1 + b2 * n2;
	ns.Normalize();

	//ns = faceNormal;

	
	//isect->hit = b0 * p0 + b1 * p1 + b2 * p2;
	isect->mUv = Vec2(u, v);
	isect->mGeometryNormal = (Dot(faceNormal, ns) < 0) ? -faceNormal : faceNormal;
	isect->mAbsNormal = isect->mGeometryNormal.Dot(ray.mDir) < 0 ? isect->mGeometryNormal : isect->mGeometryNormal * -1;
	isect->mDpDu = dpdu;
	isect->mDpDv = dpdv;
	isect->mOutDir = -ray.mDir;
	isect->mPointError = gamma(7) * Vec3(xAbsSum, yAbsSum, zAbsSum);
	

	Vec3 dpdus, dpdvs;
	//method1
	//CoordinateSystem(ns, &dpdus, &dpdvs);
	//method2
	//dpdus = isect->dpdu.Norm();
	//dpdvs = Cross(isect->dpdu, ns);
	//ns = Cross(dpdus, dpdvs).Norm();
	//ns = (Dot(ns, isect->ng) > 0) ? ns : -ns;
	
	//method3
	real sgn = (Dot(ray.mOrig - p0, ns) > 0) ? 1.0f : -1.0f;
	dpdus = e1.Norm();
	dpdvs = Cross(sgn * ns, dpdus).Norm();
	dpdus = Cross(dpdvs, ns).Norm();
	isect->SetShading(ns, dpdus, dpdvs);

}

bool Triangle::Intersect(const Ray& ray, Intersection* isect, real* t) const {
	Vec3 s1 = Cross(ray.mDir, e2);
	real divisor = Dot(s1, e1);

	if (divisor == 0.)
		return false;
	real invDivisor = 1.f / divisor;

	// Compute first barycentric coordinate
	Vec3 s = ray.mOrig - p0;
	real b1 = Dot(s, s1) * invDivisor;
	if (b1 < 0. || b1 > 1.)
		return false;

	// Compute second barycentric coordinate
	Vec3 s2 = Cross(s, e1);
	real b2 = Dot(ray.mDir, s2) * invDivisor;
	if (b2 < 0. || b1 + b2 > 1.)
		return false;

	// Compute _t_ to intersection point
	real tHit = Dot(e2, s2) * invDivisor;
	if (tHit < ray.mTmin || tHit > ray.mTmax)
		return false;

	*t = tHit;
	isect->mPos = (1 - b1 - b2) * p0 + b1 * p1 + b2 * p2;
	isect->mB1 = b1;
	isect->mB2 = b2;
	isect->mShapeId = mShapeId;

	return true;
}


bool Triangle::Intersect(const Ray& ray) const {
	//Vec3 e1 = p1 - p0;
	//Vec3 e2 = p2 - p0;
	Vec3 s1 = Cross(ray.mDir, e2);
	real divisor = Dot(s1, e1);

	//std::cout << "divisor: " << divisor << std::endl;

	if (divisor == 0.)
		return false;
	real invDivisor = 1.f / divisor;

	// Compute first barycentric coordinate
	Vec3 s = ray.mOrig - p0;
	real b1 = Dot(s, s1) * invDivisor;

	//std::cout << "b1: " << b1 << std::endl;

	if (b1 < 0. || b1 > 1.)
		return false;

	// Compute second barycentric coordinate
	Vec3 s2 = Cross(s, e1);
	real b2 = Dot(ray.mDir, s2) * invDivisor;
	if (b2 < 0. || b1 + b2 > 1.)
		return false;

	// Compute _t_ to intersection point
	real tHit = Dot(e2, s2) * invDivisor;
	if (tHit < ray.mTmin || tHit > ray.mTmax)
		return false;

	return true;
}

Intersection Triangle::Sample(real* pdf, const Vec2& rand) const {
	Vec2 b = UniformSampleTriangle(rand);
	Intersection isect;
	isect.mPos = b[0] * p0 + b[1] * p1 + (1 - b[0] - b[1]) * p2;
	isect.mGeometryNormal = faceNormal;
	isect.mAbsNormal = faceNormal;
	isect.mNormal = b[0] * n0 + b[1] * n1 + (1 - b[0] - b[1]) * n2;

	Vec3 pAbsSum =
		Abs(b[0] * p0) + Abs(b[1] * p1) + Abs((1 - b[0] - b[1]) * p2);
	isect.mPointError = gamma(6) * Vec3(pAbsSum.x, pAbsSum.y, pAbsSum.z);
	isect.mShapeId = mShapeId;

	*pdf = 1 / Area();

	return isect;

}

GYT_NAMESPACE_END