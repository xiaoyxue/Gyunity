#include "triangle.h"
#include "sampling.h"

NAMESPACE_BEGIN

bool Triangle::Intersect(const Ray& ray, Intersection* isect, real* t) const {
	Vec3 e1 = p1 - p0;
	Vec3 e2 = p2 - p0;
	Vec3 s1 = Cross(ray.d, e2);
	real divisor = Dot(s1, e1);

	if (divisor == 0.)
		return false;
	real invDivisor = 1.f / divisor;

	// Compute first barycentric coordinate
	Vec3 s = ray.o - p0;
	real b1 = Dot(s, s1) * invDivisor;
	if (b1 < 0. || b1 > 1.)
		return false;

	// Compute second barycentric coordinate
	Vec3 s2 = Cross(s, e1);
	real b2 = Dot(ray.d, s2) * invDivisor;
	if (b2 < 0. || b1 + b2 > 1.)
		return false;

	// Compute _t_ to intersection point
	real tHit = Dot(e2, s2) * invDivisor;
	if (tHit < ray.tMin || tHit > ray.tMax)
		return false;

	// Compute triangle partial derivatives

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
	real du1 = uvs[0][0] - uvs[2][0];
	real du2 = uvs[1][0] - uvs[2][0];
	real dv1 = uvs[0][1] - uvs[2][1];
	real dv2 = uvs[1][1] - uvs[2][1];
	Vec3 dp1 = p0 - p2, dp2 = p1 - p2;
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

	*t = tHit;
	isect->hit = b0 * p0 + b1 * p1 + b2 * p2;
	isect->uv = Vec2(u, v);
	isect->n = faceNormal;
	isect->nl = isect->n.Dot(ray.d) < 0 ? isect->n : isect->n * -1;
	isect->dpdu = dpdu;
	isect->dpdv = dpdv;
	isect->wo = -ray.d;
	isect->pError = gamma(7) * Vec3(xAbsSum, yAbsSum, zAbsSum);

	return true;
}

bool Triangle::Intersect(const Ray& ray) const {
	Vec3 e1 = p1 - p0;
	Vec3 e2 = p2 - p0;
	Vec3 s1 = Cross(ray.d, e2);
	real divisor = Dot(s1, e1);

	//std::cout << "divisor: " << divisor << std::endl;

	if (divisor == 0.)
		return false;
	real invDivisor = 1.f / divisor;

	// Compute first barycentric coordinate
	Vec3 s = ray.o - p0;
	real b1 = Dot(s, s1) * invDivisor;

	//std::cout << "b1: " << b1 << std::endl;

	if (b1 < 0. || b1 > 1.)
		return false;

	// Compute second barycentric coordinate
	Vec3 s2 = Cross(s, e1);
	real b2 = Dot(ray.d, s2) * invDivisor;
	if (b2 < 0. || b1 + b2 > 1.)
		return false;

	// Compute _t_ to intersection point
	real tHit = Dot(e2, s2) * invDivisor;
	if (tHit < ray.tMin || tHit > ray.tMax)
		return false;

	return true;
}

Intersection Triangle::Sample(real* pdf, const Vec2& rand) const {
	Vec2 b = UniformSampleTriangle(rand);
	Intersection isect;
	isect.hit = b[0] * p0 + b[1] * p1 + (1 - b[0] - b[1]) * p2;
	isect.n = faceNormal;
	isect.nl = faceNormal;

	Vec3 pAbsSum =
		Abs(b[0] * p0) + Abs(b[1] * p1) + Abs((1 - b[0] - b[1]) * p2);
	isect.pError = gamma(6) * Vec3(pAbsSum.x, pAbsSum.y, pAbsSum.z);

	*pdf = 1 / Area();

	return isect;

}

NAMESPACE_END