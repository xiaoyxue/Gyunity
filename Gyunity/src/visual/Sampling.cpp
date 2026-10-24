#include "Sampling.h"

GYT_NAMESPACE_BEGIN

Vec2 ConcentricSampleDisk(const Vec2 &u) {
	// Map uniform Random numbers to $[-1,1]^2$
	Vec2 uOffset = 2.0 * u - Vec2(1, 1);

	// Handle degeneracy at the origin
	if (uOffset.x == 0 && uOffset.y == 0) return Vec2(0, 0);

	// Apply concentric mapping to point
	real theta, r;
	if (std::abs(uOffset.x) > std::abs(uOffset.y)) {
		r = uOffset.x;
		theta = PI_OVER_4 * (uOffset.y / uOffset.x);
	}
	else {
		r = uOffset.y;
		theta = PI_OVER_2 - PI_OVER_4 * (uOffset.x / uOffset.y);
	}
	return r * Vec2(std::cos(theta), std::sin(theta));
}

Vec3 UniformSampleSphere(const Vec2 &u) {
	real z = 1 - 2 * u.x;
	real r = std::sqrt(std::max((real)0, (real)1 - z * z));
	real phi = 2 * PI * u.y;
	return Vec3(r * std::cos(phi), r * std::sin(phi), z);
}

Vec3 CosineSampleHemisphere(const Vec2 &u) {
	Vec2 diskSample = ConcentricSampleDisk(u);
	real z = std::sqrt(std::max((real)0, 1 - diskSample.x * diskSample.x - diskSample.y * diskSample.y));
	return Vec3(diskSample.x, diskSample.y, z);
}

Vec2 UniformSampleTriangle(const Vec2& u) {
	real su0 = std::sqrt(u[0]);
	return Vec2(1 - su0, u[1] * su0);
}

real BalanceHeuristic(int nf, real fPdf, int ng, real gPdf) {
	return (nf * fPdf) / (nf * fPdf + ng * gPdf);
}


real CosineHemispherePdf(real cosTheta) { 
	return cosTheta * INV_PI; 
}

real UniformSpherePdf() {
	return INV_4PI;
}

Gyunity::Vec3 UniformSampleHemisphere(const Vec2& u)
{
	real x = std::cos(2 * PI * u.y) * std::sqrt(1 - u.x * u.x);
	real y = std::sin(2 * PI * u.y) * std::sqrt(1 - u.x * u.x);
	real z = u.x;
	return Vec3(x, y, z);
}

Gyunity::real UniformSampleHemispherePdf()
{
	return INV_2PI;
}

GYT_NAMESPACE_END