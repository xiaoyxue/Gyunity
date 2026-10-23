#pragma once

#include "Linalg.h"
#include <algorithm>
#include "common/Core.h"
#include "numeric/EFloat.h"
#include "common/DebugUtils.h"

GYT_NAMESPACE_BEGIN

GYT_FORCE_INLINE void CoordinateSystem(const Vec3 &v1, Vec3 *v2, Vec3 *v3) {
	if (std::abs(v1.x) > std::abs(v1.y))
		*v2 = Vec3(-v1.z, 0, v1.x) / std::sqrt(v1.x * v1.x + v1.z * v1.z);
	else
		*v2 = Vec3(0, v1.z, -v1.y) / std::sqrt(v1.y * v1.y + v1.z * v1.z);
	*v3 = v1.Cross(*v2);
}


GYT_FORCE_INLINE Vec3 Reflect(const Vec3 &inDir, const Vec3 &n) {
	return 2.f * inDir.Dot(n) * n - inDir;
}


GYT_FORCE_INLINE Vec3 SphericalDirection(real sinTheta, real cosTheta, real phi) {
	return Vec3(sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta);
}

GYT_FORCE_INLINE Vec3 SphericalDirection(real sinTheta, real cosTheta, real phi, 
	const Vec3& x, const Vec3& y, const Vec3& z) {
	return sinTheta * std::cos(phi) * x + sinTheta * std::sin(phi) * y +
		cosTheta * z;
}

GYT_FORCE_INLINE bool SameHemisphere(const Vec3& w, const Vec3& wp) {
	return w.z * wp.z > 0;
}

GYT_FORCE_INLINE Vec3 OffsetRayOrigin(const Vec3 &p, const Vec3 &pError,
	const Vec3 &n, const Vec3 &w) {
	real d = Dot(Abs(n), pError);
#ifdef GYT_USING_DOUBLE
	// We have tons of precision; for now bump up the offset a bunch just
	// to be extra sure that we start on the right side of the surface
	// (In case of any bugs in the epsilons code...)
	d *= 1024.;
#endif
	Vec3 offset = d * n;// +n * 1e-4f;

	if (Dot(w, n) < 0) {
		offset = -offset;
	}
	Vec3 po = p + offset;

	// Round offset point _po_ away from _p_
	for (int i = 0; i < 3; ++i) {
		if (offset[i] > 0)
			po[i] = NextFloatUp(po[i]);
		else if (offset[i] < 0)
			po[i] = NextFloatDown(po[i]);
	}

	return po;
}

GYT_FORCE_INLINE Vec3 Faceforward(const Vec3& v, const Vec3& v2) {
	return (Dot(v, v2) < 0.f) ? -v : v;
}

GYT_NAMESPACE_END