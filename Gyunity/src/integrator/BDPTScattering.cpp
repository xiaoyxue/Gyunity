#include "BDPTScattering.h"
#include <cmath>
#include <limits>

GYT_NAMESPACE_BEGIN

namespace {

bool Finite(const Vec3& value) {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

double Dot64(const Vec3& a, const Vec3& b) {
	return double(a.x) * double(b.x) + double(a.y) * double(b.y) + double(a.z) * double(b.z);
}

bool SameStrictSide(double a, double b) {
	return (a > 0 && b > 0) || (a < 0 && b < 0);
}

bool Normalized(const Vec3& v, Vec3* unit) {
	const double length2 = Dot64(v, v);
	if (!std::isfinite(length2) || length2 <= 0) return false;
	const double inverse = 1.0 / std::sqrt(length2);
	*unit = Vec3(real(v.x * inverse), real(v.y * inverse), real(v.z * inverse));
	return Finite(*unit);
}

}

namespace Bidirectional {

bool UnitDirection(const Vec3& v) {
	const double length2 = Dot64(v, v);
	return std::isfinite(length2) && std::abs(length2 - 1.0) <= 1e-4;
}

bool ShadingSupport(const Intersection& isect, const Vec3& w) {
	return SameStrictSide(Dot64(w, isect.mNormal), Dot64(w, isect.mGeometryNormal));
}

bool ScatteringCosine(const Intersection& isect, const Vec3& wo, const Vec3& wi, TransportMode mode, double* value) {
	*value = 0;
	const Vec3& ns = isect.mNormal;
	const Vec3& ng = isect.mGeometryNormal;
	if (!UnitDirection(ns) || !UnitDirection(ng) || !UnitDirection(wo) || !UnitDirection(wi) || Dot64(ns, ng) <= 0) {
		return Fail("BDPT requires prepared, consistently oriented unit normals and directions.");
	}
	if (mode != TransportMode::Radiance && mode != TransportMode::Importance) {
		return Fail("BDPT received an invalid transport mode.");
	}
	const double woNs = Dot64(wo, ns), woNg = Dot64(wo, ng);
	const double wiNs = Dot64(wi, ns), wiNg = Dot64(wi, ng);
	// Check support before cancelling |wi.ns| in the importance factor.
	if (!SameStrictSide(woNs, woNg) || !SameStrictSide(wiNs, wiNg)) return true;
	if (mode == TransportMode::Radiance) *value = std::abs(wiNs);
	else *value = std::abs(woNs) * std::abs(wiNg) / std::abs(woNg);
	if (!std::isfinite(*value)) {
		*value = 0;
		return Fail("BDPT scattering cosine is not finite.");
	}
	return true;
}

bool PrepareSurfaceFrame(Intersection* isect, SurfaceNormalMode mode) {
	if (mode == SurfaceNormalMode::Geometric) {
		isect->mNormal = isect->mGeometryNormal.Norm();
		CoordinateSystem(isect->mNormal, &isect->mShadingDpDu, &isect->mShadingDpDv);
		return true;
	}
	if (mode != SurfaceNormalMode::Shading) return Fail("BDPT received an invalid surface normal mode.");
	Vec3 ns, ng;
	if (!Normalized(isect->mNormal, &ns) || !Normalized(isect->mGeometryNormal, &ng)) {
		return Fail("BDPT surface has an invalid shading or geometric normal.");
	}
	if (Dot64(ns, ng) <= 0) return Fail("BDPT shading normal is not on the geometric normal's side.");
	Vec3 tangent;
	const double dpduLength2 = Dot64(isect->mDpDu, isect->mDpDu);
	const Vec3 projected = isect->mDpDu - ns * real(Dot64(ns, isect->mDpDu));
	if (!std::isfinite(dpduLength2) || dpduLength2 <= 0 || !(Dot64(projected, projected) > 1e-12 * dpduLength2) ||
		!Normalized(projected, &tangent)) {
		Vec3 unused;
		CoordinateSystem(ns, &tangent, &unused);
	}
	Vec3 bitangent;
	if (!Normalized(Cross(ns, tangent), &bitangent)) return Fail("BDPT could not build a shading frame.");
	isect->mNormal = ns;
	isect->mGeometryNormal = ng;
	isect->mShadingDpDu = tangent;
	isect->mShadingDpDv = bitangent;
	return true;
}

ScatterStatus ScatterThroughput(const PathVertex& vertex, TransportMode mode,
	const Vec3& wo, const Vec3& wi, const Vec3& f, real pdf, Vec3* throughput) {
	const Intersection& isect = vertex.mIsect;
	if (vertex.mNormalMode == SurfaceNormalMode::Geometric) {
		*throughput *= f * std::abs(Dot(isect.mNormal, wi)) / pdf;
	}
	else {
		double cosine;
		if (!ScatteringCosine(isect, wo, wi, mode, &cosine)) return ScatterStatus::Invalid;
		if (cosine == 0) return ScatterStatus::Null;
		Vec3 next;
		for (int channel = 0; channel < 3; ++channel) {
			const double value = double((*throughput)[channel]) * double(f[channel]) * cosine / double(pdf);
			if (!std::isfinite(value) || std::abs(value) > double((std::numeric_limits<real>::max)())) {
				Fail("BDPT path throughput is not representable.");
				return ScatterStatus::Invalid;
			}
			next[channel] = static_cast<real>(value);
		}
		*throughput = next;
	}
	if (!Finite(*throughput)) {
		Fail("BDPT path throughput is not finite.");
		return ScatterStatus::Invalid;
	}
	return ScatterStatus::Valid;
}

}

GYT_NAMESPACE_END
