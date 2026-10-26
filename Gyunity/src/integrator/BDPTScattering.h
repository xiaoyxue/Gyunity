#pragma once
#include "PathVertex.h"

GYT_NAMESPACE_BEGIN

namespace Bidirectional {

enum class ScatterStatus { Valid, Null, Invalid };

bool UnitDirection(const Vec3& dir);
bool ShadingSupport(const Intersection& isect, const Vec3& dir);

// Geometric keeps the legacy frame. Shading builds a ray-independent frame from ns and dpdu.
bool PrepareSurfaceFrame(Intersection* isect, SurfaceNormalMode mode);

// K = |wi.ns| * C_mode; importance C = |wo.ns| |wi.ng| / (|wo.ng| |wi.ns|).
// Both directions point away from the surface. Unsupported directions give K = 0;
// malformed inputs fail. K belongs to contributions, never to PDFs.
bool ScatteringCosine(const Intersection& isect, const Vec3& wo, const Vec3& wi,
	TransportMode mode, double* value);

// beta *= f * K / pdf. Preserve the geometric float order and shading double products.
// The caller has prepared the frame and validated the sampled f, wi and positive pdf.
// A null event leaves throughput unchanged; an invalid result has already been logged.
ScatterStatus ScatterThroughput(const PathVertex& vertex, TransportMode mode,
	const Vec3& wo, const Vec3& wi, const Vec3& f, real pdf, Vec3* throughput);

}

GYT_NAMESPACE_END
