#pragma once
#include "PathVertex.h"

GYT_NAMESPACE_BEGIN

namespace Bidirectional {

bool ShadingSupport(const Intersection& isect, const Vec3& dir);

// Geometric keeps the legacy frame. Shading builds a ray-independent frame from ns and dpdu,
// and fails only for degenerate or back-facing shading normals in the scene data.
bool PrepareSurfaceFrame(Intersection* isect, SurfaceNormalMode mode);

// K = |wi.ns| * C_mode; importance C = |wo.ns| |wi.ng| / (|wo.ng| |wi.ns|).
// Both unit directions point away from a surface prepared by PrepareSurfaceFrame.
// Unsupported directions give K = 0. K belongs to contributions, never to PDFs.
double ScatteringCosine(const Intersection& isect, const Vec3& wo, const Vec3& wi, TransportMode mode);

// beta *= f * K / pdf. Preserve the geometric float order and shading double products.
// Returns false for a null event (K = 0), which leaves the throughput unchanged.
bool ScatterThroughput(const PathVertex& vertex, TransportMode mode,
	const Vec3& wo, const Vec3& wi, const Vec3& f, real pdf, Vec3* throughput);

}

GYT_NAMESPACE_END
