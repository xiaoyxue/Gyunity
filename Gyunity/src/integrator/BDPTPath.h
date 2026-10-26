#pragma once
#include "PathVertex.h"
#include "visual/Scene.h"

GYT_NAMESPACE_BEGIN

// Environment mDirection points toward the map, opposite an emitted ray.
// Finite destinations use geometric-normal area density; infinity retains solid angle.
bool ConvertDensity(real pdf, const PathVertex& vertex, const PathVertex& next, real* converted);

namespace Bidirectional {

// Combined scattering factor K = |dot(wi, ns)| * C_mode for a shading-normal surface event, where
// C_Importance = |wo.ns| |wi.ng| / (|wo.ng| |wi.ns|) is the Veach/PBRT-v3 adjoint correction and
// C_Radiance = 1. wo points toward the subpath predecessor and wi toward the successor.
// Support: each direction must lie strictly on the same side of ns and ng; otherwise value = 0
// (a valid null event). Returns false for malformed normals or directions.
bool ScatteringCosine(const Intersection& isect, const Vec3& wo, const Vec3& wi, TransportMode mode, double* value);

// True when w lies strictly on the same side of the shading and geometric normals.
bool ShadingSupport(const Intersection& isect, const Vec3& w);

// Prepares the BSDF frame before ComputeScatteringFunction. Geometric replaces the shading
// frame with the geometric normal. Shading keeps the interpolated normal, normalizes both
// normals, and builds a ray-independent orthonormal tangent frame from dpdu.
bool PrepareSurfaceFrame(Intersection* isect, SurfaceNormalMode mode);

}

// Counts include the endpoint. -1 reports an unsupported/invalid path; discard partial output.
// BSDF pointers remain valid until the caller resets arena, after all path connections.
// Both subpaths of one render must use the same normal policy.
int GenerateCameraPath(const Scene& scene, const Camera& camera, StateSequence& rand, MemoryPool& arena,
	const Ray& cameraRay, int maxVertices, SurfaceNormalMode normalMode, std::vector<PathVertex>& cameraPath);
int GenerateLightPath(const Scene& scene, StateSequence& rand, MemoryPool& arena,
	int maxVertices, SurfaceNormalMode normalMode, std::vector<PathVertex>& lightPath);

GYT_NAMESPACE_END
