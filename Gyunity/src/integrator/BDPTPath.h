#pragma once
#include "PathVertex.h"
#include "BDPTScattering.h"
#include "visual/Scene.h"

GYT_NAMESPACE_BEGIN

// Environment mDirection points toward the map, opposite an emitted ray.
// Finite destinations use geometric-normal area density; infinity retains solid angle.
bool ConvertDensity(real pdf, const PathVertex& vertex, const PathVertex& next, real* converted);

// Counts include the endpoint. -1 reports an unsupported/invalid path; discard partial output.
// BSDF pointers remain valid until the caller resets arena, after all path connections.
// Both subpaths of one render must use the same normal policy.
int GenerateCameraPath(const Scene& scene, const Camera& camera, StateSequence& rand, MemoryPool& arena,
	const Ray& cameraRay, int maxVertices, SurfaceNormalMode normalMode, std::vector<PathVertex>& cameraPath);
int GenerateLightPath(const Scene& scene, StateSequence& rand, MemoryPool& arena,
	int maxVertices, SurfaceNormalMode normalMode, std::vector<PathVertex>& lightPath);

GYT_NAMESPACE_END
