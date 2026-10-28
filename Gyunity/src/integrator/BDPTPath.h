#pragma once
#include "PathVertex.h"
#include "BDPTScattering.h"
#include "visual/Scene.h"

GYT_NAMESPACE_BEGIN

// Internal connection/probability ingredients shared by bidirectional estimators.
// pdfPos is an endpoint's area density, pdfDir a solid-angle density, and pdfArea the
// area density of the vertex a direction reaches.
namespace Bidirectional {

bool ValidateScene(const Scene& scene);
// Unit direction from `from` toward `to`. Environment mDirection points toward the map,
// opposite an emitted ray.
Vec3 DirectionTo(const PathVertex& from, const PathVertex& to);
// pdfDir sampled at `from` toward `to`, as an area density at `to` (geometric normal).
// A delta light cannot be reached (0); an environment destination keeps the solid angle.
real DirectionToArea(real pdfDir, const PathVertex& from, const PathVertex& to);
bool Connectible(const PathVertex& vertex);
bool LightOriginPdf(const Scene& scene, const PathVertex& light, const PathVertex& next, real* pdf);
// pdfArea of `next` when `vertex`, reached from `previous`, samples toward it.
bool PdfArea(const PathVertex& vertex, const PathVertex* previous, const PathVertex& next, real* pdfArea);
bool ValidConnection(const PathVertex* lightEnd, const PathVertex& cameraEnd, size_t s, size_t t);

}

// Counts include the endpoint. -1 reports invalid shading normals in the scene data; discard
// partial output. BSDF pointers remain valid until the caller resets arena, after all path
// connections. Both subpaths of one render must use the same normal policy.
int GenerateCameraPath(const Scene& scene, const Camera& camera, StateSequence& rand, MemoryPool& arena,
	const Ray& cameraRay, int maxVertices, SurfaceNormalMode normalMode, std::vector<PathVertex>& cameraPath);
int GenerateLightPath(const Scene& scene, StateSequence& rand, MemoryPool& arena,
	int maxVertices, SurfaceNormalMode normalMode, std::vector<PathVertex>& lightPath);

GYT_NAMESPACE_END
