#pragma once
#include "PathVertex.h"
#include "visual/Scene.h"

GYT_NAMESPACE_BEGIN

// Environment mDirection points toward the map, opposite an emitted ray.
// Finite destinations use geometric-normal area density; infinity retains solid angle.
bool ConvertDensity(real pdf, const PathVertex& vertex, const PathVertex& next, real* converted);

// Internal connection/probability ingredients shared by bidirectional estimators.
namespace Bidirectional {

bool ValidateScene(const Scene& scene);
bool Direction(const PathVertex& from, const PathVertex& to, Vec3* dir);
bool Connectible(const PathVertex& vertex);
bool LightOriginPdf(const Scene& scene, const PathVertex& light, const PathVertex& next, real* pdf);
bool PdfToward(const PathVertex& vertex, const PathVertex* previous, const PathVertex& next, real* pdf);
bool ValidConnection(const PathVertex* lightEnd, const PathVertex& cameraEnd, size_t s, size_t t);

}

// Counts include the endpoint. -1 reports an unsupported/invalid path; discard partial output.
// BSDF pointers remain valid until the caller resets arena, after all path connections.
int GenerateCameraPath(const Scene& scene, const Camera& camera, StateSequence& rand, MemoryPool& arena,
	const Ray& cameraRay, int maxVertices, std::vector<PathVertex>& cameraPath);
int GenerateLightPath(const Scene& scene, StateSequence& rand, MemoryPool& arena,
	int maxVertices, std::vector<PathVertex>& lightPath);

GYT_NAMESPACE_END
