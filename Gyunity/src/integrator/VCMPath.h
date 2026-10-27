#pragma once

#include "BDPTPath.h"

GYT_NAMESPACE_BEGIN

namespace Vcm {

enum class Technique { Connection, Merging };
enum class Mode { Combined, ConnectionOnly, MergingOnly };

struct Strategy {
	Technique technique;
	int s, t;
};

// Non-owning, immutable selected-prefix view. Paths and their BSDF arenas must
// outlive evaluation. A merge retains BOTH light[s-1] and camera[t-1].
struct PathContext {
	const std::vector<PathVertex>& light;
	const std::vector<PathVertex>& camera;
	const Strategy selected;
	const double radius;
	const int64 lightPathCount;
	const Mode mode;

	PathContext(const std::vector<PathVertex>& light, const std::vector<PathVertex>& camera,
		Strategy selected, double radius, int64 lightPathCount, Mode mode = Mode::Combined)
		: light(light), camera(camera), selected(selected), radius(radius),
		lightPathCount(lightPathCount), mode(mode) {}
};

struct StrategyPdf {
	bool feasible = false;
	double logPdf = 0; // Common reduced-measure density; excludes the strategy sample count.
};

// Only selected prefixes are inspected. Invalid/unsupported input fails;
// ordinary zero support or a disabled strategy succeeds with !feasible.
// Singular factors retain sampled masses and geometry, per report (51)-(55).
bool ComputePathPdf(const Scene& scene, const PathContext& path,
	const Strategy& strategy, StrategyPdf* result);

// Enumerates the same physical depth (VC s+t-2, VM s+t-3) with balance scores
// pVC and lightPathCount*pVM. A disabled or zero-support current is an error.
bool ComputeMisWeight(const Scene& scene, const PathContext& path,
	const Strategy& current, real* weight);

// Validate a complete generated subpath even when no estimator consumes it.
// Empty light attempts are valid; sampled terminal delta events are checked too.
bool ValidateSubpath(const Scene& scene, const std::vector<PathVertex>& path, bool camera);

class PhotonMap;

// Sum-ready wVM * betaE * f_camera(eyeOut, ORIGINAL photonOut) * betaL /
// (lightPathCount * pi * radius^2). Selected technique must be Merging.
// Validates both prefixes, radius and normal compatibility. MIS/kernel normalization
// is applied once here; the caller enforces depth and must not renormalize.
bool EvaluateMerging(const Scene& scene, const PathContext& path, Vec3* contribution);

// Gather at camera[t-1], filtering s+t-3 <= maxDepth. The map must borrow THIS
// immutable batch; N_L is lightPaths.size(), including empty/missed attempts.
// Empty maps and nonmergeable queries return zero after validating the camera prefix.
bool GatherMerging(const Scene& scene, const PhotonMap& map,
	const std::vector<std::vector<PathVertex>>& lightPaths,
	const std::vector<PathVertex>& cameraPath, int t,
	int maxDepth, Mode mode, Vec3* contribution);

// Estimators clear output on failure and use double products/sums before a checked real store.
// Outputs must not alias paths. Keep scene, paths and BSDF arenas alive through all calls.
// ConnectionOnly returns zero without consuming paths/maps. Full contracts: Docs/VCM.md.

}

GYT_NAMESPACE_END
