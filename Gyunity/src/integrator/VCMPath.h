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

// Surface area/point/one zero-rotation environment, pinhole and pure mirror/glass
// chains. NOT a VCM renderer. Singular factors follow report (51)-(55), not the
// legacy BDPT zero-marker convention; sampled masses and geometry are retained.
// false means invalid/unsupported input; true + !feasible means ordinary zero
// support or a disabled strategy. Only selected prefixes are inspected.
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

// Checked LOCAL estimator, not a render loop. The selected technique must be
// Merging. Active calls require two continuous surface endpoints, in-radius and
// normal-compatible as in PhotonMap::Query, and valid selected prefixes. The
// caller supplies the true attempted light-path count and enforces its depth
// budget. The camera BSDF must be the path builder's radiance-mode instance.
//
// Returns sum-ready wVM * betaE * f_camera(eyeOut, ORIGINAL photonOut) * betaL /
// (lightPathCount * pi * radius^2). MIS and density normalization are applied
// HERE, once per pair. Do not normalize the result or a sum of results again.
bool EvaluateMerging(const Scene& scene, const PathContext& path, Vec3* contribution);

// Gathers at camera[cameraPrefixCount-1], not just the first visible hit. Active
// calls require a built map borrowing THIS complete per-attempt batch (including
// empty/missed paths). N_L is exactly lightPaths.size(), never RecordCount/hits.
// The batch must contain a positive, int64-representable number of attempts.
// Filters s=record.vertex+1, t=cameraPrefixCount by physical depth s+t-3.
// A valid empty map, camera origin/escape or pure-delta query returns zero.
// Camera prefixes are validated even with no hits; unselected suffixes are not
// consumed. Build validates the batch's surface records; candidate prefixes are
// validated by MIS. No visibility test is made across the finite-radius disk.
bool GatherMerging(const Scene& scene, const PhotonMap& map,
	const std::vector<std::vector<PathVertex>>& lightPaths,
	const std::vector<PathVertex>& cameraPath, int cameraPrefixCount,
	int maxDepth, Mode mode, Vec3* contribution);

// Both estimator calls clear output on entry/failure; false logs Error(...,false)
// for malformed/unsupported active input or nonfinite/out-of-real-range results.
// Products, normalization and gather sums use double before conversion to real.
// Outputs must not alias paths. Scene, immutable paths and BSDF arenas must
// outlive all calls. ConnectionOnly returns zero without inspecting paths/maps;
// mode/output (and selected technique or gather maxDepth) still must be valid.

}

GYT_NAMESPACE_END
