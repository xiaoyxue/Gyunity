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

}

GYT_NAMESPACE_END
