#pragma once

#include "BDPTPath.h"

GYT_NAMESPACE_BEGIN

namespace Vcm {

enum class Technique { Connection, Merging };
enum class Mode { Combined, ConnectionOnly, MergingOnly };

bool UseVc(Mode mode);
bool UseVm(Mode mode);

struct Strategy {
	Technique technique;
	int s, t;
};

// Non-owning view of the selected light/camera prefixes. Paths and their BSDF arenas
// must outlive evaluation. A merge keeps BOTH physical endpoints light[s-1] and camera[t-1].
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

// Whether the mode samples a strategy. MergingOnly keeps camera-visible LS*E emission.
bool IsStrategyEnabled(const PathContext& path, const Strategy& strategy);

// Log of pdfPath for a strategy of the selected physical path; -infinity when the strategy
// is disabled or has zero support. A VM density includes pi*radius^2 but no sample count.
bool ComputePathPdf(const Scene& scene, const PathContext& path, const Strategy& strategy, double* logPdf);

// Balance heuristic over all VC (count 1) and VM (count lightPathCount) strategies of the
// same physical path. The current strategy must be enabled and have positive density.
bool ComputeMisWeight(const Scene& scene, const PathContext& path, const Strategy& current, real* weight);

// Primary VM estimate before MIS and 1/nVm: betaE * f_x(wo, original photon wi) * betaL / (pi*radius^2).
Vector3d MergeEstimate(const PathContext& path);

// Vector3d scalar operators take real; keep double factors double by passing them as Vector3d.
inline Vector3d ToVector3d(const Vec3& value) { return Vector3d(value.x, value.y, value.z); }
inline Vec3 ToVec3(const Vector3d& value) { return Vec3(real(value[0]), real(value[1]), real(value[2])); }

}

GYT_NAMESPACE_END
