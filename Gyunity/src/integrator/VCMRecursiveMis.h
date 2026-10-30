#pragma once

#include "VCMPath.h"

GYT_NAMESPACE_BEGIN

// Recursive VCM MIS weights (Docs/VCM.md, "Recursive MIS"): the report's dVCM/dVC/dVM
// recurrence (Section 5, as in SmallVCM's vertexcm.hxx), in area measure. Each subpath is
// summarized once in O(n); a connection or merge then costs O(1) and returns the same
// balance-heuristic weight as Vcm::ComputeMisWeight.
namespace Vcm {

// The report's quantities of one subpath vertex, in that subpath's own sampling order, relative
// to the density `along` of the vertex from its subpath predecessor (Eq. 14):
//   dVCM: the connection that joins this vertex to its predecessor;
//   dVC:  every strategy further back, scored against a connection at this vertex;
//   dVM:  the same strategies scored against a merge at this vertex (dVC / eta).
struct MisVertex {
	double dVCM = 0, dVC = 0, dVM = 0;
	// Every forward density of the prefix up to this vertex is positive. A current strategy
	// never ends at an unsupported vertex.
	bool supported = false;
};

// Eq. (14) over a whole subpath. eta = nVm * pi * radius^2, or 0 without merging; mode gates VC.
void PrepareLightMis(const std::vector<PathVertex>& light, double eta, Mode mode, std::vector<MisVertex>& summary);
void PrepareCameraMis(const std::vector<PathVertex>& camera, double eta, Mode mode, std::vector<MisVertex>& summary);

// Eq. (15): weight of connection (s, t). The caller has checked the strategy is enabled and its
// contribution positive; zero support of the current strategy fails as in ComputeMisWeight.
bool RecursiveConnectionWeight(const Scene& scene, const std::vector<PathVertex>& light,
	const std::vector<MisVertex>& lightMis, const std::vector<PathVertex>& camera,
	const std::vector<MisVertex>& cameraMis, int s, int t, double eta, Mode mode, real* weight);

// Eq. (16): weight of merging photon light[s-1] at camera[t-1]; requires eta > 0.
bool RecursiveMergeWeight(const std::vector<PathVertex>& light, const std::vector<MisVertex>& lightMis,
	const std::vector<PathVertex>& camera, const std::vector<MisVertex>& cameraMis, int s, int t,
	double eta, real* weight);

}

GYT_NAMESPACE_END
