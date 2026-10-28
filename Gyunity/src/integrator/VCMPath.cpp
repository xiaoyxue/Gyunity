#include "VCMPath.h"
#include <cmath>
#include <limits>

GYT_NAMESPACE_BEGIN

namespace Vcm {
namespace {

constexpr double Infeasible = -std::numeric_limits<double>::infinity();

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

int VertexCount(const PathContext& path) {
	return path.selected.s + path.selected.t - (path.selected.technique == Technique::Merging ? 1 : 0);
}

// At a separated merge there is no single vertex at index s-1.
// Light-side walks keep y = light[s-1]; camera-side walks keep x = camera[t-1].
const PathVertex& LightVertex(const PathContext& path, int i) {
	return i < path.selected.s ? path.light[i] : path.camera[VertexCount(path) - 1 - i];
}

const PathVertex& CameraVertex(const PathContext& path, int i) {
	const int split = path.selected.s - (path.selected.technique == Technique::Merging ? 1 : 0);
	return i < split ? path.light[i] : path.camera[VertexCount(path) - 1 - i];
}

bool Delta(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf && vertex.mIsect.mBsdf->IsDelta();
}

bool Continuous(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf && !Delta(vertex);
}

bool Feasible(const PathContext& path, const Strategy& strategy) {
	const int n = VertexCount(path), s = strategy.s;
	if (strategy.technique == Technique::Merging)
		return s >= 2 && s < n && Continuous(LightVertex(path, s - 1)) && Continuous(CameraVertex(path, s - 1));
	if (strategy.t == 0 || (s == 1 && strategy.t == 1)) return false;
	if (s == 0) return !LightVertex(path, 0).mLight->IsDeltaLight();
	return (s == 1 || Continuous(LightVertex(path, s - 1))) &&
		(s == n - 1 || Continuous(CameraVertex(path, s)));
}

// Eq. (6): pdfFwd[i], the density of vertex i sampled from vertex i-1 (light-to-camera direction).
bool ForwardDensity(const Scene& scene, const PathContext& path, int i, real* pdf) {
	const int n = VertexCount(path), s = path.selected.s;
	const int source = i - 1;
	if (source > 0 && source < n - 1 && Delta(LightVertex(path, source))) {
		// Eq. (11): keep the sampled delta mass and geometry; only the singular distribution is removed.
		if (source < s) return ConvertDensity(path.light[source].mDeltaPdf, path.light[source], LightVertex(path, i), pdf);
		const auto& vertex = path.camera[n - 1 - source];
		return ConvertDensity(vertex.mDeltaReversePdf, vertex, CameraVertex(path, i), pdf);
	}
	// A recorded reverse density is reused only while its source and sampled successor are selected.
	const int j = n - 1 - i;
	if (j < path.selected.t - 2) {
		*pdf = path.camera[j].mPdfPrev;
		return true;
	}
	if (i < s) {
		*pdf = path.light[i].mPdfFwd;
		return true;
	}
	if (path.selected.technique == Technique::Merging && i == s) {
		// Eq. (10): scatter at x with the original directions, convert along x's sampled edge.
		const auto& y = path.light[s - 1];
		const auto& x = path.camera[path.selected.t - 1];
		return ConvertDensity(x.mIsect.mBsdf->Pdf(y.mIsect.mOutDir, x.mIsect.mOutDir), x, CameraVertex(path, i), pdf);
	}
	if (i == 0) return Bidirectional::LightOriginPdf(scene, CameraVertex(path, 0), CameraVertex(path, 1), pdf);
	return Bidirectional::PdfToward(CameraVertex(path, source),
		i >= 2 ? &CameraVertex(path, i - 2) : nullptr, CameraVertex(path, i), pdf);
}

// Eq. (6): pdfPrev[i], the density of vertex i sampled from vertex i+1 (camera-to-light direction).
bool ReverseDensity(const PathContext& path, int i, real* pdf) {
	const int n = VertexCount(path), s = path.selected.s;
	const bool merging = path.selected.technique == Technique::Merging;
	const int source = i + 1;
	if (source > 0 && source < n - 1 && Delta(LightVertex(path, source))) {
		if (source < s) return ConvertDensity(path.light[source].mDeltaReversePdf, path.light[source], LightVertex(path, i), pdf);
		const auto& vertex = path.camera[n - 1 - source];
		return ConvertDensity(vertex.mDeltaPdf, vertex, CameraVertex(path, i), pdf);
	}
	if (i < s - 2) {
		*pdf = path.light[i].mPdfPrev;
		return true;
	}
	if (i >= s - (merging ? 1 : 0)) {
		*pdf = path.camera[n - 1 - i].mPdfFwd;
		return true;
	}
	if (merging && i == s - 2) {
		// Eq. (10): the reverse merge scatters at x but converts along y's original light edge.
		const auto& y = path.light[s - 1];
		const auto& x = path.camera[path.selected.t - 1];
		return ConvertDensity(x.mIsect.mBsdf->Pdf(x.mIsect.mOutDir, y.mIsect.mOutDir), y, LightVertex(path, i), pdf);
	}
	return Bidirectional::PdfToward(LightVertex(path, source),
		i + 2 < n ? &LightVertex(path, i + 2) : nullptr, LightVertex(path, i), pdf);
}

// Recorded edge densities of the selected path, with the connection or merge boundary recomputed.
bool PathDensities(const Scene& scene, const PathContext& path, std::vector<real>* pdfFwd, std::vector<real>* pdfPrev) {
	const int n = VertexCount(path);
	pdfFwd->assign(n, 0);
	pdfPrev->assign(n, 0);
	for (int i = 0; i + 1 < n; ++i)
		if (!ForwardDensity(scene, path, i, &(*pdfFwd)[i])) return false;
	for (int i = 0; i < n; ++i)
		if (!ReverseDensity(path, i, &(*pdfPrev)[i])) return false;
	return true;
}

// Eqs. (7) and (8): light prefix pdfFwd[0, s), then camera prefix pdfPrev[s, n). A merge samples
// its vertex from both sides, so the camera prefix starts at s-1, and adds pi*radius^2.
double LogPdf(const PathContext& path, const std::vector<real>& pdfFwd, const std::vector<real>& pdfPrev,
	const Strategy& strategy) {
	if (!IsStrategyEnabled(path, strategy) || !Feasible(path, strategy)) return Infeasible;
	const bool merging = strategy.technique == Technique::Merging;
	double logPdf = 0;
	for (int i = 0; i < strategy.s; ++i) {
		if (pdfFwd[i] == 0) return Infeasible;
		logPdf += std::log(double(pdfFwd[i]));
	}
	for (int i = strategy.s - (merging ? 1 : 0); i < VertexCount(path); ++i) {
		if (pdfPrev[i] == 0) return Infeasible;
		logPdf += std::log(double(pdfPrev[i]));
	}
	if (merging) logPdf += std::log(std::acos(-1.0) * path.radius * path.radius);
	return logPdf;
}

}

bool UseVc(Mode mode) { return mode == Mode::Combined || mode == Mode::ConnectionOnly; }
bool UseVm(Mode mode) { return mode == Mode::Combined || mode == Mode::MergingOnly; }

bool IsStrategyEnabled(const PathContext& path, const Strategy& strategy) {
	if (strategy.technique == Technique::Merging) return UseVm(path.mode);
	if (UseVc(path.mode)) return true;
	if (strategy.s != 0) return false;
	for (int i = 1; i + 1 < VertexCount(path); ++i)
		if (!Delta(LightVertex(path, i))) return false;
	return true;
}

bool ComputePathPdf(const Scene& scene, const PathContext& path, const Strategy& strategy, double* logPdf) {
	std::vector<real> pdfFwd, pdfPrev;
	if (!PathDensities(scene, path, &pdfFwd, &pdfPrev)) return false;
	*logPdf = LogPdf(path, pdfFwd, pdfPrev, strategy);
	return true;
}

bool ComputeMisWeight(const Scene& scene, const PathContext& path, const Strategy& current, real* weight) {
	*weight = 0;
	std::vector<real> pdfFwd, pdfPrev;
	if (!PathDensities(scene, path, &pdfFwd, &pdfPrev)) return false;
	const int64 nVc = 1, nVm = path.lightPathCount;
	const double logNVc = std::log(double(nVc)), logNVm = std::log(double(nVm));
	const double currentLogPdf = LogPdf(path, pdfFwd, pdfPrev, current);
	if (currentLogPdf == Infeasible) return Fail("VCM current strategy has zero support or is disabled.");
	const double currentScore = currentLogPdf + (current.technique == Technique::Merging ? logNVm : logNVc);

	// Eq. (9): scores are nStrategy * pdfPath in the log domain; normalize against their maximum.
	std::vector<double> scores;
	double maximum = Infeasible;
	const auto addScore = [&](const Strategy& strategy, double logCount) {
		const double logPdf = LogPdf(path, pdfFwd, pdfPrev, strategy);
		if (logPdf == Infeasible) return;
		scores.push_back(logPdf + logCount);
		maximum = std::max(maximum, scores.back());
	};
	const int n = VertexCount(path);
	if (UseVc(path.mode))
		for (int s = 0; s < n; ++s) addScore({ Technique::Connection, s, n - s }, logNVc);
	else
		addScore({ Technique::Connection, 0, n }, logNVc);
	if (UseVm(path.mode))
		for (int s = 2; s < n; ++s) addScore({ Technique::Merging, s, n - s + 1 }, logNVm);

	double sum = 0;
	for (double score : scores) sum += std::exp(score - maximum);
	*weight = static_cast<real>(std::exp(currentScore - maximum) / sum);
	return true;
}

Vector3d MergeEstimate(const PathContext& path) {
	const auto& y = path.light[path.selected.s - 1];
	const auto& x = path.camera[path.selected.t - 1];
	// Eq. (3). Throughputs already include sampling compensation. There is no x-y edge, photon BSDF,
	// extra cosine, or division by a full path pdf.
	const Vec3 scattering = x.mIsect.mBsdf->Evaluate(x.mIsect.mOutDir, y.mIsect.mOutDir);
	const double area = std::acos(-1.0) * path.radius * path.radius;
	return ToVector3d(x.mThroughput) * ToVector3d(scattering) * ToVector3d(y.mThroughput) / Vector3d(area);
}

}

GYT_NAMESPACE_END
