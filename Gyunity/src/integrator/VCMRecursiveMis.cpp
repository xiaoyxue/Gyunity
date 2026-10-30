#include "VCMRecursiveMis.h"
#include <cmath>

GYT_NAMESPACE_BEGIN

namespace Vcm {
namespace {

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool Delta(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf && vertex.mIsect.mBsdf->IsDelta();
}

bool Continuous(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf && !Delta(vertex);
}

// Eq. (10) BSDF density at the merge vertex x; shading normals mask its support as in PdfPos.
real MergePdf(const PathVertex& x, const Vec3& wo, const Vec3& wi) {
	const bool supported = x.mNormalMode == SurfaceNormalMode::Geometric ||
		(Bidirectional::ShadingSupport(x.mIsect, wo) && Bidirectional::ShadingSupport(x.mIsect, wi));
	return supported ? x.mIsect.mBsdf->Pdf(wo, wi) : 0;
}

// Eq. (11): a delta vertex keeps its sampled component mass and the edge geometry.
double DeltaDensity(real mass, const PathVertex& from, const PathVertex& to) {
	return mass > 0 ? Bidirectional::DirectionToArea(mass, from, to) : 0;
}

// Eq. (6) along a subpath: vertex i sampled from its predecessor (pdfFwd of a light subpath,
// pdfPrev of a camera subpath).
double Along(const std::vector<PathVertex>& path, int i) {
	return i > 0 && Delta(path[i - 1]) ?
		DeltaDensity(path[i - 1].mDeltaPdfFwd, path[i - 1], path[i]) : double(path[i].mPdfFwd);
}

// Eq. (6) against a subpath: vertex i sampled from its successor, recorded when vertex i+1
// scattered toward i+2.
double Against(const std::vector<PathVertex>& path, int i) {
	return Delta(path[i + 1]) ?
		DeltaDensity(path[i + 1].mDeltaPdfPrev, path[i + 1], path[i]) : double(path[i].mPdfPrev);
}

// Whether the connection joining subpath vertices i-1 and i is feasible. The endpoint at index 0
// is always connectible, but a camera subpath has no t = 0 strategy and a light subpath's i = 0
// is the camera hitting the emitter.
bool Connectible(const std::vector<PathVertex>& path, int i, bool lightSide) {
	if (i == 0) return lightSide && !path[0].mLight->IsDeltaLight();
	return (i == 1 || Continuous(path[i - 1])) && Continuous(path[i]);
}

// Whether a merge at subpath vertex i is feasible: not at an endpoint, and not at a delta vertex.
bool Mergeable(const std::vector<PathVertex>& path, int i) {
	return i >= 1 && Continuous(path[i]);
}

// Eq. (14): the report's recurrence, one vertex at a time in the subpath's sampling order.
void PrepareMis(const std::vector<PathVertex>& path, double eta, Mode mode, bool lightSide,
	std::vector<MisVertex>& summary) {
	const int n = int(path.size());
	const bool vc = UseVc(mode);
	const double invEta = eta > 0 ? 1 / eta : 0;
	summary.assign(path.size(), MisVertex());
	for (int i = 0; i < n; ++i) {
		const double along = Along(path, i);
		// Once a forward density is zero, no strategy ending at or beyond this vertex has support.
		if (!(along > 0) || (i > 0 && !summary[i - 1].supported)) break;
		MisVertex& vertex = summary[i];
		vertex.supported = true;
		vertex.dVCM = (vc && Connectible(path, i, lightSide) ? 1 : 0) / along;
		if (i == 0) continue;
		const MisVertex& previous = summary[i - 1];
		const double merge = Mergeable(path, i - 1) ? 1 : 0;
		const double against = i >= 2 ? Against(path, i - 2) : 0;
		vertex.dVC = (eta * merge + previous.dVCM + against * previous.dVC) / along;
		vertex.dVM = (merge + previous.dVCM * invEta + against * previous.dVM) / along;
	}
}

}

void PrepareLightMis(const std::vector<PathVertex>& light, double eta, Mode mode, std::vector<MisVertex>& summary) {
	PrepareMis(light, eta, mode, true, summary);
}

void PrepareCameraMis(const std::vector<PathVertex>& camera, double eta, Mode mode, std::vector<MisVertex>& summary) {
	PrepareMis(camera, eta, mode, false, summary);
}

bool RecursiveConnectionWeight(const Scene& scene, const std::vector<PathVertex>& light,
	const std::vector<MisVertex>& lightMis, const std::vector<PathVertex>& camera,
	const std::vector<MisVertex>& cameraMis, int s, int t, double eta, Mode mode, real* weight) {
	*weight = 0;
	if ((s > 0 && !lightMis[s - 1].supported) || !cameraMis[t - 1].supported)
		return Fail("VCM current strategy has zero support or is disabled.");
	const PathVertex& z = camera[t - 1];
	const PathVertex* b = t >= 2 ? &camera[t - 2] : nullptr;
	// Light side, report Eq. (15): R_{s-1} * (eta + dVCM + R_{s-2} * dVC) at y.
	double wLight = 0;
	if (s > 0) {
		const PathVertex& y = light[s - 1];
		const MisVertex& my = lightMis[s - 1];
		real reverse = 0, reverseBefore = 0;
		if (!Bidirectional::PdfPos(z, b, y, &reverse)) return false;
		if (s >= 2 && !Bidirectional::PdfPos(y, &z, light[s - 2], &reverseBefore)) return false;
		const double merge = Mergeable(light, s - 1) ? 1 : 0;
		wLight = reverse * (eta * merge + my.dVCM + reverseBefore * my.dVC);
	}
	// Camera side: F_s * (eta + dVCM + F_{s+1} * dVC) at z. Light tracing has none.
	double wCamera = 0;
	if (b) {
		const MisVertex& mz = cameraMis[t - 1];
		real forward = 0, forwardAfter = 0;
		if (s == 0) {
			if (!Bidirectional::LightOriginPdf(scene, z, *b, &forward) ||
				!Bidirectional::PdfPos(z, nullptr, *b, &forwardAfter)) return false;
			// Here z is the light origin: connection (1, t-1) needs no lobe at z, and z cannot merge.
			const double connect = UseVc(mode) && t >= 3 && Continuous(*b) ? 1 : 0;
			wCamera = forward * (connect / Along(camera, t - 1) + forwardAfter * mz.dVC);
		} else {
			const PathVertex& y = light[s - 1];
			if (!Bidirectional::PdfPos(y, s >= 2 ? &light[s - 2] : nullptr, z, &forward) ||
				!Bidirectional::PdfPos(z, &y, *b, &forwardAfter)) return false;
			const double merge = Continuous(z) ? 1 : 0;
			wCamera = forward * (eta * merge + mz.dVCM + forwardAfter * mz.dVC);
		}
	}
	*weight = static_cast<real>(1 / (1 + wLight + wCamera));
	return true;
}

bool RecursiveMergeWeight(const std::vector<PathVertex>& light, const std::vector<MisVertex>& lightMis,
	const std::vector<PathVertex>& camera, const std::vector<MisVertex>& cameraMis, int s, int t,
	double eta, real* weight) {
	*weight = 0;
	if (!lightMis[s - 1].supported || !cameraMis[t - 1].supported || !(eta > 0))
		return Fail("VCM current strategy has zero support or is disabled.");
	const PathVertex& y = light[s - 1];
	const PathVertex& x = camera[t - 1];
	// Eq. (10): both boundary densities scatter at x with the original directions.
	const double reverseBefore = Bidirectional::DirectionToArea(
		MergePdf(x, x.mIsect.mOutDir, y.mIsect.mOutDir), y, light[s - 2]);
	const double forward = Bidirectional::DirectionToArea(
		MergePdf(x, y.mIsect.mOutDir, x.mIsect.mOutDir), x, camera[t - 2]);
	// Report Eq. (16), as SmallVCM's merge: dVCM / eta + (Eq. 10 density) * dVM on each side.
	const MisVertex& my = lightMis[s - 1];
	const MisVertex& mx = cameraMis[t - 1];
	const double wLight = my.dVCM / eta + reverseBefore * my.dVM;
	const double wCamera = mx.dVCM / eta + forward * mx.dVM;
	*weight = static_cast<real>(1 / (1 + wLight + wCamera));
	return true;
}

}

GYT_NAMESPACE_END
