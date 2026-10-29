#include "BDPT.h"
#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <vector>

GYT_NAMESPACE_BEGIN

namespace Bidirectional {

Vec3 DirectionTo(const PathVertex& from, const PathVertex& to) {
	if (from.mType == PathVertexType::Environment) return -from.mDirection;
	if (to.mType == PathVertexType::Environment) return to.mDirection;
	const Vec3 offset = to.mIsect.mPos - from.mIsect.mPos;
	return offset / std::sqrt(offset.Length2());
}

bool Connectible(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Camera ||
		(vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf &&
			vertex.mIsect.mBsdf->HasNonDeltaLobes());
}

bool LightOriginPdf(const Scene& scene, const PathVertex& light, const PathVertex& next, real* pdf) {
	*pdf = 0;
	real pdfPos, pdfDir;
	if (!light.mLight->PdfLight(light.mIsect, DirectionTo(light, next), &pdfPos, &pdfDir)) return false;
	// ValidateScene registers each light once.
	real selectionPdf = 0;
	const auto& lights = scene.GetLights();
	for (size_t i = 0; i < lights.size(); ++i) {
		if (lights[i].get() != light.mLight) continue;
		if (!scene.LightSelectionPdf(i, &selectionPdf)) return false;
		break;
	}
	*pdf = selectionPdf * (light.mType == PathVertexType::Environment ? pdfDir : pdfPos);
	return true;
}

bool PdfPos(const PathVertex& vertex, const PathVertex* previous, const PathVertex& next, real* pdfPos) {
	const Vec3 wi = DirectionTo(vertex, next);
	real emitterPdfPos, pdfDir = 0;
	if (vertex.mType == PathVertexType::Environment) {
		// Parallel rays from the launch disk: the disk's pdfPos, not pdfDir, carries over to `next`.
		if (!vertex.mLight->PdfLight(vertex.mIsect, wi, &emitterPdfPos, &pdfDir)) return false;
		*pdfPos = DirectionToArea(emitterPdfPos, vertex, next);
		return true;
	}
	if (vertex.mType == PathVertexType::Light || (!previous && vertex.mLight)) {
		if (!vertex.mLight->PdfLight(vertex.mIsect, wi, &emitterPdfPos, &pdfDir)) return false;
	}
	else if (vertex.mType == PathVertexType::Camera) {
		pdfDir = vertex.mCamera->PdfDir(Ray(vertex.mIsect.mPos, wi));
	}
	else {
		const Vec3 wo = DirectionTo(vertex, *previous);
		// Shading normals only mask the support; their cosine never enters a density.
		const bool supported = vertex.mNormalMode == SurfaceNormalMode::Geometric ||
			(ShadingSupport(vertex.mIsect, wo) && ShadingSupport(vertex.mIsect, wi));
		pdfDir = supported && !vertex.mIsect.mBsdf->IsDelta() ? vertex.mIsect.mBsdf->Pdf(wo, wi) : 0;
	}
	*pdfPos = DirectionToArea(pdfDir, vertex, next);
	return true;
}

bool ValidConnection(const PathVertex* lightEnd, const PathVertex& cameraEnd, size_t s, size_t t) {
	if (t == 0 || s + t < 2) return false;
	if (s == 0) return cameraEnd.mLight &&
		(cameraEnd.mLight->IsAreaLight() || cameraEnd.mType == PathVertexType::Environment);
	if (cameraEnd.mType == PathVertexType::Environment) return false;
	if (s == 1 && t == 1) return false;
	if (s == 1) return lightEnd && lightEnd->mLight && Connectible(cameraEnd);
	return lightEnd && Connectible(*lightEnd) && Connectible(cameraEnd);
}

}

namespace {

using Bidirectional::DirectionTo;
using Bidirectional::DirectionToArea;
using Bidirectional::Connectible;
using Bidirectional::LightOriginPdf;
using Bidirectional::PdfPos;
using Bidirectional::ScatteringCosine;
using Bidirectional::ValidConnection;

bool Surface(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface ||
		(vertex.mLight && vertex.mLight->IsAreaLight());
}

struct MisVertex {
	real pdfFwd, pdfPrev;
	bool delta;
};

std::vector<MisVertex> CopyMisState(const std::vector<PathVertex>& path, int count) {
	std::vector<MisVertex> state;
	state.reserve(count);
	for (int i = 0; i < count; ++i) {
		state.push_back({ path[i].mPdfFwd, path[i].mPdfPrev, path[i].mIsDelta });
	}
	if (!state.empty()) {
		state.front().delta = false;
		state.back().delta = false;
	}
	return state;
}

// pdfPrev / pdfFwd at vertex i. Only delta-generated edge markers are remapped to 1;
// ordinary zero support stays zero.
double MisRatio(const std::vector<MisVertex>& path, size_t i) {
	const MisVertex& vertex = path[i];
	const double pdfFwd = vertex.pdfFwd == 0 && i > 0 && path[i - 1].delta ? 1 : vertex.pdfFwd;
	const double pdfPrev = vertex.pdfPrev == 0 && i + 1 < path.size() && path[i + 1].delta ? 1 : vertex.pdfPrev;
	return pdfPrev / pdfFwd;
}

bool WorldToScreen(const Camera& camera, const Vec3& point, Vec2* raster) {
	if (Dot(camera.mCz, point - camera.mPos) <= 0) return false;
	const Vec3 projected = (camera.mCameraToRaster * camera.mWorldToCamera)(point);
	if (projected.x < 0 || projected.y < 0 ||
		projected.x >= camera.GetFilm()->mResX || projected.y >= camera.GetFilm()->mResY) return false;
	*raster = Vec2(projected.x, projected.y);
	return true;
}

// Channelwise product of the factors times scale, kept in double until the real store.
Vec3 ScaledProduct(std::initializer_list<Vec3> factors, double scale) {
	Vec3 result;
	for (int channel = 0; channel < 3; ++channel) {
		double value = scale;
		for (const Vec3& factor : factors) value *= double(factor[channel]);
		result[channel] = static_cast<real>(value);
	}
	return result;
}

// Emitter-side connection factor for shading-normal mode: distance falloff and the area
// emitter's geometric cosine. The receiver's scattering cosine is carried by its K instead.
double EmitterConnectionFactor(const PathVertex& emitter, const PathVertex& receiver) {
	if (emitter.mType == PathVertexType::Environment) return 1;
	const double dx = double(receiver.mIsect.mPos.x) - double(emitter.mIsect.mPos.x);
	const double dy = double(receiver.mIsect.mPos.y) - double(emitter.mIsect.mPos.y);
	const double dz = double(receiver.mIsect.mPos.z) - double(emitter.mIsect.mPos.z);
	double value = 1 / (dx * dx + dy * dy + dz * dz);
	// Only an emission endpoint contributes its geometric cosine; a scattering vertex that lies
	// on an emissive surface carries its cosine in K.
	if (emitter.mType == PathVertexType::Light && emitter.mLight && emitter.mLight->IsAreaLight()) {
		value *= std::abs(double(Dot(emitter.mIsect.mGeometryNormal, DirectionTo(emitter, receiver))));
	}
	return value;
}

Vec3 EvaluateEmission(const std::vector<PathVertex>& cameraPath, int t) {
	const PathVertex& pt = cameraPath[t - 1];
	if (!pt.mLight) return Vec3(0);
	if (pt.mType == PathVertexType::Environment) return pt.mThroughput * pt.mLight->Emission(pt.mDirection);
	if (!pt.mLight->IsAreaLight()) return Vec3(0);
	return pt.mThroughput * pt.mLight->Emission(pt.mIsect, DirectionTo(pt, cameraPath[t - 2]));
}

Vec3 ConnectCamera(const Camera& camera, const PathVertex& qs, Vec2* raster) {
	if (!Connectible(qs) || !WorldToScreen(camera, qs.mIsect.mPos, raster)) return Vec3(0);
	Vec3 wi;
	real pdf;
	const Vec3 importance = camera.SampleWi(qs.mIsect, &pdf, &wi, Vec3(0.5f));
	if (pdf == 0) return Vec3(0);
	// SampleWi supplies the camera-end geometry through importance / pdf.
	const Vec3 f = qs.mIsect.mBsdf->Evaluate(qs.mIsect.mOutDir, wi);
	if (qs.mNormalMode == SurfaceNormalMode::Shading) {
		const double lightCosine = ScatteringCosine(qs.mIsect, qs.mIsect.mOutDir, wi, TransportMode::Importance);
		return lightCosine == 0 ? Vec3(0) : ScaledProduct({ qs.mThroughput, f, importance }, lightCosine / double(pdf));
	}
	return qs.mThroughput * f * importance * std::abs(Dot(qs.mIsect.mNormal, wi)) / pdf;
}

bool ConnectEmitter(const Scene& scene, const PathVertex& qs, const PathVertex& pt, Vec3* contribution) {
	if (!Connectible(pt) || !qs.mLight) return true;
	real originPdf;
	if (!LightOriginPdf(scene, qs, pt, &originPdf)) return false;
	if (originPdf <= 0) return true;
	const Vec3 toCamera = DirectionTo(qs, pt);
	const Vec3 emission = qs.mType == PathVertexType::Environment ?
		qs.mLight->Emission(qs.mDirection) : qs.mLight->IsAreaLight() ?
		qs.mLight->Emission(qs.mIsect, toCamera) : qs.mLight->Emission();
	const Vec3 f = pt.mIsect.mBsdf->Evaluate(pt.mIsect.mOutDir, -toCamera);
	// Reuse the sampled light origin, compensating its selection and origin density once.
	if (pt.mNormalMode == SurfaceNormalMode::Shading) {
		const double cameraCosine = ScatteringCosine(pt.mIsect, pt.mIsect.mOutDir, -toCamera, TransportMode::Radiance);
		if (cameraCosine != 0) {
			*contribution = ScaledProduct({ emission, pt.mThroughput, f },
				cameraCosine * EmitterConnectionFactor(qs, pt) / double(originPdf));
		}
		return true;
	}
	*contribution = emission / originPdf * pt.mThroughput * f * GeometryTerm(qs, pt);
	return true;
}

Vec3 ConnectSurfaces(const PathVertex& qs, const PathVertex& pt) {
	if (!Connectible(qs) || !Connectible(pt)) return Vec3(0);
	const Vec3 wi = DirectionTo(qs, pt);
	const Vec3 fq = qs.mIsect.mBsdf->Evaluate(qs.mIsect.mOutDir, wi);
	const Vec3 fp = pt.mIsect.mBsdf->Evaluate(pt.mIsect.mOutDir, -wi);
	if (pt.mNormalMode == SurfaceNormalMode::Shading) {
		// Each K carries its scattering cosine; only distance falloff remains.
		const double lightCosine = ScatteringCosine(qs.mIsect, qs.mIsect.mOutDir, wi, TransportMode::Importance);
		const double cameraCosine = ScatteringCosine(pt.mIsect, pt.mIsect.mOutDir, -wi, TransportMode::Radiance);
		if (lightCosine == 0 || cameraCosine == 0) return Vec3(0);
		return ScaledProduct({ qs.mThroughput, pt.mThroughput, fq, fp },
			lightCosine * cameraCosine * EmitterConnectionFactor(qs, pt));
	}
	return qs.mThroughput * pt.mThroughput * GeometryTerm(qs, pt) * fq * fp;
}

}

real GeometryTerm(const PathVertex& first, const PathVertex& second) {
	const Vec3 dir = DirectionTo(first, second);
	real value = first.mType == PathVertexType::Environment || second.mType == PathVertexType::Environment
		? 1 : 1 / (second.mIsect.mPos - first.mIsect.mPos).Length2();
	if (Surface(first)) value *= std::abs(Dot(first.mIsect.mGeometryNormal, dir));
	if (Surface(second)) value *= std::abs(Dot(second.mIsect.mGeometryNormal, dir));
	return value;
}

bool MisWeight(const Scene& scene, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, real* weight) {
	*weight = 0;
	const size_t count = static_cast<size_t>(s) + t;
	const PathVertex* lightEnd = s > 0 ? &lightPath[s - 1] : nullptr;
	const PathVertex* lightPrevious = s > 1 ? &lightPath[s - 2] : nullptr;
	const PathVertex& cameraEnd = cameraPath[t - 1];
	const PathVertex* cameraPrevious = t > 1 ? &cameraPath[t - 2] : nullptr;
	if (!ValidConnection(lightEnd, cameraEnd, s, t)) return true;
	if (count == 2) { *weight = 1; return true; }

	// Local snapshots replace PBRT's ScopedAssignment: generated paths never need restoration.
	auto lightMis = CopyMisState(lightPath, s);
	auto cameraMis = CopyMisState(cameraPath, t);

	// Recompute the camera-side connection densities.
	if (s == 0) {
		if (!LightOriginPdf(scene, cameraEnd, *cameraPrevious, &cameraMis[t - 1].pdfPrev) ||
			!PdfPos(cameraEnd, nullptr, *cameraPrevious, &cameraMis[t - 2].pdfPrev)) return false;
	}
	else {
		if (!PdfPos(*lightEnd, lightPrevious, cameraEnd, &cameraMis[t - 1].pdfPrev)) return false;
		if (cameraPrevious &&
			!PdfPos(cameraEnd, lightEnd, *cameraPrevious, &cameraMis[t - 2].pdfPrev)) return false;

		// Recompute the light-side connection densities.
		if (!PdfPos(cameraEnd, cameraPrevious, *lightEnd, &lightMis[s - 1].pdfPrev)) return false;
		if (lightPrevious &&
			!PdfPos(*lightEnd, &cameraEnd, *lightPrevious, &lightMis[s - 2].pdfPrev)) return false;
	}

	// PBRT-v3 balance heuristic: walk alternative splits in both directions.
	double sumRatios = 0, ratio = 1;
	for (int i = t - 1; i > 0; --i) {
		ratio *= MisRatio(cameraMis, i);
		if (!cameraMis[i].delta && !cameraMis[i - 1].delta &&
			ValidConnection(&cameraPath[i], cameraPath[i - 1], count - i, i)) sumRatios += ratio;
	}
	ratio = 1;
	for (int i = s - 1; i >= 0; --i) {
		ratio *= MisRatio(lightMis, i);
		if (!lightMis[i].delta && !(i > 0 && lightMis[i - 1].delta) &&
			ValidConnection(i > 0 ? &lightPath[i - 1] : nullptr, lightPath[i], i, count - i)) sumRatios += ratio;
	}
	*weight = static_cast<real>(1 / (1 + sumRatios));
	return true;
}

// Log-pdf balance heuristic; equation numbers refer to Docs/BDPT.md "MIS". The densities mirror
// Vcm::PathDensities (Docs/VCM.md Eqs. (6) and (11)) without merging, so these weights equal
// Vcm::ComputeMisWeight in ConnectionOnly mode on the same subpaths.
namespace {

constexpr double Infeasible = -std::numeric_limits<double>::infinity();

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

// The selected path x_0 .. x_{n-1} in light-to-camera order, n = s + t.
struct JoinedPath {
	const std::vector<PathVertex>& light;
	const std::vector<PathVertex>& camera;
	int s, t;

	int Count() const { return s + t; }
	const PathVertex& operator[](int i) const { return i < s ? light[i] : camera[s + t - 1 - i]; }
};

// A delta event at x_k that generated a neighbor of this path. The connection endpoints
// x_{s-1} and x_s scatter through the connection instead; a flag left there by their unused
// continuation does not count, as in CopyMisState.
bool SampledDelta(const JoinedPath& path, int k) {
	return k > 0 && k < path.Count() - 1 && k != path.s - 1 && k != path.s && path[k].mIsDelta;
}

// Eq. (4): the sampled component mass with the edge geometry G(from -> to); only the singular
// distribution is removed. A negative reverse mass marks an unsupported component.
real DeltaDensity(real mass, const PathVertex& from, const PathVertex& to) {
	return mass > 0 ? DirectionToArea(mass, from, to) : 0;
}

// Eq. (1): pdfFwd[i], x_i sampled from x_{i-1}. Recorded densities are reused while their source
// and its predecessor are selected; only the connection boundary is recomputed.
bool ForwardDensity(const Scene& scene, const JoinedPath& path, int i, real* pdf) {
	const int n = path.Count(), s = path.s, source = i - 1;
	if (SampledDelta(path, source)) {
		// Light vertices store masses in light-to-camera order, camera vertices in reverse.
		const PathVertex& vertex = path[source];
		*pdf = DeltaDensity(source < s ? vertex.mDeltaPdfFwd : vertex.mDeltaPdfPrev, vertex, path[i]);
		return true;
	}
	if (i < s) {
		*pdf = path.light[i].mPdfFwd;
		return true;
	}
	if (n - 1 - i < path.t - 2) {
		*pdf = path.camera[n - 1 - i].mPdfPrev;
		return true;
	}
	if (i == 0) return LightOriginPdf(scene, path[0], path[1], pdf);
	return PdfPos(path[source], i >= 2 ? &path[i - 2] : nullptr, path[i], pdf);
}

// Eq. (1): pdfPrev[i], x_i sampled from x_{i+1}.
bool ReverseDensity(const JoinedPath& path, int i, real* pdf) {
	const int n = path.Count(), s = path.s, source = i + 1;
	if (SampledDelta(path, source)) {
		const PathVertex& vertex = path[source];
		*pdf = DeltaDensity(source < s ? vertex.mDeltaPdfPrev : vertex.mDeltaPdfFwd, vertex, path[i]);
		return true;
	}
	if (i >= s) {
		*pdf = path.camera[n - 1 - i].mPdfFwd;
		return true;
	}
	if (i < s - 2) {
		*pdf = path.light[i].mPdfPrev;
		return true;
	}
	return PdfPos(path[source], i + 2 < n ? &path[i + 2] : nullptr, path[i], pdf);
}

// Strategy k joins x_{k-1} and x_k. It cannot reproduce the path when either endpoint
// scattered a delta event of the path, or when the connection itself is unsupported.
bool Feasible(const JoinedPath& path, int k) {
	return !SampledDelta(path, k - 1) && !SampledDelta(path, k) &&
		ValidConnection(k > 0 ? &path[k - 1] : nullptr, path[k], k, path.Count() - k);
}

}

bool MisWeightLogPdf(const Scene& scene, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, real* weight) {
	*weight = 0;
	if (!ValidConnection(s > 0 ? &lightPath[s - 1] : nullptr, cameraPath[t - 1], s, t)) return true;
	if (s + t == 2) { *weight = 1; return true; }
	const JoinedPath path{ lightPath, cameraPath, s, t };
	const int n = path.Count();
	std::vector<real> pdfFwd(n, 0), pdfPrev(n, 0);
	for (int i = 0; i + 1 < n; ++i)
		if (!ForwardDensity(scene, path, i, &pdfFwd[i])) return false;
	for (int i = 0; i < n; ++i)
		if (!ReverseDensity(path, i, &pdfPrev[i])) return false;

	// Eq. (2) for every strategy at once: light[k] = log prod_{i<k} pdfFwd[i] and
	// camera[k] = log prod_{i>=k} pdfPrev[i]. A zero factor makes the strategy infeasible.
	std::vector<double> light(n, 0), camera(n + 1, 0);
	for (int k = 1; k < n; ++k)
		light[k] = light[k - 1] == Infeasible || !(pdfFwd[k - 1] > 0) ? Infeasible : light[k - 1] + std::log(double(pdfFwd[k - 1]));
	for (int k = n - 1; k >= 0; --k)
		camera[k] = camera[k + 1] == Infeasible || !(pdfPrev[k] > 0) ? Infeasible : camera[k + 1] + std::log(double(pdfPrev[k]));
	std::vector<double> logPdf(n);
	double maximum = Infeasible;
	for (int k = 0; k < n; ++k) {
		const bool feasible = Feasible(path, k) && light[k] != Infeasible && camera[k] != Infeasible;
		logPdf[k] = feasible ? light[k] + camera[k] : Infeasible;
		maximum = std::max(maximum, logPdf[k]);
	}
	// The sampled strategy's factors are all recorded or delta masses, so they are positive.
	if (logPdf[s] == Infeasible) return Fail("BDPT current strategy has zero path density.");

	// Eq. (3): balance heuristic normalized against the largest log pdf, so long paths do not underflow.
	double sum = 0;
	for (double value : logPdf)
		if (value != Infeasible) sum += std::exp(value - maximum);
	*weight = static_cast<real>(std::exp(logPdf[s] - maximum) / sum);
	return true;
}

bool EvaluateConnection(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster) {
	*contribution = Vec3(0);
	*raster = Vec2(0, 0);
	if (s + t < 2 || (s == 1 && t == 1)) return true;
	if (s == 0) {
		*contribution = EvaluateEmission(cameraPath, t);
		return true;
	}
	const PathVertex& pt = cameraPath[t - 1];
	if (pt.mType == PathVertexType::Environment) return true;
	const PathVertex& qs = lightPath[s - 1];
	Vec3 unweightedContribution(0);
	if (t == 1) unweightedContribution = ConnectCamera(camera, qs, raster);
	else if (s == 1) {
		if (!ConnectEmitter(scene, qs, pt, &unweightedContribution)) return false;
	}
	else unweightedContribution = ConnectSurfaces(qs, pt);
	if (unweightedContribution == Vec3(0)) return true;
	const bool visible = qs.mType == PathVertexType::Environment ?
		!scene.Intersect(pt.mIsect.SpawnRay(qs.mDirection)) :
		VisibilityTester(qs.mIsect, pt.mIsect).Unoccluded(scene);
	if (visible) *contribution = unweightedContribution;
	return true;
}

bool ConnectPaths(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster, BdptMis mis) {
	*contribution = Vec3(0);
	Vec3 unweightedContribution(0);
	if (!EvaluateConnection(scene, camera, lightPath, cameraPath, s, t, &unweightedContribution, raster)) return false;
	if (unweightedContribution == Vec3(0)) return true;
	real weight;
	const bool weighted = mis == BdptMis::LogPdf ? MisWeightLogPdf(scene, lightPath, cameraPath, s, t, &weight) :
		MisWeight(scene, lightPath, cameraPath, s, t, &weight);
	if (!weighted) return false;
	*contribution = unweightedContribution * weight;
	return true;
}

GYT_NAMESPACE_END
