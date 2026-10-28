#include "BDPT.h"
#include <cmath>
#include <initializer_list>

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

bool PdfArea(const PathVertex& vertex, const PathVertex* previous, const PathVertex& next, real* pdfArea) {
	const Vec3 wi = DirectionTo(vertex, next);
	real pdfPos, pdfDir = 0;
	if (vertex.mType == PathVertexType::Environment) {
		if (!vertex.mLight->PdfLight(vertex.mIsect, wi, &pdfPos, &pdfDir)) return false;
		*pdfArea = DirectionToArea(pdfPos, vertex, next);
		return true;
	}
	if (vertex.mType == PathVertexType::Light || (!previous && vertex.mLight)) {
		if (!vertex.mLight->PdfLight(vertex.mIsect, wi, &pdfPos, &pdfDir)) return false;
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
	*pdfArea = DirectionToArea(pdfDir, vertex, next);
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
using Bidirectional::Connectible;
using Bidirectional::LightOriginPdf;
using Bidirectional::PdfArea;
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
			!PdfArea(cameraEnd, nullptr, *cameraPrevious, &cameraMis[t - 2].pdfPrev)) return false;
	}
	else {
		if (!PdfArea(*lightEnd, lightPrevious, cameraEnd, &cameraMis[t - 1].pdfPrev)) return false;
		if (cameraPrevious &&
			!PdfArea(cameraEnd, lightEnd, *cameraPrevious, &cameraMis[t - 2].pdfPrev)) return false;

		// Recompute the light-side connection densities.
		if (!PdfArea(cameraEnd, cameraPrevious, *lightEnd, &lightMis[s - 1].pdfPrev)) return false;
		if (lightPrevious &&
			!PdfArea(*lightEnd, &cameraEnd, *lightPrevious, &lightMis[s - 2].pdfPrev)) return false;
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
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster) {
	*contribution = Vec3(0);
	Vec3 unweightedContribution(0);
	if (!EvaluateConnection(scene, camera, lightPath, cameraPath, s, t, &unweightedContribution, raster)) return false;
	if (unweightedContribution == Vec3(0)) return true;
	real weight;
	if (!MisWeight(scene, lightPath, cameraPath, s, t, &weight)) return false;
	*contribution = unweightedContribution * weight;
	return true;
}

GYT_NAMESPACE_END
