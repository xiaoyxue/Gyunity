#include "BDPT.h"
#include <cmath>
#include <initializer_list>
#include <limits>

GYT_NAMESPACE_BEGIN

namespace {

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool Finite(const Vec3& value) {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool Direction(const PathVertex& from, const PathVertex& to, Vec3* dir) {
	if (from.mType == PathVertexType::Environment || to.mType == PathVertexType::Environment) {
		if (from.mType == PathVertexType::Environment && to.mType == PathVertexType::Environment) {
			return Fail("BDPT cannot connect two environment endpoints.");
		}
		*dir = from.mType == PathVertexType::Environment ? -from.mDirection : to.mDirection;
		if (!Finite(*dir) || std::abs(dir->Length2() - 1) > 1e-4f) {
			return Fail("BDPT environment endpoint requires a unit map direction.");
		}
		return true;
	}
	const Vec3 offset = to.mIsect.mPos - from.mIsect.mPos;
	const real length2 = offset.Length2();
	if (!std::isfinite(length2) || length2 <= 0) return Fail("BDPT connection endpoints must be distinct and finite.");
	*dir = offset / std::sqrt(length2);
	return true;
}

bool Surface(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface ||
		(vertex.mLight && vertex.mLight->IsAreaLight());
}

bool Connectible(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Camera ||
		(vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf &&
			vertex.mIsect.mBsdf->HasNonDeltaLobes());
}

bool Counts(const std::vector<PathVertex>& light, const std::vector<PathVertex>& camera, int s, int t) {
	return s >= 0 && t >= 1 && static_cast<size_t>(s) <= light.size() &&
		static_cast<size_t>(t) <= camera.size();
}

bool LightOriginPdf(const Scene& scene, const PathVertex& light, const PathVertex& next, real* pdf) {
	*pdf = 0;
	if (!light.mLight) return Fail("BDPT emitter has no light reference.");
	Vec3 dir;
	if (!Direction(light, next, &dir)) return false;
	real positionPdf, directionPdf;
	if (!light.mLight->PdfLight(light.mIsect, dir, &positionPdf, &directionPdf)) return false;
	real selectionPdf = 0;
	for (size_t i = 0; i < scene.GetLights().size(); ++i) {
		if (scene.GetLights()[i].get() == light.mLight) {
			real slotPdf;
			if (!scene.LightSelectionPdf(i, &slotPdf)) return false;
			selectionPdf += slotPdf;
		}
	}
	*pdf = selectionPdf * (light.mType == PathVertexType::Environment ? directionPdf : positionPdf);
	return true;
}

bool PdfToward(const PathVertex& vertex, const PathVertex* previous, const PathVertex& next, real* pdf) {
	Vec3 wi;
	if (!Direction(vertex, next, &wi)) return false;
	real directionalPdf = 0;
	if (vertex.mType == PathVertexType::Environment) {
		if (!vertex.mLight) return Fail("BDPT environment endpoint has no light reference.");
		real positionPdf, emissionPdf;
		if (!vertex.mLight->PdfLight(vertex.mIsect, wi, &positionPdf, &emissionPdf)) return false;
		return ConvertDensity(positionPdf, vertex, next, pdf);
	}
	if (vertex.mType == PathVertexType::Light || (!previous && vertex.mLight)) {
		real positionPdf;
		if (!vertex.mLight->PdfLight(vertex.mIsect, wi, &positionPdf, &directionalPdf)) return false;
	}
	else if (vertex.mType == PathVertexType::Camera) {
		if (!vertex.mCamera) return Fail("BDPT camera endpoint has no camera reference.");
		directionalPdf = vertex.mCamera->PdfDir(Ray(vertex.mIsect.mPos, wi));
	}
	else {
		if (!previous || !vertex.mIsect.mBsdf) return Fail("BDPT surface PDF requires a predecessor and BSDF.");
		Vec3 wo;
		if (!Direction(vertex, *previous, &wo)) return false;
		if (vertex.mNormalMode == SurfaceNormalMode::Shading) {
			// Radiance mode only queries the symmetric support mask; its cosine never enters a density.
			double support;
			if (!Bidirectional::ScatteringCosine(vertex.mIsect, wo, wi, TransportMode::Radiance, &support)) return false;
			directionalPdf = support == 0 || vertex.mIsect.mBsdf->IsDelta() ? 0 : vertex.mIsect.mBsdf->Pdf(wo, wi);
		}
		else {
			directionalPdf = vertex.mIsect.mBsdf->IsDelta() ? 0 : vertex.mIsect.mBsdf->Pdf(wo, wi);
		}
	}
	return ConvertDensity(directionalPdf, vertex, next, pdf);
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

bool ValidateMisState(const std::vector<MisVertex>& path) {
	for (size_t i = 0; i < path.size(); ++i) {
		const MisVertex& vertex = path[i];
		if (!std::isfinite(vertex.pdfFwd) || vertex.pdfFwd < 0 ||
			!std::isfinite(vertex.pdfPrev) || vertex.pdfPrev < 0) {
			return Fail("BDPT MIS encountered an invalid density.");
		}
		if (vertex.pdfFwd == 0 && !(i > 0 && path[i - 1].delta)) {
			return Fail("BDPT current strategy has zero probability.");
		}
	}
	return true;
}

bool AdvanceMisRatio(const std::vector<MisVertex>& path, size_t i, double* ratio) {
	const MisVertex& vertex = path[i];
	// Only delta-generated edge markers are remapped; ordinary zero support stays zero.
	const double pdfFwd = vertex.pdfFwd == 0 && i > 0 && path[i - 1].delta ? 1 : vertex.pdfFwd;
	const double pdfPrev = vertex.pdfPrev == 0 && i + 1 < path.size() && path[i + 1].delta ? 1 : vertex.pdfPrev;
	*ratio *= pdfPrev / pdfFwd;
	return std::isfinite(*ratio) || Fail("BDPT MIS density ratio overflowed.");
}

bool WorldToScreen(const Camera& camera, const Vec3& point, Vec2* raster) {
	if (Dot(camera.mCz, point - camera.mPos) <= 0) return false;
	const Vec3 projected = (camera.mCameraToRaster * camera.mWorldToCamera)(point);
	if (!Finite(projected) || projected.x < 0 || projected.y < 0 ||
		projected.x >= camera.GetFilm()->mResX || projected.y >= camera.GetFilm()->mResY) return false;
	*raster = Vec2(projected.x, projected.y);
	return true;
}

// Channelwise product of the factors times scale, kept in double until the checked real store.
bool ScaledProduct(std::initializer_list<Vec3> factors, double scale, Vec3* result) {
	for (int channel = 0; channel < 3; ++channel) {
		double value = scale;
		for (const Vec3& factor : factors) value *= double(factor[channel]);
		if (!std::isfinite(value) || std::abs(value) > double((std::numeric_limits<real>::max)())) {
			return Fail("BDPT connection contribution is not representable.");
		}
		(*result)[channel] = static_cast<real>(value);
	}
	return true;
}

// Emitter-side connection factor for shading-normal mode: distance falloff and the area
// emitter's geometric cosine. The receiver's scattering cosine is carried by its K instead.
bool EmitterConnectionFactor(const PathVertex& emitter, const PathVertex& receiver, double* value) {
	*value = 0;
	Vec3 dir;
	if (!Direction(emitter, receiver, &dir)) return false;
	if (emitter.mType == PathVertexType::Environment) {
		*value = 1;
		return true;
	}
	const double dx = double(receiver.mIsect.mPos.x) - double(emitter.mIsect.mPos.x);
	const double dy = double(receiver.mIsect.mPos.y) - double(emitter.mIsect.mPos.y);
	const double dz = double(receiver.mIsect.mPos.z) - double(emitter.mIsect.mPos.z);
	const double distance2 = dx * dx + dy * dy + dz * dz;
	if (!std::isfinite(distance2) || distance2 <= 0) return Fail("BDPT connection distance is invalid.");
	*value = 1 / distance2;
	// Only an emission endpoint contributes its geometric cosine; a scattering vertex that lies
	// on an emissive surface carries its cosine in K.
	if (emitter.mType == PathVertexType::Light && emitter.mLight && emitter.mLight->IsAreaLight()) {
		*value *= std::abs(double(Dot(emitter.mIsect.mGeometryNormal, dir)));
	}
	return std::isfinite(*value) || Fail("BDPT emitter connection factor is not finite.");
}

bool EvaluateEmission(const std::vector<PathVertex>& cameraPath, int t, Vec3* contribution) {
	const PathVertex& pt = cameraPath[t - 1];
	if (!pt.mLight) return true;
	if (pt.mType == PathVertexType::Environment) {
		*contribution = pt.mThroughput * pt.mLight->Emission(pt.mDirection);
	}
	else {
		if (!pt.mLight->IsAreaLight()) return true;
		Vec3 dir;
		if (!Direction(pt, cameraPath[t - 2], &dir)) return false;
		*contribution = pt.mThroughput * pt.mLight->Emission(pt.mIsect, dir);
	}
	return true;
}

bool ConnectCamera(const Camera& camera, const PathVertex& qs, Vec3* contribution, Vec2* raster) {
	if (!Connectible(qs) || !WorldToScreen(camera, qs.mIsect.mPos, raster)) return true;
	Vec3 wi;
	real pdf;
	const Vec3 importance = camera.SampleWi(qs.mIsect, &pdf, &wi, Vec3(0.5f));
	if (!std::isfinite(pdf) || pdf < 0) return Fail("BDPT camera connection PDF is invalid.");
	if (pdf == 0) return true;
	// SampleWi supplies the camera-end geometry through importance / pdf.
	if (qs.mNormalMode == SurfaceNormalMode::Shading) {
		double lightCosine;
		if (!Bidirectional::ScatteringCosine(qs.mIsect, qs.mIsect.mOutDir, wi, TransportMode::Importance, &lightCosine)) return false;
		if (lightCosine == 0) return true;
		const Vec3 f = qs.mIsect.mBsdf->Evaluate(qs.mIsect.mOutDir, wi);
		return ScaledProduct({ qs.mThroughput, f, importance }, lightCosine / double(pdf), contribution);
	}
	*contribution = qs.mThroughput * qs.mIsect.mBsdf->Evaluate(qs.mIsect.mOutDir, wi) *
		importance * std::abs(Dot(qs.mIsect.mNormal, wi)) / pdf;
	return true;
}

bool ConnectEmitter(const Scene& scene, const PathVertex& qs, const PathVertex& pt, Vec3* contribution) {
	if (!Connectible(pt) || !qs.mLight) return true;
	Vec3 toCamera;
	if (!Direction(qs, pt, &toCamera)) return false;
	real originPdf;
	if (!LightOriginPdf(scene, qs, pt, &originPdf)) return false;
	if (originPdf <= 0) return true;
	const Vec3 emission = qs.mType == PathVertexType::Environment ?
		qs.mLight->Emission(qs.mDirection) : qs.mLight->IsAreaLight() ?
		qs.mLight->Emission(qs.mIsect, toCamera) : qs.mLight->Emission();
	// Reuse the sampled light origin, compensating its selection and origin density once.
	if (pt.mNormalMode == SurfaceNormalMode::Shading) {
		double cameraCosine, falloff;
		if (!Bidirectional::ScatteringCosine(pt.mIsect, pt.mIsect.mOutDir, -toCamera, TransportMode::Radiance, &cameraCosine)) return false;
		if (cameraCosine == 0) return true;
		if (!EmitterConnectionFactor(qs, pt, &falloff)) return false;
		const Vec3 f = pt.mIsect.mBsdf->Evaluate(pt.mIsect.mOutDir, -toCamera);
		return ScaledProduct({ emission, pt.mThroughput, f }, cameraCosine * falloff / double(originPdf), contribution);
	}
	real geometry;
	if (!GeometryTerm(qs, pt, &geometry)) return false;
	*contribution = emission / originPdf * pt.mThroughput *
		pt.mIsect.mBsdf->Evaluate(pt.mIsect.mOutDir, -toCamera) * geometry;
	return true;
}

bool ConnectSurfaces(const PathVertex& qs, const PathVertex& pt, Vec3* contribution) {
	if (!Connectible(qs) || !Connectible(pt)) return true;
	Vec3 wi;
	if (!Direction(qs, pt, &wi)) return false;
	if (pt.mNormalMode == SurfaceNormalMode::Shading) {
		// Each K carries its scattering cosine; only distance falloff remains.
		double lightCosine, cameraCosine, falloff;
		if (!Bidirectional::ScatteringCosine(qs.mIsect, qs.mIsect.mOutDir, wi, TransportMode::Importance, &lightCosine) ||
			!Bidirectional::ScatteringCosine(pt.mIsect, pt.mIsect.mOutDir, -wi, TransportMode::Radiance, &cameraCosine)) return false;
		if (lightCosine == 0 || cameraCosine == 0) return true;
		if (!EmitterConnectionFactor(qs, pt, &falloff)) return false;
		const Vec3 fq = qs.mIsect.mBsdf->Evaluate(qs.mIsect.mOutDir, wi);
		const Vec3 fp = pt.mIsect.mBsdf->Evaluate(pt.mIsect.mOutDir, -wi);
		if (!Finite(fq) || !Finite(fp)) return Fail("BDPT connection BSDF is not finite.");
		return ScaledProduct({ qs.mThroughput, pt.mThroughput, fq, fp }, lightCosine * cameraCosine * falloff, contribution);
	}
	real geometry;
	if (!GeometryTerm(qs, pt, &geometry)) return false;
	*contribution = qs.mThroughput * pt.mThroughput * geometry *
		qs.mIsect.mBsdf->Evaluate(qs.mIsect.mOutDir, wi) *
		pt.mIsect.mBsdf->Evaluate(pt.mIsect.mOutDir, -wi);
	return true;
}

}

bool GeometryTerm(const PathVertex& first, const PathVertex& second, real* value) {
	*value = 0;
	Vec3 dir;
	if (!Direction(first, second, &dir)) return false;
	*value = first.mType == PathVertexType::Environment || second.mType == PathVertexType::Environment
		? 1 : 1 / (second.mIsect.mPos - first.mIsect.mPos).Length2();
	if (Surface(first)) *value *= std::abs(Dot(first.mIsect.mGeometryNormal, dir));
	if (Surface(second)) *value *= std::abs(Dot(second.mIsect.mGeometryNormal, dir));
	return std::isfinite(*value) || Fail("BDPT geometry factor is not finite.");
}

bool MisWeight(const Scene& scene, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, real* weight) {
	*weight = 0;
	if (!Counts(lightPath, cameraPath, s, t)) return Fail("BDPT MIS received invalid vertex counts.");
	const size_t count = static_cast<size_t>(s) + t;
	for (int i = 0; i < s; ++i) {
		if (lightPath[i].mType == PathVertexType::Environment && i != 0) {
			return Fail("An environment can only be the light subpath origin.");
		}
	}
	for (int i = 0; i < t; ++i) {
		if (cameraPath[i].mType == PathVertexType::Environment && i != t - 1) {
			return Fail("An environment can only be the camera subpath endpoint.");
		}
	}

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
			!PdfToward(cameraEnd, nullptr, *cameraPrevious, &cameraMis[t - 2].pdfPrev)) return false;
	}
	else {
		if (!PdfToward(*lightEnd, lightPrevious, cameraEnd, &cameraMis[t - 1].pdfPrev)) return false;
		if (cameraPrevious &&
			!PdfToward(cameraEnd, lightEnd, *cameraPrevious, &cameraMis[t - 2].pdfPrev)) return false;

		// Recompute the light-side connection densities.
		if (!PdfToward(cameraEnd, cameraPrevious, *lightEnd, &lightMis[s - 1].pdfPrev)) return false;
		if (lightPrevious &&
			!PdfToward(*lightEnd, &cameraEnd, *lightPrevious, &lightMis[s - 2].pdfPrev)) return false;
	}
	if (!ValidateMisState(lightMis) || !ValidateMisState(cameraMis)) return false;

	// PBRT-v3 balance heuristic: walk alternative splits in both directions.
	double sumRatios = 0, ratio = 1;
	for (int i = t - 1; i > 0; --i) {
		if (!AdvanceMisRatio(cameraMis, i, &ratio)) return false;
		if (!cameraMis[i].delta && !cameraMis[i - 1].delta &&
			ValidConnection(&cameraPath[i], cameraPath[i - 1], count - i, i)) sumRatios += ratio;
	}
	ratio = 1;
	for (int i = s - 1; i >= 0; --i) {
		if (!AdvanceMisRatio(lightMis, i, &ratio)) return false;
		if (!lightMis[i].delta && !(i > 0 && lightMis[i - 1].delta) &&
			ValidConnection(i > 0 ? &lightPath[i - 1] : nullptr, lightPath[i], i, count - i)) sumRatios += ratio;
	}
	if (!std::isfinite(sumRatios)) return Fail("BDPT MIS ratio sum overflowed.");
	*weight = static_cast<real>(1 / (1 + sumRatios));
	return true;
}

bool EvaluateConnection(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster) {
	*contribution = Vec3(0);
	*raster = Vec2(0, 0);
	if (!Counts(lightPath, cameraPath, s, t)) return Fail("BDPT connection received invalid vertex counts.");
	if (static_cast<int64>(s) + t < 2 || (s == 1 && t == 1)) return true;
	const PathVertex& pt = cameraPath[t - 1];
	if (pt.mType == PathVertexType::Environment && s != 0) return true;
	Vec3 unweightedContribution(0);
	if (s == 0) {
		if (!EvaluateEmission(cameraPath, t, &unweightedContribution)) return false;
	}
	else {
		const PathVertex& qs = lightPath[s - 1];
		if (qs.mNormalMode != pt.mNormalMode) return Fail("BDPT subpaths must share one surface normal policy.");
		if (t == 1) {
			if (!ConnectCamera(camera, qs, &unweightedContribution, raster)) return false;
		}
		else if (s == 1) {
			if (!ConnectEmitter(scene, qs, pt, &unweightedContribution)) return false;
		}
		else {
			if (!ConnectSurfaces(qs, pt, &unweightedContribution)) return false;
		}
		if (unweightedContribution != Vec3(0)) {
			if (qs.mType == PathVertexType::Environment) {
				if (scene.Intersect(pt.mIsect.SpawnRay(qs.mDirection))) return true;
			}
			else if (!VisibilityTester(qs.mIsect, pt.mIsect).Unoccluded(scene)) return true;
		}
	}
	if (!Finite(unweightedContribution)) return Fail("BDPT connection contribution is not finite.");
	*contribution = unweightedContribution;
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
