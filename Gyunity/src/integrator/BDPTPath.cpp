#include "BDPTPath.h"
#include <cmath>

GYT_NAMESPACE_BEGIN

namespace {

bool Finite(const Vec3& value) {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool ValidPdf(real pdf) {
	return std::isfinite(pdf) && pdf >= 0;
}

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool OnSurface(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface ||
		(vertex.mType == PathVertexType::Light && vertex.mLight && vertex.mLight->IsAreaLight());
}

bool ValidRay(const Ray& ray) {
	if (!Finite(ray.mOrig) || !Finite(ray.mDir) || std::abs(ray.mDir.Length2() - 1) > 1e-4f) {
		return Fail("BDPT requires finite ray origins and unit directions.");
	}
	if (ray.mMedium) return Fail("Surface BDPT does not support participating media.");
	return true;
}

int Trace(const Scene& scene, StateSequence& rand, MemoryPool& arena, Ray ray, Vec3 throughput,
	real pdfFwd, int maxVertices, TransportMode mode, std::vector<PathVertex>& path) {
	while (static_cast<int>(path.size()) < maxVertices) {
		Intersection isect;
		if (!scene.Intersect(ray, &isect)) {
			if (mode == TransportMode::Radiance && scene.GetEnvironmentLight()) {
				PathVertex endpoint;
				endpoint.mType = PathVertexType::Environment;
				endpoint.mLight = scene.GetEnvironmentLight();
				endpoint.mDirection = ray.mDir;
				endpoint.mThroughput = throughput;
				endpoint.mPdfFwd = pdfFwd;
				path.push_back(endpoint);
			}
			break;
		}
		scene.QueryIntersectionInfo(ray, &isect);
		if (isect.mMediumInterface.inside || isect.mMediumInterface.outside) {
			Fail("Surface BDPT does not support medium boundaries.");
			return -1;
		}
		if (!Finite(isect.mGeometryNormal) || isect.mGeometryNormal.Length2() == 0) {
			Fail("BDPT surface has an invalid geometric normal.");
			return -1;
		}
		// This initial BDPT path builder explicitly uses geometric-normal shading only.
		isect.mNormal = isect.mGeometryNormal.Norm();
		CoordinateSystem(isect.mNormal, &isect.mShadingDpDu, &isect.mShadingDpDv);
		isect.ComputeScatteringFunction(arena, mode);
		if (!isect.mBsdf) {
			Fail("Surface BDPT requires a material at every surface vertex.");
			return -1;
		}
		if (isect.mBsdf->HasMixedDeltaLobes()) {
			Fail("BDPT does not yet support mixed delta and continuous BSDF lobes.");
			return -1;
		}
		PathVertex vertex;
		vertex.mIsect = isect;
		vertex.mLight = isect.mPrimitive->GetLight();
		vertex.mThroughput = throughput;
		if (!ConvertDensity(pdfFwd, path.back(), vertex, &vertex.mPdfFwd)) return -1;
		path.push_back(vertex);
		if (static_cast<int>(path.size()) == maxVertices) break;

		PathVertex& current = path.back();
		PathVertex& previous = path[path.size() - 2];
		const Vec3 wo = -ray.mDir;
		Vec3 wi;
		real pdf = 0;
		const real u0 = rand(), u1 = rand(), u2 = rand();
		const Vec3 f = isect.mBsdf->Sample(wo, &wi, &pdf, Vec3(u0, u1, u2));
		if (!ValidPdf(pdf) || !Finite(f)) {
			Fail("BDPT encountered a nonfinite or negative BSDF density/contribution.");
			return -1;
		}
		if (pdf == 0 || f == Vec3(0)) break;
		if (!Finite(wi) || std::abs(wi.Length2() - 1) > 1e-4f) {
			Fail("BDPT BSDF sampling returned an invalid direction.");
			return -1;
		}
		throughput *= f * std::abs(Dot(isect.mNormal, wi)) / pdf;
		if (!Finite(throughput)) {
			Fail("BDPT path throughput is not finite.");
			return -1;
		}
		current.mIsDelta = isect.mBsdf->IsDelta();
		pdfFwd = current.mIsDelta ? 0 : pdf;
		// Evaluate with the saved incoming direction, before replacing the ray.
		const real pdfPrev = current.mIsDelta ? 0 : isect.mBsdf->Pdf(wi, wo);
		if (!ConvertDensity(pdfPrev, current, previous, &previous.mPdfPrev)) return -1;
		ray = isect.SpawnRay(wi);
	}
	return static_cast<int>(path.size());
}

}

namespace Bidirectional {

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

bool Connectible(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Camera ||
		(vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf &&
			!vertex.mIsect.mBsdf->IsDelta());
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
		directionalPdf = vertex.mIsect.mBsdf->IsDelta() ? 0 : vertex.mIsect.mBsdf->Pdf(wo, wi);
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

}

bool ConvertDensity(real pdf, const PathVertex& vertex, const PathVertex& next, real* converted) {
	*converted = 0;
	if (!ValidPdf(pdf)) return Fail("BDPT density must be finite and nonnegative.");
	if (next.mType == PathVertexType::Environment) {
		*converted = pdf;
		return true;
	}
	if (next.mType == PathVertexType::Light && next.mLight && next.mLight->IsDeltaLight()) {
		return true; // A continuous ray cannot sample a point-light position.
	}
	if (vertex.mType == PathVertexType::Environment) {
		if (!Finite(vertex.mDirection) || std::abs(vertex.mDirection.Length2() - 1) > 1e-4f) {
			return Fail("BDPT environment endpoint requires a unit map direction.");
		}
		*converted = pdf;
		if (OnSurface(next)) *converted *= std::abs(Dot(next.mIsect.mGeometryNormal, vertex.mDirection));
	}
	else {
		const Vec3 offset = next.mIsect.mPos - vertex.mIsect.mPos;
		const real distance2 = offset.Length2();
		if (!std::isfinite(distance2) || distance2 <= 0) return Fail("BDPT finite vertices must have distinct finite positions.");
		*converted = pdf / distance2;
		if (OnSurface(next)) {
			*converted *= std::abs(Dot(next.mIsect.mGeometryNormal, offset / std::sqrt(distance2)));
		}
	}
	if (!ValidPdf(*converted)) {
		*converted = 0;
		return Fail("BDPT density conversion produced a nonfinite result.");
	}
	return true;
}

int GenerateCameraPath(const Scene& scene, const Camera& camera, StateSequence& rand, MemoryPool& arena,
	const Ray& cameraRay, int maxVertices, std::vector<PathVertex>& cameraPath) {
	cameraPath.clear();
	if (maxVertices < 0) { Fail("BDPT maximum vertex count must be nonnegative."); return -1; }
	if (maxVertices == 0) return 0;
	if (!ValidRay(cameraRay)) return -1;
	PathVertex endpoint;
	endpoint.mType = PathVertexType::Camera;
	endpoint.mCamera = &camera;
	endpoint.mIsect.mPos = cameraRay.mOrig;
	endpoint.mThroughput = Vec3(1);
	endpoint.mPdfPos = camera.PdfPos();
	endpoint.mPdfDir = camera.PdfDir(cameraRay);
	endpoint.mPdfFwd = endpoint.mPdfPos;
	if (!ValidPdf(endpoint.mPdfPos) || !ValidPdf(endpoint.mPdfDir) ||
		endpoint.mPdfPos == 0 || endpoint.mPdfDir == 0) {
		Fail("BDPT camera ray has an invalid endpoint density.");
		return -1;
	}
	cameraPath.push_back(endpoint);
	return Trace(scene, rand, arena, cameraRay, Vec3(1), endpoint.mPdfDir,
		maxVertices, TransportMode::Radiance, cameraPath);
}

int GenerateLightPath(const Scene& scene, StateSequence& rand, MemoryPool& arena,
	int maxVertices, std::vector<PathVertex>& lightPath) {
	lightPath.clear();
	if (maxVertices < 0) { Fail("BDPT maximum vertex count must be nonnegative."); return -1; }
	if (maxVertices == 0 || scene.GetLights().empty()) return 0;
	PathVertex endpoint;
	Light* light = scene.SampleOneLight(&endpoint.mLightSelectionPdf, rand());
	if (!light || !ValidPdf(endpoint.mLightSelectionPdf) || endpoint.mLightSelectionPdf == 0) {
		Fail("BDPT light selection returned an invalid PMF.");
		return -1;
	}
	if (!light->IsAreaLight() && !light->IsDeltaLight() && !light->IsEnvironmentLight()) {
		Fail("BDPT supports area, point and environment lights only.");
		return -1;
	}
	const real u0 = rand(), u1 = rand(), v0 = rand(), v1 = rand();
	Vec3 dir;
	const Vec3 radiance = light->SampleLight(&endpoint.mIsect, &dir, &endpoint.mPdfPos, &endpoint.mPdfDir,
		Vec2(u0, u1), Vec2(v0, v1));
	if (!ValidPdf(endpoint.mPdfPos) || !ValidPdf(endpoint.mPdfDir) || !Finite(radiance)) {
		Fail("BDPT emitted-ray sampling returned invalid densities/radiance.");
		return -1;
	}
	if (endpoint.mPdfPos == 0 || endpoint.mPdfDir == 0) return 0;
	endpoint.mLight = light;
	endpoint.mType = light->IsEnvironmentLight() ? PathVertexType::Environment : PathVertexType::Light;
	if (light->IsAreaLight()) endpoint.mIsect.mGeometryNormal = endpoint.mIsect.mNormal;
	endpoint.mDirection = -dir;
	endpoint.mThroughput = radiance;
	endpoint.mPdfFwd = endpoint.mLightSelectionPdf *
		(light->IsEnvironmentLight() ? endpoint.mPdfDir : endpoint.mPdfPos);
	const Ray ray = endpoint.mIsect.SpawnRay(dir);
	if (!ValidRay(ray)) return -1;
	lightPath.push_back(endpoint);
	if (radiance == Vec3(0) || maxVertices == 1) return 1;
	const real cosine = light->IsAreaLight() ? std::abs(Dot(endpoint.mIsect.mNormal, dir)) : 1;
	const Vec3 throughput = radiance * cosine / endpoint.mLightSelectionPdf / endpoint.mPdfPos / endpoint.mPdfDir;
	if (!Finite(throughput)) { Fail("BDPT emitted path throughput is not finite."); return -1; }
	return Trace(scene, rand, arena, ray, throughput,
		light->IsEnvironmentLight() ? endpoint.mPdfPos : endpoint.mPdfDir,
		maxVertices, TransportMode::Importance, lightPath);
}

GYT_NAMESPACE_END
