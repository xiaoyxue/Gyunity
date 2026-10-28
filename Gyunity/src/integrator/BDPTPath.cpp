#include "BDPTPath.h"
#include <cmath>

GYT_NAMESPACE_BEGIN

namespace {

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool OnSurface(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface ||
		(vertex.mType == PathVertexType::Light && vertex.mLight && vertex.mLight->IsAreaLight());
}

int Trace(const Scene& scene, StateSequence& rand, MemoryPool& arena, Ray ray, Vec3 throughput,
	real pdfDir, int maxVertices, TransportMode mode, SurfaceNormalMode normalMode, std::vector<PathVertex>& path) {
	while (static_cast<int>(path.size()) < maxVertices) {
		Intersection isect;
		if (!scene.Intersect(ray, &isect)) {
			if (mode == TransportMode::Radiance && scene.GetEnvironmentLight()) {
				PathVertex endpoint;
				endpoint.mType = PathVertexType::Environment;
				endpoint.mLight = scene.GetEnvironmentLight();
				endpoint.mDirection = ray.mDir;
				endpoint.mThroughput = throughput;
				endpoint.mPdfFwd = pdfDir;
				path.push_back(endpoint);
			}
			break;
		}
		scene.QueryIntersectionInfo(ray, &isect);
		if (!Bidirectional::PrepareSurfaceFrame(&isect, normalMode)) return -1;
		isect.ComputeScatteringFunction(arena, mode);
		PathVertex vertex;
		vertex.mIsect = isect;
		vertex.mLight = isect.mPrimitive->GetLight();
		vertex.mThroughput = throughput;
		vertex.mNormalMode = normalMode;
		vertex.mPdfFwd = Bidirectional::DirectionToArea(pdfDir, path.back(), vertex);
		// Rounded endpoints can lose continuous area support; delta zeros remain valid markers.
		if (vertex.mPdfFwd == 0 && !path.back().mIsDelta) break;
		path.push_back(vertex);
		if (static_cast<int>(path.size()) == maxVertices) break;

		PathVertex& current = path.back();
		PathVertex& previous = path[path.size() - 2];
		const Vec3 wo = -ray.mDir;
		// An incoming direction outside the shading support is a null event: the vertex stays
		// available as a connection endpoint, but the walk cannot continue from it.
		if (normalMode == SurfaceNormalMode::Shading && !Bidirectional::ShadingSupport(isect, wo)) break;
		Vec3 wi;
		real pdf = 0, deltaPdfPrev = -1;
		const real u0 = rand(), u1 = rand(), u2 = rand();
		ScatterEventType sampledType = ScatterEventType(0);
		const Vec3 f = isect.mBsdf->Sample(wo, &wi, &pdf, Vec3(u0, u1, u2), BSDF_ALL,
			&sampledType, nullptr, &deltaPdfPrev);
		if (pdf == 0 || f == Vec3(0)) break;
		if (!Bidirectional::ScatterThroughput(current, mode, wo, wi, f, pdf, &throughput)) break;
		current.mIsDelta = SurfaceEventClassifier::IsDelta(sampledType);
		if (current.mIsDelta) {
			current.mDeltaPdfFwd = pdf;
			current.mDeltaPdfPrev = deltaPdfPrev;
		}
		pdfDir = current.mIsDelta ? 0 : pdf;
		// Evaluate with the saved incoming direction, before replacing the ray.
		const real pdfPrev = current.mIsDelta ? 0 : isect.mBsdf->Pdf(wi, wo);
		previous.mPdfPrev = Bidirectional::DirectionToArea(pdfPrev, current, previous);
		ray = isect.SpawnRay(wi);
	}
	return static_cast<int>(path.size());
}

}

namespace Bidirectional {

real DirectionToArea(real pdfDir, const PathVertex& from, const PathVertex& to) {
	if (to.mType == PathVertexType::Environment) return pdfDir;
	// A continuous ray cannot sample a point-light position.
	if (to.mType == PathVertexType::Light && to.mLight && to.mLight->IsDeltaLight()) return 0;
	if (from.mType == PathVertexType::Environment) {
		return OnSurface(to) ? pdfDir * std::abs(Dot(to.mIsect.mGeometryNormal, from.mDirection)) : pdfDir;
	}
	const Vec3 offset = to.mIsect.mPos - from.mIsect.mPos;
	const real distance2 = offset.Length2();
	real pdfArea = pdfDir / distance2;
	if (OnSurface(to)) pdfArea *= std::abs(Dot(to.mIsect.mGeometryNormal, offset / std::sqrt(distance2)));
	return pdfArea;
}

}

int GenerateCameraPath(const Scene& scene, const Camera& camera, StateSequence& rand, MemoryPool& arena,
	const Ray& cameraRay, int maxVertices, SurfaceNormalMode normalMode, std::vector<PathVertex>& cameraPath) {
	cameraPath.clear();
	if (maxVertices == 0) return 0;
	PathVertex endpoint;
	endpoint.mType = PathVertexType::Camera;
	endpoint.mNormalMode = normalMode;
	endpoint.mCamera = &camera;
	endpoint.mIsect.mPos = cameraRay.mOrig;
	endpoint.mThroughput = Vec3(1);
	endpoint.mPdfPos = camera.PdfPos();
	endpoint.mPdfDir = camera.PdfDir(cameraRay);
	endpoint.mPdfFwd = endpoint.mPdfPos;
	cameraPath.push_back(endpoint);
	return Trace(scene, rand, arena, cameraRay, Vec3(1), endpoint.mPdfDir,
		maxVertices, TransportMode::Radiance, normalMode, cameraPath);
}

int GenerateLightPath(const Scene& scene, StateSequence& rand, MemoryPool& arena,
	int maxVertices, SurfaceNormalMode normalMode, std::vector<PathVertex>& lightPath) {
	lightPath.clear();
	if (maxVertices == 0 || scene.GetLights().empty()) return 0;
	PathVertex endpoint;
	Light* light = scene.SampleOneLight(&endpoint.mLightSelectionPdf, rand());
	// SampleOneLight reports an unusable light power distribution with nullptr.
	if (!light) {
		Fail("BDPT light selection failed; light powers must be finite and positive.");
		return -1;
	}
	const real u0 = rand(), u1 = rand(), v0 = rand(), v1 = rand();
	Vec3 dir;
	const Vec3 radiance = light->SampleLight(&endpoint.mIsect, &dir, &endpoint.mPdfPos, &endpoint.mPdfDir,
		Vec2(u0, u1), Vec2(v0, v1));
	if (endpoint.mPdfPos == 0 || endpoint.mPdfDir == 0) return 0;
	endpoint.mLight = light;
	endpoint.mType = light->IsEnvironmentLight() ? PathVertexType::Environment : PathVertexType::Light;
	endpoint.mNormalMode = normalMode;
	endpoint.mDirection = -dir;
	endpoint.mThroughput = radiance;
	endpoint.mPdfFwd = endpoint.mLightSelectionPdf *
		(light->IsEnvironmentLight() ? endpoint.mPdfDir : endpoint.mPdfPos);
	lightPath.push_back(endpoint);
	if (radiance == Vec3(0) || maxVertices == 1) return 1;
	// Emission is tied to the oriented geometric surface; shapes populate it when sampled.
	const real cosine = light->IsAreaLight() ? std::abs(Dot(endpoint.mIsect.mGeometryNormal, dir)) : 1;
	const Vec3 throughput = radiance * cosine / endpoint.mLightSelectionPdf / endpoint.mPdfPos / endpoint.mPdfDir;
	return Trace(scene, rand, arena, endpoint.mIsect.SpawnRay(dir), throughput,
		light->IsEnvironmentLight() ? endpoint.mPdfPos : endpoint.mPdfDir,
		maxVertices, TransportMode::Importance, normalMode, lightPath);
}

GYT_NAMESPACE_END
