#pragma once
#include "common/DebugUtils.h"
#include <vector>
#include "ForwardDecl.h"
#include "Integrator.h"
#include "sampler/Sampler.h"
#include "PathVertex.h"
#include <mutex>

GYT_NAMESPACE_BEGIN

class LightTracing : public Integrator {

public:
	LightTracing(std::shared_ptr<Sampler> sampler, int maxDepth, int spp) :
		mSampler(sampler), mSpp(spp), mMaxDepth(maxDepth) { }

	void Render(const Scene& scene, const Camera& camera) override {

		if (mMaxDepth < 0 || mSpp <= 0) {
			GetDefaultLogger()->Error("LT requires nonnegative depth and positive samples per pixel.", false);
			return;
		}
		int64 totalPhotons = mSpp * (int64)(camera.GetFilm()->mResX) * (int64)(camera.GetFilm()->mResY);
		int64 photonsPerRenderStage = (int64)(camera.GetFilm()->mResX) * (int64)(camera.GetFilm()->mResY);
		std::atomic<int64> workDone = 0;

		int64 resX = camera.GetFilm()->mResX;
		int64 resY = camera.GetFilm()->mResY;
		std::vector<MemoryPool> memoryArenas(ThreadsNumber());
		ParallelFor(int64(0), resY, [&](int64 y) {
			std::shared_ptr<Sampler> clonedSampler = mSampler->Clone(y);
			MemoryPool& arena = memoryArenas[ThreadIndex()];
			for (int64 x = 0; x < resX; ++x) {
				for (int p = 0; p < mSpp; ++p) {
					RandomStateSequence rand(clonedSampler, resX * resY * p + y * resX + x);
					std::vector<PathVertex> lightPath(mMaxDepth + 1);
					int nVertices = GenerateLightPath(scene, camera, rand, arena, lightPath);
					for (int s = 0; s < nVertices; ++s) {
						Vec3 pRaster;
						bool inScreen;
						Vec3 L = ConnectToCamera(scene, camera, rand, lightPath[s], s, &pRaster, &inScreen);
						if (inScreen) {
							//L = L / mSpp;
							L = L * camera.GetFilm()->mResX * camera.GetFilm()->mResY / totalPhotons;
							camera.GetFilm()->AddSplat(pRaster.x, pRaster.y, L);
						}
					}
					workDone++;
				}
			}
			arena.Reset();
			real percentage = 100.f * workDone / totalPhotons;
			fprintf(stderr, "\rPercentage: %5.2f%%", percentage);
		});


		//RandomStateSequence rand(mSampler, 123);
		//MemoryPool arena;
		//for (int p = 0; p < totalPhotons; ++p) {
		//	std::vector<PathVertex> lightPath(mMaxDepth + 1);
		//	int nVertices = GenerateLightPath(scene, camera, rand, arena, lightPath);
		//	for (int s = 0; s < nVertices; ++s) {
		//		Vec3 pRaster;
		//		bool inScreen;
		//		Vec3 L = ConnectToCamera(scene, camera, rand, lightPath[s], s, &pRaster, &inScreen);
		//		if (inScreen) {
		//			L = L / mSpp;
		//			camera.GetFilm()->AddSplat(pRaster.x, pRaster.y, L);
		//		}
		//	}
		//	arena.Reset();
		//	real percentage = 100.f * p / totalPhotons;
		//	fprintf(stderr, "\rPercentage: %5.2f%%", percentage);
		//}


	}

	int Trace(const Scene& scene, const Camera& camera, const Ray& r, MemoryPool& arena, StateSequence& rand,
		int depth, Vec3 throughput, real pdfFwd, std::vector<PathVertex>& lightPath) const {
		
		Ray ray = r;
		int bounce = depth;
		real pdfDir = pdfFwd;
		while (bounce <= mMaxDepth && bounce < static_cast<int>(lightPath.size())) {
			if (!Finite(throughput) || throughput == Vec3(0) ||
				!PositivePdf(pdfDir) || !Finite(ray.mDir) || ray.mDir.Length2() == 0) break;
			Intersection& isect = lightPath[bounce].mIsect;

			if (!scene.Intersect(ray, &isect)) break;

			scene.QueryIntersectionInfo(ray, &isect);
			isect.ComputeScatteringFunction(arena, TransportMode::Importance);
			if (!isect.mBsdf) break;

			lightPath[bounce].mIsDelta = false;
			lightPath[bounce].mPdfFwd = pdfDir;
			lightPath[bounce].mThroughput = throughput;

			bounce++;
			if (bounce > mMaxDepth) break;

			Vec3 wi;
			ScatterEventType event = ScatterEventType(0);
			Vec3 f = isect.mBsdf->Sample(-1 * ray.mDir, &wi, &pdfDir, Vec3(rand(), rand(), rand()), BSDF_ALL, &event);
			if (!Finite(f) || f == Vec3() || !PositivePdf(pdfDir) ||
				!Finite(wi) || wi.Length2() == 0) break;
			lightPath[bounce - 1].mIsDelta = SurfaceEventClassifier::IsDelta(event);
			ray = isect.SpawnRay(wi);

			throughput = throughput * f * std::abs(Dot(wi, isect.mNormal)) / pdfDir;

		}
		return bounce;
	}

	int GenerateLightPath(const Scene& scene, const Camera& camera, StateSequence& rand, MemoryPool& arena, 
		std::vector<PathVertex>& lightPath) const {
		
		if (mMaxDepth < 0 || lightPath.empty()) return 0;
		real pdfLight;
		real pdfPos;
		real pdfDir;
		Vec3 Le;
		Vec3 dir;
		Intersection isect;
		Light* light = scene.SampleOneLight(&pdfLight, rand());
		if (!light || !PositivePdf(pdfLight)) return 0;
		if (light->IsAreaLight()) {
			Le = light->Emission();
			light->SampleOnLight(&isect, &dir, &pdfPos, &pdfDir, Vec2(rand(), rand()), Vec2(rand(), rand()));
		}
		else {
			Le = light->SampleLight(&isect, &dir, &pdfPos, &pdfDir, Vec2(rand(), rand()), Vec2(rand(), rand()));
		}
		if (!Finite(Le) || !Finite(isect.mPos) || !PositivePdf(pdfPos)) return 0;
		lightPath[0].mIsect = isect;
		lightPath[0].mPdfFwd = pdfPos;
		lightPath[0].mThroughput = Le;
		lightPath[0].mIsDelta = light->IsDeltaLight();
		lightPath[0].mType = light->IsEnvironmentLight() ? PathVertexType::Environment : PathVertexType::Light;
		lightPath[0].mLight = light;
		lightPath[0].mLightSelectionPdf = pdfLight;
		lightPath[0].mPdfPos = pdfPos;
		lightPath[0].mPdfDir = pdfDir;
		if (mMaxDepth == 0 || !PositivePdf(pdfDir) || !Finite(dir) || dir.Length2() == 0) return 1;
		real cosTheta = std::abs(Dot(light->IsAreaLight() ? isect.mGeometryNormal : isect.mNormal, dir));
		Vec3 throughput = Le * cosTheta / pdfLight / pdfPos / pdfDir;
		Ray ray = isect.SpawnRay(dir);

		int nVertices = Trace(scene, camera, ray, arena, rand, 1, throughput, pdfDir, lightPath);
		


		return nVertices;
	}

	Vec3 WorldToScreen(const Camera& camera, const Vec3& point, bool* inScreen) const {
		Transform World2Raster = camera.mCameraToRaster * camera.mWorldToCamera;
		Vec3 rasterPoint = World2Raster(point);
		if (Finite(rasterPoint) && rasterPoint.x >= 0 && rasterPoint.x < camera.GetFilm()->mResX &&
			rasterPoint.y >= 0 && rasterPoint.y < camera.GetFilm()->mResY) {
			*inScreen = true;
		}
		else {
			*inScreen = false;
		}
		return Vec3(rasterPoint.x, rasterPoint.y, 0);
	}

	//Vec3 WorldToScreen(const Camera& camera, const Vec3& point, bool* inScreen) const {
	//	*inScreen = true;
	//	Vec3 dir = (point - camera.mPos).Norm();
	//	real cosCamera = camera.mCz.Dot(dir);
	//	real cameraToScreenDis = camera.mFilmDistance / cosCamera;
	//	Vec3 screenPoint = camera.mPos + dir * cameraToScreenDis;

	//	Vec3 screenCenter = camera.mPos + camera.mFilmDistance * camera.mCz;
	//	Vec3 centerToPoint = screenPoint - screenCenter;
	//	if (std::abs(centerToPoint.Dot(camera.mCx)) > (camera.GetFilm()->mUpperLeft - camera.GetFilm()->mUpperRight).Length() / 2.f ||
	//		std::abs(centerToPoint.Dot(camera.mCy)) > (camera.GetFilm()->mLowerLeft - camera.GetFilm()->mUpperLeft).Length() / 2.f) {
	//		*inScreen = false;
	//		return Vec3(-1, -1, -1); //out of screen
	//	}

	//	Vec3 pointLU = screenPoint - camera.GetFilm()->mUpperLeft;
	//	real disLUP = pointLU.Length();
	//	real cosTheta = (camera.GetFilm()->mLowerLeft - camera.GetFilm()->mUpperLeft).Norm().Dot(pointLU.Norm());
	//	real sinTheta = std::sqrt(1 - cosTheta * cosTheta);
	//	real pH = disLUP * cosTheta;
	//	real pW = disLUP * sinTheta;
	//	real alpha = pW / (camera.GetFilm()->mUpperRight - camera.GetFilm()->mUpperLeft).Length();
	//	real beta = pH / (camera.GetFilm()->mLowerLeft - camera.GetFilm()->mUpperLeft).Length();

	//	int px = (int)(camera.GetFilm()->mResX * alpha);
	//	int py = (int)(camera.GetFilm()->mResX * beta);

	//	return Vec3(px, py, 0);
	//}

	Vec3 ConnectToCamera(const Scene& scene, const Camera& camera, StateSequence& rand,
		const PathVertex& vertex, int s, Vec3* pRaster, bool* inScreen) const {
		*inScreen = false;
		if (s == 0 && (!vertex.mLight || !vertex.mLight->IsAreaLight() ||
			vertex.mLight->IsDeltaLight() || vertex.mLight->IsEnvironmentLight() ||
			!PositivePdf(vertex.mLightSelectionPdf) || !PositivePdf(vertex.mPdfPos))) return Vec3(0);
		Vec3 pos = vertex.mIsect.mPos;
		Vec3 hitToCameraDir = camera.mPos - pos;
		if (!Finite(hitToCameraDir) || hitToCameraDir.Length2() == 0) return Vec3(0);
		Normalize(hitToCameraDir);
		if (Dot(camera.mCz, -1 * hitToCameraDir) <= 0) {
			*inScreen = false;
			return Vec3();
		}
		
		*pRaster = WorldToScreen(camera, pos, inScreen);
		if (!*inScreen) return Vec3(0);

		
		//GYT_DEBUG_PIXEL_V3(pRaster->x, pRaster->y, 428, 926, 563, 977, ThreadIndex());
		SelectDebugRegion(pRaster->x, pRaster->y, 429, 640, 565, 644);
		

		Ray ray = vertex.mIsect.SpawnRay(hitToCameraDir);
		ray.mTmax = (camera.mPos - ray.mOrig).Length();
		if (scene.Intersect(ray))
		{
			*inScreen = false;
			return Vec3(0);
		}

		if (s == 0) {
			real pdfCamera;
			Vec3 wi;
			const Vec3 importance = camera.SampleWi(vertex.mIsect, &pdfCamera, &wi, Vec3(0.5f));
			if (!PositivePdf(pdfCamera) || !Finite(importance)) return Vec3(0);
			const Vec3 emission = vertex.mLight->Emission(vertex.mIsect, wi);
			// The sampled emission direction is not part of this camera connection.
			const Vec3 L = emission * importance * std::abs(Dot(wi, vertex.mIsect.mGeometryNormal)) /
				pdfCamera / vertex.mLightSelectionPdf / vertex.mPdfPos;
			return Finite(L) ? L : Vec3(0);
		}

		if (!vertex.mIsect.mBsdf) return Vec3(0);
		real pdfDir;
		Vec3 wi;
		Vec3 We = camera.SampleWi(vertex.mIsect, &pdfDir, &wi, Vec3(rand(), rand(), rand()));
		if (!PositivePdf(pdfDir) || !Finite(We)) return Vec3(0);
		Vec3 f = vertex.mIsect.mBsdf->Evaluate(vertex.mIsect.mOutDir, wi);
		Vec3 L = We * f * vertex.mThroughput * std::abs(Dot(wi, vertex.mIsect.mNormal)) / pdfDir;

		//{
		//	std::lock_guard<std::mutex> lock(mMutex);
		//	GYT_DEBUG_PIXEL_IF(ThreadIndex()) {
		//		GYT_PRINT("---------------------------------------------------s: {}--------------------------------------------------------\n", s);
		//		GYT_PRINT("--------------------------------------------------Raster: {}----------------------------------------------------\n", *pRaster);
		//		const PathVertex& v = vertex;
		//		real cos = std::abs(Dot(wi, vertex.mIsect.mNormal));
		//		GYT_PRINT("Throughput: {}, pdfFwd: {}, pos: {}, normal: {}, f: {}, We: {}, pdfDir: {}, cos: {}, mOutDir: {}\n",
		//			v.mThroughput, v.mPdfFwd, v.mIsect.mPos, v.mIsect.mNormal, f, We, pdfDir, cos, v.mIsect.mOutDir);
		//		GYT_PRINT("L: {}\n", L);
		//	}
		//}

		return Finite(L) ? L : Vec3(0);
	}

private:
	static bool Finite(const Vec3& value) {
		return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
	}

	static bool PositivePdf(real pdf) {
		return std::isfinite(pdf) && pdf > 0;
	}

	int mSpp, mMaxDepth;
	std::shared_ptr<Sampler> mSampler;

	//debug
	mutable std::mutex mMutex;
};

GYT_NAMESPACE_END