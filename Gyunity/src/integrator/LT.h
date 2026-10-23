#pragma once
#include <vector>
#include "ForwardDecl.h"
#include "Integrator.h"
#include "sampler/Sampler.h"
#include "PathVertex.h"
#include <mutex>

GYT_NAMESPACE_BEGIN

class LightTracing : public Integrator {

public:
	LightTracing(std::shared_ptr<Sampler> pSampler, int maxDepth, int spp) :
		mSampler(pSampler), mSpp(spp), mMaxDepth(maxDepth) { }

	void Render(const Scene& scene, const Camera& camera) override {

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
		while (true) {
			Intersection& isect = lightPath[bounce].mIsect;

			if (!scene.Intersect(ray, &isect)) break;

			scene.QueryIntersectionInfo(ray, &isect);
			isect.ComputeScatteringFunction(arena, TransportMode::Importance);

			lightPath[bounce].mIsDelta = isect.mBsdf->IsDelta();
			lightPath[bounce].mPdfFwd = pdfDir;
			lightPath[bounce].mThroughput = throughput;

			bounce++;
			if (bounce >= mMaxDepth) break;

			Vec3 wi;
			Vec3 f = isect.mBsdf->Sample(-1 * ray.mDir, &wi, &pdfDir, Vec3(rand(), rand(), rand()));
			ray = Ray(isect.mPos + wi * RAY_EPS, wi);

			throughput = throughput * f * std::abs(Dot(wi, isect.mNormal)) / pdfDir;

		}
		return bounce;
	}

	int GenerateLightPath(const Scene& scene, const Camera& camera, StateSequence& rand, MemoryPool& arena, 
		std::vector<PathVertex>& lightPath) const {
		
		real pdfLight;
		real pdfA;
		real pdfDir;
		Vec3 Le;
		Vec3 dir;
		Intersection isect;
		Light* light = scene.SampleOneLight(&pdfLight, rand());
		if (light->IsAreaLight()) {
			Le = light->Emission();
			light->SampleOnLight(&isect, &dir, &pdfA, &pdfDir, Vec2(rand(), rand()), Vec2(rand(), rand()));
		}
		else {
			Le = light->Emission();
			light->SampleLight(&isect, &dir, &pdfA, &pdfDir, Vec2(rand(), rand()), Vec2(rand(), rand()));
		}
		lightPath[0].mIsect = isect;
		lightPath[0].mPdfFwd = pdfA;
		lightPath[0].mThroughput = Le;
		lightPath[0].mIsDelta = light->IsDeltaLight();
		real cosTheta = std::abs(Dot(isect.mNormal, dir));
		Vec3 throughput = Le * cosTheta / pdfLight / pdfA / pdfDir;
		Ray ray(isect.mPos + dir * RAY_EPS, dir);

		int nVertices = Trace(scene, camera, ray, arena, rand, 1, throughput, pdfDir, lightPath);
		


		return nVertices;
	}

	Vec3 WorldToScreen(const Camera& camera, const Vec3& point, bool* inScreen) const {
		Transform World2Raster = camera.mCameraToRaster * camera.mWorldToCamera;
		Vec3 rasterPoint = World2Raster(point);
		Vec2i rasterPointPrim = Vec2i(int(rasterPoint.x), int(rasterPoint.y));
		if (rasterPointPrim.x >= 0 && rasterPointPrim.x <= camera.GetFilm()->mResX &&
			rasterPointPrim.y >= 0 && rasterPointPrim.y <= camera.GetFilm()->mResY) {
			*inScreen = true;
		}
		else {
			*inScreen = false;
		}
		return Vec3(rasterPointPrim.x, rasterPointPrim.y, 0);
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
		Vec3 pos = vertex.mIsect.mPos;
		Vec3 hitToCameraDir = camera.mPos - pos;
		real hitToCamLength = hitToCameraDir.Length();
		Normalize(hitToCameraDir);
		if (Dot(camera.mCz, -1 * hitToCameraDir) < 0) {
			*inScreen = false;
			return Vec3();
		}
		
		*pRaster = WorldToScreen(camera, pos, inScreen);

		
		//GYT_DEBUG_PIXEL_V3(pRaster->x, pRaster->y, 428, 926, 563, 977, ThreadIndex());
		GYT_DEBUG_PIXEL_V3(pRaster->x, pRaster->y, 429, 640, 565, 644, ThreadIndex());
		

		//Ray ray(pos + hitToCameraDir * RAY_EPS, hitToCameraDir, hitToCamLength);
		Ray ray(pos, hitToCameraDir, hitToCamLength);
		if (scene.Intersect(ray))
		{
			*inScreen = false;
		}

		if (s == 0) {
			return vertex.mThroughput;
		}

		real pdfW;
		Vec3 wi;
		Vec3 We = camera.SampleWi(vertex.mIsect, &pdfW, &wi, Vec3(rand(), rand(), rand()));
		Vec3 f = vertex.mIsect.mBsdf->Evaluate(vertex.mIsect.mOutDir, wi);
		Vec3 L = We * f * vertex.mThroughput * std::abs(Dot(wi, vertex.mIsect.mNormal)) / pdfW;

		//{
		//	std::lock_guard<std::mutex> lock(mMutex);
		//	GYT_DEBUG_PIXEL_IF(ThreadIndex()) {
		//		GYT_PRINT("---------------------------------------------------s: {}--------------------------------------------------------\n", s);
		//		GYT_PRINT("--------------------------------------------------Raster: {}----------------------------------------------------\n", *pRaster);
		//		const PathVertex& v = vertex;
		//		real cos = std::abs(Dot(wi, vertex.mIsect.mNormal));
		//		GYT_PRINT("Throughput: {}, pdfFwd: {}, pos: {}, normal: {}, f: {}, We: {}, pdfW: {}, cos: {}, mOutDir: {}\n",
		//			v.mThroughput, v.mPdfFwd, v.mIsect.mPos, v.mIsect.mNormal, f, We, pdfW, cos, v.mIsect.mOutDir);
		//		GYT_PRINT("L: {}\n", L);
		//	}
		//}

		return L;
	}

private:
	int mSpp, mMaxDepth;
	std::shared_ptr<Sampler> mSampler;

	//debug
	mutable std::mutex mMutex;
};

GYT_NAMESPACE_END