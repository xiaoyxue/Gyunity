#pragma once
#include "common/DebugUtils.h"
#include <vector>
#include "ForwardDecl.h"
#include "Integrator.h"
#include "sampler/Sampler.h"
#include "PathVertex.h"
#include <mutex>
#include <limits>

GYT_NAMESPACE_BEGIN

class LightTracing : public Integrator {

public:
	LightTracing(std::shared_ptr<Sampler> sampler, int maxDepth, int spp) :
		mSampler(sampler), mSpp(spp), mMaxDepth(maxDepth) { }

	void Render(const Scene& scene, const Camera& camera) override {
		RenderChecked(scene, camera);
	}

	bool RenderChecked(const Scene& scene, const Camera& camera) override {
		if (!mSampler || mMaxDepth < 0 || mMaxDepth == std::numeric_limits<int>::max() || mSpp <= 0) {
			return Fail("LT requires a sampler, nonnegative depth and positive samples per pixel.");
		}
		if (!camera.GetFilm() || camera.GetFilm()->mResX <= 0 || camera.GetFilm()->mResY <= 0) {
			return Fail("LT requires a film with positive dimensions.");
		}
		const int64 pixels = int64(camera.GetFilm()->mResX) * camera.GetFilm()->mResY;
		if (pixels > std::numeric_limits<int64>::max() / mSpp) {
			return Fail("LT sample count exceeds the supported range.");
		}
		int64 totalPhotons = mSpp * (int64)(camera.GetFilm()->mResX) * (int64)(camera.GetFilm()->mResY);
		int64 photonsPerRenderStage = (int64)(camera.GetFilm()->mResX) * (int64)(camera.GetFilm()->mResY);
		std::atomic<int64> workDone = 0;
		std::atomic<bool> success(true);

		int64 resX = camera.GetFilm()->mResX;
		int64 resY = camera.GetFilm()->mResY;
		std::vector<MemoryPool> memoryArenas(ThreadsNumber());
		ParallelFor(int64(0), resY, [&](int64 y) {
			if (!success) return;
			std::shared_ptr<Sampler> clonedSampler = mSampler->Clone(y);
			if (!clonedSampler) {
				success = Fail("LT sampler cloning failed.");
				return;
			}
			MemoryPool& arena = memoryArenas[ThreadIndex()];
			for (int64 x = 0; x < resX && success; ++x) {
				for (int p = 0; p < mSpp && success; ++p) {
					RandomStateSequence rand(clonedSampler, resX * resY * p + y * resX + x);
					std::vector<PathVertex> lightPath(mMaxDepth + 1);
					int nVertices = GenerateLightPath(scene, camera, rand, arena, lightPath);
					if (nVertices < 0) { success = false; break; }
					for (int s = 0; s < nVertices; ++s) {
						Vec3 pRaster;
						bool inScreen;
						Vec3 L;
						if (!ConnectToCameraChecked(scene, camera, rand, lightPath[s], s, &pRaster, &inScreen, &L)) {
							success = false;
							break;
						}
						if (inScreen) {
							//L = L / mSpp;
							L = L * camera.GetFilm()->mResX * camera.GetFilm()->mResY / totalPhotons;
							if (!Finite(L)) {
								success = Fail("LT splat normalization produced nonfinite radiance.");
								break;
							}
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
		return success;


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
		if (mMaxDepth < 0 || depth < 0) return FailPath("LT path depth must be nonnegative.");
		Ray ray = r;
		int bounce = depth;
		real pdfDir = pdfFwd;
		while (bounce <= mMaxDepth && bounce < static_cast<int>(lightPath.size())) {
			if (!Finite(throughput) || !ValidPdf(pdfDir)) return FailPath("LT continuation has invalid throughput or density.");
			if (throughput == Vec3(0) || pdfDir == 0) break;
			if (!Finite(ray.mOrig) || !Finite(ray.mDir) || ray.mDir.Length2() == 0)
				return FailPath("LT continuation has an invalid ray.");
			Intersection& isect = lightPath[bounce].mIsect;

			if (!scene.Intersect(ray, &isect)) break;

			scene.QueryIntersectionInfo(ray, &isect);
			if (!Finite(isect.mPos) || !Finite(isect.mNormal))
				return FailPath("LT encountered a nonfinite surface frame.");
			isect.ComputeScatteringFunction(arena, TransportMode::Importance);
			if (!isect.mBsdf) return FailPath("LT requires a BSDF at each scattering vertex.");

			lightPath[bounce].mIsDelta = false;
			lightPath[bounce].mPdfFwd = pdfDir;
			lightPath[bounce].mThroughput = throughput;

			bounce++;
			if (bounce > mMaxDepth) break;

			Vec3 wi;
			ScatterEventType event = ScatterEventType(0);
			Vec3 f = isect.mBsdf->Sample(-1 * ray.mDir, &wi, &pdfDir, Vec3(rand(), rand(), rand()), BSDF_ALL, &event);
			if (!Finite(f) || !ValidPdf(pdfDir)) return FailPath("LT BSDF sampling returned invalid radiance or density.");
			if (f == Vec3() || pdfDir == 0) break;
			if (!Finite(wi) || wi.Length2() == 0) return FailPath("LT BSDF sampling returned an invalid direction.");
			lightPath[bounce - 1].mIsDelta = SurfaceEventClassifier::IsDelta(event);
			ray = isect.SpawnRay(wi);

			throughput = throughput * f * std::abs(Dot(wi, isect.mNormal)) / pdfDir;
			if (!Finite(throughput)) return FailPath("LT scattering produced nonfinite throughput.");

		}
		return bounce;
	}

	int GenerateLightPath(const Scene& scene, const Camera& camera, StateSequence& rand, MemoryPool& arena, 
		std::vector<PathVertex>& lightPath) const {
		
		if (mMaxDepth < 0 || lightPath.empty()) return FailPath("LT requires nonnegative depth and light-path storage.");
		real pdfLight;
		real pdfPos;
		real pdfDir;
		Vec3 Le;
		Vec3 dir;
		Intersection isect;
		Light* light = scene.SampleOneLight(&pdfLight, rand());
		if (!ValidPdf(pdfLight)) return FailPath("LT light selection returned an invalid density.");
		if (pdfLight == 0) return 0;
		if (!light) return FailPath("LT light selection returned no light with positive density.");
		if (light->IsAreaLight()) {
			Le = light->Emission();
			light->SampleOnLight(&isect, &dir, &pdfPos, &pdfDir, Vec2(rand(), rand()), Vec2(rand(), rand()));
		}
		else {
			Le = light->SampleLight(&isect, &dir, &pdfPos, &pdfDir, Vec2(rand(), rand()), Vec2(rand(), rand()));
		}
		if (!Finite(Le) || !Finite(isect.mPos) || !ValidPdf(pdfPos) || !ValidPdf(pdfDir))
			return FailPath("LT emission sampling returned invalid radiance, position or density.");
		if (pdfPos == 0) return 0;
		lightPath[0].mIsect = isect;
		lightPath[0].mPdfFwd = pdfPos;
		lightPath[0].mThroughput = Le;
		lightPath[0].mIsDelta = light->IsDeltaLight();
		lightPath[0].mType = light->IsEnvironmentLight() ? PathVertexType::Environment : PathVertexType::Light;
		lightPath[0].mLight = light;
		lightPath[0].mLightSelectionPdf = pdfLight;
		lightPath[0].mPdfPos = pdfPos;
		lightPath[0].mPdfDir = pdfDir;
		if (mMaxDepth == 0 || pdfDir == 0 || Le == Vec3(0)) return 1;
		if (!Finite(dir) || dir.Length2() == 0) return FailPath("LT emission sampling returned an invalid direction.");
		real cosTheta = std::abs(Dot(light->IsAreaLight() ? isect.mGeometryNormal : isect.mNormal, dir));
		Vec3 throughput = Le * cosTheta / pdfLight / pdfPos / pdfDir;
		if (!Finite(throughput)) return FailPath("LT emission sampling produced nonfinite throughput.");
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
		Vec3 contribution;
		ConnectToCameraChecked(scene, camera, rand, vertex, s, pRaster, inScreen, &contribution);
		return contribution;
	}

	bool ConnectToCameraChecked(const Scene& scene, const Camera& camera, StateSequence& rand,
		const PathVertex& vertex, int s, Vec3* pRaster, bool* inScreen, Vec3* contribution) const {
		*contribution = Vec3(0);
		*inScreen = false;
		if (!Finite(vertex.mThroughput)) return Fail("LT camera connection has nonfinite throughput.");
		if (s == 0) {
			if (!vertex.mLight) return Fail("LT light origin is missing its light.");
			if (!vertex.mLight->IsAreaLight() || vertex.mLight->IsDeltaLight() ||
				vertex.mLight->IsEnvironmentLight()) return true;
			if (!ValidPdf(vertex.mLightSelectionPdf) || !ValidPdf(vertex.mPdfPos))
				return Fail("LT light origin has an invalid density.");
			if (vertex.mLightSelectionPdf == 0 || vertex.mPdfPos == 0) return true;
			if (!Finite(vertex.mIsect.mGeometryNormal)) return Fail("LT light origin has a nonfinite geometric normal.");
		}
		Vec3 pos = vertex.mIsect.mPos;
		Vec3 hitToCameraDir = camera.mPos - pos;
		if (!Finite(hitToCameraDir) || hitToCameraDir.Length2() == 0)
			return Fail("LT camera connection requires distinct finite positions.");
		Normalize(hitToCameraDir);
		if (Dot(camera.mCz, -1 * hitToCameraDir) <= 0) {
			*inScreen = false;
			return true;
		}
		
		*pRaster = WorldToScreen(camera, pos, inScreen);
		if (!Finite(*pRaster)) return Fail("LT camera projection returned a nonfinite raster position.");
		if (!*inScreen) return true;

		
		//GYT_DEBUG_PIXEL_V3(pRaster->x, pRaster->y, 428, 926, 563, 977, ThreadIndex());
		SelectDebugRegion(pRaster->x, pRaster->y, 429, 640, 565, 644);
		

		Ray ray = vertex.mIsect.SpawnRay(hitToCameraDir);
		ray.mTmax = (camera.mPos - ray.mOrig).Length();
		if (scene.Intersect(ray))
		{
			*inScreen = false;
			return true;
		}

		if (s == 0) {
			real pdfCamera;
			Vec3 wi;
			const Vec3 importance = camera.SampleWi(vertex.mIsect, &pdfCamera, &wi, Vec3(0.5f));
			if (!ValidPdf(pdfCamera) || !Finite(importance)) return Fail("LT camera sampling returned invalid importance or density.");
			if (pdfCamera == 0 || importance == Vec3(0)) return true;
			if (!Finite(wi) || wi.Length2() == 0) return Fail("LT camera sampling returned an invalid direction.");
			const Vec3 emission = vertex.mLight->Emission(vertex.mIsect, wi);
			if (!Finite(emission)) return Fail("LT directional emission returned nonfinite radiance.");
			// The sampled emission direction is not part of this camera connection.
			const Vec3 L = emission * importance * std::abs(Dot(wi, vertex.mIsect.mGeometryNormal)) /
				pdfCamera / vertex.mLightSelectionPdf / vertex.mPdfPos;
			if (!Finite(L)) return Fail("LT light-origin connection produced nonfinite radiance.");
			*contribution = L;
			return true;
		}

		if (!vertex.mIsect.mBsdf || !Finite(vertex.mIsect.mNormal))
			return Fail("LT surface connection requires a BSDF and finite normal.");
		real pdfDir;
		Vec3 wi;
		Vec3 We = camera.SampleWi(vertex.mIsect, &pdfDir, &wi, Vec3(rand(), rand(), rand()));
		if (!ValidPdf(pdfDir) || !Finite(We)) return Fail("LT camera sampling returned invalid importance or density.");
		if (pdfDir == 0 || We == Vec3(0)) return true;
		if (!Finite(wi) || wi.Length2() == 0) return Fail("LT camera sampling returned an invalid direction.");
		Vec3 f = vertex.mIsect.mBsdf->Evaluate(vertex.mIsect.mOutDir, wi);
		if (!Finite(f)) return Fail("LT BSDF evaluation returned nonfinite radiance.");
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

		if (!Finite(L)) return Fail("LT surface connection produced nonfinite radiance.");
		*contribution = L;
		return true;
	}

private:
	static bool Finite(const Vec3& value) {
		return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
	}

	static bool ValidPdf(real pdf) {
		return std::isfinite(pdf) && pdf >= 0;
	}

	static bool Fail(const char* message) {
		GetDefaultLogger()->Error(message, false);
		return false;
	}

	static int FailPath(const char* message) {
		Fail(message);
		return -1;
	}

	int mSpp, mMaxDepth;
	std::shared_ptr<Sampler> mSampler;

	//debug
	mutable std::mutex mMutex;
};

GYT_NAMESPACE_END