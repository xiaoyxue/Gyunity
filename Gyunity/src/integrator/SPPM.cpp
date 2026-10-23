#include "SPPM.h"
#include "image/ImageIO.h"
#include "visual/Scene.h"
#include "system/Threading.h"
#include "light/Light.h"
#include "bsdf/BSDF.h"
#include "visual/Intersection.h"

GYT_NAMESPACE_BEGIN

void SPPM::Initialize(int w, int h)
{
	mRadius2.resize((int64)w * h);
	mPhotonNums.resize((int64)w * h);
	mFlux.resize((int64)w * h);
	mHitPoints.resize((int64)w * h);
	mDirectillum.resize((int64)w * h);
	mColor.resize((int64)w * h);
	mPixelLocks.resize((int64)w * h);
	for (int i = 0; i < w * h; ++i) {
		mRadius2[i] = mInitialRadius * mInitialRadius;
		mPhotonNums[i] = 0;
		mFlux[i] = Vec3();
		mHitPoints[i].m = 0;
	}
}

void SPPM::GeneratePhoton(const Scene& scene, Ray* pr, Vec3* f, real u, const Vec2& v, const Vec2& w) {
	real lightPdf;
	const Light* light = scene.SampleOneLight(&lightPdf, u);
	Vec3 lightDir;
	Intersection lightPoint;
	real pdfPos, pdfDir;
	Vec3 Le = light->SampleLight(&lightPoint, &lightDir, &pdfPos, &pdfDir, v, w);
	*pr = lightPoint.SpawnRay(lightDir);
	real cosTheta = std::abs(lightPoint.mNormal.Dot(lightDir));
	if (pdfPos == 0 || pdfDir == 0) {
		*f = 0;
	}
	else {
		*f = Le * cosTheta / (pdfPos * pdfDir * lightPdf);
	}
}

void SPPM::TraceEyePath(const Scene& scene, StateSequence& rand, const Ray& ray, int64 pixel, MemoryPool& arena) {
	Ray r = ray;
	bool deltaBoundEvent = false;
	Vec3 importance(1.0, 1.0, 1.0);
	for (int i = 0; i < mMaxDepth; ++i) {
		Intersection isect;
		if (!scene.Intersect(r, &isect)) {
			//Environment 
			if ((i == 0 || deltaBoundEvent) && scene.GetEnvironmentLight()) {
				mDirectillum[pixel] += importance * scene.GetEnvironmentLight()->Emission(r);
			}
			return;
		}
		scene.QueryIntersectionInfo(r, &isect);
		isect.ComputeScatteringFunction(arena);
		BSDF* bsdf = isect.mBsdf;
		Vec3 wi;
		real pdf;

		if (bsdf->IsDelta()) {
			Vec3 f = bsdf->Sample(-1 * r.mDir, &wi, &pdf, Vec3(rand(), rand(), rand()));
			if (f == Vec3() || pdf == 0) return;
			importance = f * std::abs(wi.Dot(isect.mNormal)) * importance / pdf;
			//r.o = isect.hit + wi * rayeps + wi.Dot(isect.n) * nEps * isect.n;
			//r.d = wi;
			//r = Ray(isect.hit, wi, Inf, isect.rayEps);
			r = isect.SpawnRay(wi);
			deltaBoundEvent = true;
		}
		else if (mTraceGlossyRay && bsdf->MatchScatterType(ScatterEventType::BSDF_GLOSSY) && i < mMaxDepth - 1) {
			//Trace the ray, when bsdf is glossy
			//But this is not standard photon mapping
			Vec3 Ld = importance * DirectIllumination(scene, isect, rand(), Vec2(rand(), rand()), Vec3(rand(), rand(), rand()), rand);
			mDirectillum[pixel] = mDirectillum[pixel] + Ld;

			Vec3 f = bsdf->Sample(-1 * r.mDir, &wi, &pdf, Vec3(rand(), rand(), rand()));
			if (f == Vec3() || pdf == 0) return;
			importance = f * std::abs(wi.Dot(isect.mNormal)) * importance / pdf;
			r = isect.SpawnRay(wi);
			deltaBoundEvent = false;
		}
		else {
			HPoint& hp = mHitPoints[pixel];
			hp.used = true;
			hp.importance = importance;
			hp.pos = isect.mPos;
			hp.nrm = isect.mAbsNormal;
			hp.pix = pixel;
			hp.outDir = -1 * r.mDir;
			mHitPoints[pixel] = hp;
			if ((i == 0 || deltaBoundEvent) && isect.mPrimitive->IsLight()) {
				const Light* emissionShape = isect.mPrimitive->GetLight();
				mDirectillum[hp.pix] = mDirectillum[hp.pix] + importance * emissionShape->Emission(isect, isect.mOutDir);
				
			}
			else {
				Vec3 Ld = hp.importance * DirectIllumination(scene, isect, rand(), Vec2(rand(), rand()), Vec3(rand(), rand(), rand()), rand);
				mDirectillum[hp.pix] = mDirectillum[hp.pix] + Ld;

			}
			return;
		}
	}
}

void SPPM::TracePhoton(const Scene& scene, StateSequence& rand, const Ray& ray, Vec3 photonFlux, MemoryPool& arena) {
	Ray r = ray;
	for (int i = 0; i < mMaxDepth; ++i) {
		Intersection isect;
		if (!scene.Intersect(r, &isect)) return;
		scene.QueryIntersectionInfo(r, &isect);
		isect.ComputeScatteringFunction(arena, TransportMode::Importance);
		BSDF* bsdf = isect.mBsdf;

		Vec3 wi;
		real pdf;
		Vec3 f = bsdf->Sample(-1 * r.mDir, &wi, &pdf, Vec3(rand(), rand(), rand()));
		if (f == Vec3() || pdf == 0) break;
		Vec3 estimation = f * std::abs(wi.Dot(isect.mNormal)) / pdf;
		if (bsdf->IsDelta()) {
			//photonFlux = photonFlux * estimation;
			//r = isect.SpawnRay(wi);
		}
		else {
			if (i > 0) {
				std::vector<HPoint*>& hp = mHashGrid.GetGrid(isect.mPos);
				for (HPoint* hitpoint : hp) {
					//Use spinlock, but racing condition is rare when using QMC
					//std::lock_guard<Spinlock> lock(mPixelLocks[hitpoint->pix]);
					Vec3 v = hitpoint->pos - isect.mPos;
					//if ((hitpoint->nrm.Dot(isect.n) > PhtotonEdgeEps) && (v.Dot(v) <= radius2[hitpoint->pix])) {
					if ((hitpoint->nrm.Dot(isect.mAbsNormal) > 0.0015) && (v.Dot(v) <= mRadius2[hitpoint->pix])) {
						if (!mBatchShrink) {
							// unlike N in the paper, hitpoint->n stores "N / ALPHA" to make it an integer value
							real g = (mPhotonNums[hitpoint->pix] * mAlpha + mAlpha) / (mPhotonNums[hitpoint->pix] * mAlpha + 1.f);
							mRadius2[hitpoint->pix] = mRadius2[hitpoint->pix] * g;
							mPhotonNums[hitpoint->pix] += 1;
							Vec3 contribution = hitpoint->importance * bsdf->Evaluate(hitpoint->outDir, -1 * r.mDir) * photonFlux;
							mFlux[hitpoint->pix] = (mFlux[hitpoint->pix] + contribution) * g;
						}
						else {
							hitpoint->m++;
							Vec3 contribution = hitpoint->importance * bsdf->Evaluate(hitpoint->outDir, -1 * r.mDir) * photonFlux;
							mFlux[hitpoint->pix] = (mFlux[hitpoint->pix] + contribution);
						}
					}
				}
			}
		}
		//real p = estimation.maxValue();
		real p = std::min((real)1.0, (estimation * photonFlux).Y() / photonFlux.Y());
		if (p < 1) {
			if (rand() < p) {
				photonFlux = photonFlux / p;
			}
			else {
				break;
			}
		}
		photonFlux = photonFlux * estimation;
		r = isect.SpawnRay(wi);
	}
}

void SPPM::GenerateRadiusImage(const Scene& scene, const Camera &camera) {
	int resX = camera.GetFilm()->mResX;
	int resY = camera.GetFilm()->mResY;
	std::vector<Vec3> radImg((int64)resX * resY);
	real minRadius2 = Inf, maxRadius2 = 0.0, avgRadius = 0.0;
	for (int y = 0; y < resY; ++y) {
		for (int x = 0; x < resX; ++x) {
			int index = x + y * resX;
			minRadius2 = std::min(minRadius2, mRadius2[index]);
			maxRadius2 = std::max(maxRadius2, mRadius2[index]);
			avgRadius += std::sqrt(mRadius2[index]);
		}
	}
	std::cout << "minRasius: " << std::sqrt(minRadius2) << std::endl;
	std::cout << "avgRasius: " << avgRadius / resX / resY << std::endl;
	for (int y = 0; y < resY; ++y) {
		for (int x = 0; x < resX; ++x) {
			int index = x + y * resX;
			real val = 1.f - (std::sqrt(mRadius2[index]) - std::sqrt(minRadius2)) /
				(std::sqrt(maxRadius2) - std::sqrt(minRadius2));
			radImg[index].x = val;
			radImg[index].y = val;
			radImg[index].z = val;
		}
	}

	ImageIO::WriteBmpFile("rad_image.bmp", radImg, resX, resY, 2.2f);
}

void SPPM::Render(const Scene& scene, const Camera &camera) {
	//fprintf(stderr, "Rendering ...\n");
	GYT_Print("Rendering ...\n");
	int resX = camera.GetFilm()->mResX;
	int resY = camera.GetFilm()->mResY;
	Initialize(resX, resY);
	std::vector<MemoryPool> memoryArenas(ThreadsNumber());
	for (int iter = 0; iter < mIterations; ++iter) {
		//Trace eye_path
		ParallelFor(0, resY, [&](int y) {
			for (int x = 0; x < resX; x++) {
				MemoryPool& arena = memoryArenas[ThreadIndex()];
				int pixel = x + y * resX;
				mHitPoints[pixel].used = false;
				uint64 instance = mSamplerEnum->GetIndex(iter, x, y);
				RandomStateSequence rand(mSampler, instance);
				real u = mSamplerEnum->SampleX(x, rand());
				real v = mSamplerEnum->SampleY(y, rand());
				Vec2 pixelSample(u, v);
				Ray ray = camera.GenerateRay(x, y, pixelSample);
				TraceEyePath(scene, rand, ray, pixel, arena);
			}
		});
			
		mHashGrid.ClearHashGrid();
		real maxRadius2 = 0.0;
		for (int i = 0; i < (int)mHitPoints.size(); ++i) {
			HPoint& hp = mHitPoints[i];
			if (hp.used) {
				maxRadius2 = std::max(maxRadius2, mRadius2[i]);
				mHashGrid.AddPoint(std::move(std::pair<Vec3, HPoint*>(hp.pos, &hp)), std::sqrt(mRadius2[i]));
			}
		}
		mHashGrid.BuildHashGrid(std::sqrt(maxRadius2) + Eps);

		//Trace photon
		ParallelFor((int64)0, mPhotonsPerRenderStage, [&](int64 j) {
			MemoryPool& arena = memoryArenas[ThreadIndex()];
			Ray ray;
			Vec3 photonFlux;
			RandomStateSequence rand(mSampler, iter * mPhotonsPerRenderStage + j);
			GeneratePhoton(scene, &ray, &photonFlux, rand(), Vec2(rand(), rand()), Vec2(rand(), rand()));
			TracePhoton(scene, rand, ray, photonFlux, arena);
			arena.Reset();
		});

		//Update flux, radius, photonNums if batchShrink
		if (mBatchShrink) {
			size_t nHitPoints = mHitPoints.size();
			ParallelFor(size_t(0), nHitPoints, [&](size_t i) {
				HPoint& hp = mHitPoints[i];
				if (hp.m > 0) {
					real g = (mPhotonNums[hp.pix] + mAlpha * hp.m) / (mPhotonNums[hp.pix] + hp.m);
					mRadius2[hp.pix] = mRadius2[hp.pix] * g;
					mFlux[hp.pix] = mFlux[hp.pix] * g;
					mPhotonNums[hp.pix] = mPhotonNums[hp.pix] + mAlpha * hp.m;
					hp.m = 0;
				}
			});
		}

		for (int i = 0; i < memoryArenas.size(); ++i)
			memoryArenas[i].Reset();

		real percentage = 100.f * (iter + 1) / mIterations;
		//fprintf(stderr, "\rIterations: %5.2f%%", percentage);
		GYT_Print("\rIterations: {:5.2f}%", percentage);
	}

	// density estimation
	std::cout << "\nFlux size: " << mFlux.size() << std::endl;
	ParallelFor(0, (int)mFlux.size(), [&](int i) {
		mColor[i] = mColor[i] + mFlux[i] * (1.f / (PI * mRadius2[i] * mIterations * mPhotonsPerRenderStage))
			+ mDirectillum[i] / (real)mIterations;
		//c[i] = c[i] + flux[i] * (1.f / (PI * radius2[i] * nIterations * nPhotonsPerRenderStage));
	});

	camera.GetFilm()->SetImage(mColor);
	//GenerateRadiusImage(scene);
}

SPPM::SPPM(int iterations, int nPhotonsPerStage, int maxDepth, real initialRadius, real alpha, bool batchShrink, const std::shared_ptr<Sampler>& pSampler, const std::shared_ptr<SamplerEnum>& pSmplerEnum, bool traceGlossyRay /*= false*/)
	: mIterations(iterations), mPhotonsPerRenderStage(nPhotonsPerStage), mBatchShrink(batchShrink),
	mMaxDepth(maxDepth), mInitialRadius(initialRadius), mAlpha(alpha), mSampler(pSampler), mSamplerEnum(pSmplerEnum), mTraceGlossyRay(traceGlossyRay)
{

}

GYT_NAMESPACE_END