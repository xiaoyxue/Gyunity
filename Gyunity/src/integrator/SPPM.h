#pragma once

#include "common/Core.h"
#include "math/Linalg.h"
#include "Integrator.h"
#include "sampler/SamplerEnum.h"
#include "sampler/Sampler.h"
#include "HashGrid.h"
#include "common/DebugUtils.h"

GYT_NAMESPACE_BEGIN

struct HPoint {
	Vec3 importance, pos, nrm, flux, outDir;
	real r2;
	int64 n; // n = N / ALPHA in the paper
	int64 pix;
	bool used;
	int64 m;
};

class MemoryPool;
class Scene;
class Camera;

class SPPM : public Integrator 
{
private:
	HashGrid<HPoint*> mHashGrid;
	std::shared_ptr<Sampler> mSampler;
	std::shared_ptr<SamplerEnum> mSamplerEnum;
	std::vector<real> mRadius2;
	std::vector<real> mPhotonNums;
	std::vector<Vec3> mFlux;
	std::vector<Vec3> mDirectillum;
	std::vector<HPoint> mHitPoints;
	std::vector<Vec3> mColor;
	std::vector<Spinlock> mPixelLocks;
	const real mInitialRadius;
	const int mMaxDepth;
	const int mIterations;
	const int64 mPhotonsPerRenderStage;
	const real mAlpha;
	bool mBatchShrink;
	bool mTraceGlossyRay;

public:
	SPPM(int iterations, int nPhotonsPerStage, int maxDepth, real initialRadius, real alpha, bool batchShrink, const std::shared_ptr<Sampler>& sampler,
		const std::shared_ptr<SamplerEnum>& samplerEnum, bool traceGlossyRay = false);
	void GeneratePhoton(const Scene& scene, Ray* pr, Vec3* f, real u, const Vec2& v, const Vec2& w);

	void TraceEyePath(const Scene& scene, StateSequence& rand, const Ray& ray, int64 pixel, MemoryPool& arena);

	void TracePhoton(const Scene& scene, StateSequence& rand, const Ray& ray, Vec3 photonFlux, MemoryPool& arena);

	void GenerateRadiusImage(const Scene& scene, const Camera &camera);

	void Render(const Scene& scene, const Camera &camera) override;

private:
	void Initialize(int w, int h);
};

GYT_NAMESPACE_END