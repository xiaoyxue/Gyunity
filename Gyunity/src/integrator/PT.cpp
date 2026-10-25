#include "PT.h"
#include "common/Core.h"
#include "sampler/Sampler.h"
#include "system/Memory.h"
#include "common/DebugUtils.h"
#include "visual/Intersection.h"

GYT_NAMESPACE_BEGIN

Vec3 PathTracing::Li(const Ray& r, const Scene& scene, StateSequence& rand, MemoryPool& arena) const
{
	Ray ray = r;
	bool specularBounce = false;
	Vec3 throughput = Vec3(1, 1, 1);
	Vec3 L;
	for (int i = 0; i < mMaxDepth; ++i) {
		Intersection isect;
		if (!scene.Intersect(ray, &isect)) {
			if ((i == 0 || specularBounce) && scene.GetEnvironmentLight()) {
				L += throughput * scene.GetEnvironmentLight()->Emission(ray);
			}
			break;
		}
		scene.QueryIntersectionInfo(ray, &isect);
		isect.ComputeScatteringFunction(arena);
		BSDF* bsdf = isect.mBsdf;

		if (IsPixelDebugEnabled()) {
			std::cout << "Depth: " << i << " ************************************ \n";
			std::cout << "wo: " << -ray.mDir << std::endl;
		}

		if ((i == 0 || specularBounce) && isect.mPrimitive->IsLight()) {
			const Light* emitter = isect.mPrimitive->GetLight();
			L += throughput * emitter->Emission(isect, isect.mOutDir);
		}
		else if (!bsdf->IsDelta()) {
			L += throughput * DirectIllumination(scene, isect, rand(), Vec2(rand(), rand()), Vec3(rand(), rand(), rand()), rand);

		}

		if (IsPixelDebugEnabled()) {
			std::cout << "Path Tracing: Sample BSDF " << std::endl;
		}
		Vec3 wi;
		real pdf;
		Vec3 f = bsdf->Sample(-ray.mDir, &wi, &pdf, Vec3(rand(), rand(), rand()));
		if (f == Vec3() || pdf == 0) break;
		Vec3 estimation = f * std::abs(Dot(isect.mNormal, wi)) / pdf;
		specularBounce = bsdf->IsDelta();

		real survivalProb = std::min((real)1.0, (estimation * throughput).Y() / throughput.Y());
		if (survivalProb < 1 && i > 5) {
			if (rand() < survivalProb) {
				throughput = throughput / survivalProb;
			}
			else {
				break;
			}
		}
		throughput = throughput * estimation;
		ray = isect.SpawnRay(wi);

		//if (!(bsdf->IsDelta() || (bsdf->MatchScatterType(ScatterEventType::BSDF_GLOSSY) && i < maxDepth - 1))) break;
	}
	return L;
}

PathTracing::PathTracing(int samplePerPixel, int maxDepth, const std::shared_ptr<Sampler>& sampler, const std::shared_ptr<SamplerEnum>& samplerEnum)
	:TiledIntegrator(samplePerPixel, sampler, samplerEnum), mMaxDepth(maxDepth)
{

}

GYT_NAMESPACE_END