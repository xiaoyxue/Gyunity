#pragma once
#include "TiledIntegrator.h"


GYT_NAMESPACE_BEGIN

class StateSequence;
class PathTracing : public TiledIntegrator {
private:
	int mMaxDepth;
public:
	PathTracing(int samplePerPixel, int maxDepth, const std::shared_ptr<Sampler>& sampler, const std::shared_ptr<SamplerEnum>& samplerEnum);
	Vec3 Li(const Ray &r, const Scene& scene, StateSequence& rand, MemoryPool& arena) const override;
};

GYT_NAMESPACE_END
