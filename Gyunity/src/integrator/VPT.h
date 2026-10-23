#pragma once
#include "TiledIntegrator.h"

GYT_NAMESPACE_BEGIN

class VolPathTracing : public TiledIntegrator 
{
private:
	int mMaxDepth;
	bool mUseEquiAngularSample;
public:
	VolPathTracing(int samplePerPixel, int maxDepth, const std::shared_ptr<Sampler>& sampler, 
		const std::shared_ptr<SamplerEnum>& samplerEnum, bool equalAngular = true) :
		TiledIntegrator(samplePerPixel, sampler, samplerEnum), 
		mMaxDepth(maxDepth), mUseEquiAngularSample(equalAngular)
	{

	}

	Vec3 Li(const Ray& ray, const Scene& scene, StateSequence& rand, MemoryPool& arena) const override;

private:
	Vec3 ConnectToLight(const Scene& scene, StateSequence& rand,
		const Intersection& isect, const Intersection& lightPoint,
		const Vec3 &Le, real lightPdf) const;
};

GYT_NAMESPACE_END