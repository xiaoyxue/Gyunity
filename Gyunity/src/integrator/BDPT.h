#pragma once
#include "Integrator.h"
#include "BDPTPath.h"

GYT_NAMESPACE_BEGIN

bool MisWeight(const Scene& scene, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, real* weight);
bool GeometryTerm(const PathVertex& first, const PathVertex& second, real* value);
// Without MIS, but with the original sampling compensation and visibility checks.
bool EvaluateConnection(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster);
bool ConnectPaths(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster);

class BidirectionalPathTracing : public Integrator {
public:
	BidirectionalPathTracing(std::shared_ptr<Sampler> sampler, int maxDepth, int spp)
		: mSampler(sampler), mMaxDepth(maxDepth), mSpp(spp) {}
	void Render(const Scene& scene, const Camera& camera) override;
	bool RenderChecked(const Scene& scene, const Camera& camera) override;

private:
	std::shared_ptr<Sampler> mSampler;
	int mMaxDepth, mSpp;
};

GYT_NAMESPACE_END
