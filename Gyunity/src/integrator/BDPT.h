#pragma once
#include "Integrator.h"
#include "BDPTPath.h"

GYT_NAMESPACE_BEGIN

bool MisWeight(const Scene& scene, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, real* weight);
real GeometryTerm(const PathVertex& first, const PathVertex& second);
// Includes sampling compensation and visibility, but not MIS.
bool EvaluateConnection(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster);
bool ConnectPaths(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster);

class BidirectionalPathTracing : public Integrator {
public:
	// useShadingNormals = true enables smooth shading normals with adjoint-corrected transport
	// (#148) on both subpaths; the default keeps the geometric-normal model.
	BidirectionalPathTracing(std::shared_ptr<Sampler> sampler, int maxDepth, int spp, bool useShadingNormals = false)
		: mSampler(sampler), mMaxDepth(maxDepth), mSpp(spp), mUseShadingNormals(useShadingNormals) {}
	void Render(const Scene& scene, const Camera& camera) override;
	bool RenderChecked(const Scene& scene, const Camera& camera) override;

private:
	std::shared_ptr<Sampler> mSampler;
	int mMaxDepth, mSpp;
	bool mUseShadingNormals;
};

GYT_NAMESPACE_END
