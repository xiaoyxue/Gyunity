#pragma once
#include "Integrator.h"
#include "BDPTPath.h"

GYT_NAMESPACE_BEGIN

// Two evaluations of the same balance heuristic over the (s,t) strategies of a path; see
// Docs/BDPT.md "MIS". They agree on continuous paths and differ only at delta vertices:
// Ratio drops a delta edge's sampled mass and geometry, LogPdf keeps them as VCM does.
enum class BdptMis { Ratio, LogPdf };

// PBRT-v3 ratio walk outward from the connection over pdfPrev / pdfFwd.
bool MisWeight(const Scene& scene, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, real* weight);
// Eqs. (1)-(4): enumerates log pdfPath of every strategy, as Vcm::ComputeMisWeight in ConnectionOnly mode.
bool MisWeightLogPdf(const Scene& scene, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, real* weight);
real GeometryTerm(const PathVertex& first, const PathVertex& second);
// Includes sampling compensation and visibility, but not MIS.
bool EvaluateConnection(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster);
bool ConnectPaths(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster,
	BdptMis mis = BdptMis::Ratio);

class BidirectionalPathTracing : public Integrator {
public:
	// useShadingNormals = true enables smooth shading normals with adjoint-corrected transport
	// (#148) on both subpaths; the default keeps the geometric-normal model.
	BidirectionalPathTracing(std::shared_ptr<Sampler> sampler, int maxDepth, int spp, bool useShadingNormals = false,
		BdptMis mis = BdptMis::Ratio)
		: mSampler(sampler), mMaxDepth(maxDepth), mSpp(spp), mUseShadingNormals(useShadingNormals), mMis(mis) {}
	void Render(const Scene& scene, const Camera& camera) override;
	bool RenderChecked(const Scene& scene, const Camera& camera) override;

private:
	std::shared_ptr<Sampler> mSampler;
	int mMaxDepth, mSpp;
	bool mUseShadingNormals;
	BdptMis mMis;
};

GYT_NAMESPACE_END
