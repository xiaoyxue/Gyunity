#pragma once

#include "Integrator.h"
#include "VCMPath.h"
#include "system/ThreadedTaskManager.h"
#include <atomic>
#include <functional>

GYT_NAMESPACE_BEGIN

namespace Vcm {

// One weighted estimate and its film position, as in Yuanming's bidirectional renderer.
// Camera-pixel estimates are averaged into the pixel sample; t = 1 light tracing is splatted.
struct Contribution {
	Vec2 mRaster;
	Vector3d mValue = Vector3d(0);
	bool mIsSplat = false;
};

struct PathContribution {
	std::vector<Contribution> mContributions;

	void push_back(const Contribution& contribution) { mContributions.push_back(contribution); }
	void clear() { mContributions.clear(); }
};

}

class VertexConnectionMerging : public Integrator {
public:
	struct Statistics {
		int mIterations = 0;
		int64 mLightAttempts = 0, mCameraSamples = 0;
		size_t mPeakPhotons = 0;
		real mLastRadius = 0;
		bool mSucceeded = false;
	};

	VertexConnectionMerging(std::shared_ptr<Sampler> sampler, int maxDepth, int iterations,
		real initialRadius, real alpha = real(2.0 / 3.0), Vcm::Mode mode = Vcm::Mode::Combined,
		int concurrency = ThreadedTaskManager::DefaultConcurrency(), bool useShadingNormals = false)
		: mSampler(sampler), mMaxDepth(maxDepth), mIterations(iterations), mInitialRadius(initialRadius),
		mAlpha(alpha), mMode(mode), mConcurrency(concurrency), mUseShadingNormals(useShadingNormals) {}

	// Only geometric-normal VCM is implemented; requesting shading normals fails.
	void Render(const Scene& scene, const Camera& camera) override;
	bool RenderChecked(const Scene& scene, const Camera& camera) override;
	// Cooperative cancellation of an active render. All tasks drain before return.
	void RequestCancellation() { mCancelled.store(true); }
	// Read only after RenderChecked returns. One instance cannot render concurrently.
	const Statistics& GetStatistics() const { return mStatistics; }

private:
	struct Worker;
	struct RenderState;

	bool Prepare(RenderState& state) const;
	bool RenderIteration(RenderState& state);
	bool TraceLightPaths(RenderState& state);
	bool BuildPhotonMap(RenderState& state);
	bool TraceCameraPaths(RenderState& state);
	bool RenderPixel(RenderState& state, int64 pixel, Worker& worker) const;
	bool Connect(const RenderState& state, const std::vector<PathVertex>& lightPath,
		const std::vector<PathVertex>& cameraPath, int t, Vcm::PathContribution& connection) const;
	bool Merge(const RenderState& state, const std::vector<PathVertex>& cameraPath, int t,
		Worker& worker, Vcm::PathContribution& merging) const;
	void WritePathContribution(RenderState& state, const Vec2& raster,
		const Vcm::PathContribution& connection, const Vcm::PathContribution& merging) const;
	bool ForEachPixel(RenderState& state, const char* phase, const std::function<bool(int64, Worker&)>& task);
	std::shared_ptr<Sampler> PathSampler(const RenderState& state, int64 sampleIndex) const;
	void ReportIteration(const RenderState& state, size_t photons) const;
	bool UseVm() const { return Vcm::UseVm(mMode); }

	std::shared_ptr<Sampler> mSampler;
	int mMaxDepth, mIterations;
	real mInitialRadius, mAlpha;
	Vcm::Mode mMode;
	int mConcurrency;
	bool mUseShadingNormals;
	std::atomic<bool> mCancelled{ false };
	Statistics mStatistics;
};

GYT_NAMESPACE_END
