#pragma once

#include "Integrator.h"
#include "VCMPath.h"
#include "system/ThreadedTaskManager.h"
#include <array>
#include <atomic>

GYT_NAMESPACE_BEGIN

namespace Vcm {
namespace Detail {

// Renderer-local checks; legacy sampler implementations and outputs are unchanged.
bool ValidateSamplingBudget(const Sampler& sampler, int maxDepth, int64 pixels, int iterations);

class CheckedSequence : public StateSequence {
public:
	CheckedSequence(Sampler& sampler, int64 instance);
	real Sample() override;
	bool Valid() const { return mValid; }
private:
	Sampler& mSampler;
	int64 mInstance;
	unsigned mDimensions;
	uint64 mMaximumIndex;
	bool mRoundedUnitEndpoint, mValid = true;
};

}
}

class VertexConnectionMerging : public Integrator {
public:
	struct Statistics {
		int mIterations = 0;
		int64 mLightAttempts = 0, mCameraSamples = 0;
		size_t mLightPhases = 0, mCameraPhases = 0, mMapPhases = 0, mMapBuilds = 0;
		size_t mPeakLightArenaBytes = 0, mPeakLightPathBytes = 0, mPeakPhotons = 0;
		real mLastRadius = 0;
		double mLastArea = 0, mLightSeconds = 0, mCameraSeconds = 0, mMapSeconds = 0;
		// Actual mode-weighted RGB sums before film filtering. Camera sums are
		// unaveraged; splat sums already include 1/iterations. Not separate-mode images.
		std::array<double, 3> mCameraConnectionSum{}, mCameraMergingSum{}, mSplatConnectionSum{};
		ThreadedTaskManager::Result mLastLightPhase, mLastCameraPhase, mLastMapPhase;
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
	// Render stages, in the order RenderChecked runs them each iteration:
	//   TraceLightPaths  one light path per pixel (BDPT GenerateLightPath)
	//   BuildPhotonMap   index the batch's continuous light vertices (not in ConnectionOnly)
	//   TraceCameraPaths one camera path per pixel; RenderPixel connects (Connect) and merges
	//                    (Merge) every camera prefix
	using Tasks = ThreadedTaskManager;
	struct Tile;
	struct RenderState;

	bool ValidateRender(const Scene& scene, const Camera& camera, RenderState& state);
	bool PrepareRender(RenderState& state);
	bool RenderIteration(const Scene& scene, const Camera& camera, int iteration, RenderState& state);
	bool TraceLightPaths(const Scene& scene, int iteration, RenderState& state);
	bool BuildPhotonMap(int iteration, RenderState& state);
	bool TraceCameraPaths(const Scene& scene, const Camera& camera, int iteration, RenderState& state);
	Tasks::Status RenderPixel(const Scene& scene, const Camera& camera, int iteration,
		int x, int y, Tile& tile, RenderState& state, const Tasks::Context& context);
	bool Connect(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& light,
		const std::vector<PathVertex>& eye, int t, Tile& tile, const RenderState& state, double sum[3]);
	bool Merge(const Scene& scene, const std::vector<PathVertex>& eye, int t,
		Tile& tile, const RenderState& state, double sum[3]);
	static void ReleaseIteration(RenderState& state);
	void CollectStatistics(const RenderState& state);
	bool Cancelled(const Tasks::Context& context) const;
	bool UsesMerging() const { return mMode != Vcm::Mode::ConnectionOnly; }

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
