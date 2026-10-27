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
	struct Tile;
	struct RenderState;

	bool Prepare(RenderState& state) const;
	bool RenderIteration(RenderState& state, ProgressReporter& progress);
	ThreadedTaskManager::Status TraceLightPaths(RenderState& state, Tile& tile,
		const ThreadedTaskManager::Context& context);
	ThreadedTaskManager::Status TraceCameraPaths(RenderState& state, Tile& tile,
		const ThreadedTaskManager::Context& context);
	bool Connect(const std::vector<PathVertex>& light, int t,
		const RenderState& state, Tile& tile, double sum[3]) const;
	bool Merge(int t, const RenderState& state, Tile& tile, double sum[3]) const;

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
