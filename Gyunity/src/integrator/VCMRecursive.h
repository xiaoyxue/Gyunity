#pragma once

#include "VCM.h"

GYT_NAMESPACE_BEGIN

// VertexConnectionMerging with recursive MIS (Docs/VCM.md, "Recursive MIS") and a compact photon
// grid ("Photon grid"). Sampling, stage frequency, estimators and film writes are the same, and the
// grid accepts the same photons; each subpath is summarized once, so a connection or merge weight
// costs O(1) instead of a full path-density evaluation.
// VertexConnectionMerging stays the log-domain reference for these weights.
class RecursiveVertexConnectionMerging : public Integrator {
public:
	using Statistics = VertexConnectionMerging::Statistics;

	RecursiveVertexConnectionMerging(std::shared_ptr<Sampler> sampler, int maxDepth, int iterations,
		real initialRadius, real alpha = real(2.0 / 3.0), Vcm::Mode mode = Vcm::Mode::Combined,
		int concurrency = ThreadedTaskManager::DefaultConcurrency(), bool useShadingNormals = false,
		int stageFrequency = 1)
		: mSampler(sampler), mMaxDepth(maxDepth), mIterations(iterations), mInitialRadius(initialRadius),
		mAlpha(alpha), mMode(mode), mConcurrency(concurrency), mUseShadingNormals(useShadingNormals),
		mStageFrequency(stageFrequency) {}

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
	bool GenerateCameraSample(RenderState& state, int64 sample, Worker& worker, Vec2* raster) const;
	bool ConnectAndMerge(RenderState& state, int64 sample, const Vec2& raster, Worker& worker) const;
	bool Connect(const RenderState& state, int64 sample, const Worker& worker, int t,
		Vcm::PathContribution& connection) const;
	bool Merge(const RenderState& state, int t, Worker& worker, Vcm::PathContribution& merging) const;
	void WritePathContribution(RenderState& state, const Vec2& raster,
		const Vcm::PathContribution& connection, const Vcm::PathContribution& merging) const;
	double SplatScale(const RenderState& state) const;
	bool ForEachPath(RenderState& state, const char* phase, int64 count,
		const std::function<bool(int64, Worker&)>& task);
	std::shared_ptr<Sampler> PathSampler(const RenderState& state, int64 sampleIndex) const;
	void ReportIteration(const RenderState& state, size_t photons) const;
	bool UseVm() const { return Vcm::UseVm(mMode); }
	SurfaceNormalMode NormalMode() const {
		return mUseShadingNormals ? SurfaceNormalMode::Shading : SurfaceNormalMode::Geometric;
	}

	std::shared_ptr<Sampler> mSampler;
	int mMaxDepth, mIterations;
	real mInitialRadius, mAlpha;
	Vcm::Mode mMode;
	int mConcurrency;
	bool mUseShadingNormals;
	int mStageFrequency;
	std::atomic<bool> mCancelled{ false };
	Statistics mStatistics;
};

GYT_NAMESPACE_END
