#include "VCM.h"
#include "BDPT.h"
#include "VCMPhotonMap.h"
#include "system/ProgressReporter.h"
#include "sampler/Halton.h"
#include "sampler/Sobol.h"
#include "tbb/enumerable_thread_specific.h"
#include "tbb/task_arena.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <typeinfo>

GYT_NAMESPACE_BEGIN

namespace {

using Tasks = ThreadedTaskManager;

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

std::string FormatTime(double seconds) {
	const auto total = static_cast<uint64>(seconds);
	return fmt::format("{}:{:02}:{:02}", total / 3600, (total / 60) % 60, total % 60);
}

// VCM classifies a vertex by its whole BSDF, so a BSDF mixing delta and continuous lobes is unsupported.
bool SupportedPath(const std::vector<PathVertex>& path) {
	for (const auto& vertex : path)
		if (vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf->HasMixedDeltaLobes())
			return Fail("VCM does not support BSDFs that mix delta and continuous lobes.");
	return true;
}

// Low-discrepancy samplers are pure functions of (dimension, index) and can be shared.
bool IndexedSampler(const Sampler& sampler) {
	return typeid(sampler) == typeid(HaltonSampler) || typeid(sampler) == typeid(SobolSampler) ||
		typeid(sampler) == typeid(RegularHaltonSampler);
}

// Light walks draw five emission samples and three per scattering vertex; camera walks also
// draw two jitter samples. Each pixel uses two sample indices per iteration.
bool ValidSamplingBudget(const Sampler& sampler, int maxDepth, int64 pixels, int iterations) {
	uint64 dimensions = 1229, maximumIndex = uint64((std::numeric_limits<int64>::max)()) - 1;
	if (dynamic_cast<const HaltonSampler*>(&sampler)) dimensions = HaltonSampler::get_num_dimensions();
	if (dynamic_cast<const SobolSampler*>(&sampler)) {
		dimensions = sobol::Matrices::num_dimensions;
		maximumIndex = (uint64(1) << sobol::Matrices::size) - 1;
	}
	if ((std::max)(uint64(5), uint64(3) * maxDepth + 2) > dimensions)
		return Fail("VCM path dimension budget exceeds the sampler's supported dimensions.");
	if (uint64(pixels) > ((maximumIndex + 1) / 2) / uint64(iterations))
		return Fail("VCM sample indices exceed the sampler's supported index range.");
	return true;
}

}

struct VertexConnectionMerging::Worker {
	MemoryPool lightArena, cameraArena;
	std::vector<PathVertex> cameraPath;
	std::vector<Vcm::PhotonRecord> photons;
	Vcm::PathContribution connection, merging;
};

struct VertexConnectionMerging::RenderState {
	const Scene& scene;
	const Camera& camera;
	const std::shared_ptr<Film> film;
	const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	int width = 0, height = 0, iteration = 0;
	// MIS sample counts of the VCM paper Eq. (9). nVm counts light-path attempts,
	// including empty ones: neither stored photons nor query hits.
	int64 nVc = 1;              // Number of light sub-paths connected per camera path
	int64 nVm = 0;              // Number of light sub-paths
	int64 screenPixelCount = 0; // Number of pixels
	real radius = 0;
	bool indexedSampling = false;
	ProgressReporter* progress = nullptr;
	std::vector<std::vector<PathVertex>> lightPaths;
	Vcm::PhotonMap photonMap;
	tbb::enumerable_thread_specific<Worker> workers;

	RenderState(const Scene& scene, const Camera& camera) : scene(scene), camera(camera), film(camera.GetFilm()) {}
};

void VertexConnectionMerging::Render(const Scene& scene, const Camera& camera) {
	RenderChecked(scene, camera);
}

bool VertexConnectionMerging::RenderChecked(const Scene& scene, const Camera& camera) {
	mStatistics = {};
	mCancelled.store(false);
	RenderState state(scene, camera);
	if (!Prepare(state)) return false;
	ProgressReporter progress(fmt::format("Rendering VCM ({} x {}, {} iterations, depth {}, concurrency {})",
		state.width, state.height, mIterations, mMaxDepth, mConcurrency),
		(size_t(state.nVm) + size_t(state.screenPixelCount)) * size_t(mIterations));
	state.progress = &progress;
	bool success = true;
	for (state.iteration = 0; state.iteration < mIterations && success; ++state.iteration) {
		success = RenderIteration(state);
		const size_t photons = state.photonMap.RecordCount();
		// Release the map and path references before resetting the arenas that own their BSDFs.
		state.photonMap.Clear();
		for (auto& path : state.lightPaths) path.clear();
		for (auto& worker : state.workers) worker.lightArena.Reset();
		if (success) {
			++mStatistics.mIterations;
			ReportIteration(state, photons);
		}
	}
	mStatistics.mSucceeded = progress.Finish(success);
	return mStatistics.mSucceeded;
}

bool VertexConnectionMerging::Prepare(RenderState& state) const {
	if (!mSampler || mMaxDepth < 0 || mIterations <= 0 || !std::isfinite(mInitialRadius) || mInitialRadius <= 0 ||
		!(mAlpha > 0 && mAlpha < 1) || (!Vcm::UseVc(mMode) && !Vcm::UseVm(mMode)))
		return Fail("VCM requires a sampler, nonnegative depth, positive iterations and radius, alpha in (0,1) and a valid mode.");
	if (!Bidirectional::ValidateScene(state.scene)) return false;
	state.width = state.film->mResX;
	state.height = state.film->mResY;
	// While we have the same number of pixels (camera paths) and light sub-paths, we do
	// keep them separate for clarity reasons.
	state.screenPixelCount = int64(state.width) * state.height;
	state.nVm = int64(state.width) * state.height;
	if (state.screenPixelCount <= 0) return Fail("VCM requires positive image dimensions.");
	if (!ValidSamplingBudget(*mSampler, mMaxDepth, state.screenPixelCount, mIterations)) return false;
	state.indexedSampling = IndexedSampler(*mSampler);
	state.lightPaths.resize(size_t(state.nVm));
	return true;
}

bool VertexConnectionMerging::RenderIteration(RenderState& state) {
	// Eq. (1): progressive radius r_i = r_0 * (i + 1)^((alpha - 1) / 2).
	state.radius = real(double(mInitialRadius) * std::pow(double(state.iteration) + 1, -(1 - double(mAlpha)) / 2));
	mStatistics.mLastRadius = state.radius;
	if (!TraceLightPaths(state)) return false;
	if (UseVm() && !BuildPhotonMap(state)) return false;
	return TraceCameraPaths(state);
}

bool VertexConnectionMerging::TraceLightPaths(RenderState& state) {
	if (!ForEachPath(state, "light", state.nVm, [&](int64 pathIndex, Worker& worker) {
		// Both phases stride by the same per-iteration path count; light takes the even indices.
		const int64 sampleIndex = 2 * (int64(state.iteration) * state.screenPixelCount + pathIndex);
		const auto sampler = PathSampler(state, sampleIndex);
		if (!sampler) return false;
		RandomStateSequence random(sampler, sampleIndex);
		auto& lightPath = state.lightPaths[size_t(pathIndex)];
		return GenerateLightPath(state.scene, random, worker.lightArena, mMaxDepth + 1,
			NormalMode(), lightPath) >= 0 && SupportedPath(lightPath);
	})) return false;
	mStatistics.mLightAttempts += state.nVm;
	return true;
}

bool VertexConnectionMerging::BuildPhotonMap(RenderState& state) {
	if (mCancelled.load()) return false;
	bool built = false;
	tbb::task_arena arena(mConcurrency);
	arena.execute([&]() { built = state.photonMap.Build(state.lightPaths, state.radius); });
	mStatistics.mPeakPhotons = (std::max)(mStatistics.mPeakPhotons, state.photonMap.RecordCount());
	return built && !mCancelled.load();
}

bool VertexConnectionMerging::TraceCameraPaths(RenderState& state) {
	if (!ForEachPath(state, "camera", state.screenPixelCount, [&](int64 pixel, Worker& worker) {
		const bool success = RenderPixel(state, pixel, worker);
		worker.cameraArena.Reset();
		return success;
	})) return false;
	mStatistics.mCameraSamples += state.screenPixelCount;
	return true;
}

// One camera path: connect it to this pixel's light path and merge it with the photon map.
bool VertexConnectionMerging::RenderPixel(RenderState& state, int64 pixel, Worker& worker) const {
	const int64 sampleIndex = 2 * (int64(state.iteration) * state.screenPixelCount + pixel) + 1;
	const auto sampler = PathSampler(state, sampleIndex);
	if (!sampler) return false;
	RandomStateSequence random(sampler, sampleIndex);
	const int x = int(pixel % state.width), y = int(pixel / state.width);
	const real u = random(), v = random();
	const Vec2 raster(x + u, y + v);
	const Ray ray = state.camera.GenerateRay(x, y, Vec2(u, v), 0);
	auto& cameraPath = worker.cameraPath;
	const int nCamera = GenerateCameraPath(state.scene, state.camera, random, worker.cameraArena, ray,
		mMaxDepth + 2, NormalMode(), cameraPath);
	if (nCamera < 0 || !SupportedPath(cameraPath)) return false;

	const auto& lightPath = state.lightPaths[size_t(pixel)];
	worker.connection.clear();
	worker.merging.clear();
	for (int t = 1; t <= nCamera; ++t) {
		if (!Connect(state, lightPath, cameraPath, t, worker.connection)) return false;
		if (UseVm() && !Merge(state, cameraPath, t, worker, worker.merging)) return false;
	}
	WritePathContribution(state, raster, worker.connection, worker.merging);
	return true;
}

// Eq. (2): vertex connection of camera prefix t with every light prefix s, using BDPT's estimator.
bool VertexConnectionMerging::Connect(const RenderState& state, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int t, Vcm::PathContribution& connection) const {
	for (int s = 0; s <= int(lightPath.size()); ++s) {
		const int depth = s + t - 2;
		if (depth < 0 || depth > mMaxDepth ||
			!Bidirectional::ValidConnection(s ? &lightPath[s - 1] : nullptr, cameraPath[t - 1], s, t)) continue;
		const Vcm::Strategy strategy{ Vcm::Technique::Connection, s, t };
		const Vcm::PathContext path(lightPath, cameraPath, strategy, state.radius, state.nVm, mMode);
		if (!Vcm::IsStrategyEnabled(path, strategy)) continue;
		Vec3 contributionVc;
		Vec2 raster;
		if (!EvaluateConnection(state.scene, state.camera, lightPath, cameraPath, s, t, &contributionVc, &raster))
			return false;
		if (contributionVc == Vec3(0)) continue;
		real weightVc;
		if (!Vcm::ComputeMisWeight(state.scene, path, strategy, &weightVc)) return false;
		if (t == 1)
			connection.push_back({ raster, Vcm::ToVector3d(contributionVc * (weightVc / real(mIterations))), true });
		else
			connection.push_back({ Vec2(), Vcm::ToVector3d(contributionVc) * Vector3d(weightVc) / Vector3d(double(state.nVc)) });
	}
	return true;
}

// Eq. (4): vertex merging of camera vertex t-1 with every photon of this iteration within the radius.
bool VertexConnectionMerging::Merge(const RenderState& state, const std::vector<PathVertex>& cameraPath, int t,
	Worker& worker, Vcm::PathContribution& merging) const {
	const PathVertex& x = cameraPath[t - 1];
	if (t < 2 || t - 1 > mMaxDepth || x.mType != PathVertexType::Surface || x.mIsect.mBsdf->IsDelta()) return true;
	state.photonMap.Query(x.mIsect.mPos, x.mIsect.mGeometryNormal, worker.photons);
	for (const auto& photon : worker.photons) {
		const int s = int(photon.mVertexIndex) + 1;
		if (s + t - 3 > mMaxDepth) continue;
		const Vcm::Strategy strategy{ Vcm::Technique::Merging, s, t };
		const Vcm::PathContext path(state.lightPaths[photon.mLightPathIndex], cameraPath, strategy,
			state.radius, state.nVm, mMode);
		const Vector3d contributionVm = Vcm::MergeEstimate(path);
		if (contributionVm == Vector3d(0)) continue;
		real weightVm;
		if (!Vcm::ComputeMisWeight(state.scene, path, strategy, &weightVm)) return false;
		merging.push_back({ Vec2(), Vector3d(weightVm) * contributionVm / Vector3d(double(state.nVm)) });
	}
	return true;
}

// Eq. (5): splats go to the film directly; the camera-pixel estimates form one sample, VC sum plus VM sum.
void VertexConnectionMerging::WritePathContribution(RenderState& state, const Vec2& raster,
	const Vcm::PathContribution& connection, const Vcm::PathContribution& merging) const {
	Vector3d pixelVc(0), pixelVm(0);
	for (const auto& contribution : connection.mContributions) {
		if (contribution.mIsSplat)
			state.film->AddSplat(contribution.mRaster.x, contribution.mRaster.y, Vcm::ToVec3(contribution.mValue));
		else
			pixelVc += contribution.mValue;
	}
	for (const auto& contribution : merging.mContributions) pixelVm += contribution.mValue;
	state.film->AddSample(raster.x, raster.y, Vcm::ToVec3(pixelVc + pixelVm));
}

bool VertexConnectionMerging::ForEachPath(RenderState& state, const char* phase, int64 count,
	const std::function<bool(int64, Worker&)>& task) {
	const auto name = fmt::format("VCM {} phase {}/{}", phase, state.iteration + 1, mIterations);
	state.progress->SetPhase(name);
	const auto result = Tasks::Run(name, 0, size_t(count), [&](size_t index, const Tasks::Context& context) {
		if (mCancelled.load() || context.IsCancellationRequested()) return Tasks::Status::Cancelled;
		return task(int64(index), state.workers.local()) ? Tasks::Status::Success : Tasks::Status::Failed;
	}, mConcurrency, state.progress);
	return result.mStatus == Tasks::Status::Success && !mCancelled.load();
}

// Indexed samplers are shared; other samplers get an independent stream per path.
std::shared_ptr<Sampler> VertexConnectionMerging::PathSampler(const RenderState& state, int64 sampleIndex) const {
	return state.indexedSampling ? mSampler : mSampler->CloneForPath(uint64(sampleIndex));
}

void VertexConnectionMerging::ReportIteration(const RenderState& state, size_t photons) const {
	const int completed = state.iteration + 1;
	const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - state.start).count();
	const double remaining = elapsed * (mIterations - completed) / completed;
	const auto message = fmt::format("VCM iteration {}/{}: radius {:.6g}, {} photons; elapsed {}, remaining ~{}",
		completed, mIterations, state.radius, photons, FormatTime(elapsed), FormatTime(remaining));
	std::fprintf(stderr, "\n%s\n", message.c_str());
	std::fflush(stderr);
}

GYT_NAMESPACE_END
