#include "VCMRecursive.h"
#include "BDPT.h"
#include "VCMPhotonGrid.h"
#include "VCMRecursiveMis.h"
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
// draw two jitter samples. Each path uses two sample indices per iteration.
bool ValidSamplingBudget(const Sampler& sampler, int maxDepth, int64 paths, int iterations) {
	uint64 dimensions = 1229, maximumIndex = uint64((std::numeric_limits<int64>::max)()) - 1;
	if (dynamic_cast<const HaltonSampler*>(&sampler)) dimensions = HaltonSampler::get_num_dimensions();
	if (dynamic_cast<const SobolSampler*>(&sampler)) {
		dimensions = sobol::Matrices::num_dimensions;
		maximumIndex = (uint64(1) << sobol::Matrices::size) - 1;
	}
	if ((std::max)(uint64(5), uint64(3) * maxDepth + 2) > dimensions)
		return Fail("VCM path dimension budget exceeds the sampler's supported dimensions.");
	if (uint64(paths) > ((maximumIndex + 1) / 2) / uint64(iterations))
		return Fail("VCM sample indices exceed the sampler's supported index range.");
	return true;
}

}

struct RecursiveVertexConnectionMerging::Worker {
	MemoryPool lightArena, cameraArena;
	std::vector<PathVertex> cameraPath;
	std::vector<Vcm::MisVertex> cameraMis;
	std::vector<Vcm::PhotonRecord> photons;
	Vcm::PathContribution connection, merging;
};

struct RecursiveVertexConnectionMerging::RenderState {
	const Scene& scene;
	const Camera& camera;
	const std::shared_ptr<Film> film;
	const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	int width = 0, height = 0, iteration = 0;
	int64 nVc = 1;              // Number of light sub-paths connected per camera path
	int64 nVm = 0;              // Number of light sub-paths, and of camera paths
	int64 screenPixelCount = 0; // Number of pixels, used only to normalize the film
	real radius = 0;
	double eta = 0;             // nVm * pi * radius^2, or 0 without merging (Eq. 14)
	bool indexedSampling = false;
	ProgressReporter* progress = nullptr;
	std::vector<std::vector<PathVertex>> lightPaths;
	std::vector<std::vector<Vcm::MisVertex>> lightMis;
	Vcm::PhotonGrid photonMap;
	tbb::enumerable_thread_specific<Worker> workers;

	RenderState(const Scene& scene, const Camera& camera) : scene(scene), camera(camera), film(camera.GetFilm()) {}
};

void RecursiveVertexConnectionMerging::Render(const Scene& scene, const Camera& camera) {
	RenderChecked(scene, camera);
}

bool RecursiveVertexConnectionMerging::RenderChecked(const Scene& scene, const Camera& camera) {
	mStatistics = {};
	mCancelled.store(false);
	RenderState state(scene, camera);
	if (!Prepare(state)) return false;
	ProgressReporter progress(fmt::format("Rendering recursive VCM ({} x {}, {} iterations, depth {}, concurrency {})",
		state.width, state.height, mIterations, mMaxDepth, mConcurrency),
		2 * size_t(state.nVm) * size_t(mIterations));
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

bool RecursiveVertexConnectionMerging::Prepare(RenderState& state) const {
	if (!mSampler || mMaxDepth < 0 || mIterations <= 0 || !std::isfinite(mInitialRadius) || mInitialRadius <= 0 ||
		!(mAlpha > 0 && mAlpha < 1) || (!Vcm::UseVc(mMode) && !Vcm::UseVm(mMode)))
		return Fail("VCM requires a sampler, nonnegative depth, positive iterations and radius, alpha in (0,1) and a valid mode.");
	if (!Bidirectional::ValidateScene(state.scene)) return false;
	state.width = state.film->mResX;
	state.height = state.film->mResY;
	state.screenPixelCount = int64(state.width) * state.height;
	if (state.screenPixelCount <= 0) return Fail("VCM requires positive image dimensions.");
	if (mStageFrequency < 1 || mIterations < mStageFrequency)
		return Fail("VCM requires a positive stage frequency and at least that many iterations.");
	state.nVm = (state.screenPixelCount + mStageFrequency - 1) / mStageFrequency;
	if (!ValidSamplingBudget(*mSampler, mMaxDepth, state.nVm, mIterations)) return false;
	state.indexedSampling = IndexedSampler(*mSampler);
	state.lightPaths.resize(size_t(state.nVm));
	state.lightMis.resize(size_t(state.nVm));
	return true;
}

bool RecursiveVertexConnectionMerging::RenderIteration(RenderState& state) {
	// Eq. (1): progressive radius r_i = r_0 * (i + 1)^((alpha - 1) / 2).
	state.radius = real(double(mInitialRadius) * std::pow(double(state.iteration) + 1, -(1 - double(mAlpha)) / 2));
	state.eta = UseVm() ? double(state.nVm) * std::acos(-1.0) * double(state.radius) * double(state.radius) : 0;
	mStatistics.mLastRadius = state.radius;
	if (!TraceLightPaths(state)) return false;
	if (UseVm() && !BuildPhotonMap(state)) return false;
	return TraceCameraPaths(state);
}

bool RecursiveVertexConnectionMerging::TraceLightPaths(RenderState& state) {
	if (!ForEachPath(state, "light", state.nVm, [&](int64 pathIndex, Worker& worker) {
		// Both phases stride by the same per-iteration path count; light takes the even indices.
		const int64 sampleIndex = 2 * (int64(state.iteration) * state.nVm + pathIndex);
		const auto sampler = PathSampler(state, sampleIndex);
		if (!sampler) return false;
		RandomStateSequence random(sampler, sampleIndex);
		auto& lightPath = state.lightPaths[size_t(pathIndex)];
		if (GenerateLightPath(state.scene, random, worker.lightArena, mMaxDepth + 1,
			NormalMode(), lightPath) < 0 || !SupportedPath(lightPath)) return false;
		Vcm::PrepareLightMis(lightPath, state.eta, mMode, state.lightMis[size_t(pathIndex)]);
		return true;
	})) return false;
	mStatistics.mLightAttempts += state.nVm;
	return true;
}

bool RecursiveVertexConnectionMerging::BuildPhotonMap(RenderState& state) {
	if (mCancelled.load()) return false;
	bool built = false;
	tbb::task_arena arena(mConcurrency);
	arena.execute([&]() { built = state.photonMap.Build(state.lightPaths, state.radius); });
	mStatistics.mPeakPhotons = (std::max)(mStatistics.mPeakPhotons, state.photonMap.RecordCount());
	return built && !mCancelled.load();
}

bool RecursiveVertexConnectionMerging::TraceCameraPaths(RenderState& state) {
	if (!ForEachPath(state, "camera", state.nVm, [&](int64 sample, Worker& worker) {
		Vec2 raster;
		if (!GenerateCameraSample(state, sample, worker, &raster)) return false;
		return ConnectAndMerge(state, sample, raster, worker);
	})) return false;
	mStatistics.mCameraSamples += state.nVm;
	return true;
}

// The same film placement as VertexConnectionMerging: sample k of iteration i takes pixel
// (k + i * nVm) mod P. The camera subpath is summarized once for every weight that reads it.
bool RecursiveVertexConnectionMerging::GenerateCameraSample(RenderState& state, int64 sample, Worker& worker,
	Vec2* raster) const {
	worker.cameraArena.Reset();
	const int64 sampleIndex = 2 * (int64(state.iteration) * state.nVm + sample) + 1;
	const auto sampler = PathSampler(state, sampleIndex);
	if (!sampler) return false;
	RandomStateSequence random(sampler, sampleIndex);
	const int64 pixel = (sample + int64(state.iteration) * state.nVm) % state.screenPixelCount;
	const int x = int(pixel % state.width), y = int(pixel / state.width);
	const real u = random(), v = random();
	*raster = Vec2(x + u, y + v);
	const Ray ray = state.camera.GenerateRay(x, y, Vec2(u, v), 0);
	const int nCamera = GenerateCameraPath(state.scene, state.camera, random, worker.cameraArena,
		ray, mMaxDepth + 2, NormalMode(), worker.cameraPath);
	if (nCamera < 0 || !SupportedPath(worker.cameraPath)) return false;
	Vcm::PrepareCameraMis(worker.cameraPath, state.eta, mMode, worker.cameraMis);
	return true;
}

bool RecursiveVertexConnectionMerging::ConnectAndMerge(RenderState& state, int64 sample, const Vec2& raster,
	Worker& worker) const {
	worker.connection.clear();
	worker.merging.clear();
	for (int t = 1; t <= int(worker.cameraPath.size()); ++t) {
		if (!Connect(state, sample, worker, t, worker.connection)) return false;
		if (UseVm() && !Merge(state, t, worker, worker.merging)) return false;
	}
	WritePathContribution(state, raster, worker.connection, worker.merging);
	return true;
}

// Eq. (2) with the Eq. (15) weight: camera prefix t with every light prefix s of its light path.
bool RecursiveVertexConnectionMerging::Connect(const RenderState& state, int64 sample, const Worker& worker,
	int t, Vcm::PathContribution& connection) const {
	const auto& lightPath = state.lightPaths[size_t(sample)];
	const auto& cameraPath = worker.cameraPath;
	for (int s = 0; s <= int(lightPath.size()); ++s) {
		const int depth = s + t - 2;
		if (depth < 0 || depth > mMaxDepth ||
			!Bidirectional::ValidConnection(s ? &lightPath[s - 1] : nullptr, cameraPath[t - 1], s, t)) continue;
		const Vcm::Strategy strategy{ Vcm::Technique::Connection, s, t };
		if (!Vcm::IsStrategyEnabled(Vcm::PathContext(lightPath, cameraPath, strategy, state.radius, state.nVm, mMode),
			strategy)) continue;
		Vec3 contributionVc;
		Vec2 raster;
		if (!EvaluateConnection(state.scene, state.camera, lightPath, cameraPath, s, t, &contributionVc, &raster))
			return false;
		if (contributionVc == Vec3(0)) continue;
		real weightVc;
		if (!Vcm::RecursiveConnectionWeight(state.scene, lightPath, state.lightMis[size_t(sample)], cameraPath,
			worker.cameraMis, s, t, state.eta, mMode, &weightVc)) return false;
		if (t == 1)
			connection.push_back({ raster, Vcm::ToVector3d(contributionVc * weightVc) * Vector3d(SplatScale(state)), true });
		else
			connection.push_back({ Vec2(), Vcm::ToVector3d(contributionVc) * Vector3d(weightVc) / Vector3d(double(state.nVc)) });
	}
	return true;
}

// Eq. (4) with the Eq. (16) weight: camera vertex t-1 with every photon of this iteration within the radius.
bool RecursiveVertexConnectionMerging::Merge(const RenderState& state, int t, Worker& worker,
	Vcm::PathContribution& merging) const {
	const auto& cameraPath = worker.cameraPath;
	const PathVertex& x = cameraPath[t - 1];
	if (t < 2 || t - 1 > mMaxDepth || x.mType != PathVertexType::Surface || x.mIsect.mBsdf->IsDelta()) return true;
	state.photonMap.Query(x.mIsect.mPos, x.mIsect.mGeometryNormal, worker.photons);
	for (const auto& photon : worker.photons) {
		const int s = int(photon.mVertexIndex) + 1;
		if (s + t - 3 > mMaxDepth) continue;
		const auto& lightPath = state.lightPaths[photon.mLightPathIndex];
		const Vcm::Strategy strategy{ Vcm::Technique::Merging, s, t };
		const Vector3d contributionVm = Vcm::MergeEstimate(
			Vcm::PathContext(lightPath, cameraPath, strategy, state.radius, state.nVm, mMode));
		if (contributionVm == Vector3d(0)) continue;
		real weightVm;
		if (!Vcm::RecursiveMergeWeight(lightPath, state.lightMis[photon.mLightPathIndex], cameraPath,
			worker.cameraMis, s, t, state.eta, &weightVm)) return false;
		merging.push_back({ Vec2(), Vector3d(weightVm) * contributionVm / Vector3d(double(state.nVm)) });
	}
	return true;
}

// Eq. (5), as in VertexConnectionMerging.
void RecursiveVertexConnectionMerging::WritePathContribution(RenderState& state, const Vec2& raster,
	const Vcm::PathContribution& connection, const Vcm::PathContribution& merging) const {
	Vector3d pixelVc(0), pixelVm(0);
	for (const auto& contribution : connection.mContributions) {
		if (contribution.mIsSplat)
			state.film->AddSplat(contribution.mRaster.x, contribution.mRaster.y,
				Vcm::ToVec3(contribution.mValue));
		else
			pixelVc += contribution.mValue;
	}
	for (const auto& contribution : merging.mContributions) pixelVm += contribution.mValue;
	state.film->AddSample(raster.x, raster.y, Vcm::ToVec3(pixelVc + pixelVm));
}

double RecursiveVertexConnectionMerging::SplatScale(const RenderState& state) const {
	return double(state.screenPixelCount) / (double(state.nVm) * mIterations);
}

bool RecursiveVertexConnectionMerging::ForEachPath(RenderState& state, const char* phase, int64 count,
	const std::function<bool(int64, Worker&)>& task) {
	const auto name = fmt::format("Recursive VCM {} phase {}/{}", phase, state.iteration + 1, mIterations);
	state.progress->SetPhase(name);
	const auto result = Tasks::Run(name, 0, size_t(count), [&](size_t index, const Tasks::Context& context) {
		if (mCancelled.load() || context.IsCancellationRequested()) return Tasks::Status::Cancelled;
		return task(int64(index), state.workers.local()) ? Tasks::Status::Success : Tasks::Status::Failed;
	}, mConcurrency, state.progress);
	return result.mStatus == Tasks::Status::Success && !mCancelled.load();
}

// Indexed samplers are shared; other samplers get an independent stream per path.
std::shared_ptr<Sampler> RecursiveVertexConnectionMerging::PathSampler(const RenderState& state, int64 sampleIndex) const {
	return state.indexedSampling ? mSampler : mSampler->CloneForPath(uint64(sampleIndex));
}

void RecursiveVertexConnectionMerging::ReportIteration(const RenderState& state, size_t photons) const {
	const int completed = state.iteration + 1;
	const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - state.start).count();
	const double remaining = elapsed * (mIterations - completed) / completed;
	const auto message = fmt::format("Recursive VCM iteration {}/{}: radius {:.6g}, {} photons; elapsed {}, remaining ~{}",
		completed, mIterations, state.radius, photons, FormatTime(elapsed), FormatTime(remaining));
	std::fprintf(stderr, "\n%s\n", message.c_str());
	std::fflush(stderr);
}

GYT_NAMESPACE_END
