#include "VCM.h"
#include "BDPT.h"
#include "VCMPhotonMap.h"
#include "camera/PinholeCamera.h"
#include "light/EnvLight.h"
#include "system/ProgressReporter.h"
#include "sampler/Halton.h"
#include "sampler/Sobol.h"
#include "tbb/task_arena.h"
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <mutex>
#include <typeinfo>
#include <unordered_set>

GYT_NAMESPACE_BEGIN

namespace {

using Tasks = ThreadedTaskManager;
using Vcm::Contribution;
using Vcm::PathContribution;
using Vcm::Detail::CheckedSequence;

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool Unit(const Vec3& value) {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z) &&
		std::abs(value.Length2() - 1) <= 1e-4f;
}

bool MergingRadius(real initial, real alpha, int iteration, int64 nVm, real* radius, double* area) {
	*radius = real(double(initial) * std::pow(double(iteration) + 1, -(1 - double(alpha)) / 2));
	*area = std::acos(-1.0) * double(*radius) * double(*radius);
	return (std::isfinite(*radius) && *radius > 0 && std::isnormal(*area) &&
		std::isfinite(double(nVm) * *area) && double(nVm) * *area > 0) ||
		Fail("VCM iteration radius, disk area and attempted-count normalization must be representable.");
}

bool Store(const std::array<double, 3>& value, Vec3* result) {
	for (double channel : value)
		if (!std::isfinite(channel) || std::abs(channel) > (std::numeric_limits<real>::max)())
			return Fail("VCM contribution is nonfinite or outside the real range.");
	*result = Vec3(real(value[0]), real(value[1]), real(value[2]));
	return true;
}

std::string FormatTime(double seconds) {
	const auto total = static_cast<uint64>(seconds);
	return fmt::format("{}:{:02}:{:02}", total / 3600, (total / 60) % 60, total % 60);
}

bool IndexedSampler(const Sampler& sampler) {
	// Subclasses may override Sample or Clone with stateful behavior.
	return typeid(sampler) == typeid(HaltonSampler) || typeid(sampler) == typeid(SobolSampler) ||
		typeid(sampler) == typeid(RegularHaltonSampler);
}

struct ArenaDeleter {
	void operator()(MemoryPool* arena) const {
		if (arena) { arena->~MemoryPool(); FreeAligned(arena); }
	}
};
using Arena = std::unique_ptr<MemoryPool, ArenaDeleter>;

Arena CreateArena() {
	void* memory = AllocAligned(sizeof(MemoryPool));
	return Arena(memory ? new (memory) MemoryPool() : nullptr);
}

struct SamplerDomain {
	unsigned dimensions = 1229;
	uint64 maximumIndex = uint64((std::numeric_limits<int64>::max)()) - 1;
	bool roundedUnitEndpoint = false;
};

SamplerDomain Domain(const Sampler& sampler) {
	SamplerDomain domain;
	if (dynamic_cast<const HaltonSampler*>(&sampler)) domain.dimensions = HaltonSampler::get_num_dimensions();
	else if (dynamic_cast<const SobolSampler*>(&sampler)) {
		domain.dimensions = sobol::Matrices::num_dimensions;
		domain.maximumIndex = (uint64(1) << sobol::Matrices::size) - 1;
		domain.roundedUnitEndpoint = true;
	}
	else if (dynamic_cast<const RegularHaltonSampler*>(&sampler)) domain.roundedUnitEndpoint = true;
	return domain;
}

}

struct VertexConnectionMerging::Worker {
	Arena lightArena, cameraArena;
	std::array<std::shared_ptr<Sampler>, 2> indexedSamplers;
	std::vector<PathVertex> cameraPath;
	PathContribution connection, merging;
	const Tasks::Context* context = nullptr;
	int64 lightAttempts = 0, cameraSamples = 0;
};

struct VertexConnectionMerging::RenderState {
	struct Sums {
		std::array<double, 3> connection{}, merging{}, splat{};
	};
	const Scene& scene;
	const Camera& camera;
	const std::shared_ptr<Film> film;
	const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
	int width = 0, height = 0, iteration = 0, concurrency = 1;
	int64 nVc = 1, nVm = 0;
	size_t workPerIteration = 0;
	real radius = 0;
	double area = 0;
	bool indexedSampling = false, trustedClones = false;
	ProgressReporter* progress = nullptr;
	std::mutex samplerMutex;
	std::unordered_set<const Sampler*> activeSamplers;
	std::vector<std::unique_ptr<Worker>> workers;
	std::vector<Sums> sums;
	std::vector<std::vector<PathVertex>> lightPaths;
	Vcm::PhotonMap photonMap;

	RenderState(const Scene& scene, const Camera& camera)
		: scene(scene), camera(camera), film(camera.GetFilm()) {}

	Worker* CurrentWorker() const {
		const int index = tbb::this_task_arena::current_thread_index();
		if (index < 0 || size_t(index) >= workers.size()) {
			Fail("VCM task has no valid arena worker slot.");
			return nullptr;
		}
		return workers[size_t(index)].get();
	}

	void ClearPaths() {
		photonMap.Clear();
		for (auto& path : lightPaths) path.clear();
		for (auto& worker : workers) {
			worker->cameraPath.clear();
			worker->connection.mContributions.clear();
			worker->merging.mContributions.clear();
			worker->cameraArena->Reset();
			worker->lightArena->Reset();
			worker->context = nullptr;
		}
	}

	void CollectStatistics(Statistics& statistics) const {
		size_t arenaBytes = 0, pathBytes = lightPaths.capacity() * sizeof(std::vector<PathVertex>);
		statistics.mLightAttempts = statistics.mCameraSamples = 0;
		statistics.mCameraConnectionSum = statistics.mCameraMergingSum = statistics.mSplatConnectionSum = {};
		for (const auto& worker : workers) {
			arenaBytes += worker->lightArena->TotalAllocated();
			statistics.mLightAttempts += worker->lightAttempts;
			statistics.mCameraSamples += worker->cameraSamples;
		}
		// Logical-pixel reduction keeps diagnostics independent of worker assignment.
		for (const auto& sum : sums) for (int channel = 0; channel < 3; ++channel) {
			statistics.mCameraConnectionSum[channel] += sum.connection[channel];
			statistics.mCameraMergingSum[channel] += sum.merging[channel];
			statistics.mSplatConnectionSum[channel] += sum.splat[channel];
		}
		for (const auto& path : lightPaths) pathBytes += path.capacity() * sizeof(PathVertex);
		statistics.mPeakLightArenaBytes = std::max(statistics.mPeakLightArenaBytes, arenaBytes);
		statistics.mPeakLightPathBytes = std::max(statistics.mPeakLightPathBytes, pathBytes);
	}
};

struct VertexConnectionMerging::PixelTask {
	int64 index;
	Vec2 raster;
	Worker& worker;
	std::vector<PathVertex>& cameraPath;
	PathContribution& connection;
	PathContribution& merging;
	RenderState::Sums& sums;

	PixelTask(int64 index, Worker& worker, RenderState::Sums& sums)
		: index(index), worker(worker), cameraPath(worker.cameraPath),
		connection(worker.connection), merging(worker.merging), sums(sums) {}
};

struct VertexConnectionMerging::SamplerLease {
	std::shared_ptr<Sampler> sampler;
	RenderState* registered = nullptr;
	~SamplerLease() {
		if (registered) {
			std::lock_guard<std::mutex> lock(registered->samplerMutex);
			registered->activeSamplers.erase(sampler.get());
		}
	}
	SamplerLease() = default;
	SamplerLease(const SamplerLease&) = delete;
	SamplerLease& operator=(const SamplerLease&) = delete;
};

bool VertexConnectionMerging::RenderIteration(RenderState& state) {
	if (!MergingRadius(mInitialRadius, mAlpha, state.iteration, state.nVm, &state.radius, &state.area)) return false;
	mStatistics.mLastRadius = state.radius;
	mStatistics.mLastArea = state.area;
	if (!TraceLightSubpaths(state)) return false;
	if (UseVm() && !BuildHashGrid(state)) return false;

	return ForEachPixel(state, Phase::Camera, [&](PixelTask& pixel) {
		if (!TraceEyeSubpath(state, pixel)) return false;
		const auto& cameraPath = pixel.cameraPath;
		const auto& lightPath = state.lightPaths[size_t(pixel.index)];
		auto& weightedContributionVc = pixel.connection;
		auto& weightedContributionVm = pixel.merging;
		if (!VertexConnection(state, lightPath, cameraPath, state.radius, &weightedContributionVc)) return false;
		if (UseVm() && !VertexMerging(state, cameraPath, state.radius, &weightedContributionVm)) return false;
		return Combine(state, pixel, weightedContributionVc, weightedContributionVm);
	});
}

bool VertexConnectionMerging::VertexConnection(const RenderState& state,
	const std::vector<PathVertex>& lightPath, const std::vector<PathVertex>& cameraPath,
	real radius, PathContribution* weightedContributionVc) const {
	auto& records = weightedContributionVc->mContributions;
	records.clear();
	records.reserve(lightPath.size() + 1);
	records.emplace_back();
	auto& cameraContribution = records.front().mValue;
	const int nLight = int(lightPath.size()), nCamera = int(cameraPath.size());
	for (int t = 1; t <= nCamera; ++t) {
		if (Cancelled(state)) return false;
		for (int s = 0; s <= nLight; ++s) {
			const int depth = s + t - 2;
			if (depth < 0 || depth > mMaxDepth ||
				!Bidirectional::ValidConnection(s ? &lightPath[s - 1] : nullptr, cameraPath[t - 1], s, t)) continue;
			const Vcm::Strategy strategy{ Vcm::Technique::Connection, s, t };
			const Vcm::PathContext path(lightPath, cameraPath, strategy, radius, state.nVm, mMode);
			if (!Vcm::IsStrategyEnabled(path, strategy)) continue;
			Vec3 contributionVc;
			Vec2 projectedRaster;
			if (!EvaluateConnection(state.scene, state.camera, lightPath, cameraPath,
				s, t, &contributionVc, &projectedRaster)) return false;
			if (contributionVc == Vec3(0)) continue;
			real weightVc;
			if (!Vcm::ComputeMisWeight(state.scene, path, strategy, &weightVc)) return false;
			if (t == 1) {
				if (!std::isfinite(projectedRaster.x) || !std::isfinite(projectedRaster.y) ||
					projectedRaster.x < 0 || projectedRaster.y < 0 ||
					projectedRaster.x >= state.width || projectedRaster.y >= state.height)
					return Fail("VCM camera splat lies outside the film.");
				const Vec3 value = contributionVc * (weightVc / real(mIterations));
				Contribution splat;
				splat.mRaster = projectedRaster;
				splat.mIsSplat = true;
				for (int channel = 0; channel < 3; ++channel) splat.mValue[channel] = value[channel];
				records.push_back(splat);
			}
			else for (int channel = 0; channel < 3; ++channel)
				cameraContribution[channel] += double(contributionVc[channel]) * weightVc / state.nVc;
		}
	}
	return true;
}

bool VertexConnectionMerging::VertexMerging(const RenderState& state,
	const std::vector<PathVertex>& cameraPath, real radius, PathContribution* weightedContributionVm) const {
	auto& records = weightedContributionVm->mContributions;
	records.clear();
	records.emplace_back();
	auto& cameraContribution = records.front().mValue;
	std::vector<Vcm::PhotonRecord> photons;
	for (int t = 2; t <= int(cameraPath.size()) && t - 1 <= mMaxDepth; ++t) {
		if (Cancelled(state)) return false;
		const auto& cameraVertex = cameraPath[t - 1];
		if (cameraVertex.mType != PathVertexType::Surface || cameraVertex.mIsect.mBsdf->IsDelta()) continue;
		if (!state.photonMap.Query(cameraVertex.mIsect.mPos, cameraVertex.mIsect.mGeometryNormal, photons)) return false;
		for (const auto& photon : photons) {
			if (Cancelled(state)) return false;
			const int s = int(photon.mVertexIndex) + 1;
			if (s + t - 3 > mMaxDepth) continue;
			const auto& lightPath = state.lightPaths[photon.mLightPathIndex];
			const Vcm::Strategy strategy{ Vcm::Technique::Merging, s, t };
			const Vcm::PathContext path(lightPath, cameraPath, strategy, radius, state.nVm, mMode);
			if (!Vcm::IsStrategyEnabled(path, strategy)) continue;
			std::array<double, 3> contributionVm{};
			if (!Vcm::MergeEstimate(path, &contributionVm)) return false;
			real weightVm;
			if (!Vcm::ComputeMisWeight(state.scene, path, strategy, &weightVm)) return false;
			for (int channel = 0; channel < 3; ++channel)
				cameraContribution[channel] += weightVm * contributionVm[channel] / state.nVm;
		}
	}
	return true;
}

bool VertexConnectionMerging::Combine(RenderState& state, PixelTask& pixel,
	const PathContribution& weightedContributionVc, const PathContribution& weightedContributionVm) {
	std::array<double, 3> pixelContribution{};
	for (const auto* pathContribution : { &weightedContributionVc, &weightedContributionVm }) {
		for (const auto& contribution : pathContribution->mContributions) {
			if (contribution.mIsSplat) {
				Vec3 splat;
				if (!Store(contribution.mValue, &splat)) return false;
				state.film->AddSplat(contribution.mRaster.x, contribution.mRaster.y, splat);
				for (int channel = 0; channel < 3; ++channel) pixel.sums.splat[channel] += splat[channel];
			}
			else for (int channel = 0; channel < 3; ++channel) {
				pixelContribution[channel] += contribution.mValue[channel];
				auto& sum = pathContribution == &weightedContributionVc ? pixel.sums.connection : pixel.sums.merging;
				sum[channel] += contribution.mValue[channel];
			}
		}
	}
	Vec3 value;
	if (!Store(pixelContribution, &value)) return false;
	state.film->AddSample(pixel.raster.x, pixel.raster.y, value);
	++pixel.worker.cameraSamples;
	return true;
}

void VertexConnectionMerging::Render(const Scene& scene, const Camera& camera) {
	RenderChecked(scene, camera);
}

bool VertexConnectionMerging::RenderChecked(const Scene& scene, const Camera& camera) {
	mStatistics = {};
	mCancelled.store(false);
	RenderState state(scene, camera);
	if (!Prepare(state)) return false;
	ProgressReporter progress(fmt::format("Rendering VCM ({} x {}, {} iterations, depth {}, concurrency {})",
		state.width, state.height, mIterations, mMaxDepth, mConcurrency), state.workPerIteration * size_t(mIterations));
	state.progress = &progress;
	bool success = true;
	for (state.iteration = 0; state.iteration < mIterations && success; ++state.iteration) {
		success = RenderIteration(state);
		if (success) ++mStatistics.mIterations;
		const size_t photons = state.photonMap.RecordCount();
		state.ClearPaths();
		if (success) ReportIteration(state, photons);
	}
	mStatistics.mSucceeded = progress.Finish(success);
	return mStatistics.mSucceeded;
}

void VertexConnectionMerging::ReportIteration(const RenderState& state, size_t photons) const {
	const int completed = state.iteration + 1;
	const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - state.start).count();
	const double remaining = elapsed * (mIterations - completed) / completed;
	const auto message = fmt::format(
		"VCM iteration {}/{}: radius {:.6g}, {} photons, light {:.1f}s, map {:.1f}s, camera {:.1f}s; "
		"elapsed {}, remaining ~{}",
		completed, mIterations, state.radius, photons, mStatistics.mLastLightPhase.mElapsed.count(),
		mStatistics.mLastMapPhase.mElapsed.count(), mStatistics.mLastCameraPhase.mElapsed.count(),
		FormatTime(elapsed), FormatTime(remaining));
	std::fprintf(stderr, "\n%s\n", message.c_str());
	std::fflush(stderr);
}

bool VertexConnectionMerging::ForEachPixel(RenderState& state, Phase phase,
	const std::function<bool(PixelTask&)>& task) {
	const auto name = fmt::format("VCM {} phase {}/{}", phase == Phase::Light ? "light" : "camera",
		state.iteration + 1, mIterations);
	state.progress->SetPhase(name);
	const auto result = Tasks::Run(name, 0, size_t(state.nVm),
		[&](size_t index, const Tasks::Context& context) {
			auto* worker = state.CurrentWorker();
			if (!worker) return Tasks::Status::Failed;
			worker->context = &context;
			worker->connection.mContributions.clear();
			worker->merging.mContributions.clear();
			PixelTask pixel(int64(index), *worker, state.sums[index]);
			const bool success = !mCancelled.load() && !context.IsCancellationRequested() && task(pixel);
			worker->cameraPath.clear();
			worker->cameraArena->Reset();
			const bool cancelled = mCancelled.load() || context.IsCancellationRequested();
			worker->context = nullptr;
			return cancelled ? Tasks::Status::Cancelled : success ? Tasks::Status::Success : Tasks::Status::Failed;
		}, state.concurrency, state.progress);
	if (phase == Phase::Light) {
		mStatistics.mLastLightPhase = result;
		++mStatistics.mLightPhases;
		mStatistics.mLightSeconds += result.mElapsed.count();
	}
	else {
		mStatistics.mLastCameraPhase = result;
		++mStatistics.mCameraPhases;
		mStatistics.mCameraSeconds += result.mElapsed.count();
	}
	state.CollectStatistics(mStatistics);
	if (result.mStatus != Tasks::Status::Success || mCancelled.load()) return false;
	return phase != Phase::Camera || state.film->HasFiniteSamples() ||
		Fail("VCM film accumulation is nonfinite or has missing sample weights.");
}

bool VertexConnectionMerging::TraceLightSubpaths(RenderState& state) {
	return ForEachPixel(state, Phase::Light, [&](PixelTask& pixel) {
		const int64 sampleIndex = 2 * (int64(state.iteration) * state.nVm + pixel.index);
		SamplerLease stream;
		if (!OpenSampleStream(state, pixel.worker, Phase::Light, sampleIndex, &stream)) return false;
		CheckedSequence random(*stream.sampler, sampleIndex);
		auto& lightPath = state.lightPaths[size_t(pixel.index)];
		const int nLight = GenerateLightPath(state.scene, random, *pixel.worker.lightArena,
			mMaxDepth + 1, SurfaceNormalMode::Geometric, lightPath);
		++pixel.worker.lightAttempts;
		return nLight >= 0 && random.Valid() && Vcm::ValidateSubpath(state.scene, lightPath, false);
	});
}

bool VertexConnectionMerging::BuildHashGrid(RenderState& state) {
	const auto name = fmt::format("VCM map phase {}/{}", state.iteration + 1, mIterations);
	state.progress->SetPhase(name);
	bool built = false;
	mStatistics.mLastMapPhase = Tasks::Run(name, 0, 1, [&](size_t, const Tasks::Context& context) {
		if (mCancelled.load() || context.IsCancellationRequested()) return Tasks::Status::Cancelled;
		if (!state.photonMap.Build(state.lightPaths, state.radius)) return Tasks::Status::Failed;
		built = true;
		return mCancelled.load() || context.IsCancellationRequested() ? Tasks::Status::Cancelled : Tasks::Status::Success;
	}, state.concurrency, state.progress);
	++mStatistics.mMapPhases;
	if (built) ++mStatistics.mMapBuilds;
	mStatistics.mMapSeconds += mStatistics.mLastMapPhase.mElapsed.count();
	mStatistics.mPeakPhotons = std::max(mStatistics.mPeakPhotons, state.photonMap.RecordCount());
	return mStatistics.mLastMapPhase.mStatus == Tasks::Status::Success && !mCancelled.load();
}

bool VertexConnectionMerging::TraceEyeSubpath(RenderState& state, PixelTask& pixel) {
	const int64 sampleIndex = 2 * (int64(state.iteration) * state.nVm + pixel.index) + 1;
	SamplerLease stream;
	if (!OpenSampleStream(state, pixel.worker, Phase::Camera, sampleIndex, &stream)) return false;
	CheckedSequence random(*stream.sampler, sampleIndex);
	const real u = random(), v = random();
	if (!random.Valid()) return false;
	const int x = int(pixel.index % state.width), y = int(pixel.index / state.width);
	const double exactX = double(x) + u, exactY = double(y) + v;
	const real sampleX = x + u, sampleY = y + v;
	if (exactX < x || exactX >= double(x) + 1 || exactY < y || exactY >= double(y) + 1 ||
		!std::isfinite(sampleX) || !std::isfinite(sampleY) ||
		double(sampleX) < 0 || double(sampleX) > state.width || double(sampleY) < 0 || double(sampleY) > state.height)
		return Fail("VCM jittered film coordinates lie outside the supported raster.");
	pixel.raster = Vec2(sampleX, sampleY);
	// Preserve the camera/Film rounding at closed raster edges; do not clamp valid jitter.
	const Ray ray = state.camera.GenerateRay(x, y, Vec2(u, v), 0);
	const int nCamera = GenerateCameraPath(state.scene, state.camera, random, *pixel.worker.cameraArena,
		ray, mMaxDepth + 2, SurfaceNormalMode::Geometric, pixel.cameraPath);
	return nCamera >= 0 && random.Valid() && Vcm::ValidateSubpath(state.scene, pixel.cameraPath, true);
}

bool VertexConnectionMerging::OpenSampleStream(RenderState& state, Worker& worker,
	Phase phase, int64 sampleIndex, SamplerLease* stream) const {
	std::unique_lock<std::mutex> lock(state.samplerMutex, std::defer_lock);
	if (!state.trustedClones) lock.lock();
	stream->sampler = state.indexedSampling ? worker.indexedSamplers[size_t(phase)] :
		mSampler->CloneForPath(uint64(sampleIndex));
	if (!stream->sampler || stream->sampler.get() == mSampler.get())
		return Fail("VCM requires independent non-null path sampler clones.");
	if (!state.trustedClones) {
		if (!Vcm::Detail::ValidateSamplingBudget(*stream->sampler, mMaxDepth, state.nVm, mIterations)) return false;
		if (!state.activeSamplers.insert(stream->sampler.get()).second)
			return Fail("VCM path sampler aliases another active stream.");
		stream->registered = &state;
	}
	return true;
}

bool VertexConnectionMerging::Cancelled(const RenderState& state) const {
	const auto* worker = state.CurrentWorker();
	return mCancelled.load() || !worker || (worker->context && worker->context->IsCancellationRequested());
}

bool VertexConnectionMerging::Prepare(RenderState& state) const {
	const auto& scene = state.scene;
	const auto& camera = state.camera;
	const auto& film = state.film;
	if (mUseShadingNormals)
		return Fail("VCM shading-normal transport is not implemented; useShadingNormals must be false.");
	if (!mSampler || mMaxDepth < 0 || mMaxDepth > 409 || mIterations <= 0 ||
		!std::isfinite(mInitialRadius) || mInitialRadius <= 0 ||
		!std::isfinite(mAlpha) || mAlpha <= 0 || mAlpha >= 1 ||
		(mMode != Vcm::Mode::Combined && mMode != Vcm::Mode::ConnectionOnly && mMode != Vcm::Mode::MergingOnly) ||
		(mConcurrency != -1 && mConcurrency <= 0))
		return Fail("VCM requires a sampler, supported depth in [0,409], positive iterations/radius, alpha in (0,1), valid mode/concurrency.");
	if (!dynamic_cast<const PinHoleCamera*>(&camera) || !film || !film->HasValidStorage() ||
		!std::isfinite(film->Area()) || film->Area() <= 0 ||
		!std::isfinite(film->mArea) || film->mArea <= 0 ||
		film->mArea != film->Area() || film->mAspectRatio != real(film->mResX) / real(film->mResY) ||
		!std::isfinite(camera.mFilmDistance) || camera.mFilmDistance <= 0 ||
		!std::isfinite(camera.mFovy) || camera.mFovy <= 0 || camera.mFovy >= 180 ||
		!Unit(camera.mCx) || !Unit(camera.mCy) || !Unit(camera.mCz) ||
		std::abs(Dot(camera.mCx, camera.mCy)) > 1e-4f ||
		std::abs(Dot(camera.mCx, camera.mCz)) > 1e-4f ||
		std::abs(Dot(camera.mCy, camera.mCz)) > 1e-4f)
		return Fail("VCM requires a valid pinhole camera and matching finite film storage.");
	if (!scene.IsInitialized()) return Fail("VCM requires an initialized scene.");
	if (!Bidirectional::ValidateScene(scene)) return false;
	for (size_t i = 0; i < scene.GetLights().size(); ++i) {
		const auto& light = scene.GetLights()[i];
		if (light->IsEnvironmentLight() && (!dynamic_cast<const EnvironmentLight*>(light.get()) ||
			scene.GetEnvironmentLight() != light.get()))
			return Fail("VCM requires the supported environment endpoint capability.");
		real pmf;
		if (!scene.LightSelectionPdf(i, &pmf) || !std::isfinite(pmf) || pmf < 0)
			return Fail("VCM scene light-selection probability is invalid.");
	}
	state.width = film->mResX;
	state.height = film->mResY;
	state.nVm = int64(state.width) * state.height;
	if (!Vcm::Detail::ValidateSamplingBudget(*mSampler, mMaxDepth, state.nVm, mIterations)) return false;
	if (uint64(state.nVm) > (std::numeric_limits<size_t>::max)() / 2 ||
		uint64(state.nVm) > (std::numeric_limits<size_t>::max)() / sizeof(PathVertex) / (mMaxDepth + 1))
		return Fail("VCM path or progress storage budget overflows.");
	state.workPerIteration = 2 * size_t(state.nVm) + (UseVm() ? 1 : 0);
	if (state.workPerIteration > (std::numeric_limits<size_t>::max)() / mIterations)
		return Fail("VCM progress budget overflows.");
	if (!MergingRadius(mInitialRadius, mAlpha, 0, state.nVm, &state.radius, &state.area) ||
		!MergingRadius(mInitialRadius, mAlpha, mIterations - 1, state.nVm, &state.radius, &state.area)) return false;
	const Ray probe = camera.GenerateRay(state.width / 2, state.height / 2, Vec2(0.5f), 0);
	if (!Unit(probe.mDir) || !std::isfinite(probe.mOrig.x) || !std::isfinite(probe.mOrig.y) ||
		!std::isfinite(probe.mOrig.z) || probe.mMedium ||
		!std::isfinite(camera.PdfPos()) || camera.PdfPos() <= 0 ||
		!std::isfinite(camera.PdfDir(probe)) || camera.PdfDir(probe) <= 0)
		return Fail("VCM camera ray or endpoint probability is invalid.");

	state.indexedSampling = IndexedSampler(*mSampler);
	state.trustedClones = state.indexedSampling || typeid(*mSampler) == typeid(RandomSampler);
	const uint64 last = 2 * uint64(state.nVm) * uint64(mIterations) - 1;
	std::vector<uint64> keys = state.indexedSampling ? std::vector<uint64>{ 0, 1 } :
		std::vector<uint64>{ 0, 1, last - 1, last };
	std::sort(keys.begin(), keys.end());
	keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
	std::unordered_set<const Sampler*> streams{ mSampler.get() };
	std::vector<std::shared_ptr<Sampler>> probes;
	for (uint64 key : keys) {
		auto sampler = state.indexedSampling ? mSampler->Clone(uint32(key)) : mSampler->CloneForPath(key);
		if (!sampler || !streams.insert(sampler.get()).second)
			return Fail("VCM requires distinct non-null light/camera path samplers.");
		if (!Vcm::Detail::ValidateSamplingBudget(*sampler, mMaxDepth, state.nVm, mIterations)) return false;
		probes.push_back(std::move(sampler));
	}
	tbb::task_arena arena(mConcurrency);
	arena.initialize();
	state.concurrency = arena.max_concurrency();
	for (int i = 0; i < state.concurrency; ++i) {
		auto worker = std::make_unique<Worker>();
		worker->lightArena = CreateArena();
		worker->cameraArena = CreateArena();
		if (!worker->lightArena || !worker->cameraArena) return Fail("VCM could not allocate its aligned worker arenas.");
		worker->cameraPath.reserve(mMaxDepth + 2);
		if (state.indexedSampling) for (uint32 phase = 0; phase < 2; ++phase) {
			auto sampler = mSampler->Clone(phase);
			if (!sampler || !streams.insert(sampler.get()).second)
				return Fail("VCM indexed sampler clones must be independent.");
			if (!Vcm::Detail::ValidateSamplingBudget(*sampler, mMaxDepth, state.nVm, mIterations)) return false;
			worker->indexedSamplers[phase] = std::move(sampler);
		}
		state.workers.push_back(std::move(worker));
	}
	state.sums.resize(size_t(state.nVm));
	state.lightPaths.resize(size_t(state.nVm));
	for (auto& path : state.lightPaths) path.reserve(mMaxDepth + 1);
	return true;
}

namespace Vcm {
namespace Detail {

bool ValidateSamplingBudget(const Sampler& sampler, int maxDepth, int64 pixels, int iterations) {
	if (maxDepth < 0 || maxDepth > 409 || pixels <= 0 || iterations <= 0)
		return Fail("VCM requires valid depth, pixel and iteration sampling budgets.");
	const auto domain = Domain(sampler);
	// Light: five emission draws and three per nonterminal surface; eye: two jitter draws.
	const uint64 dimensions = std::max(uint64(5), uint64(3) * maxDepth + 2);
	if (dimensions > domain.dimensions)
		return Fail("VCM path dimension budget exceeds the sampler's supported dimensions.");
	if (uint64(pixels) > ((domain.maximumIndex + 1) / 2) / uint64(iterations))
		return Fail("VCM phase sample indices exceed the sampler's supported index range.");
	return true;
}

CheckedSequence::CheckedSequence(Sampler& sampler, int64 instance) : mSampler(sampler), mInstance(instance) {
	const auto domain = Domain(sampler);
	mDimensions = domain.dimensions;
	mMaximumIndex = domain.maximumIndex;
	mRoundedUnitEndpoint = domain.roundedUnitEndpoint;
}

real CheckedSequence::Sample() {
	if (!mValid) return 0;
	if (mCursor < 0 || unsigned(mCursor) >= mDimensions || mInstance < 0 || uint64(mInstance) > mMaximumIndex) {
		mValid = Fail("VCM sample dimension or index lies outside the sampler's supported domain.");
		return 0;
	}
	const real value = mSampler.Sample(mCursor++, mInstance);
	if (!std::isfinite(value) || value < 0 || value > 1 || (value == 1 && !mRoundedUnitEndpoint)) {
		mValid = Fail("VCM sampler returned an invalid value outside its endpoint convention.");
		return 0;
	}
	// Preserve the legacy exact-one wrap only for the known Sobol/RegularHalton convention.
	return value == 1 ? 0 : value;
}

}
}

GYT_NAMESPACE_END
