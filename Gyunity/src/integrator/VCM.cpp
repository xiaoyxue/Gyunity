#include "VCM.h"
#include "BDPT.h"
#include "VCMPhotonMap.h"
#include "camera/PinholeCamera.h"
#include "light/EnvLight.h"
#include "system/ProgressReporter.h"
#include "sampler/Halton.h"
#include "sampler/Sobol.h"
#include <cmath>
#include <limits>
#include <unordered_set>

GYT_NAMESPACE_BEGIN

namespace {

using Tasks = ThreadedTaskManager;

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool Unit(const Vec3& v) {
	return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
		std::abs(v.Length2() - 1) <= 1e-4f;
}

bool Radius(real initial, real alpha, int iteration, int64 count, real* radius, double* area) {
	*radius = real(double(initial) * std::pow(double(iteration) + 1, -(1 - double(alpha)) / 2));
	*area = std::acos(-1.0) * double(*radius) * double(*radius);
	return (std::isfinite(*radius) && *radius > 0 && std::isnormal(*area) &&
		std::isfinite(double(count) * *area) && double(count) * *area > 0) ||
		Fail("VCM iteration radius, disk area and attempted-count normalization must be representable.");
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

namespace Vcm {
namespace Detail {

bool ValidateSamplingBudget(const Sampler& sampler, int maxDepth, int64 pixels, int iterations) {
	if (maxDepth < 0 || maxDepth > 409 || pixels <= 0 || iterations <= 0)
		return Fail("VCM requires valid depth, pixel and iteration sampling budgets.");
	const auto domain = Domain(sampler);
	// Light: 5 emission draws + 3 per nonterminal surface (maxDepth-1).
	// Eye: 2 jitter draws + 3 per nonterminal surface (maxDepth).
	const uint64 dimensions = std::max(uint64(5), uint64(3) * maxDepth + 2);
	if (dimensions > domain.dimensions)
		return Fail("VCM path dimension budget exceeds the sampler's supported dimensions.");
	// The largest phase index is 2*(iterations*pixels)-1. Sobol has 52
	// direction columns, and RegularHalton must also represent index+1.
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
		// Safe indexing value while the current builder unwinds. A failed
		// sequence's path is never consumed by an estimator or the film.
		mValid = Fail("VCM sampler returned an invalid value outside its endpoint convention.");
		return 0;
	}
	// Sobol's uint32->float and RegularHalton's float sum can round to one.
	// Match RandomStateSequence's existing exact-one wrap for these samplers;
	// Halton's scaled [0,1) outputs and unknown adapters remain strictly checked.
	return value == 1 ? 0 : value;
}

}
}

namespace {

using Vcm::Detail::CheckedSequence;

// The project also builds in C++14: ordinary new need not honor alignas(64).
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

struct Tile {
	int minX, minY, maxX, maxY;
	std::shared_ptr<Sampler> lightSampler, cameraSampler;
	Arena lightArena, cameraArena;
	std::vector<PathVertex> cameraPath;
	int64 lightAttempts = 0, cameraSamples = 0;
	std::array<double, 3> cameraConnectionSum{}, cameraMergingSum{}, splatConnectionSum{};
};

bool Store(const double sum[3], Vec3* result) {
	for (int c = 0; c < 3; ++c)
		if (!std::isfinite(sum[c]) || std::abs(sum[c]) > (std::numeric_limits<real>::max)())
			return Fail("VCM camera contribution is nonfinite or outside the real range.");
	*result = Vec3(real(sum[0]), real(sum[1]), real(sum[2]));
	return true;
}

}

void VertexConnectionMerging::Render(const Scene& scene, const Camera& camera) {
	RenderChecked(scene, camera);
}

bool VertexConnectionMerging::RenderChecked(const Scene& scene, const Camera& camera) {
	mStatistics = {};
	mCancelled.store(false);
	if (mUseShadingNormals)
		return Fail("VCM shading-normal transport is not implemented; useShadingNormals must be false.");
	if (!mSampler || mMaxDepth < 0 || mMaxDepth > 409 || mIterations <= 0 ||
		!std::isfinite(mInitialRadius) || mInitialRadius <= 0 ||
		!std::isfinite(mAlpha) || mAlpha <= 0 || mAlpha >= 1 ||
		(mMode != Vcm::Mode::Combined && mMode != Vcm::Mode::ConnectionOnly && mMode != Vcm::Mode::MergingOnly) ||
		(mConcurrency != -1 && mConcurrency <= 0))
		return Fail("VCM requires a sampler, supported depth in [0,409], positive iterations/radius, alpha in (0,1), valid mode/concurrency.");
	const auto film = camera.GetFilm();
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
	const int width = film->mResX, height = film->mResY;
	const int64 count = int64(width) * height;
	const size_t columns = (size_t(width) + 31) / 32, rows = (size_t(height) + 31) / 32;
	const size_t tileCount = columns * rows;
	const size_t workPerIteration = 2 * tileCount + (mMode != Vcm::Mode::ConnectionOnly ? 1 : 0);
	if (!Vcm::Detail::ValidateSamplingBudget(*mSampler, mMaxDepth, count, mIterations)) return false;
	if (tileCount > size_t((std::numeric_limits<uint32>::max)()) / 2 ||
		workPerIteration > (std::numeric_limits<size_t>::max)() / mIterations ||
		size_t(count) > (std::numeric_limits<size_t>::max)() / sizeof(PathVertex) / (mMaxDepth + 1))
		return Fail("VCM sample indices, tile seeds or path storage budget overflow.");
	real radius;
	double area;
	if (!Radius(mInitialRadius, mAlpha, 0, count, &radius, &area) ||
		!Radius(mInitialRadius, mAlpha, mIterations - 1, count, &radius, &area)) return false;
	// Check camera endpoint PDFs before allocating the light batch.
	const Ray probe = camera.GenerateRay(width / 2, height / 2, Vec2(0.5f), 0);
	if (!Unit(probe.mDir) || !std::isfinite(probe.mOrig.x) || !std::isfinite(probe.mOrig.y) ||
		!std::isfinite(probe.mOrig.z) || probe.mMedium ||
		!std::isfinite(camera.PdfPos()) || camera.PdfPos() <= 0 ||
		!std::isfinite(camera.PdfDir(probe)) || camera.PdfDir(probe) <= 0)
		return Fail("VCM camera ray or endpoint probability is invalid.");

	std::vector<Tile> tiles;
	tiles.reserve(tileCount);
	std::unordered_set<const Sampler*> streams{ mSampler.get() };
	for (size_t i = 0; i < tileCount; ++i) {
		Tile tile;
		tile.minX = int(i % columns) * 32;
		tile.minY = int(i / columns) * 32;
		tile.maxX = tile.minX + std::min(32, width - tile.minX);
		tile.maxY = tile.minY + std::min(32, height - tile.minY);
		tile.lightSampler = mSampler->Clone(uint32(2 * i));
		tile.cameraSampler = mSampler->Clone(uint32(2 * i + 1));
		if (!tile.lightSampler || !tile.cameraSampler ||
			!streams.insert(tile.lightSampler.get()).second || !streams.insert(tile.cameraSampler.get()).second)
			return Fail("VCM requires distinct non-null tile/phase sampler clones.");
		if (!Vcm::Detail::ValidateSamplingBudget(*tile.lightSampler, mMaxDepth, count, mIterations) ||
			!Vcm::Detail::ValidateSamplingBudget(*tile.cameraSampler, mMaxDepth, count, mIterations)) return false;
		tile.lightArena = CreateArena();
		tile.cameraArena = CreateArena();
		if (!tile.lightArena || !tile.cameraArena) return Fail("VCM could not allocate its aligned tile arenas.");
		tile.cameraPath.reserve(mMaxDepth + 2);
		tiles.push_back(std::move(tile));
	}
	std::vector<std::vector<PathVertex>> lightPaths(static_cast<size_t>(count));
	for (auto& path : lightPaths) path.reserve(mMaxDepth + 1);
	Vcm::PhotonMap map;
	ProgressReporter progress(fmt::format("Rendering VCM ({} x {}, {} iterations, depth {}, concurrency {})",
		width, height, mIterations, mMaxDepth, mConcurrency), workPerIteration * size_t(mIterations));

	const auto cleanup = [&]() {
		// Both Run calls are synchronous: no borrower can outlive these references.
		map.Clear();
		for (auto& path : lightPaths) path.clear();
		for (auto& tile : tiles) {
			tile.cameraPath.clear();
			tile.cameraArena->Reset();
			tile.lightArena->Reset();
		}
	};
	const auto storage = [&]() {
		size_t bytes = 0, pathBytes = lightPaths.capacity() * sizeof(std::vector<PathVertex>);
		mStatistics.mLightAttempts = mStatistics.mCameraSamples = 0;
		mStatistics.mCameraConnectionSum = {};
		mStatistics.mCameraMergingSum = {};
		mStatistics.mSplatConnectionSum = {};
		for (const auto& tile : tiles) {
			bytes += tile.lightArena->TotalAllocated();
			mStatistics.mLightAttempts += tile.lightAttempts;
			mStatistics.mCameraSamples += tile.cameraSamples;
			for (int c = 0; c < 3; ++c) {
				mStatistics.mCameraConnectionSum[c] += tile.cameraConnectionSum[c];
				mStatistics.mCameraMergingSum[c] += tile.cameraMergingSum[c];
				mStatistics.mSplatConnectionSum[c] += tile.splatConnectionSum[c];
			}
		}
		for (const auto& path : lightPaths) pathBytes += path.capacity() * sizeof(PathVertex);
		mStatistics.mPeakLightArenaBytes = std::max(mStatistics.mPeakLightArenaBytes, bytes);
		mStatistics.mPeakLightPathBytes = std::max(mStatistics.mPeakLightPathBytes, pathBytes);
	};
	bool success = true;
	for (int iteration = 0; iteration < mIterations && success; ++iteration) {
		if (!Radius(mInitialRadius, mAlpha, iteration, count, &radius, &area)) { success = false; break; }
		mStatistics.mLastRadius = radius;
		mStatistics.mLastArea = area;
		const std::string lightName = fmt::format("VCM light phase {}/{}", iteration + 1, mIterations);
		progress.SetPhase(lightName);
		mStatistics.mLastLightPhase = Tasks::Run(lightName, 0, tileCount,
			[&](size_t index, const Tasks::Context& context) {
				auto& tile = tiles[index];
				for (int y = tile.minY; y < tile.maxY; ++y) for (int x = tile.minX; x < tile.maxX; ++x) {
					if (mCancelled.load() || context.IsCancellationRequested()) return Tasks::Status::Cancelled;
					const int64 pixel = int64(y) * width + x;
					CheckedSequence random(*tile.lightSampler, 2 * (int64(iteration) * count + pixel));
					const int n = GenerateLightPath(scene, random, *tile.lightArena, mMaxDepth + 1,
						SurfaceNormalMode::Geometric, lightPaths[size_t(pixel)]);
					++tile.lightAttempts;
					if (n < 0 || !random.Valid() || !Vcm::ValidateSubpath(scene, lightPaths[size_t(pixel)], false))
						return Tasks::Status::Failed;
				}
				return mCancelled.load() || context.IsCancellationRequested() ? Tasks::Status::Cancelled : Tasks::Status::Success;
			}, mConcurrency, &progress);
		++mStatistics.mLightPhases;
		mStatistics.mLightSeconds += mStatistics.mLastLightPhase.mElapsed.count();
		storage();
		if (mStatistics.mLastLightPhase.mStatus != Tasks::Status::Success || mCancelled.load()) { success = false; break; }
		if (mMode != Vcm::Mode::ConnectionOnly) {
			const std::string mapName = fmt::format("VCM map phase {}/{}", iteration + 1, mIterations);
			progress.SetPhase(mapName);
			bool built = false;
			mStatistics.mLastMapPhase = Tasks::Run(mapName, 0, 1,
				[&](size_t, const Tasks::Context& context) {
					if (mCancelled.load() || context.IsCancellationRequested()) return Tasks::Status::Cancelled;
					if (!map.Build(lightPaths, radius)) return Tasks::Status::Failed;
					built = true;
					return mCancelled.load() || context.IsCancellationRequested() ?
						Tasks::Status::Cancelled : Tasks::Status::Success;
				}, mConcurrency, &progress);
			++mStatistics.mMapPhases;
			if (built) ++mStatistics.mMapBuilds;
			mStatistics.mMapSeconds += mStatistics.mLastMapPhase.mElapsed.count();
			mStatistics.mPeakPhotons = std::max(mStatistics.mPeakPhotons, map.RecordCount());
			if (mStatistics.mLastMapPhase.mStatus != Tasks::Status::Success || mCancelled.load()) { success = false; break; }
		}
		const std::string cameraName = fmt::format("VCM camera phase {}/{}", iteration + 1, mIterations);
		progress.SetPhase(cameraName);
		mStatistics.mLastCameraPhase = Tasks::Run(cameraName, 0, tileCount,
			[&](size_t index, const Tasks::Context& context) {
				auto& tile = tiles[index];
				for (int y = tile.minY; y < tile.maxY; ++y) for (int x = tile.minX; x < tile.maxX; ++x) {
					if (mCancelled.load() || context.IsCancellationRequested()) return Tasks::Status::Cancelled;
					const int64 pixel = int64(y) * width + x;
					CheckedSequence random(*tile.cameraSampler, 2 * (int64(iteration) * count + pixel) + 1);
					const real u = random(), v = random();
					if (!random.Valid()) return Tasks::Status::Failed;
					const double exactX = double(x) + u, exactY = double(y) + v;
					const real sampleX = x + u, sampleY = y + v;
					if (exactX < x || exactX >= double(x) + 1 || exactY < y || exactY >= double(y) + 1 ||
						!std::isfinite(sampleX) || !std::isfinite(sampleY) ||
						double(sampleX) < 0 || double(sampleX) > width || double(sampleY) < 0 || double(sampleY) > height) {
						Fail("VCM jittered film coordinates lie outside the supported raster.");
						return Tasks::Status::Failed;
					}
					// The pinhole camera also adds integer pixel + real jitter.
					// Preserve that same rounding and Film's existing closed-edge
					// filter behavior; a valid near-one jitter is not clamped/wrapped.
					const Ray ray = camera.GenerateRay(x, y, Vec2(u, v), 0);
					auto& eye = tile.cameraPath;
					const auto& light = lightPaths[size_t(pixel)];
					const int n = GenerateCameraPath(scene, camera, random, *tile.cameraArena, ray, mMaxDepth + 2,
						SurfaceNormalMode::Geometric, eye);
					if (n < 0 || !random.Valid() || !Vcm::ValidateSubpath(scene, eye, true)) return Tasks::Status::Failed;
					double sum[3] = {};
					for (int t = 1; t <= n; ++t) {
						if (mCancelled.load() || context.IsCancellationRequested()) return Tasks::Status::Cancelled;
						for (int s = 0; s <= int(light.size()); ++s) {
							const int depth = s + t - 2;
							if (depth < 0 || depth > mMaxDepth ||
								!Bidirectional::ValidConnection(s ? &light[s - 1] : nullptr, eye[t - 1], s, t)) continue;
							if (mMode == Vcm::Mode::MergingOnly) {
								if (s != 0) continue;
								bool specular = true;
								for (int j = 1; j + 1 < t; ++j)
									specular &= eye[j].mIsect.mBsdf && eye[j].mIsect.mBsdf->IsDelta();
								if (!specular) continue;
							}
							Vec3 primary;
							Vec2 raster;
							if (!EvaluateConnection(scene, camera, light, eye, s, t, &primary, &raster)) return Tasks::Status::Failed;
							if (primary == Vec3(0)) continue;
							const Vcm::Strategy strategy{ Vcm::Technique::Connection, s, t };
							const Vcm::PathContext path(light, eye, strategy, radius, count, mMode);
							real weight;
							if (!Vcm::ComputeMisWeight(scene, path, strategy, &weight)) return Tasks::Status::Failed;
							if (t == 1) {
								if (!std::isfinite(raster.x) || !std::isfinite(raster.y) ||
									raster.x < 0 || raster.y < 0 || raster.x >= width || raster.y >= height) {
									Fail("VCM camera splat lies outside the film.");
									return Tasks::Status::Failed;
								}
								const Vec3 splat = primary * (weight / real(mIterations));
								film->AddSplat(raster.x, raster.y, splat);
								for (int c = 0; c < 3; ++c) tile.splatConnectionSum[c] += splat[c];
							}
							else for (int c = 0; c < 3; ++c) {
								const double value = double(primary[c]) * weight;
								sum[c] += value;
								tile.cameraConnectionSum[c] += value;
							}
						}
						if (mMode != Vcm::Mode::ConnectionOnly && t >= 2 && t - 1 <= mMaxDepth) {
							Vec3 merged;
							if (!Vcm::GatherMerging(scene, map, lightPaths, eye, t, mMaxDepth, mMode, &merged)) return Tasks::Status::Failed;
							for (int c = 0; c < 3; ++c) {
								sum[c] += merged[c];
								tile.cameraMergingSum[c] += merged[c];
							}
						}
					}
					Vec3 contribution;
					if (!Store(sum, &contribution)) return Tasks::Status::Failed;
					film->AddSample(sampleX, sampleY, contribution);
					++tile.cameraSamples;
					eye.clear();
					tile.cameraArena->Reset();
				}
				return mCancelled.load() || context.IsCancellationRequested() ? Tasks::Status::Cancelled : Tasks::Status::Success;
			}, mConcurrency, &progress);
		++mStatistics.mCameraPhases;
		mStatistics.mCameraSeconds += mStatistics.mLastCameraPhase.mElapsed.count();
		success = mStatistics.mLastCameraPhase.mStatus == Tasks::Status::Success && !mCancelled.load();
		if (success && !film->HasFiniteSamples()) success = Fail("VCM film accumulation is nonfinite or has missing sample weights.");
		storage();
		if (success) ++mStatistics.mIterations;
		cleanup();
	}
	cleanup();
	mStatistics.mSucceeded = progress.Finish(success);
	return mStatistics.mSucceeded;
}

GYT_NAMESPACE_END
