#include "BDPT.h"
#include "TiledIntegrator.h"
#include "system/ProgressReporter.h"
#include <atomic>
#include <cmath>
#include <limits>

GYT_NAMESPACE_BEGIN

namespace {

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool Finite(const Vec3& value) {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool Surface(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface ||
		(vertex.mLight && vertex.mLight->IsAreaLight());
}

bool Counts(const std::vector<PathVertex>& light, const std::vector<PathVertex>& camera, int s, int t) {
	return s >= 0 && t >= 1 && static_cast<size_t>(s) <= light.size() &&
		static_cast<size_t>(t) <= camera.size();
}

struct MisVertex {
	real pdfFwd, pdfPrev;
	bool delta;
};

std::vector<MisVertex> CopyMisState(const std::vector<PathVertex>& path, int count) {
	std::vector<MisVertex> state;
	state.reserve(count);
	for (int i = 0; i < count; ++i) {
		state.push_back({ path[i].mPdfFwd, path[i].mPdfPrev, path[i].mIsDelta });
	}
	if (!state.empty()) {
		state.front().delta = false;
		state.back().delta = false;
	}
	return state;
}

bool ValidateMisState(const std::vector<MisVertex>& path) {
	for (size_t i = 0; i < path.size(); ++i) {
		const MisVertex& vertex = path[i];
		if (!std::isfinite(vertex.pdfFwd) || vertex.pdfFwd < 0 ||
			!std::isfinite(vertex.pdfPrev) || vertex.pdfPrev < 0) {
			return Fail("BDPT MIS encountered an invalid density.");
		}
		if (vertex.pdfFwd == 0 && !(i > 0 && path[i - 1].delta)) {
			return Fail("BDPT current strategy has zero probability.");
		}
	}
	return true;
}

bool AdvanceMisRatio(const std::vector<MisVertex>& path, size_t i, double* ratio) {
	const MisVertex& vertex = path[i];
	// Only delta-generated edge markers are remapped; ordinary zero support stays zero.
	const double pdfFwd = vertex.pdfFwd == 0 && i > 0 && path[i - 1].delta ? 1 : vertex.pdfFwd;
	const double pdfPrev = vertex.pdfPrev == 0 && i + 1 < path.size() && path[i + 1].delta ? 1 : vertex.pdfPrev;
	*ratio *= pdfPrev / pdfFwd;
	return std::isfinite(*ratio) || Fail("BDPT MIS density ratio overflowed.");
}

bool WorldToScreen(const Camera& camera, const Vec3& point, Vec2* raster) {
	if (Dot(camera.mCz, point - camera.mPos) <= 0) return false;
	const Vec3 projected = (camera.mCameraToRaster * camera.mWorldToCamera)(point);
	if (!Finite(projected) || projected.x < 0 || projected.y < 0 ||
		projected.x >= camera.GetFilm()->mResX || projected.y >= camera.GetFilm()->mResY) return false;
	*raster = Vec2(projected.x, projected.y);
	return true;
}

}

bool GeometryTerm(const PathVertex& first, const PathVertex& second, real* value) {
	*value = 0;
	Vec3 dir;
	if (!Bidirectional::Direction(first, second, &dir)) return false;
	*value = first.mType == PathVertexType::Environment || second.mType == PathVertexType::Environment
		? 1 : 1 / (second.mIsect.mPos - first.mIsect.mPos).Length2();
	if (Surface(first)) *value *= std::abs(Dot(first.mIsect.mGeometryNormal, dir));
	if (Surface(second)) *value *= std::abs(Dot(second.mIsect.mGeometryNormal, dir));
	return std::isfinite(*value) || Fail("BDPT geometry factor is not finite.");
}

bool MisWeight(const Scene& scene, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, real* weight) {
	*weight = 0;
	if (!Counts(lightPath, cameraPath, s, t)) return Fail("BDPT MIS received invalid vertex counts.");
	const size_t count = static_cast<size_t>(s) + t;
	for (int i = 0; i < s; ++i) {
		if (lightPath[i].mType == PathVertexType::Environment && i != 0) {
			return Fail("An environment can only be the light subpath origin.");
		}
	}
	for (int i = 0; i < t; ++i) {
		if (cameraPath[i].mType == PathVertexType::Environment && i != t - 1) {
			return Fail("An environment can only be the camera subpath endpoint.");
		}
	}

	const PathVertex* lightEnd = s > 0 ? &lightPath[s - 1] : nullptr;
	const PathVertex* lightPrevious = s > 1 ? &lightPath[s - 2] : nullptr;
	const PathVertex& cameraEnd = cameraPath[t - 1];
	const PathVertex* cameraPrevious = t > 1 ? &cameraPath[t - 2] : nullptr;
	if (!Bidirectional::ValidConnection(lightEnd, cameraEnd, s, t)) return true;
	if (count == 2) { *weight = 1; return true; }

	// Local snapshots replace PBRT's ScopedAssignment: generated paths never need restoration.
	auto lightMis = CopyMisState(lightPath, s);
	auto cameraMis = CopyMisState(cameraPath, t);

	// Recompute the camera-side connection densities.
	if (s == 0) {
		if (!Bidirectional::LightOriginPdf(scene, cameraEnd, *cameraPrevious, &cameraMis[t - 1].pdfPrev) ||
			!Bidirectional::PdfToward(cameraEnd, nullptr, *cameraPrevious, &cameraMis[t - 2].pdfPrev)) return false;
	}
	else {
		if (!Bidirectional::PdfToward(*lightEnd, lightPrevious, cameraEnd, &cameraMis[t - 1].pdfPrev)) return false;
		if (cameraPrevious &&
			!Bidirectional::PdfToward(cameraEnd, lightEnd, *cameraPrevious, &cameraMis[t - 2].pdfPrev)) return false;

		// Recompute the light-side connection densities.
		if (!Bidirectional::PdfToward(cameraEnd, cameraPrevious, *lightEnd, &lightMis[s - 1].pdfPrev)) return false;
		if (lightPrevious &&
			!Bidirectional::PdfToward(*lightEnd, &cameraEnd, *lightPrevious, &lightMis[s - 2].pdfPrev)) return false;
	}
	if (!ValidateMisState(lightMis) || !ValidateMisState(cameraMis)) return false;

	// PBRT-v3 balance heuristic: walk alternative splits in both directions.
	double sumRatios = 0, ratio = 1;
	for (int i = t - 1; i > 0; --i) {
		if (!AdvanceMisRatio(cameraMis, i, &ratio)) return false;
		if (Bidirectional::ValidConnection(&cameraPath[i], cameraPath[i - 1], count - i, i)) sumRatios += ratio;
	}
	ratio = 1;
	for (int i = s - 1; i >= 0; --i) {
		if (!AdvanceMisRatio(lightMis, i, &ratio)) return false;
		if (Bidirectional::ValidConnection(i > 0 ? &lightPath[i - 1] : nullptr, lightPath[i], i, count - i)) sumRatios += ratio;
	}
	if (!std::isfinite(sumRatios)) return Fail("BDPT MIS ratio sum overflowed.");
	*weight = static_cast<real>(1 / (1 + sumRatios));
	return true;
}

bool EvaluateConnection(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster) {
	*contribution = Vec3(0);
	*raster = Vec2(0, 0);
	if (!Counts(lightPath, cameraPath, s, t)) return Fail("BDPT connection received invalid vertex counts.");
	if (static_cast<int64>(s) + t < 2 || (s == 1 && t == 1)) return true;
	const PathVertex& pt = cameraPath[t - 1];
	if (pt.mType == PathVertexType::Environment && s != 0) return true;
	Vec3 unweightedContribution(0);
	if (s == 0) {
		if (!pt.mLight) return true;
		if (pt.mType == PathVertexType::Environment) {
			unweightedContribution = pt.mThroughput * pt.mLight->Emission(pt.mDirection);
		}
		else {
			if (!pt.mLight->IsAreaLight()) return true;
			Vec3 dir;
			if (!Bidirectional::Direction(pt, cameraPath[t - 2], &dir)) return false;
			unweightedContribution = pt.mThroughput * pt.mLight->Emission(pt.mIsect, dir);
		}
	}
	else {
		const PathVertex& qs = lightPath[s - 1];
		if (t == 1) {
			if (s < 2 || !Bidirectional::Connectible(qs) || !WorldToScreen(camera, qs.mIsect.mPos, raster)) return true;
			Vec3 wi;
			real pdf;
			const Vec3 importance = camera.SampleWi(qs.mIsect, &pdf, &wi, Vec3(0.5f));
			if (!std::isfinite(pdf) || pdf < 0) return Fail("BDPT camera connection PDF is invalid.");
			if (pdf == 0) return true;
			unweightedContribution = qs.mThroughput * qs.mIsect.mBsdf->Evaluate(qs.mIsect.mOutDir, wi) *
				importance * std::abs(Dot(qs.mIsect.mNormal, wi)) / pdf;
		}
		else if (s == 1) {
			if (!Bidirectional::Connectible(pt) || !qs.mLight) return true;
			Vec3 toCamera;
			if (!Bidirectional::Direction(qs, pt, &toCamera)) return false;
			real originPdf;
			if (!Bidirectional::LightOriginPdf(scene, qs, pt, &originPdf)) return false;
			if (originPdf <= 0) return true;
			const Vec3 emission = qs.mType == PathVertexType::Environment ?
				qs.mLight->Emission(qs.mDirection) : qs.mLight->IsAreaLight() ?
				qs.mLight->Emission(qs.mIsect, toCamera) : qs.mLight->Emission();
			real geometry;
			if (!GeometryTerm(qs, pt, &geometry)) return false;
			unweightedContribution = emission / originPdf * pt.mThroughput *
				pt.mIsect.mBsdf->Evaluate(pt.mIsect.mOutDir, -toCamera) * geometry;
		}
		else {
			if (!Bidirectional::Connectible(qs) || !Bidirectional::Connectible(pt)) return true;
			Vec3 wi;
			if (!Bidirectional::Direction(qs, pt, &wi)) return false;
			real geometry;
			if (!GeometryTerm(qs, pt, &geometry)) return false;
			unweightedContribution = qs.mThroughput * pt.mThroughput * geometry *
				qs.mIsect.mBsdf->Evaluate(qs.mIsect.mOutDir, wi) *
				pt.mIsect.mBsdf->Evaluate(pt.mIsect.mOutDir, -wi);
		}
		if (unweightedContribution != Vec3(0)) {
			if (qs.mType == PathVertexType::Environment) {
				if (scene.Intersect(pt.mIsect.SpawnRay(qs.mDirection))) return true;
			}
			else if (!VisibilityTester(qs.mIsect, pt.mIsect).Unoccluded(scene)) return true;
		}
	}
	if (!Finite(unweightedContribution)) return Fail("BDPT connection contribution is not finite.");
	if (unweightedContribution == Vec3(0)) return true;
	*contribution = unweightedContribution;
	return true;
}

bool ConnectPaths(const Scene& scene, const Camera& camera, const std::vector<PathVertex>& lightPath,
	const std::vector<PathVertex>& cameraPath, int s, int t, Vec3* contribution, Vec2* raster) {
	*contribution = Vec3(0);
	Vec3 unweightedContribution(0);
	if (!EvaluateConnection(scene, camera, lightPath, cameraPath, s, t, &unweightedContribution, raster)) return false;
	if (unweightedContribution == Vec3(0)) return true;
	real weight;
	if (!MisWeight(scene, lightPath, cameraPath, s, t, &weight)) return false;
	*contribution = unweightedContribution * weight;
	return true;
}

void BidirectionalPathTracing::Render(const Scene& scene, const Camera& camera) {
	RenderChecked(scene, camera);
}

bool BidirectionalPathTracing::RenderChecked(const Scene& scene, const Camera& camera) {
	if (!mSampler || mSpp <= 0 || mMaxDepth < 0 || mMaxDepth > std::numeric_limits<int>::max() - 2) {
		return Fail("BDPT requires a sampler, positive sample count and valid maximum depth.");
	}
	if (!Bidirectional::ValidateScene(scene)) return false;
	const int width = camera.GetFilm()->mResX, height = camera.GetFilm()->mResY;
	if (width <= 0 || height <= 0) return Fail("BDPT requires positive image dimensions.");
	const int tileSize = 32;
	std::vector<Tile> tiles;
	for (int y = 0; y < height;) {
		const int maxY = y + std::min(tileSize, height - y);
		for (int x = 0; x < width;) {
			const int maxX = x + std::min(tileSize, width - x);
			tiles.emplace_back(x, y, maxX, maxY);
			x = maxX;
		}
		y = maxY;
	}
	ProgressReporter progress(fmt::format("Rendering BDPT ({} x {}, {} spp, depth {})",
		width, height, mSpp, mMaxDepth), tiles.size());
	std::atomic<bool> success(true);
	ParallelFor(size_t(0), tiles.size(), [&](size_t tileIndex) {
		if (!success) return;
		const Tile& tile = tiles[tileIndex];
		auto sampler = mSampler->Clone(static_cast<uint32>(tileIndex));
		MemoryPool arena;
		std::vector<PathVertex> lightPath, cameraPath;
		for (int y = tile.minY; y < tile.maxY && success; ++y) {
			for (int x = tile.minX; x < tile.maxX && success; ++x) {
				for (int sample = 0; sample < mSpp && success; ++sample) {
					RandomStateSequence rand(sampler, (int64(y) * width + x) * mSpp + sample);
					const real u = rand(), v = rand();
					const Ray ray = camera.GenerateRay(x, y, Vec2(u, v));
					const int nCamera = GenerateCameraPath(scene, camera, rand, arena, ray, mMaxDepth + 2, cameraPath);
					const int nLight = GenerateLightPath(scene, rand, arena, mMaxDepth + 1, lightPath);
					if (nCamera < 0 || nLight < 0) { success = false; break; }
					Vec3 pixelContribution(0);
					for (int t = 1; t <= nCamera && success; ++t) {
						for (int s = 0; s <= nLight && success; ++s) {
							const int64 depth = static_cast<int64>(s) + t - 2;
							if (depth < 0 || depth > mMaxDepth || (s == 1 && t == 1)) continue;
							Vec3 contribution;
							Vec2 raster;
							if (!ConnectPaths(scene, camera, lightPath, cameraPath, s, t, &contribution, &raster)) {
								success = false;
								break;
							}
							if (t == 1) camera.GetFilm()->AddSplat(raster.x, raster.y, contribution / real(mSpp));
							else pixelContribution += contribution;
						}
					}
					if (success) camera.GetFilm()->AddSample(x + u, y + v, pixelContribution);
					arena.Reset();
				}
			}
		}
		if (success) progress.Advance();
	});
	return progress.Finish(success);
}

GYT_NAMESPACE_END
