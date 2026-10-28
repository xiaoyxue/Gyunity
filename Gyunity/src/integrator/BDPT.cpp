#include "BDPT.h"
#include "TiledIntegrator.h"
#include "system/ProgressReporter.h"
#include "light/PointLight.h"
#include "light/EnvLight.h"
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

}

namespace Bidirectional {

bool ValidateScene(const Scene& scene) {
	int environments = 0;
	for (size_t i = 0; i < scene.GetLights().size(); ++i) {
		const auto& light = scene.GetLights()[i];
		if (!light || (!light->IsAreaLight() && !light->IsEnvironmentLight() && !dynamic_cast<const PointLight*>(light.get()))) {
			return Fail("BDPT supports area lights, point lights and an environment light.");
		}
		if (light->IsEnvironmentLight()) {
			++environments;
			const auto* environment = dynamic_cast<const EnvironmentLight*>(light.get());
			if (environment && environment->GetRotation() != 0) {
				return Fail("BDPT currently supports environment maps with zero rotation only.");
			}
		}
		for (size_t j = 0; j < i; ++j) {
			if (scene.GetLights()[j] == light) return Fail("BDPT requires each light to be registered once.");
		}
	}
	if (environments > 1) return Fail("BDPT currently supports one environment light per scene.");
	if (environments == 1) {
		Vec3 center;
		real radius;
		scene.GetBoundingSphere(&center, &radius);
		if (!std::isfinite(center.x) || !std::isfinite(center.y) || !std::isfinite(center.z) ||
			!std::isfinite(radius) || radius <= 0) {
			return Fail("Environment BDPT requires finite nonempty scene bounds.");
		}
	}
	for (const auto& primitive : scene.GetPrimitives()) {
		if (!primitive || !primitive->GetMaterial() || primitive->mMediumInterface.inside || primitive->mMediumInterface.outside) {
			return Fail("Surface BDPT requires materials without participating media.");
		}
	}
	return true;
}

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
	ProgressReporter progress(fmt::format("Rendering BDPT ({} x {}, {} spp, depth {}{})",
		width, height, mSpp, mMaxDepth, mUseShadingNormals ? ", shading normals" : ""), tiles.size());
	const SurfaceNormalMode normalMode = mUseShadingNormals ? SurfaceNormalMode::Shading : SurfaceNormalMode::Geometric;
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
					const int nCamera = GenerateCameraPath(scene, camera, rand, arena, ray, mMaxDepth + 2, normalMode, cameraPath);
					const int nLight = GenerateLightPath(scene, rand, arena, mMaxDepth + 1, normalMode, lightPath);
					if (nCamera < 0 || nLight < 0) { success = false; break; }
					Vec3 pixelContribution(0);
					for (int t = 1; t <= nCamera && success; ++t) {
						for (int s = 0; s <= nLight && success; ++s) {
							// s/t count vertices, including their light/camera endpoints.
							const int64 depth = static_cast<int64>(s) + t - 2;
							if (depth < 0 || depth > mMaxDepth || (s == 1 && t == 1)) continue;
							Vec3 contribution;
							Vec2 raster;
							if (!ConnectPaths(scene, camera, lightPath, cameraPath, s, t, &contribution, &raster)) {
								success = false;
								break;
							}
							// The one output check: internal stages trust their components, so a
							// faulty BSDF, light or camera surfaces here instead of in the film.
							if (!Finite(contribution)) {
								success = Fail("BDPT produced a nonfinite path contribution.");
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
