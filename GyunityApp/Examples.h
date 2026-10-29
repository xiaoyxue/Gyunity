#pragma once
#include "SceneRenderer.h"
#include "integrator/VCM.h"
#include <optional>

namespace Examples {

enum class SurfaceIntegrator { Path, Bdpt, Sppm };

struct Options {
	std::optional<int> width, height, spp, maxDepth;
	std::optional<int> iterations, photonsPerStage;
	std::optional<SceneAccelerator> accelerator;
	std::string output, outputDirectory;
};

bool OutputPath(const std::string& filename, std::string& output);
bool PathTracing(const Options& options = {});
bool Sppm(const Options& options = {});
bool LightTracing(const Options& options = {});
bool VolumePathTracing(const Options& options = {});
bool BidirectionalPathTracing(const std::string& scene = "mesh", const Options& options = {});
bool VertexConnectionMerging(const std::string& scene = "mesh", const Options& options = {},
	real initialRadius = 0.05f, real alpha = real(2.0 / 3.0),
	Vcm::Mode mode = Vcm::Mode::Combined, int concurrency = ThreadedTaskManager::DefaultConcurrency(),
	bool useShadingNormals = false, Gyunity::VertexConnectionMerging::Statistics* statistics = nullptr);

bool CornellBoxStandard(const Options& options = {}, SurfaceIntegrator integrator = SurfaceIntegrator::Path);
bool VeachMis(const Options& options = {}, SurfaceIntegrator integrator = SurfaceIntegrator::Path);
bool VeachBidir(const Options& options = {}, SurfaceIntegrator integrator = SurfaceIntegrator::Bdpt);
// SmallVCM's veach-bidir VCM comparison: 1024^2, 512 iterations, depth 7, shading normals.
bool VeachBidirVcm(const Options& options = {},
	Gyunity::VertexConnectionMerging::Statistics* statistics = nullptr);
bool Matpreview(const std::string& profile, const Options& options = {},
	SurfaceIntegrator integrator = SurfaceIntegrator::Path);
bool FeatureScenes(const Options& options = {});
bool All(const Options& options = {});
bool Named(const std::string& name, const Options& options = {});

std::vector<std::string> Names();

}
