#pragma once
#include "SceneRenderer.h"
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

bool CornellBoxStandard(const Options& options = {}, SurfaceIntegrator integrator = SurfaceIntegrator::Path);
bool VeachMis(const Options& options = {}, SurfaceIntegrator integrator = SurfaceIntegrator::Path);
bool VeachBidir(const Options& options = {}, SurfaceIntegrator integrator = SurfaceIntegrator::Bdpt);
bool Matpreview(const std::string& profile, const Options& options = {},
	SurfaceIntegrator integrator = SurfaceIntegrator::Path);
bool FeatureScenes(const Options& options = {});
bool All(const Options& options = {});
bool Named(const std::string& name, const Options& options = {});

std::vector<std::string> Names();

}
