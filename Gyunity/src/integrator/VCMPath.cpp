#include "VCMPath.h"
#include "VCMPhotonMap.h"
#include "camera/PinholeCamera.h"
#include "light/EnvLight.h"
#include "light/PointLight.h"
#include <cmath>
#include <limits>

GYT_NAMESPACE_BEGIN

namespace Vcm {
namespace {

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool Finite(const Vec3& value) {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool Unit(const Vec3& value) {
	return Finite(value) && std::abs(value.Length2() - 1) <= 1e-4f;
}

bool ValidPdf(double pdf) {
	return std::isfinite(pdf) && pdf >= 0;
}

bool ValidTechnique(Technique technique) {
	return technique == Technique::Connection || technique == Technique::Merging;
}

// Joined-path notation (technical report Sec. 3). Vertex i is numbered from the light origin
// (i = 0) to the camera (i = n - 1). A connection joins light[0, s) with camera[t - 1 .. 0], so
// n = s + t. A merge keeps two physical vertices at the shared index s - 1, the photon
// y = light[s - 1] and the query x = camera[t - 1], so n = s + t - 1.
int VertexCount(const PathContext& path) {
	return path.selected.s + path.selected.t - (path.selected.technique == Technique::Merging ? 1 : 0);
}

// Which physical vertex stands at a merge's shared index. Elsewhere both choices agree.
enum class MergeEnd { Photon, Query };

const PathVertex& Vertex(const PathContext& path, int i, MergeEnd end) {
	const int split = path.selected.s;
	if (path.selected.technique == Technique::Connection) {
		return i < split ? path.light[i] : path.camera[VertexCount(path) - 1 - i];
	}
	return i < split - 1 || (i == split - 1 && end == MergeEnd::Photon) ?
		path.light[i] : path.camera[VertexCount(path) - 1 - i];
}

bool Delta(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf &&
		vertex.mIsect.mBsdf->IsDelta();
}

bool Continuous(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf && !Delta(vertex);
}

bool ValidateVertex(const PathVertex& vertex, bool lightOrigin, bool cameraOrigin, bool continued) {
	if (vertex.mNormalMode != SurfaceNormalMode::Geometric)
		return Fail("VCM path evaluation only supports geometric-normal transport.");
	if (!ValidPdf(vertex.mPdfFwd) || !ValidPdf(vertex.mPdfPrev) ||
		!ValidPdf(vertex.mPdfPos) || !ValidPdf(vertex.mPdfDir) ||
		!ValidPdf(vertex.mLightSelectionPdf)) return Fail("VCM path contains an invalid density.");
	if (!Finite(vertex.mIsect.mPos)) return Fail("VCM path contains a nonfinite position.");
	if (cameraOrigin) {
		if (vertex.mType != PathVertexType::Camera ||
			!dynamic_cast<const PinHoleCamera*>(vertex.mCamera) || vertex.mLight) {
			return Fail("VCM requires a pinhole camera origin.");
		}
	}
	else {
		if (lightOrigin) {
			if (!vertex.mLight) return Fail("VCM light origin has no emitter.");
			if (vertex.mLight->IsEnvironmentLight()) {
				const auto* environment = dynamic_cast<const EnvironmentLight*>(vertex.mLight);
				if (!environment || vertex.mType != PathVertexType::Environment ||
					!Unit(vertex.mDirection) || environment->GetRotation() != 0) {
					return Fail("VCM requires a zero-rotation environment with unit map direction.");
				}
			}
			else if (vertex.mLight->IsDeltaLight()) {
				if (vertex.mType != PathVertexType::Light || !dynamic_cast<const PointLight*>(vertex.mLight)) {
					return Fail("VCM discrete origins must be point-light records.");
				}
			}
			else if (!vertex.mLight->IsAreaLight() ||
				(vertex.mType != PathVertexType::Light && vertex.mType != PathVertexType::Surface)) {
				return Fail("VCM light origin has mismatched or unsupported endpoint metadata.");
			}
		}
		else if (vertex.mType != PathVertexType::Surface || !vertex.mIsect.mBsdf) {
			return Fail("VCM requires a BSDF at each interior surface.");
		}
		if ((!lightOrigin || vertex.mLight->IsAreaLight()) &&
			(!Unit(vertex.mIsect.mGeometryNormal) || !Unit(vertex.mIsect.mNormal) ||
			(vertex.mIsect.mNormal - vertex.mIsect.mGeometryNormal).Length2() > 1e-8f)) {
			return Fail("VCM requires unit geometric-normal shading.");
		}
	}
	// Endpoints emit/measure rather than scatter. A terminal continuation is
	// likewise not part of the selected event, even if the builder sampled one.
	if (!lightOrigin && !cameraOrigin) {
		if (vertex.mIsect.mBsdf->HasMixedDeltaLobes()) return Fail("VCM does not support mixed delta/continuous lobes.");
		if (continued && (vertex.mIsDelta != Delta(vertex) ||
			(Delta(vertex) && (!ValidPdf(vertex.mDeltaPdf) || vertex.mDeltaPdf <= 0 || vertex.mDeltaPdf > 1 ||
				!ValidPdf(vertex.mDeltaReversePdf) || vertex.mDeltaReversePdf > 1 || !Unit(vertex.mIsect.mOutDir))))) {
			return Fail("VCM requires the actual forward/reverse masses of each selected delta event.");
		}
	}
	if (vertex.mIsect.mMediumInterface.inside || vertex.mIsect.mMediumInterface.outside) {
		return Fail("VCM does not support participating media.");
	}
	return true;
}

bool Validate(const Scene& scene, const PathContext& path) {
	const Strategy& selected = path.selected;
	if (!ValidTechnique(selected.technique) ||
		(path.mode != Mode::Combined && path.mode != Mode::ConnectionOnly && path.mode != Mode::MergingOnly)) {
		return Fail("VCM received an invalid technique or mode.");
	}
	const int64 count = int64(selected.s) + selected.t;
	if (selected.s < 0 || selected.t < 1 || count > std::numeric_limits<int>::max() ||
		static_cast<size_t>(selected.s) > path.light.size() ||
		static_cast<size_t>(selected.t) > path.camera.size() ||
		(selected.technique == Technique::Connection && (count < 2 || (selected.s == 1 && selected.t == 1))) ||
		(selected.technique == Technique::Merging && (selected.s < 2 || selected.t < 2))) {
		return Fail("VCM received invalid selected prefix counts.");
	}
	const double area = std::acos(-1.0) * path.radius * path.radius;
	if (!std::isfinite(path.radius) || path.radius <= 0 || !std::isfinite(area) || area <= 0 ||
		path.lightPathCount <= 0) return Fail("VCM requires a finite positive disk area and attempted light-path count.");
	for (int i = 0; i < selected.s; ++i) {
		if (!ValidateVertex(path.light[i], i == 0, false, i + 1 < selected.s)) return false;
	}
	for (int i = 0; i < selected.t; ++i) {
		if (!ValidateVertex(path.camera[i], selected.s == 0 && i == selected.t - 1, i == 0,
			i + 1 < selected.t)) return false;
	}
	const PathVertex& origin = Vertex(path, 0, MergeEnd::Photon);
	if (!origin.mLight) return Fail("VCM path has no light origin.");
	real selection = 0;
	bool registered = false;
	int environments = 0;
	for (size_t i = 0; i < scene.GetLights().size(); ++i) {
		if (scene.GetLights()[i]->IsEnvironmentLight()) {
			const auto* environment = dynamic_cast<const EnvironmentLight*>(scene.GetLights()[i].get());
			if (!environment || environment->GetRotation() != 0 || ++environments > 1) {
				return Fail("VCM supports at most one zero-rotation environment.");
			}
		}
		if (scene.GetLights()[i].get() == origin.mLight) {
			registered = true;
			real slot;
			if (!scene.LightSelectionPdf(i, &slot)) return false;
			selection += slot;
		}
	}
	if (!registered || !ValidPdf(selection)) return Fail("VCM origin requires a registered light-selection probability.");
	if (!origin.mLight->IsAreaLight()) {
		Vec3 emitted;
		if (!Bidirectional::Direction(origin, Vertex(path, 1, MergeEnd::Photon), &emitted)) return false;
		real position, direction;
		if (!origin.mLight->PdfLight(origin.mIsect, emitted, &position, &direction)) return false;
		if (!ValidPdf(position) || position <= 0 || !ValidPdf(direction) ||
			(origin.mType == PathVertexType::Environment &&
				(environments != 1 || scene.GetEnvironmentLight() != origin.mLight))) {
			return Fail("VCM singular origin has invalid position, direction or scene metadata.");
		}
		if (selected.s > 0) {
			auto same = [](real a, real b) { return a > 0 && std::abs(double(a) - b) <= 1e-4 * std::abs(double(b)); };
			if (!same(origin.mPdfPos, position) || !same(origin.mPdfDir, direction) ||
				!same(origin.mLightSelectionPdf, selection) ||
				!same(origin.mPdfFwd, selection * (origin.mType == PathVertexType::Environment ? direction : position))) {
				return Fail("VCM sampled singular-origin densities do not match their endpoint metadata.");
			}
		}
	}
	Vec3 direction;
	for (int i = 1; i < selected.s; ++i) {
		if (!Bidirectional::Direction(path.light[i - 1], path.light[i], &direction)) return false;
	}
	for (int i = 1; i < selected.t; ++i) {
		if (!Bidirectional::Direction(path.camera[i - 1], path.camera[i], &direction)) return false;
	}
	if (selected.technique == Technique::Connection) {
		if (selected.s > 0 &&
			!Bidirectional::Direction(path.light[selected.s - 1], path.camera[selected.t - 1], &direction)) return false;
	}
	else {
		const PathVertex& y = path.light[selected.s - 1];
		const PathVertex& x = path.camera[selected.t - 1];
		double distance2 = 0;
		for (int axis = 0; axis < 3; ++axis) {
			const double offset = double(y.mIsect.mPos[axis]) - double(x.mIsect.mPos[axis]);
			distance2 += offset * offset;
		}
		if (!std::isfinite(distance2) || distance2 > path.radius * path.radius ||
			!Unit(y.mIsect.mOutDir) || !Unit(x.mIsect.mOutDir)) {
			return Fail("VCM merge requires in-radius endpoints and original unit incoming directions.");
		}
	}
	return true;
}

// Whether `mode` uses this strategy. MergingOnly keeps a connection only for camera-visible
// emission through zero or more delta surfaces (L S* E), which no merge can estimate.
bool Enabled(const PathContext& path, const Strategy& strategy) {
	if (strategy.technique == Technique::Merging) return path.mode != Mode::ConnectionOnly;
	if (path.mode != Mode::MergingOnly) return true;
	if (strategy.s != 0) return false;
	for (int i = 1; i + 1 < VertexCount(path); ++i) {
		if (!Delta(Vertex(path, i, MergeEnd::Photon))) return false;
	}
	return true; // Camera-visible LS*E emission, including the empty S* prefix.
}

// Whether the strategy can generate this joined path at all: connections and merges need
// continuous endpoints, and an emitter hit (s = 0) cannot reach a delta light.
bool Feasible(const PathContext& path, const Strategy& strategy) {
	const int n = VertexCount(path), s = strategy.s;
	if (strategy.technique == Technique::Merging) {
		return s >= 2 && s < n && Continuous(Vertex(path, s - 1, MergeEnd::Photon)) &&
			Continuous(Vertex(path, s - 1, MergeEnd::Query));
	}
	if (strategy.t == 0 || (s == 1 && strategy.t == 1)) return false;
	if (s == 0) return !Vertex(path, 0, MergeEnd::Photon).mLight->IsDeltaLight();
	return (s == 1 || Continuous(Vertex(path, s - 1, MergeEnd::Photon))) &&
		(s == n - 1 || Continuous(Vertex(path, s, MergeEnd::Query)));
}

// Which subpath generates joined vertex i: the light subpath walks i - 1 -> i and the camera
// subpath walks i + 1 -> i. A strategy multiplies light-generated densities over its light
// prefix and camera-generated densities over its camera prefix (report Eq. 10).
enum class Generator { Light, Camera };

// How the density of vertex i under one generator is obtained, tested in this order.
enum class DensitySource {
	DeltaEvent,    // the generating source vertex scattered through a delta lobe
	CachedReverse, // a reverse density stored while tracing, still valid inside its prefix
	Sampled,       // the density the generating subpath actually sampled
	MergeAdjacent, // the vertex just beyond a merge (report Sec. 4.3)
	Recomputed,    // re-evaluated across the connection on the joined path
};

DensitySource ClassifyDensity(const PathContext& path, int i, Generator by) {
	const int n = VertexCount(path), s = path.selected.s, t = path.selected.t;
	const bool light = by == Generator::Light;
	const int source = light ? i - 1 : i + 1;
	if (source > 0 && source < n - 1 && Delta(Vertex(path, source, MergeEnd::Photon)))
		return DensitySource::DeltaEvent;
	// A reverse event is unchanged only when its source AND sampled successor remain in the
	// selected prefix. Its cached PDF retains the actual offset-ray directions;
	// reconstructing them from hit positions changes the event.
	if (!light && i < s - 2) return DensitySource::CachedReverse;
	if (light && n - 1 - i < t - 2) return DensitySource::CachedReverse;
	// Each subpath sampled its own prefix: the light prefix ends at s - 1 (the photon for a
	// merge); the camera prefix ends at s, or at the query x = s - 1 for a merge.
	const bool merge = path.selected.technique == Technique::Merging;
	if (light && i <= s - 1) return DensitySource::Sampled;
	if (!light && i >= (merge ? s - 1 : s)) return DensitySource::Sampled;
	if (merge && i == (light ? s : s - 2)) return DensitySource::MergeAdjacent;
	return DensitySource::Recomputed;
}

bool VertexDensity(const Scene& scene, const PathContext& path, int i, Generator by, real* pdf) {
	const int n = VertexCount(path), s = path.selected.s, t = path.selected.t;
	const bool light = by == Generator::Light;
	const int source = light ? i - 1 : i + 1;
	switch (ClassifyDensity(path, i, by)) {
	case DensitySource::DeltaEvent: {
		// The event belongs to the subpath that traced the source vertex.
		const bool lightOwned = source < s;
		const int index = lightOwned ? source : n - 1 - source;
		if (index + 1 >= (lightOwned ? s : t)) {
			return Fail("VCM cannot invent a delta event across an unsampled connection.");
		}
		const PathVertex& vertex = lightOwned ? path.light[index] : path.camera[index];
		// The owner's walking direction uses the forward mass; the other side the reverse mass.
		const real mass = light == lightOwned ? vertex.mDeltaPdf : vertex.mDeltaReversePdf;
		// Report (52): remove only the shared singular distribution, not its
		// discrete component mass or geometric conversion. Never remap Pdf()==0.
		return ConvertDensity(mass, vertex, Vertex(path, i, lightOwned ? MergeEnd::Photon : MergeEnd::Query), pdf);
	}
	case DensitySource::CachedReverse:
		*pdf = light ? path.camera[n - 1 - i].mPdfPrev : path.light[i].mPdfPrev;
		return true;
	case DensitySource::Sampled:
		*pdf = light ? path.light[i].mPdfFwd : path.camera[n - 1 - i].mPdfFwd;
		return true;
	case DensitySource::MergeAdjacent: {
		const PathVertex& y = path.light[s - 1];
		const PathVertex& x = path.camera[t - 1];
		// Report 4.3 / Fig. 3: scatter at x with the ORIGINAL eye/photon
		// directions, but convert along the sampled edge on that side.
		// There is no x-y edge, and neither x->a nor y->b is a sampled edge.
		const real pdfDir = light ? x.mIsect.mBsdf->Pdf(y.mIsect.mOutDir, x.mIsect.mOutDir) :
			x.mIsect.mBsdf->Pdf(x.mIsect.mOutDir, y.mIsect.mOutDir);
		return ConvertDensity(pdfDir, light ? x : y, Vertex(path, i, light ? MergeEnd::Query : MergeEnd::Photon), pdf);
	}
	case DensitySource::Recomputed: {
		// A light-side walk past a merge continues on the camera prefix (x); a camera-side
		// walk continues on the light prefix (y).
		const MergeEnd end = light ? MergeEnd::Query : MergeEnd::Photon;
		if (light && i == 0) {
			return Bidirectional::LightOriginPdf(scene, Vertex(path, 0, end), Vertex(path, 1, end), pdf);
		}
		const int previous = light ? i - 2 : i + 2;
		return Bidirectional::PdfToward(Vertex(path, source, end),
			previous >= 0 && previous < n ? &Vertex(path, previous, end) : nullptr,
			Vertex(path, i, end), pdf);
	}
	}
	return Fail("VCM encountered an unknown vertex density source.");
}

// Densities depend only on the selected joined path, never on the strategy being scored, so
// one evaluation computes each (vertex, generator) density at most once, on first use and in
// the same order as an uncached evaluation. Failures are never cached; they end evaluation.
class VertexDensities {
public:
	VertexDensities(const Scene& scene, const PathContext& path)
		: mScene(scene), mPath(path), mLight(VertexCount(path)), mCamera(VertexCount(path)) {}

	bool Get(int i, Generator by, real* pdf) {
		Entry& entry = (by == Generator::Light ? mLight : mCamera)[i];
		if (!entry.known) {
			if (!VertexDensity(mScene, mPath, i, by, &entry.pdf)) return false;
			entry.known = true;
		}
		*pdf = entry.pdf;
		return true;
	}

private:
	struct Entry {
		real pdf = 0;
		bool known = false;
	};
	const Scene& mScene;
	const PathContext& mPath;
	std::vector<Entry> mLight, mCamera;
};

bool HasSelectedDepth(const PathContext& path, const Strategy& strategy) {
	return ValidTechnique(strategy.technique) && strategy.s >= 0 && strategy.t >= 0 &&
		int64(strategy.s) + strategy.t - (strategy.technique == Technique::Merging ? 1 : 0) == VertexCount(path);
}

// Report Eq. (10): one strategy's density multiplies its light prefix, generated by the light
// subpath, with its camera prefix, generated by the camera subpath. A merge's prefixes share
// index s - 1, so the photon landing and the query each contribute a density (one density is
// duplicated, not one position), and the acceptance disk pi r^2 is applied once. The sample
// count (N_L for merges) is left to the MIS weight. The context must already be validated.
bool StrategyLogPdf(VertexDensities& densities, const PathContext& path,
	const Strategy& strategy, StrategyPdf* result) {
	*result = {};
	if (!Enabled(path, strategy) || !Feasible(path, strategy)) return true;
	double logPdf = 0;
	bool positive = true;
	const auto multiply = [&](int i, Generator by) {
		real pdf;
		if (!densities.Get(i, by, &pdf)) return false;
		if (!ValidPdf(pdf)) return Fail("VCM evaluated a nonfinite or negative density.");
		if (pdf == 0) positive = false;
		else logPdf += std::log(double(pdf));
		return true;
	};
	const bool merge = strategy.technique == Technique::Merging;
	for (int i = 0; i < strategy.s; ++i) {
		if (!multiply(i, Generator::Light)) return false;
	}
	for (int i = strategy.s - (merge ? 1 : 0); i < VertexCount(path); ++i) {
		if (!multiply(i, Generator::Camera)) return false;
	}
	if (!positive) return true;
	if (merge) logPdf += std::log(std::acos(-1.0) * path.radius * path.radius);
	if (!std::isfinite(logPdf)) return Fail("VCM path log density is not finite.");
	result->feasible = true;
	result->logPdf = logPdf;
	return true;
}

}

bool IsStrategyEnabled(const PathContext& path, const Strategy& strategy) {
	return Enabled(path, strategy);
}

bool ComputePathPdf(const Scene& scene, const PathContext& path,
	const Strategy& strategy, StrategyPdf* result) {
	if (!result) return Fail("VCM PDF output is null.");
	*result = {};
	if (!Validate(scene, path)) return false;
	if (!HasSelectedDepth(path, strategy)) {
		return Fail("VCM alternative must have valid counts at the selected physical depth.");
	}
	VertexDensities densities(scene, path);
	return StrategyLogPdf(densities, path, strategy, result);
}

bool ComputeMisWeight(const Scene& scene, const PathContext& path,
	const Strategy& current, real* weight) {
	if (!weight) return Fail("VCM weight output is null.");
	*weight = 0;
	// Every strategy below scores the same immutable joined path; validate it once.
	if (!Validate(scene, path)) return false;
	if (!HasSelectedDepth(path, current)) {
		return Fail("VCM alternative must have valid counts at the selected physical depth.");
	}
	VertexDensities densities(scene, path);
	StrategyPdf currentPdf;
	if (!StrategyLogPdf(densities, path, current, &currentPdf)) return false;
	if (!currentPdf.feasible) return Fail("VCM current strategy has zero support or is disabled.");

	// Balance heuristic in the log domain: w_i = c_i p_i / sum_j c_j p_j, with sample count
	// c = 1 for a connection and N_L for a merge. Every strategy of the same physical depth
	// competes: connections s = 0..n-1 (t = n - s) and merges s = 2..n-1 (t = n - s + 1).
	const double logCount = std::log(double(path.lightPathCount));
	const auto score = [&](const Strategy& strategy, const StrategyPdf& pdf) {
		return pdf.logPdf + (strategy.technique == Technique::Merging ? logCount : 0);
	};
	const double currentScore = score(current, currentPdf);
	std::vector<double> scores;
	double maximum = -std::numeric_limits<double>::infinity();
	const int n = VertexCount(path);
	for (Technique technique : { Technique::Connection, Technique::Merging }) {
		for (int s = technique == Technique::Connection ? 0 : 2; s < n; ++s) {
			const Strategy strategy{ technique, s, n - s + (technique == Technique::Merging ? 1 : 0) };
			StrategyPdf pdf;
			if (!StrategyLogPdf(densities, path, strategy, &pdf)) return false;
			if (!pdf.feasible) continue;
			scores.push_back(score(strategy, pdf));
			maximum = std::max(maximum, scores.back());
		}
	}
	double sum = 0;
	for (double value : scores) sum += std::exp(value - maximum);
	const double normalized = std::exp(currentScore - maximum) / sum;
	if (!std::isfinite(normalized) || normalized < 0 || normalized > 1) return Fail("VCM normalized weight is invalid.");
	*weight = static_cast<real>(normalized);
	return true;
}

namespace {

bool ValidMode(Mode mode) {
	return mode == Mode::Combined || mode == Mode::ConnectionOnly || mode == Mode::MergingOnly;
}

bool ValidLocalScene(const Scene& scene) {
	int environments = 0;
	for (const auto& light : scene.GetLights()) {
		if (!light) return Fail("VCM merging requires valid scene lights.");
		if (!light->IsEnvironmentLight()) continue;
		const auto* environment = dynamic_cast<const EnvironmentLight*>(light.get());
		if (!environment || environment->GetRotation() != 0 || ++environments > 1 ||
			scene.GetEnvironmentLight() != light.get())
			return Fail("VCM merging supports at most one zero-rotation environment.");
	}
	return true;
}

bool ValidLocalPrefix(const std::vector<PathVertex>& vertices, int count, bool camera, bool complete = false) {
	if (count < 1 || size_t(count) > vertices.size())
		return Fail("VCM merging received an invalid prefix count.");
	for (int i = 0; i < count; ++i) {
		const auto& vertex = vertices[i];
		const bool endpoint = i == 0 || (camera && i + 1 == count &&
			(vertex.mType == PathVertexType::Environment || vertex.mType == PathVertexType::Light));
		if (!ValidateVertex(vertex, endpoint && !(camera && i == 0), camera && i == 0,
			i + 1 < count || (complete && vertex.mIsDelta)))
			return false;
		if (!Finite(vertex.mThroughput))
			return Fail("VCM merging requires finite selected throughputs.");
		if (!endpoint && (!Unit(vertex.mIsect.mOutDir) || vertex.mCamera ||
			!vertex.mIsect.mBsdf->MatchScatterType(BSDF_ALL) ||
			(vertex.mIsDelta && !Delta(vertex))))
			return Fail("VCM merging requires original unit directions and consistent scattering metadata.");
	}
	Vec3 direction;
	for (int i = 1; i < count; ++i)
		if (!Bidirectional::Direction(vertices[i - 1], vertices[i], &direction)) return false;
	return true;
}

bool Normalization(double radius, int64 count, double* normalization) {
	const double area = std::acos(-1.0) * radius * radius;
	*normalization = double(count) * area;
	if (!std::isfinite(radius) || radius <= 0 || !std::isfinite(area) || area <= 0 ||
		count <= 0 || !std::isfinite(*normalization) || *normalization <= 0)
		return Fail("VCM merging requires a representable positive disk area times attempted path count.");
	return true;
}

bool StoreContribution(const double value[3], Vec3* contribution) {
	for (int axis = 0; axis < 3; ++axis)
		if (!std::isfinite(value[axis]) || std::abs(value[axis]) > (std::numeric_limits<real>::max)())
			return Fail("VCM merging contribution is nonfinite or outside the real range.");
	*contribution = Vec3(real(value[0]), real(value[1]), real(value[2]));
	return true;
}

// The merge contribution of photon y at query x, with both selected prefixes already checked:
// wVM * betaE * f_x(eyeOut, ORIGINAL photonOut) * betaL / (N_L * pi * r^2). MIS and the
// density normalization are applied here, once per pair.
bool MergeContribution(const Scene& scene, const PathContext& path, double value[3]) {
	const auto& y = path.light[path.selected.s - 1];
	const auto& x = path.camera[path.selected.t - 1];
	if (!Continuous(x) || !Continuous(y))
		return Fail("VCM merging endpoints must be continuous scattering surfaces.");
	double cosine = 0, xLength2 = 0, yLength2 = 0;
	for (int axis = 0; axis < 3; ++axis) {
		const double nx = x.mIsect.mGeometryNormal[axis], ny = y.mIsect.mGeometryNormal[axis];
		xLength2 += nx * nx;
		yLength2 += ny * ny;
	}
	for (int axis = 0; axis < 3; ++axis)
		cosine += (double(x.mIsect.mGeometryNormal[axis]) / std::sqrt(xLength2)) *
			(double(y.mIsect.mGeometryNormal[axis]) / std::sqrt(yLength2));
	if (cosine < 0.9)
		return Fail("VCM merging endpoints must have compatible oriented geometric normals.");
	double normalization;
	if (!Normalization(path.radius, path.lightPathCount, &normalization)) return false;
	real weight;
	if (!ComputeMisWeight(scene, path, path.selected, &weight)) return false;
	const Vec3 scattering = x.mIsect.mBsdf->Evaluate(x.mIsect.mOutDir, y.mIsect.mOutDir);
	if (!Finite(scattering)) return Fail("VCM merging evaluated nonfinite camera scattering.");
	for (int axis = 0; axis < 3; ++axis) {
		// Throughputs already include sampling compensation. There is no x-y
		// transport edge, second BSDF, cosine, or division by a full path PDF.
		value[axis] = double(x.mThroughput[axis]) * scattering[axis] *
			double(y.mThroughput[axis]) * double(weight) / normalization;
		if (!std::isfinite(value[axis]))
			return Fail("VCM merging evaluated a nonfinite normalized pair.");
	}
	return true;
}

bool HasMergePrefixes(const PathContext& path) {
	return (path.selected.s >= 2 && path.selected.t >= 2) ||
		Fail("VCM merging requires valid light and camera surface prefixes.");
}

// One selected pair: checks both prefixes, then evaluates.
bool MergePair(const Scene& scene, const PathContext& path, double value[3]) {
	return HasMergePrefixes(path) &&
		ValidLocalPrefix(path.light, path.selected.s, false) &&
		ValidLocalPrefix(path.camera, path.selected.t, true) &&
		MergeContribution(scene, path, value);
}

// One gathered photon: the caller already checked the camera prefix shared by every photon.
bool MergeGatheredPhoton(const Scene& scene, const PathContext& path, double value[3]) {
	return HasMergePrefixes(path) &&
		ValidLocalPrefix(path.light, path.selected.s, false) &&
		MergeContribution(scene, path, value);
}

}

bool ValidateSubpath(const Scene& scene, const std::vector<PathVertex>& path, bool camera) {
	if (path.empty()) return !camera || Fail("VCM camera subpath must contain its origin.");
	if (path.size() > size_t((std::numeric_limits<int>::max)()) ||
		!ValidLocalPrefix(path, static_cast<int>(path.size()), camera, true)) return false;
	for (size_t i = 0; i < path.size(); ++i) {
		const auto& vertex = path[i];
		if (vertex.mPdfFwd == 0 && !(i > 0 && path[i - 1].mIsDelta))
			return Fail("VCM generated subpath has zero sampled probability without a delta marker.");
		if (!vertex.mLight) continue;
		bool registered = false;
		for (const auto& light : scene.GetLights()) registered |= light.get() == vertex.mLight;
		if (!registered || (vertex.mType == PathVertexType::Environment &&
			scene.GetEnvironmentLight() != vertex.mLight))
			return Fail("VCM generated subpath contains an unregistered emitter.");
	}
	if (!camera) {
		const auto& origin = path.front();
		if (!Unit(origin.mDirection)) return Fail("VCM emitted subpath must retain its original unit direction.");
		real position, direction;
		if (!origin.mLight->PdfLight(origin.mIsect, -origin.mDirection, &position, &direction)) return false;
		real pmf = 0;
		for (size_t i = 0; i < scene.GetLights().size(); ++i)
			if (scene.GetLights()[i].get() == origin.mLight && !scene.LightSelectionPdf(i, &pmf)) return false;
		auto same = [](real a, real b) {
			return ValidPdf(a) && a > 0 && ValidPdf(b) && b > 0 &&
				std::abs(double(a) - b) <= 1e-4 * std::abs(double(b));
		};
		if (!same(origin.mPdfPos, position) || !same(origin.mPdfDir, direction) ||
			!same(origin.mLightSelectionPdf, pmf) ||
			!same(origin.mPdfFwd, pmf * (origin.mType == PathVertexType::Environment ? direction : position))) {
			const auto message = fmt::format(
				"VCM generated light endpoint probabilities do not match their sampling metadata: "
				"position {} / {}, direction {} / {}, selection {} / {}, map direction ({}, {}, {}).",
				origin.mPdfPos, position, origin.mPdfDir, direction, origin.mLightSelectionPdf, pmf,
				origin.mDirection.x, origin.mDirection.y, origin.mDirection.z);
			return Fail(message.c_str());
		}
	}
	return true;
}

bool EvaluateMerging(const Scene& scene, const PathContext& path, Vec3* contribution) {
	if (!contribution) return Fail("VCM merging output is null.");
	*contribution = Vec3(0);
	if (!ValidMode(path.mode) || path.selected.technique != Technique::Merging)
		return Fail("VCM merging requires a valid mode and selected merging technique.");
	if (path.mode == Mode::ConnectionOnly) return true;
	if (!ValidLocalScene(scene)) return false;
	double value[3];
	return MergePair(scene, path, value) && StoreContribution(value, contribution);
}

bool GatherMerging(const Scene& scene, const PhotonMap& map,
	const std::vector<std::vector<PathVertex>>& lightPaths,
	const std::vector<PathVertex>& cameraPath, int cameraPrefixCount,
	int maxDepth, Mode mode, Vec3* contribution) {
	if (!contribution) return Fail("VCM merging gather output is null.");
	*contribution = Vec3(0);
	if (!ValidMode(mode) || maxDepth < 0) return Fail("VCM merging gather requires a valid mode and depth.");
	if (mode == Mode::ConnectionOnly) return true;
	if (!map.References(lightPaths))
		return Fail("VCM merging gather requires a map built from the supplied immutable light-path batch.");
	if (lightPaths.empty() || lightPaths.size() > uint64((std::numeric_limits<int64>::max)()))
		return Fail("VCM merging gather requires a positive representable number of attempted light paths.");
	const int64 count = static_cast<int64>(lightPaths.size());
	double normalization;
	if (!Normalization(map.Radius(), count, &normalization) || !ValidLocalScene(scene) ||
		!ValidLocalPrefix(cameraPath, cameraPrefixCount, true)) return false;
	const auto& x = cameraPath[cameraPrefixCount - 1];
	if (x.mType == PathVertexType::Environment || x.mType == PathVertexType::Light) {
		bool registered = false;
		for (const auto& light : scene.GetLights()) registered |= light.get() == x.mLight;
		if (!registered || (x.mType == PathVertexType::Environment && scene.GetEnvironmentLight() != x.mLight))
			return Fail("VCM merging camera escape requires a registered endpoint.");
	}
	if (cameraPrefixCount == 1 || !Continuous(x)) return true;
	// Candidates come from the map; each accepted photon y is one merge strategy
	// (s = vertex + 1, t = cameraPrefixCount) of physical depth s + t - 3.
	std::vector<PhotonRecord> photons;
	if (!map.Query(x.mIsect.mPos, x.mIsect.mGeometryNormal, photons)) return false;
	double sum[3] = {};
	for (const auto& photon : photons) {
		if (photon.mLightPathIndex >= lightPaths.size() ||
			photon.mVertexIndex >= lightPaths[photon.mLightPathIndex].size() ||
			photon.mVertexIndex >= size_t((std::numeric_limits<int>::max)()))
			return Fail("VCM merging photon record is outside supported path indices.");
		const int s = static_cast<int>(photon.mVertexIndex) + 1;
		if (int64(s) + cameraPrefixCount - 3 > maxDepth) continue;
		const PathContext path(lightPaths[photon.mLightPathIndex], cameraPath,
			{ Technique::Merging, s, cameraPrefixCount }, map.Radius(), count, mode);
		double value[3];
		if (!MergeGatheredPhoton(scene, path, value)) return false;
		for (int axis = 0; axis < 3; ++axis) {
			sum[axis] += value[axis];
			if (!std::isfinite(sum[axis])) return Fail("VCM merging gather sum is nonfinite.");
		}
	}
	return StoreContribution(sum, contribution);
}

}

GYT_NAMESPACE_END
