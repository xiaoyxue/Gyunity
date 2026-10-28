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

int VertexCount(const PathContext& path) {
	return path.selected.s + path.selected.t - (path.selected.technique == Technique::Merging ? 1 : 0);
}

// At a separated merge there is no universal geometric vertex at index s-1.
// Reverse walks on the light side retain y; forward walks on the eye side retain x.
const PathVertex& LightVertex(const PathContext& path, int i) {
	return i < path.selected.s ? path.light[i] : path.camera[VertexCount(path) - 1 - i];
}

const PathVertex& CameraVertex(const PathContext& path, int i) {
	const int split = path.selected.s - (path.selected.technique == Technique::Merging ? 1 : 0);
	return i < split ? path.light[i] : path.camera[VertexCount(path) - 1 - i];
}

bool Delta(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf &&
		vertex.mIsect.mBsdf->IsDelta();
}

bool Continuous(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface && vertex.mIsect.mBsdf && !Delta(vertex);
}

bool ValidateMergeGeometry(const PathContext& path) {
	const auto& y = path.light[path.selected.s - 1];
	const auto& x = path.camera[path.selected.t - 1];
	double distance2 = 0;
	for (int axis = 0; axis < 3; ++axis) {
		const double offset = double(y.mIsect.mPos[axis]) - double(x.mIsect.mPos[axis]);
		distance2 += offset * offset;
	}
	if (!std::isfinite(distance2) || distance2 > path.radius * path.radius ||
		!Unit(y.mIsect.mOutDir) || !Unit(x.mIsect.mOutDir))
		return Fail("VCM merge requires in-radius endpoints and original unit incoming directions.");
	return true;
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
	const PathVertex& origin = LightVertex(path, 0);
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
		if (!Bidirectional::Direction(origin, LightVertex(path, 1), &emitted)) return false;
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
		if (!ValidateMergeGeometry(path)) return false;
	}
	return true;
}

bool Feasible(const PathContext& path, const Strategy& strategy) {
	const int n = VertexCount(path), s = strategy.s;
	if (strategy.technique == Technique::Merging) {
		return s >= 2 && s < n && Continuous(LightVertex(path, s - 1)) &&
			Continuous(CameraVertex(path, s - 1));
	}
	if (strategy.t == 0 || (s == 1 && strategy.t == 1)) return false;
	if (s == 0) return !LightVertex(path, 0).mLight->IsDeltaLight();
	return (s == 1 || Continuous(LightVertex(path, s - 1))) &&
		(s == n - 1 || Continuous(CameraVertex(path, s)));
}

bool ForwardDensity(const Scene& scene, const PathContext& path, int i, real* pdf) {
	const int n = VertexCount(path), s = path.selected.s;
	const int source = i - 1;
	if (source > 0 && source < n - 1 && Delta(LightVertex(path, source))) {
		// Remove the singular distribution, not its component mass or geometry.
		if (source < s) {
			if (source + 1 >= s) return Fail("VCM cannot invent a delta event across an unsampled connection.");
			const auto& vertex = path.light[source];
			return ConvertDensity(vertex.mDeltaPdf, vertex, LightVertex(path, i), pdf);
		}
		const int j = n - 1 - source;
		if (j + 1 >= path.selected.t)
			return Fail("VCM cannot invent a delta event across an unsampled connection.");
		const auto& vertex = path.camera[j];
		return ConvertDensity(vertex.mDeltaReversePdf, vertex, CameraVertex(path, i), pdf);
	}

	// Reuse a reverse PDF only while its source AND sampled successor remain selected.
	const int j = n - 1 - i;
	if (j < path.selected.t - 2) {
		*pdf = path.camera[j].mPdfPrev;
		return true;
	}
	if (i < s) {
		*pdf = path.light[i].mPdfFwd;
		return true;
	}
	if (path.selected.technique == Technique::Merging && i == s) {
		const auto& y = path.light[s - 1];
		const auto& x = path.camera[path.selected.t - 1];
		// Report Fig. 3: scatter at x using original directions, convert along x's sampled edge.
		const real pdfDir = x.mIsect.mBsdf->Pdf(y.mIsect.mOutDir, x.mIsect.mOutDir);
		return ConvertDensity(pdfDir, x, CameraVertex(path, i), pdf);
	}
	if (i == 0)
		return Bidirectional::LightOriginPdf(scene, CameraVertex(path, 0), CameraVertex(path, 1), pdf);
	return Bidirectional::PdfToward(CameraVertex(path, source),
		i >= 2 ? &CameraVertex(path, i - 2) : nullptr, CameraVertex(path, i), pdf);
}

bool ReverseDensity(const PathContext& path, int i, real* pdf) {
	const int n = VertexCount(path), s = path.selected.s;
	const int source = i + 1;
	if (source > 0 && source < n - 1 && Delta(LightVertex(path, source))) {
		if (source < s) {
			if (source + 1 >= s) return Fail("VCM cannot invent a delta event across an unsampled connection.");
			const auto& vertex = path.light[source];
			return ConvertDensity(vertex.mDeltaReversePdf, vertex, LightVertex(path, i), pdf);
		}
		const int j = n - 1 - source;
		if (j + 1 >= path.selected.t)
			return Fail("VCM cannot invent a delta event across an unsampled connection.");
		const auto& vertex = path.camera[j];
		return ConvertDensity(vertex.mDeltaPdf, vertex, CameraVertex(path, i), pdf);
	}

	if (i < s - 2) {
		*pdf = path.light[i].mPdfPrev;
		return true;
	}
	const bool merging = path.selected.technique == Technique::Merging;
	if (i >= s - (merging ? 1 : 0)) {
		*pdf = path.camera[n - 1 - i].mPdfFwd;
		return true;
	}
	if (merging && i == s - 2) {
		const auto& y = path.light[s - 1];
		const auto& x = path.camera[path.selected.t - 1];
		// The reverse merge PDF scatters at x but converts along y's original light edge.
		const real pdfDir = x.mIsect.mBsdf->Pdf(x.mIsect.mOutDir, y.mIsect.mOutDir);
		return ConvertDensity(pdfDir, y, LightVertex(path, i), pdf);
	}
	return Bidirectional::PdfToward(LightVertex(path, source),
		i + 2 < n ? &LightVertex(path, i + 2) : nullptr, LightVertex(path, i), pdf);
}

class MisPdfs {
public:
	MisPdfs(const Scene& scene, const PathContext& path)
		: scene(scene), path(path), mPdfFwd(VertexCount(path)), mPdfPrev(VertexCount(path)),
		mHasFwd(VertexCount(path), false), mHasPrev(VertexCount(path), false) {}

	bool PdfFwd(int i, real* pdf) {
		if (!mHasFwd[i]) {
			if (!ForwardDensity(scene, path, i, &mPdfFwd[i])) return false;
			if (!ValidPdf(mPdfFwd[i])) return Fail("VCM evaluated a nonfinite or negative density.");
			mHasFwd[i] = true;
		}
		*pdf = mPdfFwd[i];
		return true;
	}

	bool PdfPrev(int i, real* pdf) {
		if (!mHasPrev[i]) {
			if (!ReverseDensity(path, i, &mPdfPrev[i])) return false;
			if (!ValidPdf(mPdfPrev[i])) return Fail("VCM evaluated a nonfinite or negative density.");
			mHasPrev[i] = true;
		}
		*pdf = mPdfPrev[i];
		return true;
	}

	const Scene& scene;
	const PathContext& path;
private:
	std::vector<real> mPdfFwd, mPdfPrev;
	std::vector<bool> mHasFwd, mHasPrev;
};

// Cached getters reuse recorded edge PDFs and lazily correct only changed boundary events.
bool StrategyDensity(MisPdfs& pdfs, const Strategy& strategy, StrategyPdf* pdfPath) {
	*pdfPath = {};
	const auto& path = pdfs.path;
	if (!ValidTechnique(strategy.technique) || strategy.s < 0 || strategy.t < 0 ||
		int64(strategy.s) + strategy.t - (strategy.technique == Technique::Merging ? 1 : 0) != VertexCount(path)) {
		return Fail("VCM alternative must have valid counts at the selected physical depth.");
	}
	if (!IsStrategyEnabled(path, strategy) || !Feasible(path, strategy)) return true;

	double logPdfPath = 0;
	bool positive = true;
	// Eq. (10): light prefix, then camera prefix in light-to-camera index order.
	for (int i = 0; i < strategy.s; ++i) {
		real pdf;
		if (!pdfs.PdfFwd(i, &pdf)) return false;
		if (pdf == 0) positive = false;
		else logPdfPath += std::log(double(pdf));
	}
	// VM samples both the landing and query densities; it does not collapse their geometry.
	const int cameraStart = strategy.s - (strategy.technique == Technique::Merging ? 1 : 0);
	for (int i = cameraStart; i < VertexCount(path); ++i) {
		real pdf;
		if (!pdfs.PdfPrev(i, &pdf)) return false;
		if (pdf == 0) positive = false;
		else logPdfPath += std::log(double(pdf));
	}
	if (!positive) return true;
	if (strategy.technique == Technique::Merging) {
		logPdfPath += std::log(std::acos(-1.0) * path.radius * path.radius);
	}
	if (!std::isfinite(logPdfPath)) return Fail("VCM path log density is not finite.");
	pdfPath->feasible = true;
	pdfPath->logPdf = logPdfPath;
	return true;
}

}

bool UseVc(Mode mode) { return mode == Mode::Combined || mode == Mode::ConnectionOnly; }
bool UseVm(Mode mode) { return mode == Mode::Combined || mode == Mode::MergingOnly; }

bool IsStrategyEnabled(const PathContext& path, const Strategy& strategy) {
	if (strategy.technique == Technique::Merging) return UseVm(path.mode);
	if (UseVc(path.mode)) return true;
	if (path.mode != Mode::MergingOnly || strategy.s != 0) return false;
	for (int i = 1; i + 1 < VertexCount(path); ++i)
		if (!Delta(LightVertex(path, i))) return false;
	return true; // Retain camera-visible LS*E emission in MergingOnly.
}

bool ComputePathPdf(const Scene& scene, const PathContext& path,
	const Strategy& strategy, StrategyPdf* pdfPath) {
	if (!pdfPath) return Fail("VCM PDF output is null.");
	*pdfPath = {};
	if (!Validate(scene, path)) return false;
	MisPdfs pdfs(scene, path);
	return StrategyDensity(pdfs, strategy, pdfPath);
}

bool ComputeMisWeight(const Scene& scene, const PathContext& path,
	const Strategy& current, real* weight) {
	if (!weight) return Fail("VCM weight output is null.");
	*weight = 0;
	if (!Validate(scene, path)) return false;
	MisPdfs pdfs(scene, path);
	StrategyPdf currentPdfPath;
	if (!StrategyDensity(pdfs, current, &currentPdfPath)) return false;
	if (!currentPdfPath.feasible) return Fail("VCM current strategy has zero support or is disabled.");
	const int64 nVc = 1, nVm = path.lightPathCount;
	const double logCountVc = std::log(double(nVc)), logCountVm = std::log(double(nVm));
	const double currentScore = currentPdfPath.logPdf + (current.technique == Technique::Merging ? logCountVm : logCountVc);
	std::vector<double> scores;
	double maximum = -std::numeric_limits<double>::infinity();
	const int n = VertexCount(path);
	const auto addScore = [&](const Strategy& strategy, double logSampleCount) {
		StrategyPdf pdfPath;
		if (!StrategyDensity(pdfs, strategy, &pdfPath)) return false;
		if (!pdfPath.feasible) return true;
		const double score = pdfPath.logPdf + logSampleCount;
		scores.push_back(score);
		maximum = std::max(maximum, score);
		return true;
	};
	if (UseVc(path.mode)) {
		for (int s = 0; s < n; ++s)
			if (!addScore({ Technique::Connection, s, n - s }, logCountVc)) return false;
	}
	else if (!addScore({ Technique::Connection, 0, n }, logCountVc)) return false;
	if (UseVm(path.mode))
		for (int s = 2; s < n; ++s)
			if (!addScore({ Technique::Merging, s, n - s + 1 }, logCountVm)) return false;
	double sum = 0;
	for (double score : scores) sum += std::exp(score - maximum);
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

bool ValidateMergePair(const PathContext& path, double* normalization) {
	if (path.selected.s < 2 || path.selected.t < 2)
		return Fail("VCM merging requires valid light and camera surface prefixes.");
	if (!ValidLocalPrefix(path.light, path.selected.s, false) ||
		!ValidLocalPrefix(path.camera, path.selected.t, true)) return false;
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
	return Normalization(path.radius, path.lightPathCount, normalization);
}

bool MergeNumerator(const PathContext& path, double value[3]) {
	const auto& y = path.light[path.selected.s - 1];
	const auto& x = path.camera[path.selected.t - 1];
	const Vec3 scattering = x.mIsect.mBsdf->Evaluate(x.mIsect.mOutDir, y.mIsect.mOutDir);
	if (!Finite(scattering)) return Fail("VCM merging evaluated nonfinite camera scattering.");
	for (int axis = 0; axis < 3; ++axis) {
		// Throughputs already include sampling compensation. There is no x-y
		// transport edge, second BSDF, cosine, or division by a full path PDF.
		value[axis] = double(x.mThroughput[axis]) * scattering[axis] * double(y.mThroughput[axis]);
		if (!std::isfinite(value[axis]))
			return Fail("VCM merging evaluated a nonfinite primary contribution.");
	}
	return true;
}

bool MergePair(const Scene& scene, const PathContext& path, double value[3]) {
	double normalization;
	if (!ValidateMergePair(path, &normalization)) return false;
	real weight;
	if (!ComputeMisWeight(scene, path, path.selected, &weight) || !MergeNumerator(path, value)) return false;
	for (int axis = 0; axis < 3; ++axis) {
		value[axis] = value[axis] * double(weight) / normalization;
		if (!std::isfinite(value[axis]))
			return Fail("VCM merging evaluated a nonfinite normalized pair.");
	}
	return true;
}

}

bool MergeEstimate(const PathContext& path, std::array<double, 3>* contributionVm) {
	if (!contributionVm) return Fail("VCM primary merge output is null.");
	*contributionVm = {};
	if (!ValidMode(path.mode) || path.selected.technique != Technique::Merging)
		return Fail("VCM primary merge requires a valid mode and merging technique.");
	if (!UseVm(path.mode)) return true;
	double normalization;
	if (!ValidateMergePair(path, &normalization) || !ValidateMergeGeometry(path)) return false;
	std::array<double, 3> value{};
	if (!MergeNumerator(path, value.data())) return false;
	const double area = std::acos(-1.0) * path.radius * path.radius;
	for (double& channel : value) {
		channel /= area;
		if (!std::isfinite(channel)) return Fail("VCM primary merge estimate is not representable.");
	}
	*contributionVm = value;
	return true;
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
	const std::vector<PathVertex>& cameraPath, int t,
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
		!ValidLocalPrefix(cameraPath, t, true)) return false;
	const auto& x = cameraPath[t - 1];
	if (x.mType == PathVertexType::Environment || x.mType == PathVertexType::Light) {
		bool registered = false;
		for (const auto& light : scene.GetLights()) registered |= light.get() == x.mLight;
		if (!registered || (x.mType == PathVertexType::Environment && scene.GetEnvironmentLight() != x.mLight))
			return Fail("VCM merging camera escape requires a registered endpoint.");
	}
	if (t == 1 || !Continuous(x)) return true;
	std::vector<PhotonRecord> photons;
	if (!map.Query(x.mIsect.mPos, x.mIsect.mGeometryNormal, photons)) return false;
	double sum[3] = {};
	for (const auto& photon : photons) {
		if (photon.mLightPathIndex >= lightPaths.size() ||
			photon.mVertexIndex >= lightPaths[photon.mLightPathIndex].size() ||
			photon.mVertexIndex >= size_t((std::numeric_limits<int>::max)()))
			return Fail("VCM merging photon record is outside supported path indices.");
		const int s = static_cast<int>(photon.mVertexIndex) + 1;
		if (int64(s) + t - 3 > maxDepth) continue;
		const PathContext path(lightPaths[photon.mLightPathIndex], cameraPath,
			{ Technique::Merging, s, t }, map.Radius(), count, mode);
		double value[3];
		if (!MergePair(scene, path, value)) return false;
		for (int axis = 0; axis < 3; ++axis) {
			sum[axis] += value[axis];
			if (!std::isfinite(sum[axis])) return Fail("VCM merging gather sum is nonfinite.");
		}
	}
	return StoreContribution(sum, contribution);
}

}

GYT_NAMESPACE_END
