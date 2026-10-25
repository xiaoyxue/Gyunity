#include "VCMPath.h"
#include "camera/PinholeCamera.h"
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
const PathVertex& Vertex(const PathContext& path, int i, bool lightSide) {
	const int split = path.selected.s;
	if (path.selected.technique == Technique::Connection) {
		return i < split ? path.light[i] : path.camera[VertexCount(path) - 1 - i];
	}
	return i < split - 1 || (i == split - 1 && lightSide) ?
		path.light[i] : path.camera[VertexCount(path) - 1 - i];
}

bool ValidateVertex(const PathVertex& vertex, bool lightOrigin, bool cameraOrigin) {
	if (!ValidPdf(vertex.mPdfFwd) || !ValidPdf(vertex.mPdfPrev) ||
		!ValidPdf(vertex.mPdfPos) || !ValidPdf(vertex.mPdfDir) ||
		!ValidPdf(vertex.mLightSelectionPdf)) return Fail("VCM path contains an invalid density.");
	if (!Finite(vertex.mIsect.mPos)) return Fail("VCM path contains a nonfinite position.");
	if (vertex.mType == PathVertexType::Environment ||
		(vertex.mLight && !vertex.mLight->IsAreaLight())) {
		return Fail("VCM continuous MIS does not yet support point or environment origins.");
	}
	if (cameraOrigin) {
		if (vertex.mType != PathVertexType::Camera ||
			!dynamic_cast<const PinHoleCamera*>(vertex.mCamera)) {
			return Fail("VCM continuous MIS requires a pinhole camera origin.");
		}
	}
	else {
		if (!Unit(vertex.mIsect.mGeometryNormal) || !Unit(vertex.mIsect.mNormal) ||
			(vertex.mIsect.mNormal - vertex.mIsect.mGeometryNormal).Length2() > 1e-8f) {
			return Fail("VCM continuous MIS requires unit geometric-normal shading.");
		}
		if (lightOrigin) {
			if ((vertex.mType != PathVertexType::Light && vertex.mType != PathVertexType::Surface) ||
				!vertex.mLight) return Fail("VCM continuous MIS requires a finite area-light origin.");
		}
		else if (vertex.mType != PathVertexType::Surface || !vertex.mIsect.mBsdf) {
			return Fail("VCM continuous MIS requires a BSDF at each scattering vertex.");
		}
	}
	if (vertex.mIsDelta || (vertex.mIsect.mBsdf &&
		(vertex.mIsect.mBsdf->IsDelta() || vertex.mIsect.mBsdf->HasMixedDeltaLobes()))) {
		return Fail("VCM continuous MIS does not yet support specular or mixed-delta paths.");
	}
	if (vertex.mIsect.mMediumInterface.inside || vertex.mIsect.mMediumInterface.outside) {
		return Fail("VCM continuous MIS does not support participating media.");
	}
	return true;
}

bool Validate(const PathContext& path) {
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
		if (!ValidateVertex(path.light[i], i == 0, false)) return false;
	}
	for (int i = 0; i < selected.t; ++i) {
		if (!ValidateVertex(path.camera[i], selected.s == 0 && i == selected.t - 1, i == 0)) return false;
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
		const Vec3 offset = y.mIsect.mPos - x.mIsect.mPos;
		const double distance2 = double(offset.x) * offset.x + double(offset.y) * offset.y + double(offset.z) * offset.z;
		if (!std::isfinite(distance2) || distance2 > path.radius * path.radius ||
			!Unit(y.mIsect.mOutDir) || !Unit(x.mIsect.mOutDir)) {
			return Fail("VCM merge requires in-radius endpoints and original unit incoming directions.");
		}
	}
	return true;
}

bool Enabled(const PathContext& path, const Strategy& strategy) {
	if (strategy.technique == Technique::Merging) return path.mode != Mode::ConnectionOnly;
	// With no singular vertices, the only LS*E path is the depth-zero emitter hit.
	return path.mode != Mode::MergingOnly || (VertexCount(path) == 2 && strategy.s == 0);
}

bool Density(const Scene& scene, const PathContext& path, int i, bool forward, real* pdf) {
	const int n = VertexCount(path), s = path.selected.s;
	if (path.selected.technique == Technique::Merging) {
		const int j = s - 1;
		if (forward && i <= j) { *pdf = path.light[i].mPdfFwd; return true; }
		if (!forward && i >= j) { *pdf = path.camera[n - 1 - i].mPdfFwd; return true; }
		if (i == (forward ? j + 1 : j - 1)) {
			const PathVertex& y = path.light[j];
			const PathVertex& x = path.camera[path.selected.t - 1];
			// Report 4.3 / Fig. 3: scatter at x with the ORIGINAL eye/photon
			// directions, but convert along the sampled edge on that side.
			// There is no x-y edge, and neither x->a nor y->b is a sampled edge.
			const real pdfDir = forward ? x.mIsect.mBsdf->Pdf(y.mIsect.mOutDir, x.mIsect.mOutDir) :
				x.mIsect.mBsdf->Pdf(x.mIsect.mOutDir, y.mIsect.mOutDir);
			return ConvertDensity(pdfDir, forward ? x : y, Vertex(path, i, !forward), pdf);
		}
	}
	else {
		if (forward && i < s) { *pdf = path.light[i].mPdfFwd; return true; }
		if (!forward && i >= s) { *pdf = path.camera[n - 1 - i].mPdfFwd; return true; }
	}
	const bool lightSide = !forward;
	if (forward && i == 0) {
		return Bidirectional::LightOriginPdf(scene, Vertex(path, 0, lightSide), Vertex(path, 1, lightSide), pdf);
	}
	const int source = i + (forward ? -1 : 1);
	const int previous = i + (forward ? -2 : 2);
	return Bidirectional::PdfToward(Vertex(path, source, lightSide),
		previous >= 0 && previous < n ? &Vertex(path, previous, lightSide) : nullptr,
		Vertex(path, i, lightSide), pdf);
}

}

bool ComputePathPdf(const Scene& scene, const PathContext& path,
	const Strategy& strategy, StrategyPdf* result) {
	if (!result) return Fail("VCM PDF output is null.");
	*result = {};
	if (!Validate(path)) return false;
	if (!ValidTechnique(strategy.technique) || strategy.s < 0 || strategy.t < 0 ||
		int64(strategy.s) + strategy.t - (strategy.technique == Technique::Merging ? 1 : 0) != VertexCount(path)) {
		return Fail("VCM alternative must have valid counts at the selected physical depth.");
	}
	if (!Enabled(path, strategy) ||
		(strategy.technique == Technique::Connection && (strategy.t == 0 || (strategy.s == 1 && strategy.t == 1))) ||
		(strategy.technique == Technique::Merging && (strategy.s < 2 || strategy.t < 2))) return true;

	double logPdf = 0;
	bool positive = true;
	// Eq. (10): independently multiply the two prefixes of THIS strategy.
	// At a merge the forward prefix includes its landing; the reverse prefix
	// includes its query, duplicating one density, not one geometric position.
	const int reverseStart = strategy.s - (strategy.technique == Technique::Merging ? 1 : 0);
	for (int side = 0; side < 2; ++side) {
		for (int i = side == 0 ? 0 : reverseStart; i < (side == 0 ? strategy.s : VertexCount(path)); ++i) {
			real pdf;
			if (!Density(scene, path, i, side == 0, &pdf)) return false;
			if (!ValidPdf(pdf)) return Fail("VCM evaluated a nonfinite or negative density.");
			if (pdf == 0) positive = false;
			else logPdf += std::log(double(pdf));
		}
	}
	if (!positive) return true;
	if (strategy.technique == Technique::Merging) {
		logPdf += std::log(std::acos(-1.0) * path.radius * path.radius);
	}
	if (!std::isfinite(logPdf)) return Fail("VCM path log density is not finite.");
	result->feasible = true;
	result->logPdf = logPdf;
	return true;
}

bool ComputeMisWeight(const Scene& scene, const PathContext& path,
	const Strategy& current, real* weight) {
	if (!weight) return Fail("VCM weight output is null.");
	*weight = 0;
	StrategyPdf currentPdf;
	if (!ComputePathPdf(scene, path, current, &currentPdf)) return false;
	if (!currentPdf.feasible) return Fail("VCM current strategy has zero support or is disabled.");
	const double logCount = std::log(double(path.lightPathCount));
	const double currentScore = currentPdf.logPdf + (current.technique == Technique::Merging ? logCount : 0);
	std::vector<double> scores;
	double maximum = -std::numeric_limits<double>::infinity();
	const int n = VertexCount(path);
	for (Technique technique : { Technique::Connection, Technique::Merging }) {
		for (int s = technique == Technique::Connection ? 0 : 2; s < n; ++s) {
			const Strategy strategy{ technique, s, n - s + (technique == Technique::Merging ? 1 : 0) };
			StrategyPdf pdf;
			if (!ComputePathPdf(scene, path, strategy, &pdf)) return false;
			if (!pdf.feasible) continue;
			const double score = pdf.logPdf + (technique == Technique::Merging ? logCount : 0);
			scores.push_back(score);
			maximum = std::max(maximum, score);
		}
	}
	double sum = 0;
	for (double score : scores) sum += std::exp(score - maximum);
	const double normalized = std::exp(currentScore - maximum) / sum;
	if (!std::isfinite(normalized) || normalized < 0 || normalized > 1) return Fail("VCM normalized weight is invalid.");
	*weight = static_cast<real>(normalized);
	return true;
}

}

GYT_NAMESPACE_END
