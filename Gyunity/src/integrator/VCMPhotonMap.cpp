#include "VCMPhotonMap.h"
#include "bsdf/BSDF.h"
#include <cmath>
#include <limits>

GYT_NAMESPACE_BEGIN

namespace Vcm {
namespace {

// HashGrid operates in radius-relative coordinates, not world coordinates.
// At this bound float rounding is <= 1/8 even for a query two units outside.
// A physical radius-one neighborhood therefore lies strictly inside the
// radius-two broad phase. Its cell width is exactly four; all backend cells
// are nonnegative and < 2^20, safely below its int casts/loop increments.
// BroadPhaseRadius is a conservative cell-insertion radius in those units,
// never the physical kernel radius used by Query's acceptance test.
constexpr double CoordinateLimit = 1048576;
constexpr real BroadPhaseRadius = 2;

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

bool ValidVector(const Vec3& value) {
	const double limit = std::sqrt((std::numeric_limits<double>::max)()) / 4;
	for (int axis = 0; axis < 3; ++axis)
		if (!std::isfinite(value[axis]) || std::abs(double(value[axis])) > limit) return false;
	return true;
}

double Length2(const Vec3& value) {
	return double(value.x) * value.x + double(value.y) * value.y + double(value.z) * value.z;
}

bool ValidNormal(const Vec3& normal) {
	return ValidVector(normal) && std::isnormal(Length2(normal));
}

bool ValidSurface(const PathVertex& vertex) {
	const auto& hit = vertex.mIsect;
	if (vertex.mNormalMode != SurfaceNormalMode::Geometric)
		return Fail("VCM photon map only supports geometric-normal transport.");
	if (!ValidVector(hit.mPos) || !ValidNormal(hit.mGeometryNormal) ||
		!ValidVector(hit.mNormal) || !ValidVector(hit.mOutDir) ||
		std::abs(Length2(hit.mOutDir) - 1) > 1e-4 ||
		!ValidVector(vertex.mThroughput) ||
		!std::isfinite(vertex.mPdfFwd) || vertex.mPdfFwd < 0 ||
		!std::isfinite(vertex.mPdfPrev) || vertex.mPdfPrev < 0) {
		return Fail("VCM photon map requires finite surface data, a geometric normal and a unit incident direction.");
	}
	const double length = std::sqrt(Length2(hit.mGeometryNormal));
	for (int axis = 0; axis < 3; ++axis)
		if (std::abs(double(hit.mNormal[axis]) - hit.mGeometryNormal[axis] / length) > 1e-4)
			return Fail("VCM photon map does not support shading-normal transport.");
	if (hit.mMediumInterface.inside || hit.mMediumInterface.outside || vertex.mCamera)
		return Fail("VCM photon map does not support medium or camera metadata on surfaces.");
	if (!hit.mBsdf || !hit.mBsdf->MatchScatterType(BSDF_ALL))
		return Fail("VCM photon map requires a scattering BSDF at every surface.");
	if (hit.mBsdf->HasMixedDeltaLobes())
		return Fail("VCM photon map does not support mixed delta and continuous BSDF lobes.");
	if (vertex.mIsDelta && !hit.mBsdf->IsDelta())
		return Fail("VCM photon map surface has inconsistent sampled-delta metadata.");
	return true;
}

double Coordinate(real value, real origin, real radius) {
	return (double(value) - double(origin)) / double(radius);
}

// Physical acceptance after the broad phase, on world-space values in double precision:
// an exact disk of the kernel radius and an oriented geometric-normal test.
constexpr double MinimumNormalCosine = 0.9;

double SquaredDistance(const Vec3& a, const Vec3& b) {
	double distance2 = 0;
	for (int axis = 0; axis < 3; ++axis) {
		const double delta = double(a[axis]) - double(b[axis]);
		distance2 += delta * delta;
	}
	return distance2;
}

double NormalCosine(const Vec3& a, const Vec3& b) {
	const double aLength = std::sqrt(Length2(a)), bLength = std::sqrt(Length2(b));
	double cosine = 0;
	for (int axis = 0; axis < 3; ++axis)
		cosine += (double(a[axis]) / aLength) * (double(b[axis]) / bLength);
	return cosine;
}

}

void PhotonMap::Clear() {
	mBuilt = false;
	mLightPaths = nullptr;
	mRecordCount = 0;
	mRadius = 0;
	mOrigin = Vec3();
	mGridBounds.Reset();
	mGrid.ClearHashGrid();
}

bool PhotonMap::Build(const std::vector<std::vector<PathVertex>>& lightPaths, real radius) {
	Clear();
	const double radius2 = double(radius) * radius;
	if (!std::isfinite(radius) || radius <= 0 || !std::isnormal(radius2))
		return Fail("VCM photon map radius must be finite, positive and have a representable normal double square.");

	size_t count = 0;
	Vec3 origin;
	// Also bound vector element counts on narrower size_t implementations. Each
	// point reaches at most 27 cells, including all duplicate bucket identities.
	const size_t maximumCount = std::min(size_t((std::numeric_limits<int>::max)()),
		std::vector<std::pair<Vec3, PhotonRecord>>().max_size() / 27);
	// Validate the entire generation before backend allocations or float->int casts.
	for (const auto& path : lightPaths) for (const auto& vertex : path) {
		switch (vertex.mType) {
		case PathVertexType::Camera:
		case PathVertexType::Light:
		case PathVertexType::Environment:
			continue;
		case PathVertexType::Surface:
			break;
		default:
			return Fail("VCM photon map encountered an unknown vertex type.");
		}
		if (!ValidSurface(vertex)) return false;
		if (vertex.mIsect.mBsdf->IsDelta()) continue;
		if (count == 0) origin = vertex.mIsect.mPos;
		if (count == maximumCount)
			return Fail("VCM photon map record count exceeds backend integer or allocation ranges.");
		++count;
		for (int axis = 0; axis < 3; ++axis) {
			const double local = Coordinate(vertex.mIsect.mPos[axis], origin[axis], radius);
			if (!std::isfinite(local) || std::abs(local) > CoordinateLimit)
				return Fail("VCM photon map extent exceeds 2^20 radii from its first photon.");
		}
	}

	mGrid.Initialize(BroadPhaseRadius);
	for (size_t pathIndex = 0; pathIndex < lightPaths.size(); ++pathIndex) {
		const auto& path = lightPaths[pathIndex];
		for (size_t vertexIndex = 0; vertexIndex < path.size(); ++vertexIndex) {
			const auto& vertex = path[vertexIndex];
			if (vertex.mType != PathVertexType::Surface || vertex.mIsect.mBsdf->IsDelta()) continue;
			Vec3 local;
			for (int axis = 0; axis < 3; ++axis)
				local[axis] = real(Coordinate(vertex.mIsect.mPos[axis], origin[axis], radius));
			mGrid.AddPoint({ local, { pathIndex, vertexIndex } }, BroadPhaseRadius);
			mGridBounds.Fit(local - BroadPhaseRadius);
			mGridBounds.Fit(local + BroadPhaseRadius);
		}
	}
	if (count) mGrid.BuildHashGrid(BroadPhaseRadius);
	mOrigin = origin;
	mRadius = radius;
	mRecordCount = count;
	mLightPaths = &lightPaths;
	mBuilt = true;
	return true;
}

bool PhotonMap::Query(const Vec3& position, const Vec3& geometricNormal,
	std::vector<PhotonRecord>& result) const {
	result.clear();
	if (!mBuilt) return Fail("VCM photon map query requires a successful build.");
	if (!ValidVector(position) || !ValidNormal(geometricNormal))
		return Fail("VCM photon map query requires a finite position and nonzero finite geometric normal.");
	if (!mRecordCount) return true;

	Vec3 local;
	for (int axis = 0; axis < 3; ++axis) {
		const double coordinate = Coordinate(position[axis], mOrigin[axis], mRadius);
		// Compare in double BEFORE casting or invoking HashGrid. These deliberately
		// loose bounds include every exact-radius candidate, even at rounded edges.
		if (coordinate < mGridBounds.minPoint[axis] || coordinate > mGridBounds.maxPoint[axis]) return true;
		if (!std::isfinite(coordinate))
			return Fail("VCM photon map query has an unrepresentable relative coordinate.");
		local[axis] = real(coordinate);
	}
	// Broad phase: every record in the query's grid cell, deduplicated in (path, vertex) order.
	const auto& bucket = mGrid.GetGrid(local);
	result.assign(bucket.begin(), bucket.end());
	std::sort(result.begin(), result.end());
	result.erase(std::unique(result.begin(), result.end()), result.end());
	// Physical acceptance: keep a candidate only inside the exact disk with a compatible normal.
	const double radius2 = double(mRadius) * mRadius;
	result.erase(std::remove_if(result.begin(), result.end(), [&](const PhotonRecord& record) {
		const auto& hit = (*mLightPaths)[record.mLightPathIndex][record.mVertexIndex].mIsect;
		return SquaredDistance(position, hit.mPos) > radius2 ||
			NormalCosine(geometricNormal, hit.mGeometryNormal) < MinimumNormalCosine;
	}), result.end());
	return true;
}

}

GYT_NAMESPACE_END
