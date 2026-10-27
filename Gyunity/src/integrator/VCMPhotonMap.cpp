#include "VCMPhotonMap.h"
#include "bsdf/BSDF.h"
#include <cmath>
#include <limits>

GYT_NAMESPACE_BEGIN

namespace Vcm {
namespace {

// In radius-relative coordinates, rounding within this extent is <= 1/8.
// The radius-two broad phase conservatively contains the physical radius-one disk.
constexpr double MaxGridCoordinate = 1048576;
constexpr real GridRadius = 2;

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

double RadiusCoordinate(real value, real origin, real radius) {
	return (double(value) - double(origin)) / double(radius);
}

bool CollectPhotons(const std::vector<std::vector<PathVertex>>& lightPaths, real radius,
	Vec3& origin, std::vector<PhotonRecord>& photons) {
	// Each indexed photon can occupy at most 27 backend cells.
	const size_t maximumCount = std::min(size_t((std::numeric_limits<int>::max)()),
		std::vector<std::pair<Vec3, PhotonRecord>>().max_size() / 27);
	for (size_t p = 0; p < lightPaths.size(); ++p) {
		const auto& path = lightPaths[p];
		for (size_t i = 0; i < path.size(); ++i) {
			const auto& vertex = path[i];
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
			if (photons.empty()) origin = vertex.mIsect.mPos;
			if (photons.size() == maximumCount)
				return Fail("VCM photon map record count exceeds backend integer or allocation ranges.");
			for (int axis = 0; axis < 3; ++axis) {
				const double local = RadiusCoordinate(vertex.mIsect.mPos[axis], origin[axis], radius);
				if (!std::isfinite(local) || std::abs(local) > MaxGridCoordinate)
					return Fail("VCM photon map extent exceeds 2^20 radii from its first photon.");
			}
			photons.push_back({ p, i });
		}
	}
	return true;
}

bool WithinRadius(const Vec3& x, const Vec3& y, double radius2) {
	double distance2 = 0;
	for (int axis = 0; axis < 3; ++axis) {
		const double delta = double(x[axis]) - double(y[axis]);
		distance2 += delta * delta;
	}
	return distance2 <= radius2;
}

bool CompatibleNormals(const Vec3& x, const Vec3& y, double xLength) {
	const double yLength = std::sqrt(Length2(y));
	double cosine = 0;
	for (int axis = 0; axis < 3; ++axis)
		cosine += (double(x[axis]) / xLength) * (double(y[axis]) / yLength);
	return cosine >= 0.9;
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

	Vec3 origin;
	std::vector<PhotonRecord> photons;
	if (!CollectPhotons(lightPaths, radius, origin, photons)) return false;
	mGrid.Initialize(GridRadius);
	for (const auto& photon : photons) {
		const auto& position = lightPaths[photon.mLightPathIndex][photon.mVertexIndex].mIsect.mPos;
		Vec3 local;
		for (int axis = 0; axis < 3; ++axis)
			local[axis] = real(RadiusCoordinate(position[axis], origin[axis], radius));
		mGrid.AddPoint({ local, photon }, GridRadius);
		mGridBounds.Fit(local - GridRadius);
		mGridBounds.Fit(local + GridRadius);
	}
	if (!photons.empty()) mGrid.BuildHashGrid(GridRadius);
	mOrigin = origin;
	mRadius = radius;
	mRecordCount = photons.size();
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
		const double coordinate = RadiusCoordinate(position[axis], mOrigin[axis], mRadius);
		// Compare in double BEFORE casting or invoking HashGrid. These deliberately
		// loose bounds include every exact-radius candidate, even at rounded edges.
		if (coordinate < mGridBounds.minPoint[axis] || coordinate > mGridBounds.maxPoint[axis]) return true;
		if (!std::isfinite(coordinate))
			return Fail("VCM photon map query has an unrepresentable relative coordinate.");
		local[axis] = real(coordinate);
	}
	const auto& bucket = mGrid.GetGrid(local);
	result.assign(bucket.begin(), bucket.end());
	std::sort(result.begin(), result.end());
	result.erase(std::unique(result.begin(), result.end()), result.end());
	const double radius2 = double(mRadius) * mRadius;
	const double queryLength = std::sqrt(Length2(geometricNormal));
	result.erase(std::remove_if(result.begin(), result.end(), [&](const PhotonRecord& record) {
		const auto& hit = (*mLightPaths)[record.mLightPathIndex][record.mVertexIndex].mIsect;
		return !WithinRadius(position, hit.mPos, radius2) ||
			!CompatibleNormals(geometricNormal, hit.mGeometryNormal, queryLength);
	}), result.end());
	return true;
}

}

GYT_NAMESPACE_END
