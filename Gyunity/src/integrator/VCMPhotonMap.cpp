#include "VCMPhotonMap.h"
#include "bsdf/BSDF.h"
#include <cmath>

GYT_NAMESPACE_BEGIN

namespace Vcm {
namespace {

// The grid stores radius-relative coordinates, where float rounding within 2^20 radii is <= 1/8.
// Its cells use a broad-phase radius of two, which conservatively contains the physical unit disk.
constexpr double MaxGridCoordinate = 1048576;
constexpr real GridRadius = 2;

double GridCoordinate(real value, real origin, real radius) {
	return (double(value) - double(origin)) / double(radius);
}

bool WithinRadius(const Vec3& x, const Vec3& y, double radius2) {
	double distance2 = 0;
	for (int axis = 0; axis < 3; ++axis) {
		const double delta = double(x[axis]) - double(y[axis]);
		distance2 += delta * delta;
	}
	return distance2 <= radius2;
}

double Length(const Vec3& value) {
	return std::sqrt(double(value.x) * value.x + double(value.y) * value.y + double(value.z) * value.z);
}

bool CompatibleNormals(const Vec3& x, const Vec3& y) {
	const double xLength = Length(x), yLength = Length(y);
	double cosine = 0;
	for (int axis = 0; axis < 3; ++axis)
		cosine += (double(x[axis]) / xLength) * (double(y[axis]) / yLength);
	return cosine >= 0.9;
}

}

void PhotonMap::Clear() {
	mLightPaths = nullptr;
	mRecordCount = 0;
	mRadius = 0;
	mOrigin = Vec3();
	mGridBounds.Reset();
	mGrid.ClearHashGrid();
}

bool PhotonMap::Build(const std::vector<std::vector<PathVertex>>& lightPaths, real radius) {
	Clear();
	mLightPaths = &lightPaths;
	mRadius = radius;
	mGrid.Initialize(GridRadius);
	for (size_t p = 0; p < lightPaths.size(); ++p) {
		for (size_t i = 0; i < lightPaths[p].size(); ++i) {
			const auto& vertex = lightPaths[p][i];
			if (vertex.mType != PathVertexType::Surface || vertex.mIsect.mBsdf->IsDelta()) continue;
			if (mRecordCount == 0) mOrigin = vertex.mIsect.mPos;
			Vec3 local;
			for (int axis = 0; axis < 3; ++axis) {
				const double coordinate = GridCoordinate(vertex.mIsect.mPos[axis], mOrigin[axis], radius);
				if (std::abs(coordinate) > MaxGridCoordinate) {
					Clear();
					GetDefaultLogger()->Error("VCM photon map extent exceeds 2^20 radii from its first photon.", false);
					return false;
				}
				local[axis] = real(coordinate);
			}
			mGrid.AddPoint({ local, PhotonRecord{ p, i } }, GridRadius);
			mGridBounds.Fit(local - GridRadius);
			mGridBounds.Fit(local + GridRadius);
			++mRecordCount;
		}
	}
	if (mRecordCount) mGrid.BuildHashGrid(GridRadius);
	return true;
}

void PhotonMap::Query(const Vec3& position, const Vec3& geometricNormal, std::vector<PhotonRecord>& result) const {
	result.clear();
	if (!mRecordCount) return;
	Vec3 local;
	for (int axis = 0; axis < 3; ++axis) {
		const double coordinate = GridCoordinate(position[axis], mOrigin[axis], mRadius);
		if (coordinate < mGridBounds.minPoint[axis] || coordinate > mGridBounds.maxPoint[axis]) return;
		local[axis] = real(coordinate);
	}
	// Grid cells are candidates only: deduplicate them, then apply the exact radius and normal tests.
	const auto& bucket = mGrid.GetGrid(local);
	result.assign(bucket.begin(), bucket.end());
	std::sort(result.begin(), result.end());
	result.erase(std::unique(result.begin(), result.end()), result.end());
	const double radius2 = double(mRadius) * mRadius;
	result.erase(std::remove_if(result.begin(), result.end(), [&](const PhotonRecord& photon) {
		const auto& hit = (*mLightPaths)[photon.mLightPathIndex][photon.mVertexIndex].mIsect;
		return !WithinRadius(position, hit.mPos, radius2) || !CompatibleNormals(geometricNormal, hit.mGeometryNormal);
	}), result.end());
}

}

GYT_NAMESPACE_END
