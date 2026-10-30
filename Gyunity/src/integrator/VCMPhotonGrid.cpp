#include "VCMPhotonGrid.h"
#include "bsdf/BSDF.h"
#include "tbb/blocked_range.h"
#include "tbb/parallel_for.h"
#include "tbb/parallel_sort.h"
#include <atomic>
#include <cmath>
#include <limits>

GYT_NAMESPACE_BEGIN

namespace Vcm {
namespace {

// Cell coordinates stay far inside int64, so neighbouring cells never overflow.
constexpr double MaxCell = 4611686018427387904.0; // 2^62
// Widens a query's cell range by a relative 1e-6 of the radius, so a photon that Eq. (12) accepts
// at the rounding limit is never in a skipped cell.
constexpr double QueryMargin = 1 + 1e-6;

bool Fail(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

// Every continuous surface vertex is a photon: the first surface and emitting surfaces included,
// pure-delta vertices excluded.
bool Stored(const PathVertex& vertex) {
	return vertex.mType == PathVertexType::Surface && !vertex.mIsect.mBsdf->IsDelta();
}

// Eq. (12) in double precision on the stored float inputs.
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

void PhotonGrid::Clear() {
	mCount = 0;
	mBucketMask = 0;
	mInvCellSize = 0;
	mRadius = 0;
}

// Cells are floor(coordinate / 2r): monotone in the coordinate, so a query's range [p - r, p + r]
// maps to a contiguous range of cells that contains every photon within r.
bool PhotonGrid::Cell(double coordinate, int64* cell) const {
	const double scaled = std::floor(coordinate * mInvCellSize);
	if (!(std::abs(scaled) < MaxCell)) return false;
	*cell = int64(scaled);
	return true;
}

uint64 PhotonGrid::Bucket(int64 x, int64 y, int64 z) const {
	uint64 hash = uint64(x) * 0x9E3779B97F4A7C15ull ^ uint64(y) * 0xC2B2AE3D27D4EB4Full ^
		uint64(z) * 0x165667B19E3779F9ull;
	hash ^= hash >> 29;
	return hash & mBucketMask;
}

bool PhotonGrid::Build(const std::vector<std::vector<PathVertex>>& lightPaths, real radius) {
	Clear();
	if (!(radius > 0) || !std::isfinite(radius)) return Fail("VCM photon grid requires a positive finite radius.");
	mRadius = radius;
	mInvCellSize = 1 / (2 * double(radius));
	const size_t paths = lightPaths.size();
	// Count each path's photons in parallel, then place them in (path, vertex) order.
	mPathOffsets.resize(paths + 1);
	mPathOffsets[0] = 0;
	tbb::parallel_for(tbb::blocked_range<size_t>(0, paths), [&](const tbb::blocked_range<size_t>& range) {
		for (size_t p = range.begin(); p != range.end(); ++p) {
			size_t count = 0;
			for (const auto& vertex : lightPaths[p]) count += Stored(vertex);
			mPathOffsets[p + 1] = count;
		}
	});
	for (size_t p = 0; p < paths; ++p) mPathOffsets[p + 1] += mPathOffsets[p];
	const size_t count = mPathOffsets[paths];
	if (count == 0) return true;
	if (count >= (size_t(1) << 32)) return Fail("VCM photon grid holds fewer than 2^32 photons.");
	size_t buckets = 1;
	while (buckets < count) buckets <<= 1;
	mBucketMask = buckets - 1;

	mUnsorted.resize(count);
	mKeys.resize(count);
	std::atomic<bool> valid{ true };
	tbb::parallel_for(tbb::blocked_range<size_t>(0, paths), [&](const tbb::blocked_range<size_t>& range) {
		for (size_t p = range.begin(); p != range.end(); ++p) {
			size_t index = mPathOffsets[p];
			for (size_t v = 0; v < lightPaths[p].size(); ++v) {
				const auto& vertex = lightPaths[p][v];
				if (!Stored(vertex)) continue;
				const Vec3& position = vertex.mIsect.mPos;
				int64 x, y, z;
				if (!Cell(position.x, &x) || !Cell(position.y, &y) || !Cell(position.z, &z)) {
					valid.store(false);
					return;
				}
				mUnsorted[index] = { position, vertex.mIsect.mGeometryNormal, uint32(p), uint32(v) };
				mKeys[index] = Bucket(x, y, z) << 32 | uint64(index);
				++index;
			}
		}
	});
	if (!valid.load()) {
		Clear();
		return Fail("VCM photon grid coordinates exceed its cell range.");
	}
	// Keys are unique, so the sorted order, and every query's order, is deterministic.
	tbb::parallel_sort(mKeys.begin(), mKeys.end());
	mPhotons.resize(count);
	tbb::parallel_for(tbb::blocked_range<size_t>(0, count), [&](const tbb::blocked_range<size_t>& range) {
		for (size_t i = range.begin(); i != range.end(); ++i) mPhotons[i] = mUnsorted[size_t(mKeys[i] & 0xffffffffu)];
	});
	mBucketStart.assign(buckets + 1, 0);
	for (size_t i = 0; i < count; ++i) ++mBucketStart[size_t(mKeys[i] >> 32) + 1];
	for (size_t b = 0; b < buckets; ++b) mBucketStart[b + 1] += mBucketStart[b];
	mCount = count;
	return true;
}

void PhotonGrid::Query(const Vec3& position, const Vec3& geometricNormal, std::vector<PhotonRecord>& result) const {
	result.clear();
	if (!mCount) return;
	const double reach = double(mRadius) * QueryMargin;
	int64 low[3], high[3];
	for (int axis = 0; axis < 3; ++axis) {
		if (!Cell(double(position[axis]) - reach, &low[axis]) || !Cell(double(position[axis]) + reach, &high[axis]))
			return; // Beyond every stored photon's cell range.
	}
	// The range spans at most three cells per axis; distinct cells may share a bucket, visited once.
	uint64 visited[27];
	int visitedCount = 0;
	const double radius2 = double(mRadius) * mRadius;
	for (int64 x = low[0]; x <= high[0]; ++x) for (int64 y = low[1]; y <= high[1]; ++y) for (int64 z = low[2]; z <= high[2]; ++z) {
		const uint64 bucket = Bucket(x, y, z);
		bool seen = false;
		for (int i = 0; i < visitedCount && !seen; ++i) seen = visited[i] == bucket;
		if (seen) continue;
		visited[visitedCount++] = bucket;
		for (uint32 i = mBucketStart[bucket]; i < mBucketStart[bucket + 1]; ++i) {
			const Photon& photon = mPhotons[i];
			if (WithinRadius(position, photon.mPosition, radius2) && CompatibleNormals(geometricNormal, photon.mNormal))
				result.push_back({ photon.mLightPathIndex, photon.mVertexIndex });
		}
	}
}

}

GYT_NAMESPACE_END
