#pragma once

#include "HashGrid.h"
#include "PathVertex.h"

GYT_NAMESPACE_BEGIN

namespace Vcm {

// A mergeable light vertex: lightPaths[mLightPathIndex][mVertexIndex].
struct PhotonRecord {
	size_t mLightPathIndex, mVertexIndex;

	bool operator==(const PhotonRecord& other) const {
		return mLightPathIndex == other.mLightPathIndex && mVertexIndex == other.mVertexIndex;
	}
	bool operator<(const PhotonRecord& other) const {
		return mLightPathIndex < other.mLightPathIndex ||
			(mLightPathIndex == other.mLightPathIndex && mVertexIndex < other.mVertexIndex);
	}
};

// Hash grid over the continuous surface vertices of one light-path batch. It borrows the
// paths; keep them and their BSDF arenas alive through the last query.
class PhotonMap {
public:
	bool Build(const std::vector<std::vector<PathVertex>>& lightPaths, real radius);
	void Clear();

	// Photons with distance^2 <= radius^2 and oriented geometric-normal cosine >= .9,
	// in (path, vertex) order. Each concurrent reader supplies its own result vector.
	void Query(const Vec3& position, const Vec3& geometricNormal, std::vector<PhotonRecord>& result) const;

	size_t RecordCount() const { return mRecordCount; }
	real Radius() const { return mRadius; }

private:
	HashGrid<PhotonRecord> mGrid;
	const std::vector<std::vector<PathVertex>>* mLightPaths = nullptr;
	Vec3 mOrigin;
	AABB mGridBounds;
	real mRadius = 0;
	size_t mRecordCount = 0;
};

}

GYT_NAMESPACE_END
