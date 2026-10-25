#pragma once

#include "HashGrid.h"
#include "PathVertex.h"

GYT_NAMESPACE_BEGIN

namespace Vcm {

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

// Spatial index only: no estimator, MIS, photon-count or kernel-area normalization.
// Borrows complete paths, their BSDF arenas, and referenced scene objects. All must
// remain alive and immutable from Build through the last Query; do not reset an
// arena or move/resize the paths. IDs belong to that generation, not a later build.
// Build/Clear/destruction require exclusive ownership and must never race queries.
class PhotonMap {
public:
	// Every rebuild first clears the old generation. Failure leaves IsBuilt false,
	// Radius/RecordCount zero and queries failing; a valid empty build is distinct.
	bool Build(const std::vector<std::vector<PathVertex>>& lightPaths, real radius);
	void Clear();

	// Caller-owned output is also the deduplication scratch; use a separate vector
	// per concurrent reader. Clears output even on failure. Returns unique IDs in
	// (path, vertex) order, retaining distinct coincident photons.
	// Acceptance: double-precision squared distance <= radius^2 and dot of
	// normalized geometric normals >= 0.9 (oriented, not absolute). No primitive
	// identity, visibility ray, geometry term, BSDF evaluation or path mutation.
	bool Query(const Vec3& position, const Vec3& geometricNormal,
		std::vector<PhotonRecord>& result) const;

	bool IsBuilt() const { return mBuilt; }
	size_t RecordCount() const { return mRecordCount; }
	real Radius() const { return mRadius; }

private:
	HashGrid<PhotonRecord> mGrid;
	const std::vector<std::vector<PathVertex>>* mLightPaths = nullptr;
	Vec3 mOrigin;
	AABB mGridBounds;
	real mRadius = 0;
	size_t mRecordCount = 0;
	bool mBuilt = false;
};

}

GYT_NAMESPACE_END
