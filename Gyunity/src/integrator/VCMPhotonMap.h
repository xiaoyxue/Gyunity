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

// Spatial index borrowing immutable light paths and their BSDF arenas.
// Keep them alive through the last query; Build/Clear must not race readers.
class PhotonMap {
public:
	// Rebuild clears old IDs. Failure leaves an unbuilt empty map; empty success is valid.
	// Checked grid limits and numerical details are documented in Docs/VCM.md.
	bool Build(const std::vector<std::vector<PathVertex>>& lightPaths, real radius);
	void Clear();

	// Clears output and returns unique IDs in (path, vertex) order; distinct photons stay distinct.
	// Physical acceptance is distance^2 <= radius^2 and oriented normal cosine >= .9.
	// Each concurrent reader supplies its own output/scratch vector.
	bool Query(const Vec3& position, const Vec3& geometricNormal,
		std::vector<PhotonRecord>& result) const;

	bool IsBuilt() const { return mBuilt; }
	size_t RecordCount() const { return mRecordCount; }
	real Radius() const { return mRadius; }
	// Batch identity, not a content comparison.
	bool References(const std::vector<std::vector<PathVertex>>& lightPaths) const {
		return mBuilt && mLightPaths == &lightPaths;
	}

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
