#pragma once

#include "VCMPhotonMap.h"

GYT_NAMESPACE_BEGIN

namespace Vcm {

// SmallVCM-style photon grid of one light-path batch (Docs/VCM.md, "Photon grid"), used by
// RecursiveVertexConnectionMerging. It has PhotonMap's interface and accepts exactly the same
// photons by Eq. (12); only the order of a query's results differs. Cells are 2r wide and each
// photon is stored once, with its position and normal, in one array sorted by bucket.
// It borrows nothing from the paths after Build, but its records index them.
class PhotonGrid {
public:
	bool Build(const std::vector<std::vector<PathVertex>>& lightPaths, real radius);
	void Clear();

	// Photons with distance^2 <= radius^2 and oriented geometric-normal cosine >= .9, in bucket
	// order, deterministic for a given build. Each concurrent reader supplies its own result vector.
	void Query(const Vec3& position, const Vec3& geometricNormal, std::vector<PhotonRecord>& result) const;

	size_t RecordCount() const { return mCount; }
	real Radius() const { return mRadius; }

private:
	struct Photon {
		Vec3 mPosition, mNormal;
		uint32 mLightPathIndex, mVertexIndex;
	};

	bool Cell(double coordinate, int64* cell) const;
	uint64 Bucket(int64 x, int64 y, int64 z) const;

	// Storage keeps its capacity across Clear, so later iterations do not reallocate.
	std::vector<size_t> mPathOffsets;   // First photon of each light path, in path order
	std::vector<Photon> mUnsorted;      // Photons in (path, vertex) order
	std::vector<uint64> mKeys;          // bucket << 32 | index into mUnsorted
	std::vector<Photon> mPhotons;       // Photons sorted by key
	std::vector<uint32> mBucketStart;   // mPhotons range of bucket b: [start[b], start[b + 1])
	size_t mCount = 0;
	uint64 mBucketMask = 0;
	double mInvCellSize = 0;
	real mRadius = 0;
};

}

GYT_NAMESPACE_END
