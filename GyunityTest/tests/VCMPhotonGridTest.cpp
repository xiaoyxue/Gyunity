#include "gtest/gtest.h"
#include "integrator/VCMPhotonGrid.h"
#include "integrator/VCMPhotonMap.h"
#include "bsdf/BSDF.h"
#include "system/Memory.h"
#include <algorithm>
#include <cmath>
#include <random>

using namespace Gyunity;

namespace {

using Paths = std::vector<std::vector<PathVertex>>;
using Records = std::vector<Vcm::PhotonRecord>;

Records Sorted(Records records) {
	std::sort(records.begin(), records.end());
	return records;
}

// Independent exhaustive oracle of Eq. (12): double squared distance against the float radius,
// and the cosine of separately normalized geometric normals >= 0.9.
Records BruteForce(const Paths& paths, real radius, const Vec3& point, const Vec3& normal) {
	Records expected;
	const double queryLength = std::sqrt(double(normal.x) * normal.x + double(normal.y) * normal.y +
		double(normal.z) * normal.z);
	for (size_t p = 0; p < paths.size(); ++p) {
		for (size_t v = 0; v < paths[p].size(); ++v) {
			const auto& vertex = paths[p][v];
			if (vertex.mType != PathVertexType::Surface || vertex.mIsect.mBsdf->IsDelta()) continue;
			const auto& hit = vertex.mIsect;
			const double x = double(point.x) - hit.mPos.x, y = double(point.y) - hit.mPos.y,
				z = double(point.z) - hit.mPos.z;
			const double nx = hit.mGeometryNormal.x, ny = hit.mGeometryNormal.y, nz = hit.mGeometryNormal.z;
			const double photonLength = std::sqrt(nx * nx + ny * ny + nz * nz);
			const double cosine = (double(normal.x) / queryLength) * (nx / photonLength) +
				(double(normal.y) / queryLength) * (ny / photonLength) + (double(normal.z) / queryLength) * (nz / photonLength);
			if (x * x + y * y + z * z <= double(radius) * radius && cosine >= 0.9) expected.push_back({ p, v });
		}
	}
	return expected;
}

class VCMPhotonGrid : public testing::Test {
protected:
	MemoryPool arena;
	std::mt19937 random{ 0x9a1d };

	PathVertex Surface(const Vec3& position, const Vec3& normal, bool delta = false) {
		PathVertex vertex;
		auto& hit = vertex.mIsect;
		hit.mPos = position;
		hit.mGeometryNormal = normal;
		hit.mNormal = normal.Norm();
		hit.mOutDir = normal.Norm();
		CoordinateSystem(hit.mNormal, &hit.mShadingDpDu, &hit.mShadingDpDv);
		hit.mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(hit);
		if (delta) hit.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(1)));
		else hit.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(Vec3(0.8f)));
		return vertex;
	}

	PathVertex Origin() {
		PathVertex vertex;
		vertex.mType = PathVertexType::Light;
		return vertex;
	}

	real Uniform(real low, real high) { return std::uniform_real_distribution<real>(low, high)(random); }

	// Photons on a few planes and a sphere, with occasional delta vertices and tilted normals.
	Paths RandomPaths(int count, real extent) {
		Paths paths(count);
		for (auto& path : paths) {
			path.push_back(Origin());
			const int length = int(random() % 6);
			for (int i = 0; i < length; ++i) {
				Vec3 position(Uniform(-extent, extent), Uniform(-extent, extent), Uniform(-extent, extent));
				Vec3 normal(0, 0, 1);
				switch (random() % 4) {
				case 0: position.z = 0; break;
				case 1: position.x = real(0.25) * extent; normal = Vec3(-1, 0, 0); break;
				case 2: normal = Vec3(Uniform(-1, 1), Uniform(-1, 1), Uniform(0.3f, 1)); break;
				default: normal = position.Length2() > 0 ? position.Norm() : Vec3(0, 0, 1); break;
				}
				path.push_back(Surface(position, normal, random() % 7 == 0));
			}
		}
		return paths;
	}

	void ExpectSameAsReference(const Paths& paths, real radius, const Vec3& point, const Vec3& normal,
		const Vcm::PhotonGrid& grid, const Vcm::PhotonMap& map) {
		Records actual, expected;
		grid.Query(point, normal, actual);
		map.Query(point, normal, expected);
		const Records sorted = Sorted(actual);
		EXPECT_EQ(std::adjacent_find(sorted.begin(), sorted.end()), sorted.end()) << "duplicate photon";
		EXPECT_EQ(sorted, expected);
		EXPECT_EQ(sorted, BruteForce(paths, radius, point, normal));
	}
};

}

TEST_F(VCMPhotonGrid, AcceptsExactlyThePhotonMapSetAtRandomAndBoundaryQueries) {
	for (real radius : { 0.05f, 0.3f, 1.7f }) {
		SCOPED_TRACE(radius);
		const Paths paths = RandomPaths(400, 3);
		Vcm::PhotonGrid grid;
		Vcm::PhotonMap map;
		ASSERT_TRUE(grid.Build(paths, radius));
		ASSERT_TRUE(map.Build(paths, radius));
		EXPECT_EQ(grid.RecordCount(), map.RecordCount());
		EXPECT_EQ(grid.Radius(), radius);
		int accepted = 0;
		for (int i = 0; i < 300; ++i) {
			const Vec3 point(Uniform(-3, 3), Uniform(-3, 3), i % 2 ? 0 : Uniform(-3, 3));
			ExpectSameAsReference(paths, radius, point, Vec3(0, 0, 1), grid, map);
		}
		// Queries placed one radius away from photons, just inside and outside, and on cell walls.
		for (const auto& path : paths) {
			for (size_t v = 1; v < path.size(); ++v) {
				const Vec3& position = path[v].mIsect.mPos;
				const Vec3& normal = path[v].mIsect.mGeometryNormal;
				for (real scale : { real(0), real(0.999999), real(1), real(1.000001) }) {
					const Vec3 offset(radius * scale, 0, 0);
					ExpectSameAsReference(paths, radius, position + offset, normal, grid, map);
					ExpectSameAsReference(paths, radius, position - offset, normal, grid, map);
				}
				const Vec3 wall(std::round(position.x / (2 * radius)) * 2 * radius, position.y, position.z);
				ExpectSameAsReference(paths, radius, wall, normal, grid, map);
				Records records;
				grid.Query(position, normal, records);
				accepted += int(records.size());
			}
		}
		// A query at a photon with its own normal accepts at least that photon.
		EXPECT_GE(accepted, int(grid.RecordCount()));
		EXPECT_GT(grid.RecordCount(), 500u);
	}
}

TEST_F(VCMPhotonGrid, QueryOrderIsDeterministicAcrossBuilds) {
	const Paths paths = RandomPaths(300, 1);
	Vcm::PhotonGrid first, second;
	ASSERT_TRUE(first.Build(paths, 0.4f));
	ASSERT_TRUE(second.Build(paths, 0.4f));
	for (int i = 0; i < 100; ++i) {
		const Vec3 point(Uniform(-1, 1), Uniform(-1, 1), 0);
		Records a, b;
		first.Query(point, Vec3(0, 0, 1), a);
		second.Query(point, Vec3(0, 0, 1), b);
		EXPECT_EQ(a, b);
	}
}

TEST_F(VCMPhotonGrid, EmptyClearedAndRebuiltGrids) {
	Vcm::PhotonGrid grid;
	Records records;
	grid.Query(Vec3(0), Vec3(0, 0, 1), records);
	EXPECT_TRUE(records.empty());
	Paths deltaOnly(3);
	for (auto& path : deltaOnly) { path.push_back(Origin()); path.push_back(Surface(Vec3(0), Vec3(0, 0, 1), true)); }
	ASSERT_TRUE(grid.Build(deltaOnly, 0.5f));
	EXPECT_EQ(grid.RecordCount(), 0u);
	grid.Query(Vec3(0), Vec3(0, 0, 1), records);
	EXPECT_TRUE(records.empty());

	const Paths large = RandomPaths(500, 2), small = RandomPaths(20, 2);
	ASSERT_TRUE(grid.Build(large, 0.5f));
	EXPECT_GT(grid.RecordCount(), 0u);
	grid.Clear();
	EXPECT_EQ(grid.RecordCount(), 0u);
	grid.Query(Vec3(0), Vec3(0, 0, 1), records);
	EXPECT_TRUE(records.empty());
	// A smaller rebuild reuses the storage but must not see the larger build's photons.
	ASSERT_TRUE(grid.Build(small, 0.5f));
	Vcm::PhotonMap map;
	ASSERT_TRUE(map.Build(small, 0.5f));
	for (int i = 0; i < 100; ++i)
		ExpectSameAsReference(small, 0.5f, Vec3(Uniform(-2, 2), Uniform(-2, 2), 0), Vec3(0, 0, 1), grid, map);
}

TEST_F(VCMPhotonGrid, RejectsInvalidRadiiAndUnindexableCoordinates) {
	const Paths paths = RandomPaths(10, 1);
	Vcm::PhotonGrid grid;
	for (real radius : { real(0), real(-1), std::numeric_limits<real>::infinity(), std::numeric_limits<real>::quiet_NaN() })
		EXPECT_FALSE(grid.Build(paths, radius));
	Paths far(1);
	far[0].push_back(Origin());
	far[0].push_back(Surface(Vec3(1e30f, 0, 0), Vec3(0, 0, 1)));
	EXPECT_FALSE(grid.Build(far, 1e-10f));
	EXPECT_EQ(grid.RecordCount(), 0u);
}
