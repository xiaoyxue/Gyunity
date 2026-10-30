#include "gtest/gtest.h"
#include "integrator/VCMPhotonGrid.h"
#include "integrator/BDPTPath.h"
#include "accelerator/BruteForceAccel.h"
#include "bsdf/BSDF.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "shape/Triangle.h"
#include "system/Memory.h"
#include "texture/ConstantTexture.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <random>
#include <thread>

using namespace Gyunity;

namespace {

using Paths = std::vector<std::vector<PathVertex>>;
using Records = std::vector<Vcm::PhotonRecord>;
static_assert(sizeof(Vcm::PhotonRecord) == 2 * sizeof(size_t), "Photon records contain only stable IDs.");
const Vec3 Up(0, 0, 1);

Records Sorted(Records records) {
	std::sort(records.begin(), records.end());
	return records;
}

// Independent exhaustive oracle of Eq. (12), sharing no production eligibility, filter or hash
// helpers: double squared distance of the stored float inputs against the float radius, and the
// dot product of separately normalized oriented geometric normals >= 0.9. In (path, vertex) order.
Records BruteForceRecords(const Paths& paths, real radius, const Vec3& point, const Vec3& normal) {
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

size_t StoredCount(const Paths& paths) {
	size_t count = 0;
	for (const auto& path : paths)
		for (const auto& vertex : path)
			count += vertex.mType == PathVertexType::Surface && !vertex.mIsect.mBsdf->IsDelta();
	return count;
}

class VCMPhotonGrid : public testing::Test {
protected:
	MemoryPool arena;
	PointLight emitter{ Vec3(0, 0, 3), Vec3(1) };
	Vcm::PhotonGrid grid;
	Records result;
	std::mt19937 random{ 0x9a1d };
	int oracleQueries = 0;

	void TearDown() override { RecordProperty("brute_force_queries", oracleQueries); }

	enum class Lobes { Diffuse, Delta, Mixed };

	PathVertex Surface(Vec3 position = Vec3(), Vec3 normal = Up, Lobes lobes = Lobes::Diffuse) {
		PathVertex vertex;
		auto& hit = vertex.mIsect;
		hit.mPos = position;
		hit.mGeometryNormal = normal;
		const double length = std::sqrt(double(normal.x) * normal.x +
			double(normal.y) * normal.y + double(normal.z) * normal.z);
		hit.mNormal = Vec3(real(normal.x / length), real(normal.y / length), real(normal.z / length));
		hit.mAbsNormal = hit.mNormal;
		hit.mOutDir = Up;
		CoordinateSystem(hit.mNormal, &hit.mShadingDpDu, &hit.mShadingDpDv);
		hit.mDpDu = hit.mShadingDpDu;
		hit.mDpDv = hit.mShadingDpDv;
		hit.mUv = Vec2(0.2f, 0.7f);
		hit.mPointError = Vec3(0.00001f);
		hit.mB1 = 0.2f; hit.mB2 = 0.7f;
		hit.mShapeId = 13; hit.mPrimId = 17; hit.mPrimitive = nullptr;
		hit.mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(hit);
		if (lobes == Lobes::Diffuse || lobes == Lobes::Mixed)
			hit.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(Vec3(0.8f)));
		if (lobes == Lobes::Delta || lobes == Lobes::Mixed)
			hit.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(1)));
		vertex.mThroughput = Vec3(0.5f, 2, 3);
		vertex.mDirection = Vec3(1, 0, 0);
		vertex.mPdfFwd = 0.2f; vertex.mPdfPrev = 0.3f;
		return vertex;
	}

	PathVertex Endpoint(PathVertexType type = PathVertexType::Light) {
		PathVertex endpoint;
		endpoint.mType = type;
		endpoint.mLight = &emitter;
		return endpoint;
	}

	real Uniform(real low, real high) { return std::uniform_real_distribution<real>(low, high)(random); }

	// Photons on a few planes and a sphere, with occasional delta vertices and tilted normals.
	Paths RandomPaths(int count, real extent) {
		Paths paths(count);
		for (auto& path : paths) {
			path.push_back(Endpoint());
			const int length = int(random() % 6);
			for (int i = 0; i < length; ++i) {
				Vec3 position(Uniform(-extent, extent), Uniform(-extent, extent), Uniform(-extent, extent));
				Vec3 normal = Up;
				switch (random() % 4) {
				case 0: position.z = 0; break;
				case 1: position.x = real(0.25) * extent; normal = Vec3(-1, 0, 0); break;
				case 2: normal = Vec3(Uniform(-1, 1), Uniform(-1, 1), Uniform(0.3f, 1)); break;
				default: normal = position.Length2() > 0 ? position.Norm() : Up; break;
				}
				path.push_back(Surface(position, normal, random() % 7 == 0 ? Lobes::Delta : Lobes::Diffuse));
			}
		}
		return paths;
	}

	// Queries the grid and leaves its records in `result`, sorted for comparison. Each photon is
	// stored once, so no query returns a record twice even when its cells share a bucket.
	void Check(const Paths& paths, real radius, const Vec3& point, const Vec3& normal = Up) {
		result = { { 999, 999 } };
		grid.Query(point, normal, result);
		result = Sorted(result);
		++oracleQueries;
		EXPECT_EQ(std::adjacent_find(result.begin(), result.end()), result.end()) << "duplicate photon";
		EXPECT_EQ(result, BruteForceRecords(paths, radius, point, normal));
	}
};

}

TEST_F(VCMPhotonGrid, EmptyBuildAndNoEligibleVerticesAreUsable) {
	Paths empty;
	ASSERT_TRUE(grid.Build(empty, 0.25f));
	EXPECT_EQ(grid.RecordCount(), 0u);
	EXPECT_EQ(grid.Radius(), 0.25f);
	Check(empty, 0.25f, Vec3());
	Paths skipped{ { Endpoint(), Endpoint(PathVertexType::Camera),
		Endpoint(PathVertexType::Environment), Surface(Vec3(), Up, Lobes::Delta) } };
	ASSERT_TRUE(grid.Build(skipped, 2));
	EXPECT_EQ(grid.RecordCount(), 0u);
	Check(skipped, 2, Vec3(-100));
}

TEST_F(VCMPhotonGrid, FirstSurfaceAndEmittingScatteringSurfaceAreIncluded) {
	Paths paths{ { Endpoint(), Surface() }, { Endpoint(), Surface(Vec3(0.5f, 0, 0)) } };
	paths[1][1].mLight = &emitter;
	ASSERT_TRUE(grid.Build(paths, 1));
	EXPECT_EQ(grid.RecordCount(), 2u);
	Check(paths, 1, Vec3());
	EXPECT_EQ(result, (Records{ { 0, 1 }, { 1, 1 } }));
}

TEST_F(VCMPhotonGrid, SampledAndUnsampledTerminalPureDeltaAreExcluded) {
	Paths paths{ { Endpoint(), Surface(Vec3(), Up, Lobes::Delta), Surface(),
		Surface(Vec3(), Up, Lobes::Delta), Surface(Vec3(), Up, Lobes::Mixed) } };
	paths[0][1].mIsDelta = true;
	ASSERT_FALSE(paths[0][3].mIsDelta);
	ASSERT_TRUE(grid.Build(paths, 1));
	EXPECT_EQ(grid.RecordCount(), 2u);
	Check(paths, 1, Vec3());
	EXPECT_EQ(result, (Records{ { 0, 2 }, { 0, 4 } }));
}

TEST_F(VCMPhotonGrid, CoincidentRecordsRetainDistinctIdentities) {
	Paths paths(25);
	for (auto& path : paths) path = { Endpoint(), Surface(), Surface() };
	ASSERT_TRUE(grid.Build(paths, 1));
	EXPECT_EQ(grid.RecordCount(), 50u);
	Check(paths, 1, Vec3());
	EXPECT_EQ(result.size(), 50u);
}

TEST_F(VCMPhotonGrid, BucketCollisionsStillUseExactDistance) {
	// Two photons give a two-bucket table, so every visited cell shares a bucket with others.
	Paths paths{ { Endpoint(), Surface() }, { Endpoint(), Surface(Vec3(8, 0, 0)) } };
	ASSERT_TRUE(grid.Build(paths, 1));
	for (int x = -10; x <= 20; ++x) Check(paths, 1, Vec3(real(x) / 2, 0, 0));
}

TEST_F(VCMPhotonGrid, AdjacentCoplanarPrimitivesDoNotCreateSeams) {
	auto material = std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
	auto a = std::make_shared<Primitive>(std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-1, -1, 0), Vec3(1, -1, 0), Vec3(1, 1, 0), Up), material);
	auto b = std::make_shared<Primitive>(std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-1, -1, 0), Vec3(1, 1, 0), Vec3(-1, 1, 0), Up), material);
	Paths paths{ { Endpoint(), Surface(Vec3(0.01f, -0.01f, 0)) },
		{ Endpoint(), Surface(Vec3(-0.01f, 0.01f, 0)) } };
	paths[0][1].mIsect.mPrimitive = a.get();
	paths[1][1].mIsect.mPrimitive = b.get();
	paths[1][1].mIsect.mPrimId = 18;
	paths[1][1].mIsect.mShapeId = 14;
	ASSERT_TRUE(grid.Build(paths, 0.1f));
	Check(paths, 0.1f, Vec3());
	EXPECT_EQ(result.size(), 2u);
}

TEST_F(VCMPhotonGrid, OrientedGeometricNormalCompatibility) {
	Paths paths{ { Endpoint(), Surface(Vec3(), Up), Surface(Vec3(), -Up),
		Surface(Vec3(), Vec3(1, 0, 0)), Surface(Vec3(), Vec3(0.3f, 0, 1)),
		Surface(Vec3(), Vec3(0.6f, 0, 1)), Surface(Vec3(), Up * 7) } };
	ASSERT_TRUE(grid.Build(paths, 1));
	Check(paths, 1, Vec3());
	EXPECT_EQ(result, (Records{ { 0, 1 }, { 0, 4 }, { 0, 6 } }));
	for (const Vec3 normal : { Up * 19, -Up, Vec3(0.43589f, 0, 0.9f), Vec3(1, 0, 0) })
		Check(paths, 1, Vec3(), normal);
}

TEST_F(VCMPhotonGrid, ExactRadiusNextafterAndOutsideBounds) {
	for (real offset : { -100.0f, -1.0f, 0.0f, 1.0f, 100.0f }) {
		Paths paths{ { Endpoint(), Surface(Vec3(offset)) } };
		const real radius = 0.5f;
		ASSERT_TRUE(grid.Build(paths, radius));
		for (int axis = 0; axis < 3; ++axis) for (real sign : { -1.0f, 1.0f }) {
			Vec3 boundary(offset);
			boundary[axis] += sign * radius;
			Check(paths, radius, boundary);
			EXPECT_EQ(result.size(), 1u);
			Vec3 inside = boundary, outside = boundary;
			inside[axis] = std::nextafter(boundary[axis], offset);
			outside[axis] = std::nextafter(boundary[axis], sign * std::numeric_limits<real>::infinity());
			Check(paths, radius, inside);
			EXPECT_EQ(result.size(), 1u);
			Check(paths, radius, outside);
			EXPECT_TRUE(result.empty());
		}
		Check(paths, radius, Vec3(1000000));
		Check(paths, radius, Vec3(-1000000));
	}
}

TEST_F(VCMPhotonGrid, RandomNegativeWorldQueriesMatchBruteForce) {
	std::mt19937 rng(128);
	std::uniform_real_distribution<real> coordinate(-20, -10);
	std::uniform_real_distribution<real> tilt(-0.7f, 0.7f);
	Paths paths(31);
	for (auto& path : paths) {
		path.push_back(Endpoint());
		for (int vertex = 0; vertex < 7; ++vertex) {
			const Vec3 point(coordinate(rng), coordinate(rng), coordinate(rng));
			const Vec3 normal(tilt(rng), tilt(rng), 1);
			path.push_back(Surface(point, normal));
		}
	}
	ASSERT_TRUE(grid.Build(paths, 2.25f));
	for (int query = 0; query < 4096; ++query) {
		const Vec3 point(coordinate(rng), coordinate(rng), coordinate(rng));
		Check(paths, 2.25f, point, Vec3(tilt(rng), tilt(rng), 1));
	}
}

TEST_F(VCMPhotonGrid, RandomScenesMatchBruteForceAtRadiusAndCellBoundaries) {
	for (real radius : { 0.05f, 0.3f, 1.7f }) {
		SCOPED_TRACE(radius);
		const Paths paths = RandomPaths(400, 3);
		ASSERT_TRUE(grid.Build(paths, radius));
		EXPECT_EQ(grid.RecordCount(), StoredCount(paths));
		EXPECT_GT(grid.RecordCount(), 500u);
		EXPECT_EQ(grid.Radius(), radius);
		for (int i = 0; i < 300; ++i)
			Check(paths, radius, Vec3(Uniform(-3, 3), Uniform(-3, 3), i % 2 ? 0 : Uniform(-3, 3)));
		// Queries one radius away from photons, just inside and outside, and on cell walls.
		size_t accepted = 0;
		for (const auto& path : paths) {
			for (size_t v = 1; v < path.size(); ++v) {
				const Vec3& position = path[v].mIsect.mPos;
				const Vec3& normal = path[v].mIsect.mGeometryNormal;
				for (real scale : { real(0), real(0.999999), real(1), real(1.000001) }) {
					const Vec3 offset(radius * scale, 0, 0);
					Check(paths, radius, position + offset, normal);
					Check(paths, radius, position - offset, normal);
				}
				Check(paths, radius, Vec3(std::round(position.x / (2 * radius)) * 2 * radius, position.y, position.z), normal);
				Check(paths, radius, position, normal);
				accepted += result.size();
			}
		}
		// A query at a photon with its own normal accepts at least that photon.
		EXPECT_GE(accepted, grid.RecordCount());
	}
}

TEST_F(VCMPhotonGrid, BoundarySweepsAcrossPhysicalScalesMatchBruteForce) {
	for (real radius : { 1e-30f, 1e-20f, 1e-5f, 0.1f, 1.0f, 1e10f, 1e20f, 1e30f }) {
		Paths paths(17);
		for (size_t p = 0; p < paths.size(); ++p)
			paths[p] = { Endpoint(), Surface(Vec3((real(p) - 8.5f) * radius,
				(real(p % 3) - 2.3f) * radius, -3.7f * radius)) };
		ASSERT_TRUE(grid.Build(paths, radius));
		for (const auto& path : paths) {
			for (int axis = 0; axis < 3; ++axis) for (real sign : { -1.0f, 1.0f }) {
				Vec3 boundary = path[1].mIsect.mPos;
				boundary[axis] += sign * radius;
				Check(paths, radius, boundary);
				boundary[axis] = std::nextafter(boundary[axis], -std::numeric_limits<real>::infinity());
				Check(paths, radius, boundary);
				boundary[axis] = std::nextafter(std::nextafter(boundary[axis],
					std::numeric_limits<real>::infinity()), std::numeric_limits<real>::infinity());
				Check(paths, radius, boundary);
			}
			Check(paths, radius, path[1].mIsect.mPos + Vec3(0.6f * radius, 0.8f * radius, 0));
		}
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
		first.Query(point, Up, a);
		second.Query(point, Up, b);
		EXPECT_EQ(a, b);
	}
}

TEST_F(VCMPhotonGrid, RebuildDisjointThenEmptyClearsIdsRadiusAndBuckets) {
	Paths first{ { Endpoint(), Surface() } };
	Paths second{ {}, {}, { Endpoint(), Surface(Vec3(-100)), Surface(Vec3(-102)) } };
	Paths empty;
	ASSERT_TRUE(grid.Build(first, 1));
	Check(first, 1, Vec3());
	ASSERT_TRUE(grid.Build(second, 0.25f));
	EXPECT_EQ(grid.RecordCount(), 2u);
	EXPECT_EQ(grid.Radius(), 0.25f);
	Check(second, 0.25f, Vec3());
	EXPECT_TRUE(result.empty());
	Check(second, 0.25f, Vec3(-100));
	EXPECT_EQ(result, (Records{ { 2, 1 } }));
	ASSERT_TRUE(grid.Build(empty, 7));
	EXPECT_EQ(grid.RecordCount(), 0u);
	EXPECT_EQ(grid.Radius(), 7);
	Check(empty, 7, Vec3(-100));
	grid.Clear();
	EXPECT_EQ(grid.Radius(), 0);
	grid.Query(Vec3(), Up, result);
	EXPECT_TRUE(result.empty());
}

TEST_F(VCMPhotonGrid, SmallerRebuildReusesStorageWithoutStalePhotons) {
	const Paths large = RandomPaths(500, 2), small = RandomPaths(20, 2);
	ASSERT_TRUE(grid.Build(large, 0.5f));
	EXPECT_GT(grid.RecordCount(), 0u);
	grid.Clear();
	EXPECT_EQ(grid.RecordCount(), 0u);
	grid.Query(Vec3(), Up, result);
	EXPECT_TRUE(result.empty());
	ASSERT_TRUE(grid.Build(small, 0.5f));
	EXPECT_EQ(grid.RecordCount(), StoredCount(small));
	for (int i = 0; i < 100; ++i) Check(small, 0.5f, Vec3(Uniform(-2, 2), Uniform(-2, 2), 0));
}

TEST_F(VCMPhotonGrid, ExtremeFiniteWorldInputsDoNotOverflowDistanceOrCellCasts) {
	const real largest = (std::numeric_limits<real>::max)();
	// The largest float is 2^127 cells of radius 0.5 or less from the origin: it cannot be indexed.
	for (real radius : { std::numeric_limits<real>::denorm_min(), (std::numeric_limits<real>::min)(), 1.0f }) {
		Paths paths{ { Endpoint(), Surface(Vec3(largest, 0, 0)) } };
		EXPECT_FALSE(grid.Build(paths, radius));
		EXPECT_EQ(grid.RecordCount(), 0u);
		grid.Query(Vec3(largest, 0, 0), Up, result);
		EXPECT_TRUE(result.empty());
	}
	Paths paths{ { Endpoint(), Surface(Vec3(largest, 0, 0)) } };
	ASSERT_TRUE(grid.Build(paths, largest));
	EXPECT_EQ(grid.Radius(), largest);
	Check(paths, largest, Vec3(largest, 0, 0));
	EXPECT_EQ(result.size(), 1u);
	Check(paths, largest, Vec3(-largest, 0, 0));
	EXPECT_TRUE(result.empty());
	Paths extremes{ { Endpoint(), Surface(Vec3(largest, 0, 0)), Surface(Vec3(-largest, 0, 0)),
		Surface(Vec3(largest, largest, largest)) } };
	ASSERT_TRUE(grid.Build(extremes, largest));
	Check(extremes, largest, Vec3());
	EXPECT_EQ(result.size(), 2u);
	EXPECT_FALSE(grid.Build(extremes, 1));
	EXPECT_EQ(grid.RecordCount(), 0u);
	Paths narrow{ { Surface(), Surface(Vec3(std::numeric_limits<real>::denorm_min(), 0, 0)) } };
	ASSERT_TRUE(grid.Build(narrow, std::numeric_limits<real>::denorm_min()));
	Check(narrow, std::numeric_limits<real>::denorm_min(), Vec3());
	EXPECT_EQ(result.size(), 2u);
	for (real scale : { std::numeric_limits<real>::denorm_min(), largest }) {
		Paths normals{ { Surface(Vec3(), Up * scale) } };
		ASSERT_TRUE(grid.Build(normals, 1));
		Check(normals, 1, Vec3(), Up * scale);
		EXPECT_EQ(result.size(), 1u);
	}
}

TEST_F(VCMPhotonGrid, CellCoordinateLimitIsExplicit) {
	// With radius 0.5 a cell is one unit wide, so a coordinate is its own cell index; the largest
	// float below 2^62 is indexable and 2^62 itself is not.
	const real limit = 4611686018427387904.0f; // 2^62
	for (real sign : { -1.0f, 1.0f }) {
		const real edge = sign * std::nextafter(limit, 0.0f);
		Paths paths{ { Endpoint(), Surface(), Surface(Vec3(edge, 0, 0)) } };
		ASSERT_TRUE(grid.Build(paths, 0.5f));
		Check(paths, 0.5f, Vec3(edge, 0, 0));
		EXPECT_EQ(result.size(), 1u);
		Check(paths, 0.5f, Vec3());
		EXPECT_EQ(result.size(), 1u);
		// A query beyond the cell range accepts nothing rather than wrapping around.
		Check(paths, 0.5f, Vec3(sign * limit, 0, 0));
		paths[0][2].mIsect.mPos.x = sign * limit;
		EXPECT_FALSE(grid.Build(paths, 0.5f));
		EXPECT_EQ(grid.RecordCount(), 0u);
	}
}

TEST_F(VCMPhotonGrid, RejectsInvalidRadii) {
	const Paths paths = RandomPaths(10, 1);
	for (real radius : { real(0), real(-1), std::numeric_limits<real>::infinity(), std::numeric_limits<real>::quiet_NaN() }) {
		EXPECT_FALSE(grid.Build(paths, radius));
		EXPECT_EQ(grid.RecordCount(), 0u);
	}
}

TEST_F(VCMPhotonGrid, ConcurrentReadOnlyQueriesOwnSeparateOutputs) {
	Paths paths(23);
	for (size_t p = 0; p < paths.size(); ++p)
		paths[p] = { Endpoint(), Surface(Vec3(real(p) / 8 - 2, 0, 0)), Surface(Vec3()) };
	ASSERT_TRUE(grid.Build(paths, 0.25f));
	std::vector<Vec3> points;
	std::vector<Records> expected;
	for (int i = 0; i < 512; ++i) {
		points.push_back(Vec3(real(i) / 128 - 2, 0, 0));
		expected.push_back(BruteForceRecords(paths, 0.25f, points.back(), Up));
	}
	std::array<Records, 8> outputs;
	std::array<int, 8> failures{};
	std::vector<std::thread> workers;
	for (size_t worker = 0; worker < outputs.size(); ++worker) {
		workers.emplace_back([&, worker]() {
			for (size_t q = 0; q < points.size(); ++q) {
				grid.Query(points[q], Up, outputs[worker]);
				if (Sorted(outputs[worker]) != expected[q]) ++failures[worker];
			}
		});
	}
	for (auto& worker : workers) worker.join();
	oracleQueries += int(outputs.size() * points.size());
	for (int failure : failures) EXPECT_EQ(failure, 0);
}

TEST_F(VCMPhotonGrid, GeneratedPathsAndArenaOwnedDataRemainUnchanged) {
	auto scene = std::make_shared<Scene>();
	auto material = std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(0.7f)));
	scene->AddPrimitive(std::make_shared<Primitive>(
		std::make_shared<Triangle>(Transform(), Transform(),
			Vec3(-100, -100, 0), Vec3(100, -100, 0), Vec3(0, 100, 0), Up), material));
	scene->AddLight(std::make_shared<PointLight>(Vec3(0, 0, 1), Vec3(3)));
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	scene->Initialize();
	class Samples : public StateSequence {
		real Sample() override { ++mCursor; return 0.75f; }
	} samples;
	Paths paths(8);
	for (auto& path : paths) ASSERT_GE(GenerateLightPath(*scene, samples, arena, 4, SurfaceNormalMode::Geometric, path), 2);
	const Paths snapshot = paths;
	const auto* storage = paths.data();
	std::vector<const PathVertex*> pathStorage;
	for (const auto& path : paths) pathStorage.push_back(path.data());
	const size_t allocated = arena.TotalAllocated();
	ASSERT_TRUE(grid.Build(paths, 1));
	for (const auto& path : paths) Check(paths, 1, path[1].mIsect.mPos);
	EXPECT_EQ(arena.TotalAllocated(), allocated);
	EXPECT_EQ(paths.data(), storage);
	for (size_t p = 0; p < paths.size(); ++p) {
		EXPECT_EQ(paths[p].data(), pathStorage[p]);
		ASSERT_EQ(paths[p].size(), snapshot[p].size());
		for (size_t v = 0; v < paths[p].size(); ++v) {
			const auto& before = snapshot[p][v];
			const auto& after = paths[p][v];
			EXPECT_EQ(after.mIsect.mPos, before.mIsect.mPos);
			EXPECT_EQ(after.mIsect.mNormal, before.mIsect.mNormal);
			EXPECT_EQ(after.mIsect.mGeometryNormal, before.mIsect.mGeometryNormal);
			EXPECT_EQ(after.mIsect.mOutDir, before.mIsect.mOutDir);
			EXPECT_EQ(after.mIsect.mBsdf, before.mIsect.mBsdf);
			EXPECT_EQ(after.mThroughput, before.mThroughput);
			EXPECT_EQ(after.mDirection, before.mDirection);
			EXPECT_EQ(after.mPdfFwd, before.mPdfFwd);
			EXPECT_EQ(after.mPdfPrev, before.mPdfPrev);
			EXPECT_EQ(after.mPdfPos, before.mPdfPos);
			EXPECT_EQ(after.mPdfDir, before.mPdfDir);
			EXPECT_EQ(after.mLightSelectionPdf, before.mLightSelectionPdf);
			EXPECT_EQ(after.mIsDelta, before.mIsDelta);
			EXPECT_EQ(after.mType, before.mType);
			EXPECT_EQ(after.mLight, before.mLight);
			EXPECT_EQ(after.mCamera, before.mCamera);
			if (v) {
				EXPECT_EQ(after.mIsect.mPrimitive, before.mIsect.mPrimitive);
				EXPECT_EQ(after.mIsect.mBsdf->Evaluate(Up, Up), Vec3(0.7f * INV_PI));
			}
		}
	}
	grid.Clear();
	arena.Reset();
}
