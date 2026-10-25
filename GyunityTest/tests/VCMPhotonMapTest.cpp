#include "gtest/gtest.h"
#include "integrator/VCMPhotonMap.h"
#include "integrator/BDPTPath.h"
#include "accelerator/BruteForceAccel.h"
#include "bsdf/BSDF.h"
#include "camera/PinholeCamera.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "shape/Triangle.h"
#include "system/Memory.h"
#include "texture/ConstantTexture.h"
#include "volume/Homogeneous.h"
#include <array>
#include <limits>
#include <random>
#include <thread>

using namespace Gyunity;

namespace {

using Paths = std::vector<std::vector<PathVertex>>;
using Records = std::vector<Vcm::PhotonRecord>;
static_assert(sizeof(Vcm::PhotonRecord) == 2 * sizeof(size_t), "Photon records contain only stable IDs.");
const Vec3 Up(0, 0, 1);

// Independent exhaustive oracle: no production eligibility/filter/hash helpers.
// Compare the stored float inputs using double squared Euclidean distance, and
// the dot product of separately normalized oriented geometric normals >= 0.9.
Records BruteForceRecords(const Paths& paths, real radius, const Vec3& point, const Vec3& normal) {
	Records expected;
	const double queryLength = std::sqrt(double(normal.x) * normal.x +
		double(normal.y) * normal.y + double(normal.z) * normal.z);
	for (size_t p = 0; p < paths.size(); ++p) {
		for (size_t v = 0; v < paths[p].size(); ++v) {
			const auto& vertex = paths[p][v];
			if (vertex.mType != PathVertexType::Surface || vertex.mIsect.mBsdf->IsDelta()) continue;
			const auto& hit = vertex.mIsect;
			const double x = double(point.x) - hit.mPos.x;
			const double y = double(point.y) - hit.mPos.y;
			const double z = double(point.z) - hit.mPos.z;
			const double nx = hit.mGeometryNormal.x, ny = hit.mGeometryNormal.y, nz = hit.mGeometryNormal.z;
			const double photonLength = std::sqrt(nx * nx + ny * ny + nz * nz);
			const double cosine = (double(normal.x) / queryLength) * (nx / photonLength) +
				(double(normal.y) / queryLength) * (ny / photonLength) +
				(double(normal.z) / queryLength) * (nz / photonLength);
			if (x * x + y * y + z * z <= double(radius) * radius && cosine >= 0.9)
				expected.push_back({ p, v });
		}
	}
	return expected;
}

class VCMPhotonMap : public testing::Test {
protected:
	MemoryPool arena;
	PointLight emitter{ Vec3(0, 0, 3), Vec3(1) };
	Vcm::PhotonMap map;
	Records result;
	int oracleQueries = 0;

	void TearDown() override { RecordProperty("brute_force_queries", oracleQueries); }

	enum class Lobes { Diffuse, Delta, Mixed, Empty };

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

	void Check(const Paths& paths, real radius, const Vec3& point, const Vec3& normal = Up) {
		result = { { 999, 999 } };
		ASSERT_TRUE(map.Query(point, normal, result));
		++oracleQueries;
		EXPECT_EQ(result, BruteForceRecords(paths, radius, point, normal));
		EXPECT_TRUE(std::is_sorted(result.begin(), result.end()));
		EXPECT_EQ(std::adjacent_find(result.begin(), result.end()), result.end());
	}
};

TEST_F(VCMPhotonMap, EmptyBuildAndNoEligibleVerticesAreUsable) {
	Paths empty;
	ASSERT_TRUE(map.Build(empty, 0.25f));
	EXPECT_TRUE(map.IsBuilt());
	EXPECT_EQ(map.RecordCount(), 0u);
	EXPECT_EQ(map.Radius(), 0.25f);
	Check(empty, 0.25f, Vec3());
	Paths skipped{ { Endpoint(), Endpoint(PathVertexType::Camera),
		Endpoint(PathVertexType::Environment), Surface(Vec3(), Up, Lobes::Delta) } };
	ASSERT_TRUE(map.Build(skipped, 2));
	EXPECT_EQ(map.RecordCount(), 0u);
	Check(skipped, 2, Vec3(-100));
}

TEST_F(VCMPhotonMap, FirstSurfaceAndEmittingScatteringSurfaceAreIncluded) {
	Paths paths{ { Endpoint(), Surface() }, { Endpoint(), Surface(Vec3(0.5f, 0, 0)) } };
	paths[1][1].mLight = &emitter;
	ASSERT_TRUE(map.Build(paths, 1));
	EXPECT_EQ(map.RecordCount(), 2u);
	Check(paths, 1, Vec3());
	EXPECT_EQ(result, (Records{ { 0, 1 }, { 1, 1 } }));
}

TEST_F(VCMPhotonMap, SampledAndUnsampledTerminalPureDeltaAreExcluded) {
	Paths paths{ { Endpoint(), Surface(Vec3(), Up, Lobes::Delta), Surface(),
		Surface(Vec3(), Up, Lobes::Delta) } };
	paths[0][1].mIsDelta = true;
	ASSERT_FALSE(paths[0][3].mIsDelta);
	ASSERT_TRUE(map.Build(paths, 1));
	EXPECT_EQ(map.RecordCount(), 1u);
	Check(paths, 1, Vec3());
	EXPECT_EQ(result, (Records{ { 0, 2 } }));
}

TEST_F(VCMPhotonMap, OneRecordRawCollisionIsDeduplicated) {
	Paths paths{ { Endpoint(), Surface() } };
	HashGrid<Vcm::PhotonRecord> raw;
	raw.AddPoint({ Vec3(), { 0, 1 } }, 2);
	raw.BuildHashGrid(2);
	const auto& readOnly = raw;
	EXPECT_EQ(readOnly.GetGrid(Vec3()).size(), 8u);
	EXPECT_EQ(&raw.GetGrid(Vec3()), &readOnly.GetGrid(Vec3()));
	ASSERT_TRUE(map.Build(paths, 1));
	Check(paths, 1, Vec3());
	EXPECT_EQ(result.size(), 1u);
}

TEST_F(VCMPhotonMap, CoincidentRecordsRetainDistinctIdentities) {
	Paths paths(25);
	for (auto& path : paths) path = { Endpoint(), Surface(), Surface() };
	ASSERT_TRUE(map.Build(paths, 1));
	EXPECT_EQ(map.RecordCount(), 50u);
	Check(paths, 1, Vec3());
	EXPECT_EQ(result.size(), 50u);
}

TEST_F(VCMPhotonMap, ForcedHashCollisionsStillUseExactDistance) {
	Paths paths{ { Endpoint(), Surface() }, { Endpoint(), Surface(Vec3(8, 0, 0)) } };
	HashGrid<int> raw;
	raw.AddPoint({ Vec3(), 0 }, 2);
	raw.AddPoint({ Vec3(8, 0, 0), 1 }, 2);
	raw.BuildHashGrid(2);
	ASSERT_NE(std::find(raw.GetGrid(Vec3()).begin(), raw.GetGrid(Vec3()).end(), 1),
		raw.GetGrid(Vec3()).end());
	ASSERT_TRUE(map.Build(paths, 1));
	for (int x = -10; x <= 20; ++x) Check(paths, 1, Vec3(real(x) / 2, 0, 0));
}

TEST_F(VCMPhotonMap, AdjacentCoplanarPrimitivesDoNotCreateSeams) {
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
	ASSERT_TRUE(map.Build(paths, 0.1f));
	Check(paths, 0.1f, Vec3());
	EXPECT_EQ(result.size(), 2u);
}

TEST_F(VCMPhotonMap, OrientedGeometricNormalCompatibility) {
	Paths paths{ { Endpoint(), Surface(Vec3(), Up), Surface(Vec3(), -Up),
		Surface(Vec3(), Vec3(1, 0, 0)), Surface(Vec3(), Vec3(0.3f, 0, 1)),
		Surface(Vec3(), Vec3(0.6f, 0, 1)), Surface(Vec3(), Up * 7) } };
	ASSERT_TRUE(map.Build(paths, 1));
	Check(paths, 1, Vec3());
	EXPECT_EQ(result, (Records{ { 0, 1 }, { 0, 4 }, { 0, 6 } }));
	for (const Vec3 normal : { Up * 19, -Up, Vec3(0.43589f, 0, 0.9f), Vec3(1, 0, 0) })
		Check(paths, 1, Vec3(), normal);
}

TEST_F(VCMPhotonMap, ExactRadiusNextafterAndOutsideBounds) {
	for (real offset : { -100.0f, -1.0f, 0.0f, 1.0f, 100.0f }) {
		Paths paths{ { Endpoint(), Surface(Vec3(offset)) } };
		const real radius = 0.5f;
		ASSERT_TRUE(map.Build(paths, radius));
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

TEST_F(VCMPhotonMap, RandomNegativeWorldQueriesMatchBruteForce) {
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
	ASSERT_TRUE(map.Build(paths, 2.25f));
	for (int query = 0; query < 4096; ++query) {
		const Vec3 point(coordinate(rng), coordinate(rng), coordinate(rng));
		Check(paths, 2.25f, point, Vec3(tilt(rng), tilt(rng), 1));
	}
}

TEST_F(VCMPhotonMap, BoundarySweepsAcrossPhysicalScalesMatchBruteForce) {
	for (real radius : { 1e-30f, 1e-20f, 1e-5f, 0.1f, 1.0f, 1e10f, 1e20f, 1e30f }) {
		Paths paths(17);
		for (size_t p = 0; p < paths.size(); ++p)
			paths[p] = { Endpoint(), Surface(Vec3((real(p) - 8.5f) * radius,
				(real(p % 3) - 2.3f) * radius, -3.7f * radius)) };
		ASSERT_TRUE(map.Build(paths, radius));
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

TEST_F(VCMPhotonMap, RebuildDisjointThenEmptyClearsIdsRadiusAndBuckets) {
	Paths first{ { Endpoint(), Surface() } };
	Paths second{ {}, {}, { Endpoint(), Surface(Vec3(-100)), Surface(Vec3(-102)) } };
	Paths empty;
	ASSERT_TRUE(map.Build(first, 1));
	Check(first, 1, Vec3());
	ASSERT_TRUE(map.Build(second, 0.25f));
	EXPECT_EQ(map.RecordCount(), 2u);
	EXPECT_EQ(map.Radius(), 0.25f);
	Check(second, 0.25f, Vec3());
	EXPECT_TRUE(result.empty());
	Check(second, 0.25f, Vec3(-100));
	EXPECT_EQ(result, (Records{ { 2, 1 } }));
	ASSERT_TRUE(map.Build(empty, 7));
	EXPECT_EQ(map.RecordCount(), 0u);
	EXPECT_EQ(map.Radius(), 7);
	Check(empty, 7, Vec3(-100));
	map.Clear();
	EXPECT_FALSE(map.IsBuilt());
	EXPECT_EQ(map.Radius(), 0);
	EXPECT_FALSE(map.Query(Vec3(), Up, result));
	EXPECT_TRUE(result.empty());
}

TEST_F(VCMPhotonMap, MalformedSurfacesFailRatherThanBecomeEmptyMaps) {
	const real nan = std::numeric_limits<real>::quiet_NaN();
	const real infinity = std::numeric_limits<real>::infinity();
	HomogeneousMedium medium(Vec3(0.1f), Vec3(0.2f), 0);
	PinHoleCamera camera(std::make_shared<Film>(2, 2), Vec3(0, 0, 1), Vec3(), Vec3(0, 1, 0), 60, 1);
	const std::vector<std::function<void(PathVertex&)>> changes = {
		[](PathVertex& v) { v.mIsect.mBsdf = nullptr; },
		[&](PathVertex& v) { v = Surface(Vec3(), Up, Lobes::Empty); },
		[&](PathVertex& v) { v = Surface(Vec3(), Up, Lobes::Mixed); },
		[](PathVertex& v) { v.mIsect.mGeometryNormal = Vec3(); },
		[&](PathVertex& v) { v.mIsect.mGeometryNormal.x = infinity; },
		[&](PathVertex& v) { v.mIsect.mNormal.x = nan; },
		[](PathVertex& v) { v.mIsect.mNormal = Vec3(1, 0, 0); },
		[&](PathVertex& v) { v.mIsect.mPos.x = nan; },
		[&](PathVertex& v) { v.mIsect.mPos.y = infinity; },
		[](PathVertex& v) { v.mIsect.mOutDir = Vec3(); },
		[&](PathVertex& v) { v.mThroughput.z = infinity; },
		[](PathVertex& v) { v.mPdfFwd = -1; },
		[&](PathVertex& v) { v.mPdfPrev = nan; },
		[&](PathVertex& v) { v.mIsect.mMediumInterface = &medium; },
		[&](PathVertex& v) { v.mCamera = &camera; },
		[](PathVertex& v) { v.mIsDelta = true; },
		[](PathVertex& v) { v.mType = static_cast<PathVertexType>(99); }
	};
	for (size_t index = 0; index < changes.size(); ++index) {
		SCOPED_TRACE(index);
		Paths good{ { Endpoint(), Surface() } };
		ASSERT_TRUE(map.Build(good, 1));
		Paths bad = good;
		changes[index](bad[0][1]);
		EXPECT_FALSE(map.Build(bad, 1));
		EXPECT_FALSE(map.IsBuilt());
		EXPECT_EQ(map.RecordCount(), 0u);
		EXPECT_EQ(map.Radius(), 0);
		result = { { 123, 456 } };
		EXPECT_FALSE(map.Query(Vec3(), Up, result));
		EXPECT_TRUE(result.empty());
		ASSERT_TRUE(map.Build(good, 1));
		Check(good, 1, Vec3());
	}
	Paths invalidDelta{ { Endpoint(), Surface(Vec3(), Up, Lobes::Delta) } };
	invalidDelta[0][1].mIsect.mGeometryNormal = Vec3();
	EXPECT_FALSE(map.Build(invalidDelta, 1));
}

TEST_F(VCMPhotonMap, InvalidRadiusAndQueryAreCheckedEvenWhenEmpty) {
	Paths empty;
	EXPECT_FALSE(map.Query(Vec3(), Up, result));
	for (real radius : { 0.0f, -1.0f, std::numeric_limits<real>::infinity(),
		std::numeric_limits<real>::quiet_NaN() }) {
		EXPECT_FALSE(map.Build(empty, radius));
		EXPECT_FALSE(map.IsBuilt());
	}
	ASSERT_TRUE(map.Build(empty, 1));
	EXPECT_FALSE(map.Query(Vec3(), Vec3(), result));
	EXPECT_FALSE(map.Query(Vec3(std::numeric_limits<real>::infinity()), Up, result));
	EXPECT_FALSE(map.Query(Vec3(), Vec3(std::numeric_limits<real>::quiet_NaN()), result));
	EXPECT_TRUE(result.empty());
	EXPECT_TRUE(map.IsBuilt());
	Check(empty, 1, Vec3());
}

TEST_F(VCMPhotonMap, ExtremeFiniteWorldInputsDoNotOverflowDistanceOrCellCasts) {
	const real largest = (std::numeric_limits<real>::max)();
	for (real radius : { std::numeric_limits<real>::denorm_min(),
		(std::numeric_limits<real>::min)(), 1.0f, largest }) {
		Paths paths{ { Endpoint(), Surface(Vec3(largest, 0, 0)) } };
		ASSERT_TRUE(map.Build(paths, radius));
		EXPECT_EQ(map.Radius(), radius);
		Check(paths, radius, Vec3(largest, 0, 0));
		EXPECT_EQ(result.size(), 1u);
		Check(paths, radius, Vec3(-largest, 0, 0));
		EXPECT_TRUE(result.empty());
	}
	Paths extremes{ { Endpoint(), Surface(Vec3(largest, 0, 0)), Surface(Vec3(-largest, 0, 0)),
		Surface(Vec3(largest, largest, largest)) } };
	ASSERT_TRUE(map.Build(extremes, largest));
	Check(extremes, largest, Vec3());
	EXPECT_EQ(result.size(), 2u);
	EXPECT_FALSE(map.Build(extremes, 1));
	EXPECT_FALSE(map.IsBuilt());
	EXPECT_EQ(map.RecordCount(), 0u);
	Paths narrow{ { Surface(), Surface(Vec3(std::numeric_limits<real>::denorm_min(), 0, 0)) } };
	ASSERT_TRUE(map.Build(narrow, std::numeric_limits<real>::denorm_min()));
	Check(narrow, std::numeric_limits<real>::denorm_min(), Vec3());
	EXPECT_EQ(result.size(), 2u);
	for (real scale : { std::numeric_limits<real>::denorm_min(), largest }) {
		Paths normals{ { Surface(Vec3(), Up * scale) } };
		ASSERT_TRUE(map.Build(normals, 1));
		Check(normals, 1, Vec3(), Up * scale);
		EXPECT_EQ(result.size(), 1u);
	}
}

TEST_F(VCMPhotonMap, BackendCoordinateLimitIsConservativeAndExplicit) {
	for (real sign : { -1.0f, 1.0f }) {
		const real edge = sign * 1048576;
		Paths paths{ { Endpoint(), Surface(), Surface(Vec3(edge, 0, 0)) } };
		ASSERT_TRUE(map.Build(paths, 1));
		for (real offset : { -1.0f, 0.0f, 1.0f }) {
			const Vec3 point(edge + offset, 0, 0);
			Check(paths, 1, point);
			EXPECT_EQ(result.size(), 1u);
			Check(paths, 1, Vec3(std::nextafter(point.x,
				sign * std::numeric_limits<real>::infinity()), 0, 0));
		}
		paths[0][2].mIsect.mPos.x = std::nextafter(edge, sign * std::numeric_limits<real>::infinity());
		EXPECT_FALSE(map.Build(paths, 1));
		EXPECT_FALSE(map.IsBuilt());
		EXPECT_EQ(map.Radius(), 0);
	}
}

TEST_F(VCMPhotonMap, ConcurrentReadOnlyQueriesOwnSeparateOutputs) {
	Paths paths(23);
	for (size_t p = 0; p < paths.size(); ++p)
		paths[p] = { Endpoint(), Surface(Vec3(real(p) / 8 - 2, 0, 0)), Surface(Vec3()) };
	ASSERT_TRUE(map.Build(paths, 0.25f));
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
			for (size_t q = 0; q < points.size(); ++q)
				if (!map.Query(points[q], Up, outputs[worker]) || outputs[worker] != expected[q])
					++failures[worker];
		});
	}
	for (auto& worker : workers) worker.join();
	oracleQueries += int(outputs.size() * points.size());
	for (int failure : failures) EXPECT_EQ(failure, 0);
}

TEST_F(VCMPhotonMap, GeneratedPathsAndArenaOwnedDataRemainUnchanged) {
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
	for (auto& path : paths) ASSERT_GE(GenerateLightPath(*scene, samples, arena, 4, path), 2);
	const Paths snapshot = paths;
	const auto* storage = paths.data();
	std::vector<const PathVertex*> pathStorage;
	for (const auto& path : paths) pathStorage.push_back(path.data());
	const size_t allocated = arena.TotalAllocated();
	ASSERT_TRUE(map.Build(paths, 1));
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
	map.Clear();
	arena.Reset();
}

}
