#include "gtest/gtest.h"
#include "integrator/HashGrid.h"
#include <array>
#include <limits>

using namespace Gyunity;

namespace {

// Compute modulo-2^32 products in 64 bits, independently of HashGrid::Hash.
uint32 WrappedHash(int x, int y, int z, size_t count) {
	const uint64 mask = 0xffffffffULL;
	const uint64 a = (uint64(uint32(x)) * 73856093ULL) & mask;
	const uint64 b = (uint64(uint32(y)) * 19349663ULL) & mask;
	const uint64 c = (uint64(uint32(z)) * 83492791ULL) & mask;
	return uint32((a ^ b ^ c) % count);
}

TEST(HashGrid, UnsignedWrapMatchesIndependentProducts) {
	HashGrid<int> grid;
	for (int i = 0; i < 37; ++i) grid.AddPoint({ Vec3(real(i), 0, 0), i }, 1);
	grid.BuildHashGrid(1);
	const int values[] = { 0, 1, -1, 29, -29, 1000000, -1000000,
		(std::numeric_limits<int>::min)(), (std::numeric_limits<int>::max)() };
	for (int x : values) for (int y : values) for (int z : values) {
		EXPECT_EQ(grid.Hash(x, y, z), WrappedHash(x, y, z, 37));
	}
}

TEST(HashGrid, ExplicitRadiusBucketsPreserveLegacyMultiplicityAndRebuild) {
	HashGrid<int> grid;
	for (int generation = 0; generation < 2; ++generation) {
		grid.ClearHashGrid();
		grid.Initialize(real(generation + 1));
		const real radius = real(generation + 1);
		const real inverseCell = 1 / (radius * 2);
		std::vector<Vec3> positions;
		Vec3 minimum(10000);
		for (int i = 0; i < 37; ++i) {
			const Vec3 position(real(i % 7 - 20), real(i % 5 - 13), real(i % 3 - 8));
			positions.push_back(position);
			grid.AddPoint({ position, i }, radius);
			for (int axis = 0; axis < 3; ++axis)
				minimum[axis] = std::min(minimum[axis], position[axis] - radius);
		}
		grid.BuildHashGrid(radius);
		std::array<std::vector<int>, 37> expected;
		for (int i = 0; i < 37; ++i) {
			const Vec3 lo = (positions[i] - radius - minimum) * inverseCell;
			const Vec3 hi = (positions[i] + radius - minimum) * inverseCell;
			for (int z = int(lo.z); z <= int(hi.z); ++z)
				for (int y = int(lo.y); y <= int(hi.y); ++y)
					for (int x = int(lo.x); x <= int(hi.x); ++x)
						expected[WrappedHash(x, y, z, 37)].push_back(i);
		}
		for (auto& bucket : expected) std::sort(bucket.begin(), bucket.end());
		for (int x = -24; x <= -10; ++x) for (int y = -16; y <= -5; ++y)
			for (int z = -10; z <= -2; ++z) {
				const Vec3 point{ real(x), real(y), real(z) };
				const Vec3 cell = (point - minimum) * inverseCell;
				const auto hash = WrappedHash(std::abs(int(cell.x)), std::abs(int(cell.y)),
					std::abs(int(cell.z)), 37);
				auto actual = grid.GetGrid(point);
				std::sort(actual.begin(), actual.end());
				EXPECT_EQ(actual, expected[hash]);
			}
	}
}

}
