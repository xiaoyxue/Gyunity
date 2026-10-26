#include "container/Array.h"
#include "texture/ImageTexture.h"
#include "gtest/gtest.h"
#include <climits>
#include <type_traits>

using namespace Gyunity;

static_assert(std::is_same<decltype(std::declval<Array2D<float>&>()[0]), float*>::value, "2D row access must remain a pointer.");
static_assert(std::is_same<decltype(std::declval<const Array2D<float>&>()[0]), const float*>::value, "Const row access must remain const.");

TEST(ArrayND, RuntimeExtentsAllocateAndPreserveTwoDimensionalLayout) {
	Array2D<Vec3> image(Vec2i(2, 3));
	ASSERT_TRUE(image.IsValid());
	EXPECT_EQ(image.Size(), 6u);
	EXPECT_EQ(image.mRes, Vec2i(2, 3));
	image(1, 2) = Vec3(7);
	EXPECT_EQ(image[1][2], Vec3(7));
	EXPECT_EQ(image[Vec2i(1, 2)], Vec3(7));
	EXPECT_EQ(image.Data()[5], Vec3(7));
	EXPECT_EQ(image.GetStrides()[0], 3u);
	EXPECT_EQ(image.GetStrides()[1], 1u);
	const auto& readOnly = image;
	EXPECT_EQ(readOnly(1, 2), Vec3(7));
	EXPECT_EQ(readOnly[1][2], Vec3(7));
}

TEST(ArrayND, OneAndThreeDimensionalNestedAccessIsContiguous) {
	Array1D<int> line(4);
	line[3] = 9;
	EXPECT_EQ(line(3), 9);
	Array3D<int> volume(2, 3, 4);
	ASSERT_TRUE(volume.IsValid());
	for (int i = 0; i < 2; ++i)
		for (int j = 0; j < 3; ++j)
			for (int k = 0; k < 4; ++k)
				volume(i, j, k) = 100 * i + 10 * j + k;
	EXPECT_EQ(volume[1][2][3], 123);
	EXPECT_EQ(volume.Data()[23], 123);
	const auto& readOnly = volume;
	EXPECT_EQ(readOnly[1][2][3], 123);
	EXPECT_EQ(volume.GetStrides(), (Array3D<int>::StrideArray{12,4,1}));
}

TEST(ArrayND, HigherRanksHaveCheckedIndexRoundTripsAndIndexIteration) {
	ArrayND<5, int> array(ArrayND<5, int>::Extents{2,3,2,2,4});
	ASSERT_TRUE(array.IsValid());
	size_t visited = 0;
	ASSERT_TRUE(array.ForEachIndex([&](const ArrayND<5, int>::Extents& index) {
		size_t offset = 0;
		ASSERT_TRUE(array.TryOffset(index, offset));
		EXPECT_EQ(offset, visited);
		array(index) = static_cast<int>(visited++);
		ArrayND<5, int>::Extents recovered{};
		ASSERT_TRUE(array.TryCoordinates(offset, recovered));
		EXPECT_EQ(recovered, index);
	}));
	EXPECT_EQ(visited, array.Size());
	EXPECT_EQ(array[1][2][1][1][3], 95);
	EXPECT_EQ(array(1,2,1,1,3), 95);
}

TEST(ArrayND, FillAssignCopyMoveAndEmptyStatesRemainConsistent) {
	Array2D<int> array(std::vector<int>{1,2,3,4}, 2, 2);
	ASSERT_TRUE(array.IsValid());
	auto copy = array;
	copy[0][0] = 8;
	EXPECT_EQ(array[0][0], 1);
	Array2D<int> moved(std::move(copy));
	EXPECT_EQ(moved[0][0], 8);
	EXPECT_TRUE(copy.IsValid());
	EXPECT_TRUE(copy.Empty());
	EXPECT_EQ(copy.mRes, Vec2i(0));
	ASSERT_TRUE(moved.Fill(5));
	for (int value : moved) EXPECT_EQ(value, 5);
	ASSERT_TRUE(moved.Resize(Vec2i(3,2), 7));
	EXPECT_EQ(moved.Size(), 6u);
	for (int value : moved) EXPECT_EQ(value, 7);
	array = std::move(moved);
	EXPECT_TRUE(moved.IsValid());
	EXPECT_TRUE(moved.Empty());
	EXPECT_EQ(array.Size(), 6u);
	ASSERT_TRUE(array.Resize(Array2D<int>::Extents{0,3}));
	EXPECT_TRUE(array.IsValid());
	EXPECT_TRUE(array.Empty());
	EXPECT_EQ(array.mRes, Vec2i(0,3));
	size_t count = 0;
	ASSERT_TRUE(array.ForEachIndex([&](const Array2D<int>::Extents&) { ++count; }));
	EXPECT_EQ(count, 0u);
	array.Clear();
	EXPECT_EQ(array.GetShape(), (Array2D<int>::Extents{0,0}));
}

TEST(ArrayND, InvalidInputsReturnStatusAndDoNotCorruptExistingData) {
	Array2D<int> array(std::vector<int>{1,2,3,4}, 2, 2);
	EXPECT_FALSE(array.Resize(Array2D<int>::Extents{-1,2}));
	EXPECT_FALSE(array.Assign(std::vector<int>{9}, Vec2i(2,2)));
	EXPECT_EQ(array.mRes, Vec2i(2,2));
	EXPECT_EQ(array[1][1], 4);
	int* value = nullptr;
	EXPECT_FALSE(array.TryAt(Array2D<int>::Extents{-1,0}, value));
	EXPECT_EQ(value, nullptr);
	EXPECT_TRUE(array.TryAt(Array2D<int>::Extents{1,1}, value));
	ASSERT_NE(value, nullptr);
	EXPECT_EQ(*value, 4);
	Array3D<int> volume;
	EXPECT_FALSE(volume.Resize(Array3D<int>::Extents{INT_MAX,INT_MAX,INT_MAX}));
	EXPECT_TRUE(volume.IsValid());
	EXPECT_TRUE(volume.Empty());
	Array2D<int> bad(-1, 2);
	EXPECT_FALSE(bad.IsValid());
	EXPECT_FALSE(bad.Fill(0));
	Array2D<int> mismatch(std::vector<int>{1}, 2, 2);
	EXPECT_FALSE(mismatch.IsValid());
	array.mRes[0] = 9;
	EXPECT_FALSE(array.IsValid());
	EXPECT_FALSE(array.TryAt(Array2D<int>::Extents{0,0}, value));
	ASSERT_TRUE(array.Resize(Vec2i(1,1)));
	EXPECT_TRUE(array.IsValid());
}

TEST(ArrayND, ExistingImageTextureConstructionAndSamplingStayCompatible) {
	Array2D<Vec3> pixels(std::vector<Vec3>{Vec3(1), Vec3(2), Vec3(3), Vec3(4)}, 2, 2);
	ImageTexture<Vec3> texture(pixels);
	EXPECT_EQ(texture.Width(), 2);
	EXPECT_EQ(texture.Height(), 2);
	EXPECT_EQ(texture.Sample(Vec2(0.25f,0.25f)), Vec3(1));
	EXPECT_EQ(texture.Sample(Vec2(0.75f,0.75f)), Vec3(4));
}
