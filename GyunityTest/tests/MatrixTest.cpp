#include "gtest/gtest.h"
#include "math/Linalg.h"
#include "math/MathUtils.h"

GYT_NAMESPACE_BEGIN
TEST(TestMatrixIsEqual, IsEqual) {
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		EXPECT_EQ(mat1 == mat2, true);
	}
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1.1, 2, 3, 4,
				5, 6, 7.3, 8,
				9, 10, 11, 12,
				13, 14.9, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(1.1, 2, 3, 4,
				5, 6, 7.3, 8,
				9, 10, 11, 12,
				13, 14.9, 15, 16);
		EXPECT_EQ(Equal(mat1, mat2), true);
	}
};

TEST(TestMatrixNotEqual, NotEqual) {
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1.1, 2, 3, 4,
				5, 6, 7.3, 8,
				9, 10, 11, 12,
				13, 14.9, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(1.1, 2, 3, 4,
				5, 6, 7.3, 8,
				9, 10, 11, 13,
				13, 14.9, 15, 16);
		EXPECT_EQ(mat1 != mat2, true);
	}
};

TEST(TestMatrixPlus, Plus) {
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat3(2, 4, 6, 8,
				10, 12, 14, 16,
				18, 20, 22, 24,
				26, 28, 30, 32);
		EXPECT_EQ(mat1 + mat2 == mat3, true);
	}
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat3(2, 4, 6, 8,
				10, 12, 14, 16,
				18, 20, 22, 24,
				26, 28, 30, 32);
		mat1 += mat2;
		EXPECT_EQ(mat1 == mat3, true);
	}
};

TEST(TestMatrixMinus, Minus) {
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat3 = Matrix<4, real, IntrinsicSet::None>::Zeros();
		EXPECT_EQ((mat1 - mat2) == mat3, true);
	}
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat3 = Matrix<4, real, IntrinsicSet::None>::Zeros();
		mat1 -= mat2;
		EXPECT_EQ(mat1 == mat3, true);
	}
};


TEST(TestMatrixMulMatrix, MulMatrix) {
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(1, 2, 3, 4,
				5, 6, 7, 8,
				3, 2, 1, 6,
				1, 3, 5, 7);
		Matrix<4, real, IntrinsicSet::None>
			mat3(24, 32, 40, 66,
				 64, 84, 104, 166,
				 104, 136, 168, 266,
				 144, 188, 232, 366);
		EXPECT_EQ(mat1 * mat2 == mat3, true);
	}
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(1, 2, 3, 4,
				5, 6, 7, 8,
				3, 2, 1, 6,
				1, 3, 5, 7);
		Matrix<4, real, IntrinsicSet::None>
			mat3(24, 32, 40, 66,
				64, 84, 104, 166,
				104, 136, 168, 266,
				144, 188, 232, 366);
		mat1 *= mat2;
		EXPECT_EQ(mat1 == mat3, true);
	}
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(2, 4, 6, 8,
				10, 12, 14, 16,
				18, 20, 22, 24,
				26, 28, 30, 32);

		EXPECT_EQ(mat1 * 2 == mat2, true);
		EXPECT_EQ(2 * mat1 == mat2, true);
	}
	{
		Matrix<4, real, IntrinsicSet::None>
			mat1(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Matrix<4, real, IntrinsicSet::None>
			mat2(2, 4, 6, 8,
				10, 12, 14, 16,
				18, 20, 22, 24,
				26, 28, 30, 32);
		mat1 *= 2;
		EXPECT_EQ(mat1 == mat2, true);

	}
};


TEST(TestMatrixMulVector, MulVector) {
	{
		Matrix<4, real, IntrinsicSet::None>
			mat(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		Vector<4, real, IntrinsicSet::None> v1(1.1, 2.2, 3.3, 4.4);
		Vector<4, real, IntrinsicSet::None> v2(33.0, 77.0, 121.0, 165.0);
		EXPECT_EQ(Equal(mat * v1, v2), true);
	}
	{
		Matrix<4, real, IntrinsicSet::None>
			mat(0.992797, 0.307158, 0.878931, 0.862293,
				0.688249, 0.42504, 0.677657, 0.540247,
				0.970518, 0.0388582, 0.482551, 0.547493,
				0.973952, 0.116856, 0.645318, 0.350542);
		Vector<4, real, IntrinsicSet::None> 
		v1(0.8310123813548647,
			0.7829980173878517,
			0.48269930108431525,
			0.4714602712916691);
		Vector<4, real, IntrinsicSet::None> 
		v2(1.8963264168581206,
			1.4865590956869474,
			1.3279869317662532,
			1.3776250350210677);
		EXPECT_EQ(Equal(mat * v1, v2), true);
	}
};


TEST(TestMatrixDeterminant, Determinant) {
	{
		Matrix<4, real, IntrinsicSet::None>
			mat(1, 2, 3, 4,
				5, 6, 7, 8,
				9, 10, 11, 12,
				13, 14, 15, 16);
		EXPECT_EQ(Determinant(mat) == 0, true);
	}
	{
		Matrix<4, real, IntrinsicSet::None>
			mat(0.992797, 0.307158, 0.878931, 0.862293,
				0.688249, 0.42504, 0.677657, 0.540247,
				0.970518, 0.0388582, 0.482551, 0.547493,
				0.973952, 0.116856, 0.645318, 0.350542);
		real expectedDeterminant = 0.02143648345371967;
		EXPECT_EQ(Equal(Determinant(mat), expectedDeterminant), true);
	}
};

TEST(TestMatrixInv, Inv) {
	{
		Matrix<4, real, IntrinsicSet::None>
			scaleMatrix(1, 0, 0, 0,
				0, 2, 0, 0,
				0, 0, 5, 0,
				0, 0, 0, 1);
		Matrix<4, real, IntrinsicSet::None>
			invScaleMatrix(1.0, 0, 0, 0,
						   0, 0.5, 0, 0,
						   0, 0, 0.2, 0,
						   0, 0, 0, 1.0);
		EXPECT_EQ(Equal(Inverse(scaleMatrix), invScaleMatrix), true);
	}
	{
		Matrix<4, real, IntrinsicSet::None>
			translateMatrix(1, 0, 0, 1.1,
							0, 1, 0, 2.2,
							0, 0, 1, 3.3,
							0, 0, 0, 1);
		Matrix<4, real, IntrinsicSet::None>
			invTranslateMatrix(1.0, 0, 0, -1.1,
							   0, 1.0, 0, -2.2,
							   0, 0, 1.0, -3.3,
							   0, 0, 0, 1.0);
		EXPECT_EQ(Equal(Inverse(translateMatrix), invTranslateMatrix), true);
	}
	{
		Matrix<4, real, IntrinsicSet::None>
			mat(0.992797, 0.307158, 0.878931, 0.862293,
				0.688249, 0.42504, 0.677657, 0.540247,
				0.970518, 0.0388582, 0.482551, 0.547493,
				0.973952, 0.116856, 0.645318, 0.350542);
		Matrix<4, real, IntrinsicSet::None>
			expectedInversedMat(-2.84865, 1.83357, 2.73568, -0.091224,
						-4.12886, 5.67329, 2.146, -1.93875,
						4.11491, -3.47325, -5.35782, 3.5988,
						1.71591, -0.591686, 1.54706, -2.8726);
		EXPECT_EQ(Equal(Inverse(mat), expectedInversedMat, 1e-4), true);
	}
};

GYT_NAMESPACE_END