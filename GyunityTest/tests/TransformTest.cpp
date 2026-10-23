#include "gtest/gtest.h"
#include "math/Linalg.h"
#include "math/MathUtils.h"
#include "math/Transform.h"

GYT_NAMESPACE_BEGIN

TEST(TestTranslateVector, TranslateVector) {
	{
		Vec3 vector3(1, 2, 3);
		Transform transform = Transform::Translate(Vec3(1, 2, 3));
		Vec3 absError;
		Vec3 transformedVector = transform.TransformVector(vector3, &absError);
		EXPECT_EQ(transformedVector == Vec3(1, 2, 3), true);
	}
};

TEST(TestTranslatePoint, TranslatePoint) {
	{
		Vec3 point3(1, 2, 3);
		Transform transform = Transform::Translate(Vec3(1, 2, 3));
		Vec3 absError;
		Vec3 transformedPoint = transform.TransformPoint(point3, &absError);
		EXPECT_EQ(transformedPoint == Vec3(2, 4, 6), true);
	}
	{
		Vec3 point3(1, 2, 3);
		Transform transform = Transform::Translate(Vec3(1, 2, 3));
		Vec3 absError;
		Vec3 transformedPoint = transform(point3, &absError);
		EXPECT_EQ(transformedPoint == Vec3(2, 4, 6), true);
	}
};

TEST(TestTranslateNormal, TranslateNormal) {
	{
		Vec3 normal(1, 2, 3);
		Transform transform = Transform::Translate(Vec3(1, 2, 3));
		Vec3 transformedNormal = transform.TransformNormal(normal);
		EXPECT_EQ(transformedNormal == Vec3(1, 2, 3), true);
	}
};

TEST(TestTranslateRay, TranslateRay) {
	{
		Vec3 origin(1, 2, 3);
		Vec3 direction = Vec3(1, 0, 0);
		Ray ray(origin, direction);
		Transform transform = Transform::Translate(Vec3(1, 2, 3));
		Vec3 originError, directionError;
		Ray transformedRay = transform.TransformRay(ray, &originError, &directionError);
		std::cout << transformedRay << std::endl;
		EXPECT_EQ(Equal(transformedRay.mOrig, Vec3(2.0002f, 4.f, 6.f)), true);
		EXPECT_EQ(Equal(transformedRay.mDir, Vec3(1.f, 0.f, 0.f)), true);
	}
};

TEST(TestTranslageAABB, TranslateAABB) {
	{
		AABB bounding(Vec3(1, 2, 3), Vec3(6, 7, 8));
		Transform transform = Transform::Translate(Vec3(1, 2, 3));
		AABB transformedBounding = transform.TransformAABB(bounding);
		EXPECT_EQ(transformedBounding.minPoint == Vec3(2, 4, 6), true);
		EXPECT_EQ(transformedBounding.maxPoint == Vec3(7, 9, 11), true);
	}
};

TEST(TestScaleVector, ScaleVector) {
	{
		Vec3 vector3(1, 2, 3);
		Transform transform = Transform::Scale(1.0, 2.5, 3.0);
		Vec3 absError;
		Vec3 scaledVector = transform.TransformVector(vector3, &absError);
		EXPECT_EQ(scaledVector == Vec3(1.f, 5.f, 9.f), true);
	}
};

TEST(TestScalePoint, ScalePoint) {
	{
		Vec3 point3(1, 2, 3);
		Transform transform = Transform::Scale(1.0, 2.5, 3.0);
		Vec3 absError;
		Vec3 scaledPoint = transform(point3, &absError);
		EXPECT_EQ(scaledPoint == Vec3(1.f, 5.f, 9.f), true);
	}
	{
		Vec3 point3(1, 2, 3);
		Transform transform = Transform::Scale(1.0, 2.5, 3.0);
		Vec3 absError;
		Vec3 scaledPoint = transform.TransformPoint(point3, &absError);
		EXPECT_EQ(scaledPoint == Vec3(1.f, 5.f, 9.f), true);
	}
};

TEST(TestScaledNormal, ScaleNormal) {
	{
		Vec3 normal(1, 1, 1);
		Transform transform = Transform::Scale(1.0, 2.5, 5.0);
		Vec3 scaledNormal = transform.TransformNormal(normal);
		std::cout << scaledNormal << std::endl;
		EXPECT_EQ(Equal(scaledNormal, Vec3(1.f, 0.4f, 0.2f)), true);
	}
};

TEST(TestScaledRay, ScaleRay) {
	{
		Vec3 origin(1, 2, 3);
		Vec3 direction = Vec3(1, 1, 1);
		Ray ray(origin, direction);
		Transform transform = Transform::Scale(1, 2, 3);
		Vec3 originError, directionError;
		Ray transformedRay = transform.TransformRay(ray, &originError, &directionError);
		std::cout << transformedRay << std::endl;
		EXPECT_EQ(Equal(transformedRay.mOrig, Vec3(1.0002f, 4.0004f, 9.0006f)), true);
		EXPECT_EQ(Equal(transformedRay.mDir, Vec3(1.f, 2.f, 3.f)), true);
	}
};

TEST(TestScaleAABB, ScaleAABB) {
	{
		AABB bounding(Vec3(1, 2, 3), Vec3(6, 7, 8));
		Transform transform = Transform::Scale(1, 2, 3);
		AABB transformedBounding = transform.TransformAABB(bounding);
		EXPECT_EQ(transformedBounding.minPoint == Vec3(1, 4, 9), true);
		EXPECT_EQ(transformedBounding.maxPoint == Vec3(6, 14, 24), true);
	}
};


TEST(TestRotateVector, RotateX) {
	{
		Vec3 vector3(1, 1, 0);
		Transform transform = Transform::RotateX(30);
		Vec3 absError;
		Vec3 transformeVector = transform.TransformVector(vector3, &absError);
		Vec3 expectedVector(1.f, std::sqrt(3) * 0.5, 0.5);
		//std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
};

TEST(TestRotateVector, RotateY) {
	{
		Vec3 vector3(1, 1, 0);
		Transform transform = Transform::RotateY(30);
		Vec3 absError;
		Vec3 transformeVector = transform.TransformVector(vector3, &absError);
		Vec3 expectedVector(std::sqrt(3) * 0.5, 1, -0.5);
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
};

TEST(TestRotateVector, RotateZ) {
	{
		Vec3 vector3(1, 0, 1);
		Transform transform = Transform::RotateZ(30);
		Vec3 absError;
		Vec3 transformeVector = transform.TransformVector(vector3, &absError);
		Vec3 expectedVector(std::sqrt(3) * 0.5, 0.5, 1);
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
};

TEST(TestRotateVector, RotateV) {
	{
		Vec3 vector3(1, 1, 0);
		Transform transform = Transform::Rotate(30, Vec3(1, 0, 0));
		Vec3 absError;
		Vec3 transformeVector = transform.TransformVector(vector3, &absError);
		Vec3 expectedVector(1.f, std::sqrt(3) * 0.5, 0.5);
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
	{
		Vec3 vector3(1, 1, 0);
		Transform transform = Transform::Rotate(30, Vec3(0, 1, 0));
		Vec3 absError;
		Vec3 transformeVector = transform.TransformVector(vector3, &absError);
		Vec3 expectedVector(std::sqrt(3) * 0.5, 1, -0.5);
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
	{
		Vec3 vector3(1, 0, 1);
		Transform transform = Transform::Rotate(30, Vec3(0, 0, 1));
		Vec3 absError;
		Vec3 transformeVector = transform.TransformVector(vector3, &absError);
		Vec3 expectedVector(std::sqrt(3) * 0.5, 0.5, 1);
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
};


TEST(TestRotatePoint, RotateX) {
	{
		Vec3 point3(1, 1, 0);
		Transform transform = Transform::RotateX(30);
		Vec3 absError;
		Vec3 transformePoint = transform.TransformPoint(point3, &absError);
		Vec3 expectedPoint(1.f, std::sqrt(3) * 0.5, 0.5);
		//std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformePoint, expectedPoint), true);
	}
};

TEST(TestRotatePoint, RotateY) {
	{
		Vec3 point3(1, 1, 0);
		Transform transform = Transform::RotateY(30);
		Vec3 absError;
		Vec3 transformePoint = transform.TransformVector(point3, &absError);
		Vec3 expectedPoint(std::sqrt(3) * 0.5, 1, -0.5);
		EXPECT_EQ(Equal(transformePoint, expectedPoint), true);
	}
};

TEST(TestRotatePoint, RotateZ) {
	{
		Vec3 point3(1, 0, 1);
		Transform transform = Transform::RotateZ(30);
		Vec3 absError;
		Vec3 transformePoint = transform.TransformVector(point3, &absError);
		Vec3 expectedPoint(std::sqrt(3) * 0.5, 0.5, 1);
		EXPECT_EQ(Equal(transformePoint, expectedPoint), true);
	}
};

TEST(TestRotatePoint, RotateV) {
	{
		Vec3 point3(1, 1, 0);
		Transform transform = Transform::Rotate(30, Vec3(1, 0, 0));
		Vec3 absError;
		Vec3 transformePoint = transform.TransformVector(point3, &absError);
		Vec3 expectedPoint(1.f, std::sqrt(3) * 0.5, 0.5);
		EXPECT_EQ(Equal(transformePoint, expectedPoint), true);
	}
	{
		Vec3 point3(1, 1, 0);
		Transform transform = Transform::Rotate(30, Vec3(0, 1, 0));
		Vec3 absError;
		Vec3 transformePoint = transform.TransformVector(point3, &absError);
		Vec3 expectedPoint(std::sqrt(3) * 0.5, 1, -0.5);
		EXPECT_EQ(Equal(transformePoint, expectedPoint), true);
	}
	{
		Vec3 point3(1, 0, 1);
		Transform transform = Transform::Rotate(30, Vec3(0, 0, 1));
		Vec3 absError;
		Vec3 transformePoint = transform.TransformVector(point3, &absError);
		Vec3 expectedPoint(std::sqrt(3) * 0.5, 0.5, 1);
		EXPECT_EQ(Equal(transformePoint, expectedPoint), true);
	}
};

TEST(TestRotateNormal, RotateX) {
	{
		Vec3 normal(1, 0, 0);
		Transform transform = Transform::RotateX(30);
		Vec3 transformeVector = transform.TransformNormal(normal);
		Vec3 expectedVector(1, 0, 0);
		std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
	{
		Vec3 normal(0, 1, 0);
		Transform transform = Transform::RotateX(30);
		Vec3 transformeVector = transform.TransformNormal(normal);
		Vec3 expectedVector(0, std::sqrt(3) * 0.5, 0.5);
		std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
};

TEST(TestRotateNormal, RotateY) {
	{
		Vec3 normal(0, 1, 0);
		Transform transform = Transform::RotateY(30);
		Vec3 transformeVector = transform.TransformNormal(normal);
		Vec3 expectedVector(0, 1, 0);
		//std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
	{
		Vec3 normal(1, 0, 0);
		Transform transform = Transform::RotateY(30);
		Vec3 transformeVector = transform.TransformNormal(normal);
		Vec3 expectedVector(std::sqrt(3) * 0.5, 0, -0.5);
		//std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
};

TEST(TestRotateNormal, RotateZ) {
	{
		Vec3 normal(0, 0, 1);
		Transform transform = Transform::RotateZ(30);
		Vec3 transformeVector = transform.TransformNormal(normal);
		Vec3 expectedVector(0, 0, 1);
		//std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
	{
		Vec3 normal(1, 0, 0);
		Transform transform = Transform::RotateZ(30);
		Vec3 transformeVector = transform.TransformNormal(normal);
		Vec3 expectedVector(std::sqrt(3) * 0.5, 0.5, 0);
		//std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
};

TEST(TestRotateNormal, RotateV) {
	{
		Vec3 normal(1, 0, 0);
		Transform transform = Transform::RotateX(30);
		Vec3 transformeVector = transform.TransformNormal(normal);
		Vec3 expectedVector(1, 0, 0);
		//std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
	{
		Vec3 normal(0, 1, 0);
		Transform transform = Transform::RotateY(30);
		Vec3 transformeVector = transform.TransformNormal(normal);
		Vec3 expectedVector(0, 1, 0);
		//std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
	{
		Vec3 normal(0, 0, 1);
		Transform transform = Transform::RotateZ(30);
		Vec3 transformeVector = transform.TransformNormal(normal);
		Vec3 expectedVector(0, 0, 1);
		//std::cout << transformeVector << std::endl << expectedVector << std::endl;
		EXPECT_EQ(Equal(transformeVector, expectedVector), true);
	}
};
GYT_NAMESPACE_END