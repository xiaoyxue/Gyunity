#include "gtest/gtest.h"
#include "camera/PinholeCamera.h"
#include <cmath>

using namespace Gyunity;

TEST(CameraBasis, ExplicitAxesMatchLookAtRays) {
	for (const Vec3& pos : { Vec3(0, 0, 3), Vec3(4, -2, 7) }) {
		for (real angle : { 0.f, 35.f, -70.f }) {
			const Transform rotation = Transform::RotateY(angle);
			const Vec3 cx = rotation.TransformVector(Vec3(1, 0, 0));
			const Vec3 cy = rotation.TransformVector(Vec3(0, 1, 0));
			const Vec3 cz = rotation.TransformVector(Vec3(0, 0, -1));
			auto film = std::make_shared<Film>(32, 24);
			auto referenceFilm = std::make_shared<Film>(32, 24);
			std::shared_ptr<Camera> camera = std::make_shared<PinHoleCamera>(
				film, pos, cz, cx, cy, 53.13010235415597f, 1);
			std::shared_ptr<Camera> reference = std::make_shared<PinHoleCamera>(
				referenceFilm, pos, pos + cz, cy, 53.13010235415597f, 1);
			EXPECT_EQ(camera->mCx, cx);
			EXPECT_EQ(camera->mCy, cy);
			EXPECT_EQ(camera->mCz, cz);
			EXPECT_EQ(camera->mLookAt, pos + cz);
			EXPECT_EQ(camera->mUp, cy);
			for (int x : { 0, 16, 31 }) {
				for (int y : { 0, 12, 23 }) {
					const Ray actual = camera->GenerateRay(x, y, Vec2(0.5f, 0.5f));
					const Ray expected = reference->GenerateRay(x, y, Vec2(0.5f, 0.5f));
					for (int axis = 0; axis < 3; ++axis) {
						ASSERT_TRUE(std::isfinite(actual.mDir[axis]));
						EXPECT_FLOAT_EQ(actual.mOrig[axis], pos[axis]);
						EXPECT_NEAR(actual.mDir[axis], expected.mDir[axis], 1e-5f);
					}
				}
			}
		}
	}
}
