#include "gtest/gtest.h"
#include "visual/Primitive.h"
#include "shape/Sphere.h"
#include "material/Mirror.h"
#include "texture/ConstantTexture.h"
#include <array>
#include <cstring>
#include <new>

using namespace Gyunity;

TEST(BoundaryScattering, DefaultInteractionInitializesBsdf) {
	alignas(Intersection) unsigned char storage[sizeof(Intersection)];
	std::memset(storage, 0xa5, sizeof(storage));
	auto* intersection = new (storage) Intersection;
	std::array<unsigned char, sizeof(BSDF*)> actual;
	std::memcpy(actual.data(), &intersection->mBsdf, actual.size());
	BSDF* expected = nullptr;
	EXPECT_EQ(std::memcmp(actual.data(), &expected, actual.size()), 0);
	intersection->~Intersection();
}

TEST(BoundaryScattering, MediumInteractionInitializesBsdf) {
	Intersection intersection(Vec3(1, 2, 3), Vec3(0, 0, 1), MediumInterface());
	EXPECT_EQ(intersection.mBsdf, nullptr);
}

TEST(BoundaryScattering, NullMaterialClearsPreviousScattering) {
	Intersection intersection;
	intersection.mNormal = intersection.mGeometryNormal = Vec3(0, 0, 1);
	intersection.mShadingDpDu = Vec3(1, 0, 0);
	intersection.mShadingDpDv = Vec3(0, 1, 0);
	BSDF previous(intersection);
	intersection.mBsdf = &previous;
	Primitive boundary(std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3(0)));
	MemoryPool arena;
	boundary.ComputeScatteringFunction(&intersection, arena);
	EXPECT_EQ(intersection.mBsdf, nullptr);
}

TEST(BoundaryScattering, MaterialStillCreatesScattering) {
	Intersection intersection;
	intersection.mNormal = intersection.mGeometryNormal = Vec3(0, 0, 1);
	intersection.mShadingDpDu = Vec3(1, 0, 0);
	intersection.mShadingDpDv = Vec3(0, 1, 0);
	auto material = std::make_shared<MirrorMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
	Primitive surface(std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3(0)), material);
	MemoryPool arena;
	surface.ComputeScatteringFunction(&intersection, arena);
	ASSERT_NE(intersection.mBsdf, nullptr);
	EXPECT_TRUE(intersection.mBsdf->IsDelta());
}
