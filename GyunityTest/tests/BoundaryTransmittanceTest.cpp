#include "gtest/gtest.h"
#include "visual/Scene.h"
#include "accelerator/BruteForceAccel.h"
#include "volume/Homogeneous.h"
#include "material/Diffuse.h"
#include "texture/ConstantTexture.h"
#include <cmath>

using namespace Gyunity;

namespace {

class FixedBoundarySamples : public StateSequence {
public:
	real Sample() override { return 0.3f; }
};

void AddBoundary(const std::shared_ptr<Scene>& scene, real z, const Vec3& normal,
	const MediumInterface& medium, const std::shared_ptr<Material>& material = nullptr) {
	auto shape = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-2, -2, z), Vec3(2, -2, z), Vec3(0, 2, z), normal);
	scene->AddPrimitive(std::make_shared<Primitive>(shape, material, nullptr, medium));
}

}

TEST(BoundaryTransmittance, ExitUsesVacuumAndFiniteAttenuation) {
	HomogeneousMedium medium(Vec3(0), Vec3(1), 0);
	auto scene = std::make_shared<Scene>();
	AddBoundary(scene, 0, Vec3(0, 0, 1), MediumInterface(&medium, nullptr));
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	FixedBoundarySamples samples;
	Intersection hit;
	Ray ray(Vec3(0, 0, -1), Vec3(0, 0, 1), 10, &medium);
	Vec3 transmittance;
	EXPECT_FALSE(scene->IntersectTr(ray, samples, &hit, &transmittance));
	EXPECT_EQ(ray.mMedium, nullptr);
	EXPECT_NEAR(transmittance.x, std::exp(-1.f), 1e-5f);
}

TEST(BoundaryTransmittance, CrossesSlabInBothDirections) {
	HomogeneousMedium medium(Vec3(0.1f), Vec3(0.9f), 0);
	auto scene = std::make_shared<Scene>();
	AddBoundary(scene, 0, Vec3(0, 0, -1), MediumInterface(&medium, nullptr));
	AddBoundary(scene, 1, Vec3(0, 0, 1), MediumInterface(&medium, nullptr));
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	for (bool reverse : { false, true }) {
		FixedBoundarySamples samples;
		Intersection hit;
		Ray ray(Vec3(0, 0, reverse ? 2 : -1), Vec3(0, 0, reverse ? -1 : 1));
		Vec3 transmittance;
		EXPECT_FALSE(scene->IntersectTr(ray, samples, &hit, &transmittance));
		EXPECT_EQ(ray.mMedium, nullptr);
		EXPECT_NEAR(transmittance.x, std::exp(-1.f), 1e-5f);
	}
}

TEST(BoundaryTransmittance, StopsAtMaterialAfterEnteringMedium) {
	HomogeneousMedium medium(Vec3(0), Vec3(1), 0);
	auto scene = std::make_shared<Scene>();
	AddBoundary(scene, 0, Vec3(0, 0, -1), MediumInterface(&medium, nullptr));
	auto material = std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)));
	AddBoundary(scene, 1, Vec3(0, 0, -1), MediumInterface(), material);
	scene->SetAccelerator(std::make_shared<BruteForce>(scene->GetPrimitives()));
	FixedBoundarySamples samples;
	Intersection hit;
	Ray ray(Vec3(0, 0, -1), Vec3(0, 0, 1));
	Vec3 transmittance;
	ASSERT_TRUE(scene->IntersectTr(ray, samples, &hit, &transmittance));
	EXPECT_EQ(hit.mPrimitive->GetMaterial(), material.get());
	EXPECT_EQ(ray.mMedium, &medium);
	EXPECT_NEAR(transmittance.x, std::exp(-1.f), 1e-5f);
}
