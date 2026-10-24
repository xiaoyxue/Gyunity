#include "gtest/gtest.h"
#include "shape/Sphere.h"
#include "light/AreaLight.h"
#include <cmath>

using namespace Gyunity;

TEST(SphereLightPdf, OutsideSamplingMatchesConeDensity) {
	auto sphere = std::make_shared<Sphere>(Transform(), Transform(), 1, Vec3(0));
	sphere->mShapeId = 0;
	AreaLight light(sphere, Vec3(1));
	Intersection reference;
	reference.mPos = Vec3(0, 0, 2);
	const real expected = 1 / (2 * PI * (1 - std::sqrt(0.75f)));
	for (const Vec2& sample : { Vec2(0.1f, 0.2f), Vec2(0.3f, 0.4f), Vec2(0.8f, 0.7f) }) {
		Vec3 direction;
		real sampledPdf;
		Intersection point;
		light.SampleLi(reference, &direction, &sampledPdf, &point, sample);
		EXPECT_NEAR(point.mPos.Length(), 1, 1e-5f);
		EXPECT_NEAR(sampledPdf, expected, 1e-5f);
		EXPECT_NEAR(light.PdfLi(reference, direction), expected, 1e-5f);
	}
}

TEST(SphereLightPdf, InteriorAreaConversionUsesSquaredDistance) {
	Sphere sphere(Transform(), Transform(), 2, Vec3(0));
	sphere.mShapeId = 0;
	for (const Vec3& pos : { Vec3(0), Vec3(0.2f, 0.1f, 0.3f) }) {
		Intersection reference;
		reference.mPos = pos;
		for (const Vec2& sample : { Vec2(0.2f, 0.3f), Vec2(0.6f, 0.7f) }) {
			real sampledPdf;
			const Intersection point = sphere.Sample(reference, &sampledPdf, sample);
			const Vec3 direction = (point.mPos - pos).Norm();
			const real expected = (point.mPos - pos).Length2() /
				(std::abs(point.mNormal.Dot(-direction)) * sphere.Area());
			EXPECT_NEAR(sampledPdf, expected, 1e-5f);
			EXPECT_NEAR(sphere.Pdf(reference, direction), expected, 1e-5f);
		}
	}
}

TEST(SphereLightPdf, TranslatedSphereUsesWorldSpaceCenter) {
	const Transform transform = Transform::Translate(Vec3(4, 2, 1));
	Sphere sphere(transform, Inverse(transform), 1, Vec3(0));
	sphere.mShapeId = 0;
	Intersection reference;
	reference.mPos = Vec3(4, 2, 3);
	real sampledPdf;
	const Intersection point = sphere.Sample(reference, &sampledPdf, Vec2(0.3f, 0.4f));
	EXPECT_NEAR(sphere.Pdf(reference, (point.mPos - reference.mPos).Norm()), sampledPdf, 1e-5f);
}

TEST(SphereLightPdf, DistantLightPdfRemainsFinite) {
	Sphere sphere(Transform(), Transform(), 1, Vec3(0));
	Intersection reference;
	reference.mPos = Vec3(0, 0, 100000);
	const real actual = sphere.Pdf(reference, Vec3(0, 0, -1));
	const double expected = 1 / (2 * double(PI) * (1e-10 / (1 + std::sqrt(1 - 1e-10))));
	EXPECT_TRUE(std::isfinite(actual));
	EXPECT_NEAR(actual, expected, expected * 1e-5);
}
