#include "gtest/gtest.h"
#include "material/Plastic.h"
#include "texture/ConstantTexture.h"
#include <cmath>
#include <limits>

GYT_NAMESPACE_BEGIN

namespace {

const auto kDelta = ScatterEventType(BSDF_REFLECTION | BSDF_SPECULAR);
const auto kDiffuse = ScatterEventType(BSDF_REFLECTION | BSDF_DIFFUSE);
const Vec3 kKd(0.240f, 0.771f, 0.361f);

Intersection PlasticPoint() {
	Intersection hit;
	hit.mNormal = hit.mGeometryNormal = Vec3(0, 0, 1);
	hit.mShadingDpDu = Vec3(1, 0, 0);
	hit.mShadingDpDv = Vec3(0, 1, 0);
	return hit;
}

std::shared_ptr<PlasticMaterial> Plastic(const Vec3& kd = kKd, const Vec3& ks = Vec3(1),
	real outside = 1.000277f, real inside = 1.9f, bool nonlinear = false) {
	return PlasticMaterial::Create(std::make_shared<ConstantTexture<Vec3>>(kd),
		std::make_shared<ConstantTexture<Vec3>>(ks), outside, inside, nonlinear);
}

void ExpectColor(const Vec3& actual, const Vec3& expected, real tolerance = 1e-6f) {
	for (int c = 0; c < 3; ++c) EXPECT_NEAR(actual[c], expected[c], tolerance) << "channel " << c;
}

struct PlasticReference {
	real outside, inside;
	bool nonlinear;
	Vec3 normal, oblique;
	real normalPdf, obliquePdf;
	real normalMass, obliqueMass;
	real normalFresnel, obliqueFresnel;
};

class HitReflectance : public Texture<Vec3> {
public:
	HitReflectance(int coordinate, const Vec3& color) : mCoordinate(coordinate), mColor(color) {}
	Vec3 Sample(const Intersection& hit) const override {
		return hit.mUv[mCoordinate] > 0 ? mColor : Vec3(0);
	}
private:
	int mCoordinate;
	Vec3 mColor;
};

// Stored independent double-precision formulas from Test\smooth-plastic\reference\plugin_table.json.
// Source: Mitsuba aa01500, plastic.cpp / fresnel.h. The separate 3.9.1 scalar_rgb wheel
// agrees within float tolerance; it is not an execution of that source revision.
const PlasticReference kReferences[] = {
	{ 1.000277f, 1.9f, false,
		Vec3(0.073143696294f, 0.234974124345f, 0.110020309842f),
		Vec3(0.070587759131f, 0.226763176208f, 0.106175754359f),
		0.258193123570f, 0.194759374497f, 0.188862379784f, 0.235181724830f,
		0.096236249249f, 0.123291562696f },
	{ 1, 1.5f, false,
		Vec3(0.077609120474f, 0.249319299522f, 0.116737052046f),
		Vec3(0.075319634514f, 0.241964325877f, 0.113293283582f),
		0.291730904372f, 0.221274929860f, 0.083500334001f, 0.131055382412f,
		0.04f, 0.064524971331f },
	{ 1, 1, false,
		Vec3(0.076410189593f, 0.245467734069f, 0.114933660180f),
		Vec3(0.076410189593f, 0.245467734069f, 0.114933660180f),
		0.318309886184f, 0.254647908947f, 0, 0, 0, 0 },
	{ 1.000277f, 1.9f, true,
		Vec3(0.021174701615f, 0.135079998120f, 0.035912725979f),
		Vec3(0.020434771730f, 0.130359755574f, 0.034657789797f),
		0.258193123570f, 0.194759374497f, 0.188862379784f, 0.235181724830f,
		0.096236249249f, 0.123291562696f }
};

}

TEST(SmoothPlastic, DiffuseFresnelPublishedFits) {
	EXPECT_NEAR(FrDiffuseDielectric(1.000277f / 1.9f), 0.7635563441483434, 1e-6);
	EXPECT_NEAR(FrDiffuseDielectric(1.9f / 1.000277f), 0.14741444071935794, 1e-6);
	EXPECT_NEAR(FrDiffuseDielectric(1 / 1.5f), 0.5968111111111111, 1e-6);
	EXPECT_NEAR(FrDiffuseDielectric(1.5f), 0.09177428395061771, 1e-6);
	EXPECT_NEAR(FrDiffuseDielectric(1), 0.000207, 1e-6);
}

TEST(SmoothPlastic, ActualMaterialMatchesIndependentSourceFixtures) {
	for (const auto& reference : kReferences) {
		SCOPED_TRACE(testing::Message() << "inside=" << reference.inside << " outside=" <<
			reference.outside << " nonlinear=" << reference.nonlinear);
		const auto material = Plastic(kKd, Vec3(1), reference.outside, reference.inside, reference.nonlinear);
		ASSERT_NE(material, nullptr);
		for (TransportMode mode : { TransportMode::Radiance, TransportMode::Importance }) {
			MemoryPool arena;
			Intersection hit = PlasticPoint();
			material->ComputeScatteringFunction(&hit, arena, mode);
			ASSERT_NE(hit.mBsdf, nullptr);
			EXPECT_FALSE(hit.mBsdf->IsDelta());
			EXPECT_TRUE(hit.mBsdf->HasNonDeltaLobes());
			EXPECT_TRUE(hit.mBsdf->HasMixedDeltaLobes());
			for (bool oblique : { false, true }) {
				const Vec3 wo = oblique ? Vec3(0.8f, 0, 0.6f) : Vec3(0, 0, 1);
				const Vec3 wi = oblique ? Vec3(0, 0.6f, 0.8f) : Vec3(0, 0, 1);
				const Vec3 expected = oblique ? reference.oblique : reference.normal;
				const real mass = oblique ? reference.obliqueMass : reference.normalMass;
				const real fresnel = oblique ? reference.obliqueFresnel : reference.normalFresnel;
				ExpectColor(hit.mBsdf->Evaluate(wo, wi), expected);
				ExpectColor(hit.mBsdf->Evaluate(wi, wo), expected);
				ExpectColor(hit.mBsdf->Evaluate(wo, wi, kDiffuse), expected);
				EXPECT_NEAR(hit.mBsdf->Pdf(wo, wi),
					oblique ? reference.obliquePdf : reference.normalPdf, 1e-6f);
				EXPECT_NEAR(hit.mBsdf->Pdf(wo, wi, kDiffuse), wi.z * INV_PI, 1e-7f);
				EXPECT_EQ(hit.mBsdf->Evaluate(wo, wi, kDelta), Vec3(0));
				EXPECT_EQ(hit.mBsdf->Pdf(wo, wi, kDelta), 0);
				Vec3 sampledWi(7);
				real pdf = 7, eventProbability = 7;
				ScatterEventType type = BSDF_ALL;
				const Vec3 f = hit.mBsdf->Sample(wo, &sampledWi, &pdf, Vec3(0), kDelta, &type, &eventProbability);
				if (mass == 0) {
					EXPECT_EQ(f, Vec3(0));
					EXPECT_EQ(pdf, 0);
					EXPECT_EQ(sampledWi, Vec3(0));
					EXPECT_EQ(type, ScatterEventType(0));
					EXPECT_EQ(eventProbability, 0);
				} else {
					EXPECT_EQ(sampledWi, Vec3(-wo.x, -wo.y, wo.z));
					EXPECT_EQ(pdf, 1);
					EXPECT_EQ(type, kDelta);
					EXPECT_EQ(eventProbability, 1);
					ExpectColor(f * sampledWi.z, Vec3(fresnel));
					const Vec3 mixedF = hit.mBsdf->Sample(wo, &sampledWi, &pdf, Vec3(0), BSDF_ALL, &type, &eventProbability);
					EXPECT_EQ(type, kDelta);
					EXPECT_NEAR(pdf, mass, 1e-6f);
					EXPECT_NEAR(eventProbability, mass, 1e-6f);
					ExpectColor(mixedF * sampledWi.z, Vec3(fresnel));
					EXPECT_NE(pdf, hit.mBsdf->Pdf(wo, sampledWi));
				}
				hit.mBsdf->Sample(wo, &sampledWi, &pdf, Vec3(0.75f, 0.5f, 0), BSDF_ALL, &type, &eventProbability);
				EXPECT_EQ(type, kDiffuse);
				EXPECT_NEAR(eventProbability, 1 - mass, 1e-6f);
			}
		}
	}
}

TEST(SmoothPlastic, RemappedDiffuseSamplesMatchStoredWheelThroughput) {
	const auto material = Plastic();
	ASSERT_NE(material, nullptr);
	MemoryPool arena;
	Intersection hit = PlasticPoint();
	material->ComputeScatteringFunction(&hit, arena);
	for (bool oblique : { false, true }) {
		const Vec3 wo = oblique ? Vec3(0.8f, 0, 0.6f) : Vec3(0, 0, 1);
		const real mass = oblique ? kReferences[0].obliqueMass : kReferences[0].normalMass;
		const Vec3 random(mass + (1 - mass) * 0.25f, 0.75f, 0);
		Vec3 wi;
		real pdf, eventProbability;
		ScatterEventType type;
		const Vec3 f = hit.mBsdf->Sample(wo, &wi, &pdf, random, BSDF_ALL, &type, &eventProbability);
		EXPECT_EQ(type, kDiffuse);
		EXPECT_NEAR(eventProbability, 1 - mass, 1e-6f);
		ExpectColor(wi, Vec3(-0.353553385f, 0.353553414f, 0.866025388f));
		EXPECT_NEAR(pdf, oblique ? 0.210833192f : 0.223601773f, 1e-6f);
		ExpectColor(f * wi.z / pdf, oblique ?
			Vec3(0.290849864f, 0.934355319f, 0.437486678f) :
			Vec3(0.282704264f, 0.908187568f, 0.425234348f));
		ExpectColor(f, hit.mBsdf->Evaluate(wo, wi));
		EXPECT_NEAR(pdf, hit.mBsdf->Pdf(wo, wi), 1e-7f);
	}
}

TEST(SmoothPlastic, OneSidedSupportAndExcludedFlagsAreNullEvents) {
	const auto material = Plastic();
	ASSERT_NE(material, nullptr);
	MemoryPool arena;
	Intersection hit = PlasticPoint();
	material->ComputeScatteringFunction(&hit, arena);
	for (const Vec3& wo : { Vec3(0, 0, -1), Vec3(1, 0, 0) }) {
		for (ScatterEventType flags : { BSDF_ALL, kDelta, kDiffuse }) {
			Vec3 wi(7);
			real pdf = 7, eventProbability = 7;
			ScatterEventType type = BSDF_ALL;
			EXPECT_EQ(hit.mBsdf->Evaluate(wo, Vec3(0, 0, 1), flags), Vec3(0));
			EXPECT_EQ(hit.mBsdf->Pdf(wo, Vec3(0, 0, 1), flags), 0);
			EXPECT_EQ(hit.mBsdf->Sample(wo, &wi, &pdf, Vec3(0.5f), flags, &type, &eventProbability), Vec3(0));
			EXPECT_EQ(eventProbability, 0);
			EXPECT_EQ(wi, Vec3(0));
			EXPECT_EQ(pdf, 0);
			EXPECT_EQ(type, ScatterEventType(0));
		}
		EXPECT_EQ(hit.mBsdf->Evaluate(Vec3(0, 0, 1), wo), Vec3(0));
		EXPECT_EQ(hit.mBsdf->Pdf(Vec3(0, 0, 1), wo), 0);
	}
	for (const auto flags : { ScatterEventType(BSDF_TRANSMISSION | BSDF_ALL_TYPES),
		ScatterEventType(BSDF_REFLECTION | BSDF_GLOSSY) }) {
		Vec3 wi(7);
		real pdf = 7, eventProbability = 7;
		ScatterEventType type = BSDF_ALL;
		const Vec3 wo(0, 0, 1);
		EXPECT_EQ(hit.mBsdf->Evaluate(wo, wo, flags), Vec3(0));
		EXPECT_EQ(hit.mBsdf->Pdf(wo, wo, flags), 0);
		EXPECT_EQ(hit.mBsdf->Sample(wo, &wi, &pdf, Vec3(0.5f), flags, &type, &eventProbability), Vec3(0));
		EXPECT_EQ(eventProbability, 0);
		EXPECT_EQ(wi, Vec3(0));
		EXPECT_EQ(pdf, 0);
		EXPECT_EQ(type, ScatterEventType(0));
	}
}

TEST(SmoothPlastic, BlackComponentsAreOmittedAndBlackBothAbsorbs) {
	for (bool blackDiffuse : { false, true }) {
		for (bool blackSpecular : { false, true }) {
			const auto material = Plastic(blackDiffuse ? Vec3(0) : kKd, blackSpecular ? Vec3(0) : Vec3(1));
			ASSERT_NE(material, nullptr);
			MemoryPool arena;
			Intersection hit = PlasticPoint();
			material->ComputeScatteringFunction(&hit, arena);
			ASSERT_NE(hit.mBsdf, nullptr);
			EXPECT_EQ(hit.mBsdf->IsDelta(), blackDiffuse && !blackSpecular);
			EXPECT_EQ(hit.mBsdf->HasNonDeltaLobes(), !blackDiffuse);
			EXPECT_EQ(hit.mBsdf->HasMixedDeltaLobes(), !blackDiffuse && !blackSpecular);
			const Vec3 wo(0, 0, 1);
			const Vec3 value = hit.mBsdf->Evaluate(wo, wo);
			ExpectColor(value, blackDiffuse ? Vec3(0) : kReferences[0].normal);
			Vec3 wi(7);
			real pdf = 7, eventProbability = 7;
			ScatterEventType type = BSDF_ALL;
			const Vec3 f = hit.mBsdf->Sample(wo, &wi, &pdf, Vec3(0.5f), BSDF_ALL, &type, &eventProbability);
			if (blackDiffuse && blackSpecular) {
				EXPECT_EQ(f, Vec3(0));
				EXPECT_EQ(wi, Vec3(0));
				EXPECT_EQ(pdf, 0);
				EXPECT_EQ(type, ScatterEventType(0));
				EXPECT_EQ(eventProbability, 0);
			} else if (blackDiffuse) {
				EXPECT_EQ(type, kDelta);
				EXPECT_EQ(pdf, 1);
				EXPECT_EQ(eventProbability, 1);
				EXPECT_EQ(hit.mBsdf->Pdf(wo, wo), 0);
				ExpectColor(f, Vec3(kReferences[0].normalFresnel));
			} else {
				EXPECT_EQ(type, kDiffuse);
				EXPECT_NEAR(pdf, hit.mBsdf->Pdf(wo, wi), 1e-7f);
				EXPECT_NEAR(eventProbability, blackSpecular ? 1 : 1 - kReferences[0].normalMass, 1e-6f);
				if (blackSpecular) {
					EXPECT_NEAR(pdf, wi.z * INV_PI, 1e-7f);
				}
			}
		}
	}
}

TEST(SmoothPlastic, EqualIndicesWithoutDiffuseIsFiniteAbsorption) {
	const auto material = Plastic(Vec3(0), Vec3(1), 1, 1);
	ASSERT_NE(material, nullptr);
	MemoryPool arena;
	Intersection hit = PlasticPoint();
	material->ComputeScatteringFunction(&hit, arena);
	for (const Vec3& wo : { Vec3(0, 0, 1), Vec3(0.8f, 0, 0.6f) }) {
		for (ScatterEventType flags : { BSDF_ALL, kDelta, kDiffuse }) {
			Vec3 wi(7);
			real pdf = 7, eventProbability = 7;
			ScatterEventType type = BSDF_ALL;
			EXPECT_EQ(hit.mBsdf->Sample(wo, &wi, &pdf, Vec3(0.5f), flags, &type, &eventProbability), Vec3(0));
			EXPECT_EQ(wi, Vec3(0));
			EXPECT_EQ(pdf, 0);
			EXPECT_EQ(type, ScatterEventType(0));
			EXPECT_EQ(hit.mBsdf->Pdf(wo, wo, flags), 0);
			EXPECT_EQ(eventProbability, 0);
		}
	}
}

TEST(SmoothPlastic, ComponentPresenceAndWeightsFollowPerHitTextures) {
	const auto material = PlasticMaterial::Create(std::make_shared<HitReflectance>(0, kKd),
		std::make_shared<HitReflectance>(1, Vec3(1)), 1.000277f, 1.9f, false);
	ASSERT_NE(material, nullptr);
	MemoryPool arena;
	for (int diffuse = 0; diffuse < 2; ++diffuse) {
		for (int specular = 0; specular < 2; ++specular) {
			Intersection hit = PlasticPoint();
			hit.mUv = Vec2(real(diffuse), real(specular));
			material->ComputeScatteringFunction(&hit, arena);
			ASSERT_NE(hit.mBsdf, nullptr);
			EXPECT_EQ(hit.mBsdf->HasNonDeltaLobes(), diffuse != 0);
			EXPECT_EQ(hit.mBsdf->IsDelta(), specular != 0 && diffuse == 0);
			Vec3 wi;
			real pdf, eventProbability;
			ScatterEventType type;
			hit.mBsdf->Sample(Vec3(0, 0, 1), &wi, &pdf, Vec3(0, 0.5f, 0), BSDF_ALL, &type, &eventProbability);
			const real expected = specular ? (diffuse ? kReferences[0].normalMass : 1) : (diffuse ? 1 : 0);
			EXPECT_NEAR(eventProbability, expected, 1e-6f);
			EXPECT_EQ(type, specular ? kDelta : (diffuse ? kDiffuse : ScatterEventType(0)));
		}
	}
}

TEST(SmoothPlastic, ColoredSpecularUsesArithmeticMeanForSelection) {
	const Vec3 ks(0.1f, 0.7f, 0.4f);
	const auto material = Plastic(kKd, ks);
	ASSERT_NE(material, nullptr);
	MemoryPool arena;
	Intersection hit = PlasticPoint();
	material->ComputeScatteringFunction(&hit, arena);
	const real fresnel = kReferences[0].normalFresnel;
	const real expectedMass = 0.4f * fresnel / (0.4f * fresnel + (1.372f / 3) * (1 - fresnel));
	const Vec3 wo(0, 0, 1);
	Vec3 wi;
	real pdf, eventProbability;
	const Vec3 f = hit.mBsdf->Sample(wo, &wi, &pdf, Vec3(0), BSDF_ALL, nullptr, &eventProbability);
	EXPECT_NEAR(pdf, expectedMass, 1e-6f);
	EXPECT_NEAR(eventProbability, expectedMass, 1e-6f);
	ExpectColor(f, ks * fresnel);
}

TEST(SmoothPlastic, TotalInternalReflectionSelectsOnlyDelta) {
	const auto material = Plastic(kKd, Vec3(1), 1.5f, 1);
	ASSERT_NE(material, nullptr);
	MemoryPool arena;
	Intersection hit = PlasticPoint();
	material->ComputeScatteringFunction(&hit, arena);
	const Vec3 wo(0.8f, 0, 0.6f);
	EXPECT_EQ(hit.mBsdf->Evaluate(wo, Vec3(0, 0, 1)), Vec3(0));
	EXPECT_EQ(hit.mBsdf->Pdf(wo, Vec3(0, 0, 1)), 0);
	Vec3 wi;
	real pdf, eventProbability;
	ScatterEventType type;
	const Vec3 f = hit.mBsdf->Sample(wo, &wi, &pdf, Vec3(0.5f), BSDF_ALL, &type, &eventProbability);
	EXPECT_EQ(type, kDelta);
	EXPECT_EQ(pdf, 1);
	EXPECT_EQ(eventProbability, 1);
	ExpectColor(f * wi.z, Vec3(1));
	EXPECT_EQ(hit.mBsdf->Sample(wo, &wi, &pdf, Vec3(0.5f), kDiffuse, &type, &eventProbability), Vec3(0));
	EXPECT_EQ(type, ScatterEventType(0));
	EXPECT_EQ(eventProbability, 0);
}

TEST(SmoothPlastic, FactoryRejectsInvalidStaticParametersAndFittedDenominators) {
	const auto color = std::make_shared<ConstantTexture<Vec3>>(kKd);
	EXPECT_EQ(PlasticMaterial::Create(nullptr, color, 1, 1.5f, false), nullptr);
	EXPECT_EQ(PlasticMaterial::Create(color, nullptr, 1, 1.5f, false), nullptr);
	for (real invalid : { real(0), real(-1), std::numeric_limits<real>::infinity(),
		std::numeric_limits<real>::quiet_NaN() }) {
		EXPECT_EQ(PlasticMaterial::Create(color, color, invalid, 1.5f, false), nullptr);
		EXPECT_EQ(PlasticMaterial::Create(color, color, 1, invalid, false), nullptr);
	}
	const real largest = (std::numeric_limits<real>::max)();
	const real smallest = (std::numeric_limits<real>::min)();
	for (bool nonlinear : { false, true }) {
		EXPECT_EQ(PlasticMaterial::Create(color, color, smallest, largest, nonlinear), nullptr);
		EXPECT_EQ(PlasticMaterial::Create(color, color, largest, smallest, nonlinear), nullptr);
		EXPECT_EQ(PlasticMaterial::Create(color, color, 1, smallest, nonlinear), nullptr);
		EXPECT_EQ(PlasticMaterial::Create(color, color, 1, largest, nonlinear), nullptr);
		for (real inside : { 4.f, 5.f }) {
			ASSERT_GE(FrDiffuseDielectric(1 / inside), 1);
			EXPECT_EQ(PlasticMaterial::Create(color, color, 1, inside, nonlinear), nullptr);
		}
		EXPECT_NE(PlasticMaterial::Create(color, color, 1, 3, nonlinear), nullptr);
	}
}

GYT_NAMESPACE_END
