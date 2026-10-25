#pragma once

#include "Light.h"
#include "EnvMap.h"

GYT_NAMESPACE_BEGIN

struct Ray;
class Intersection;

class EnvironmentLight : public Light
{
public:
	EnvironmentLight(const std::string& filename, real rotate, real scale = 1.f) : mRotation(rotate) {
		mEnvMap.reset(new EnvironmentMap(filename, rotate, scale));
	}

	void Initialize(const Scene& scene);

	Vec3 Emission() const override {
		return Vec3();
	}
	Vec3 Emission(const Ray& ray) const override;
	Vec3 Emission(const Vec3& dir, const Vec3 &normal = Vec3()) const override;
	Vec3 SampleLight(Intersection* isect, Vec3* dir, real* pdfPos, real* pdfDir, const Vec2& u, const Vec2& v) const override;
	Vec3 SampleLi(const Intersection& isect, Vec3* wi, real* pdf, Intersection* lightPoint, const Vec2& u) const override;
	real PdfLi(const Intersection& isect, const Vec3& wi) const override;
	bool PdfLight(const Intersection& point, const Vec3& dir, real* pdfPos, real* pdfDir) const override;
	Vec3 Power() const override;
	bool IsEnvironmentLight() const override { return true; }
	real GetRotation() const { return mRotation; }

private:
	std::unique_ptr<EnvironmentMap> mEnvMap;
	Vec3 mCenter;
	real mRadius = 0;
	real mRotation;
};

GYT_NAMESPACE_END