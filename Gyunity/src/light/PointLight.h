#pragma once
#include "Light.h"
#include "visual/Sampling.h"

GYT_NAMESPACE_BEGIN

class PointLight : public Light {
public:
	PointLight(const Vec3& pos, const Vec3& intensity) : mLightPosition(pos), mI(intensity) {}

	Vec3 Emission() const override { return mI; }

	bool IsDeltaLight() const override { return true; }

	Vec3 SampleLi(const Intersection& isect, Vec3* wi, real* pdf, Intersection* lightPoint, const Vec2& u) const override;

	real PdfLi(const Intersection& isect, const Vec3& wi) const override 
	{
		return 0;
	}

	GYT_FORCE_INLINE Vec3 Power() const override 
	{
		return 4 * PI * mI;
	}

	Vec3 SampleLight(Intersection* isect, Vec3* dir, real* pdfPos, real* pdfDir, const Vec2& u, const Vec2& v) const override;

	bool PdfLight(const Intersection& point, const Vec3& dir, real* pdfPos, real* pdfDir) const override {
		// A discrete mass at the point source, never a finite-area density.
		*pdfPos = point.mPos == mLightPosition ? 1 : 0;
		*pdfDir = UniformSpherePdf();
		return true;
	}

	Vec3 SampleOnePoint(Intersection* isect, real* pdf, const Vec2& u) const override 
	{
		isect->mPos = mLightPosition;
		*pdf = 1.f;
		return mI;
	}
private:
	Vec3 mLightPosition;
	Vec3 mI;
};

GYT_NAMESPACE_END