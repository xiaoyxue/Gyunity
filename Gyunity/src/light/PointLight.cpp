#include "PointLight.h"
#include "visual/Sampling.h"

GYT_NAMESPACE_BEGIN

Vec3 PointLight::SampleLi(const Intersection& isect, Vec3* wi, real* pdf, Intersection* lightPoint, const Vec2& u) const 
{
	lightPoint->mPos = mLightPosition;
	Vec3 dir = isect.mPos - mLightPosition;
	*wi = -dir.Norm();
	lightPoint->mNormal = lightPoint->mAbsNormal = lightPoint->mGeometryNormal = -(*wi);
	*pdf = 1.f;
	return mI / dir.Length2();
}


Vec3 PointLight::SampleLight(Intersection* isect, Vec3* dir, real* pdfPos, real* pdfDir, const Vec2& u, const Vec2& v) const 
{
	*pdfPos = 1;
	Vec3 sampledDir = UniformSampleSphere(v);
	*dir = sampledDir.Norm();
	*pdfDir = UniformSpherePdf();
	isect->mPos = mLightPosition;
	isect->mNormal = isect->mGeometryNormal = isect->mAbsNormal = *dir;
	return mI;
}

GYT_NAMESPACE_END