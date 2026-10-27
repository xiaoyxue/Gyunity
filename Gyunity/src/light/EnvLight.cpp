#include "EnvLight.h"
#include "visual/Scene.h"

GYT_NAMESPACE_BEGIN

void EnvironmentLight::Initialize(const Scene& scene) 
{
	scene.GetBoundingSphere(&mCenter, &mRadius);
}

Vec3 EnvironmentLight::Emission(const Ray& ray) const
{
	Vec3 w = ray.mDir.Norm();
	return mEnvMap->Lookup(w);
}

Vec3 EnvironmentLight::Emission(const Vec3& dir, const Vec3 &normal) const 
{
	return mEnvMap->Lookup(dir.Norm());
}

Vec3 EnvironmentLight::SampleLight(Intersection* isect, Vec3* dir, real* pdfPos,
	real* pdfDir, const Vec2& u, const Vec2& v) const 
{
	Vec3 sampledDir;
	Vec3 radiance = mEnvMap->Sample(&sampledDir, pdfDir, u);
	*dir = -sampledDir;
	isect->mNormal = *dir;

	Vec3 tangent, bitangent;
	CoordinateSystem(-*dir, &tangent, &bitangent);
	Vec2 diskSample = ConcentricSampleDisk(v);
	Vec3 pDisk = mCenter + mRadius * (diskSample.x * tangent + diskSample.y * bitangent);
	isect->mPos = pDisk + mRadius * (-*dir);

	*pdfPos = 1 / (PI * mRadius * mRadius);
	//std::cout << radiance << " pdfDir: " << *pdfDir << " pdfPos: " << *pdfPos << std::endl;
	return radiance;
}


Vec3 EnvironmentLight::SampleLi(const Intersection& isect, Vec3* wi, real* pdf,
	Intersection* lightPoint, const Vec2& u) const 
{
	Vec3 sampledDir;
	Vec3 radiance = mEnvMap->Sample(&sampledDir, pdf, u);
	*wi = sampledDir.Norm();
	lightPoint->mPos = isect.mPos + *wi * (2 * mRadius);
	lightPoint->mNormal = lightPoint->mAbsNormal = lightPoint->mGeometryNormal = *wi;
	return radiance;
}

real EnvironmentLight::PdfLi(const Intersection& isect, const Vec3& wi) const 
{
	return mEnvMap->PdfDir(wi);
}

bool EnvironmentLight::PdfLight(const Intersection& point, const Vec3& dir, real* pdfPos, real* pdfDir) const {
	if (!std::isfinite(mRadius) || mRadius <= 0) {
		*pdfPos = *pdfDir = 0;
		GetDefaultLogger()->Error("Environment emission PDF requires initialized, nonempty scene bounds.", false);
		return false;
	}
	*pdfPos = 1 / (PI * mRadius * mRadius);
	// Emitted rays travel into the scene, opposite the direction used to look up the map.
	*pdfDir = mEnvMap->PdfDir(-dir);
	return true;
}


Vec3 EnvironmentLight::Power() const 
{
	return mEnvMap->RadianceIntegral() * PI * mRadius * mRadius;
}


GYT_NAMESPACE_END