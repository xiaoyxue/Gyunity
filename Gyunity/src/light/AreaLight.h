#pragma once

#include "math/Linalg.h"
#include "visual/Intersection.h"
#include "light/Light.h"
#include "visual/Sampling.h"
#include "math/GeometryUtils.h"
#include "shape/Shape.h"

GYT_NAMESPACE_BEGIN

class AreaLight : public Light 
{
private:
	std::shared_ptr<Shape> mShape;
	Vec3 mLemit;
public:
	AreaLight(const std::shared_ptr<Shape>& shape) : mShape(shape) {}
	AreaLight(const std::shared_ptr<Shape>& shape, const Vec3 & Lemit) : mShape(shape), mLemit(Lemit) {}

	Vec3 SampleLi(const Intersection &isect, Vec3 *wi, real *pdf, Intersection *lightPoint, const Vec2 &u) const override 
	{
		*lightPoint = mShape->Sample(isect, pdf, u);
		*wi = (lightPoint->mPos - isect.mPos).Norm();
		return Emission(*lightPoint, -*wi);
	}

	real PdfLi(const Intersection &isect, const Vec3 &wi) const override 
	{
		return mShape->Pdf(isect, wi);
	}

	bool PdfLight(const Intersection& point, const Vec3& dir, real* pdfPos, real* pdfDir) const override {
		const real area = mShape->Area();
		if (!std::isfinite(area) || area <= 0) {
			*pdfPos = *pdfDir = 0;
			GetDefaultLogger()->Error("Area-light emission PDF requires a positive finite area.", false);
			return false;
		}
		*pdfPos = 1 / area;
		*pdfDir = std::max(real(0), Dot(point.mNormal, dir)) * INV_PI;
		return true;
	}

	Vec3 Emission() const override 
	{
		return mLemit;
	}

	Vec3 Emission(const Intersection &isect, const Vec3 &w) const override 
	{
		if (Dot(isect.mNormal, w) > 0) return mLemit;
		else return Vec3(0, 0, 0);

	}

	Vec3 Power() const override 
	{
		return Emission() * mShape->Area() * PI;
	}

	bool IsAreaLight() const override { return true; }

	std::shared_ptr<Shape> GetShapePtr() const override { return mShape; }

	Vec3 SampleOnePoint(Intersection* isect, real* pdf, const Vec2& u) const override 
	{
		*isect = mShape->Sample(pdf, u);
		return mLemit;
	}

	void SampleOnLight(Intersection* isect, Vec3* dir, real* pdfPos, real* pdfDir, const Vec2& u, const Vec2& v) const override
	{
		//sample a position
		//cos importance sampling
		*isect = mShape->Sample(pdfPos, u);
		Vec3 ss, ts;
		CoordinateSystem(isect->mNormal, &ss, &ts);
		Vec3 dirLocal = CosineSampleHemisphere(v);
		real cosTheta = dirLocal.z;
		*dir = (ss * dirLocal.x + ts * dirLocal.y + isect->mNormal * dirLocal.z).Norm();
		*pdfDir = CosineHemispherePdf(cosTheta);

		//*isect = mShape->Sample(pdfPos, u);
		//Vec3 ss, ts;
		//CoordinateSystem(isect->mNormal, &ss, &ts);
		//Vec3 dirLocal = UniformSampleHemisphere(v);
		//*dir = (ss * dirLocal.x + ts * dirLocal.y + isect->mNormal * dirLocal.z).Norm();
		//*pdfDir = UniformSampleHemispherePdf();
	}

protected:



};

GYT_NAMESPACE_END