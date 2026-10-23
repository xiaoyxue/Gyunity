#pragma once

#include "math/Linalg.h"
#include "visual/Intersection.h"
#include "shape/Shape.h"

GYT_NAMESPACE_BEGIN

class Light {
public:
	Light() {}
	//virtual Vec3 Illumination(const Intersection &isect, const std::shared_ptr<BSDF> &bsdf,
	//	const Vec3 &importance, Vec3 *dir, Intersection *lightPoint, const Vec2 &u) const = 0;
	virtual Vec3 Emission() const { return Vec3(); }
	virtual Vec3 Emission(const Intersection& isect, const Vec3& w) const { return Vec3(); }
	virtual Vec3 Emission(const Ray& ray) const { return Vec3(); }
	virtual Vec3 Emission(const Vec3& dir, const Vec3 &normal = Vec3()) const { return Vec3(); }
	virtual void Initialize(const Scene& scene){}
	virtual Vec3 SampleLight(Intersection *isect, Vec3 *dir, real *pdfPos, real *pdfDir, const Vec2 &u, const Vec2 &v) const {
		SampleOnLight(isect, dir, pdfPos, pdfDir, u, v);
		return Emission();
	}
	virtual Vec3 SampleLi(const Intersection &isect, Vec3 *wi, real *pdf, Intersection *lightPoint, const Vec2 &u) const = 0;
	virtual real PdfLi(const Intersection &isect, const Vec3 &wi) const = 0;
	//virtual int64 GetId() const = 0;
	virtual Vec3 Power() const = 0;
	virtual bool IsAreaLight() const { return false; }
	virtual bool IsDeltaLight() const { return false; }
	virtual bool IsEnvironmentLight() const { return false; }
	virtual std::shared_ptr<Shape> GetShapePtr() const { return nullptr; }
	virtual Vec3 SampleOnePoint(Intersection* isect, real *pdf, const Vec2& u) const { return Vec3(); };
	virtual void SampleOnLight(Intersection* isect, Vec3* dir, real* pdfPos, real* pdfDir, const Vec2& u, const Vec2& v) const {}
protected:
	//virtual void SampleOnLight(Vec3 *pos, Vec3 *dir, Vec3 *lightNorm, real *pdfPos, real *pdfDir, const Vec2 &u, const Vec2 &v) const = 0;


};

GYT_NAMESPACE_END