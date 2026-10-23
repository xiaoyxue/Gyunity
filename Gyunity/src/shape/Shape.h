#pragma once

#include "math/Linalg.h"
#include "math/Ray.h"
#include "math/Transform.h"
#include "visual/Intersection.h"
#include "bsdf/BSDF.h"


GYT_NAMESPACE_BEGIN

class Shape {
protected:
	const Transform mObjectToWorld, mWorldToObject;
public:
	int64 mShapeId;
public:
	Shape() {}
	Shape(const Transform &ObjectToWorld, const Transform &WorldToObject) 
		: mObjectToWorld(ObjectToWorld), mWorldToObject(WorldToObject) 
	{
	}
	virtual ~Shape(){ }

	virtual bool Intersect(const Ray &r, Intersection *isect, real *t) const = 0;
	virtual bool Intersect(const Ray &r) const = 0;
	virtual Intersection Sample(real *pdf, const Vec2 &rand) const = 0;
	virtual Intersection Sample(const Intersection& isect, real* pdf, const Vec2& u) const;
	virtual AABB ObjectBound() const = 0;
	virtual AABB WorldBould() const
	{
		return mObjectToWorld(ObjectBound());
	}
	virtual real Pdf(const Intersection &isect, const Vec3 &wi) const {
		//Ray ray(isect.hit, wi);
		Ray ray = isect.SpawnRay(wi);
		Intersection lightPoint;
		real t = Inf;
		if (!Intersect(ray, &lightPoint, &t)) {
			return 0;
		}
		QueryIntersectionInfo(ray, &lightPoint);
		Vec3 d = lightPoint.mPos - isect.mPos;
		real pdf = d.Dot(d) / (std::abs(lightPoint.mNormal.Dot(-1 * wi)) * Area());
		if (std::isinf(pdf)) return 0;
		return pdf;
	}

	virtual Vec3 GetNorm(const Vec3 &point) const = 0;

	int64 GetId() const { return mShapeId; }

	virtual real Area() const = 0;

	virtual void QueryIntersectionInfo(const Ray& ray, Intersection* isect) const {}

	friend class Scene;
};

GYT_NAMESPACE_END