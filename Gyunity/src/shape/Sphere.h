#pragma once

#include "common/Core.h"
#include "Shape.h"
#include "math/GeometryUtils.h"
#include "common/DebugUtils.h"

GYT_NAMESPACE_BEGIN

class Sphere : public Shape 
{
private:
	real mRad;
	Vec3 mCenter;
	bool mInside;
	AABB mBbox;
	const real mThetaMin = 0;
	const real mThetaMax = PI;
	const real mPhiMax = 2 * PI;

public:
	Sphere(const Transform& ObjectToWorld, const Transform& WorldToObject,
		real radius, Vec3 center, bool inside = false) :
		Shape(ObjectToWorld, WorldToObject), mRad(radius), mCenter(center), mInside(inside) {

		mBbox = AABB(mCenter - Vec3(mRad, mRad, mRad), mCenter + Vec3(mRad, mRad, mRad));
	}

	bool Intersect(const Ray &ray, Intersection *isect, real *t) const override {
		// ray-sphere Intersection returns distance

		Ray r;
		real tHit;

		r = mWorldToObject(ray);

		Vec3 offset = r.mOrig - mCenter;
		real offsetDotDir = offset.Dot(r.mDir), dirLength2 = r.mDir.Dot(r.mDir), offsetLength2 = offset.Dot(offset);
		real det = offsetDotDir * offsetDotDir - dirLength2 * offsetLength2 + dirLength2 * mRad * mRad;


		if (det < 0) {
			//*t = r.tMax;
			return false;
		}
		else {
			det = std::sqrt(det);
		}

		tHit = (-offsetDotDir - det) / dirLength2;
		if (tHit <= 0 || tHit > r.mTmax) {
			tHit = (-offsetDotDir + det) / dirLength2;
			if (tHit <= 0 || tHit > r.mTmax) {
				return false;
			}
		}

		//GYT_DEBUG_PIXEL_IF() {
		//	std::cout << "Intersection function: tmax: " << r.tMax << " tHit: " << tHit << std::endl;
		//}

		//isect->hit = r.o + r.d * (*t);
		//Vec3 scaledDir = (isect->hit - p) * rad / Distance(isect->hit, p);
		//isect->hit = p + scaledDir;
		//isect->n = (isect->hit - p).Norm();
		//isect->nl = isect->n.Dot(r.d) < 0 ? isect->n : isect->n * -1;
		//isect->wo = -1 * r.d;
		//isect->pError = Abs(p) * gamma(1) + Abs(scaledDir) * gamma(6);

		Vec3 pos = r.mOrig + r.mDir * (tHit);
		Vec3 scaledDir = (pos - mCenter) * mRad / Distance(pos, mCenter);
		pos = mCenter + scaledDir;
		Vec3 normal = GetNorm(pos);
		Vec3 absNormal = normal.Dot(r.mDir) < 0 ? normal : normal * -1;
		Vec3 wo = -1 * r.mDir;
		Vec3 pError = Abs(mCenter) * Gamma(1) + Abs(scaledDir) * Gamma(6);

		//Compute dpdu, dpdv
		//Vec3 pHit = pos - p;
		//real phi = std::atan2(pHit.y, pHit.x);
		//if (phi < 0) phi += 2 * PI;
		//real u = phi / phiMax;
		//real theta = std::acos(Clamp(pHit.z / rad, -1, 1));
		//real v = (theta - thetaMin) / (thetaMax - thetaMin);

		//real zRadius = std::sqrt(pHit.x * pHit.x + pHit.y * pHit.y);
		//real invZRadius = 1 / zRadius;
		//real cosPhi = pHit.x * invZRadius;
		//real sinPhi = pHit.y * invZRadius;
		//Vec3 dpdu(-phiMax * pHit.y, phiMax * pHit.x, 0);
		//Vec3 dpdv = (thetaMax - thetaMin) * Vec3(pHit.z * cosPhi, pHit.z * sinPhi, -rad * std::sin(theta));


		Vec3 localPos = pos - mCenter;
		real phi = std::atan2(localPos.z, localPos.x);
		if (phi < 0) phi += 2 * PI;
		real u = phi / mPhiMax;
		real theta = std::acos(Clamp(localPos.y / mRad, -1, 1));
		real v = (theta - mThetaMin) / (mThetaMax - mThetaMin);
		v = 1.f - v;

		real yRadius = std::sqrt(localPos.x * localPos.x + localPos.z * localPos.z);
		real invYRadius = 1 / yRadius;
		real cosPhi = localPos.x * invYRadius;
		real sinPhi = localPos.z * invYRadius;
		Vec3 dpdu(-mPhiMax * localPos.z, 0, mPhiMax * localPos.x);
		Vec3 dpdv = (mThetaMax - mThetaMin) * Vec3(localPos.y * cosPhi, -mRad * std::sin(theta), localPos.y * sinPhi);

		//CoordinateSystem(GetNorm(pos), &dpdu, &dpdv);

		Intersection it(pos, normal, absNormal, dpdu, dpdv, wo, pError);

		Vec3 dpdus = (dpdu - normal * Dot(normal, dpdu)).Norm();
		Vec3 dpdvs = Cross(normal, dpdus).Norm();
		it.SetShading(normal, dpdus, dpdvs);

		*isect = mObjectToWorld(it);

		isect->mUv = Vec2(u, v);
		isect->mShapeId = mShapeId;
		*t = tHit;

		return true;

	}


	bool Intersect(const Ray &ray) const override {
		// ray-sphere Intersection returns distance

		Ray r;
		real tHit;

		r = mWorldToObject(ray);

		Vec3 offset = r.mOrig - mCenter;
		real offsetDotDir = offset.Dot(r.mDir), dirLength2 = r.mDir.Dot(r.mDir), offsetLength2 = offset.Dot(offset);
		real det = offsetDotDir * offsetDotDir - dirLength2 * offset.Dot(offset) + dirLength2 * mRad * mRad;

		if (det < 0) {
			return false;
		}
		else {
			det = sqrt(det);
		}
		tHit = (-offsetDotDir - det) / dirLength2;
		if (tHit <= 0 || tHit > r.mTmax) {
			tHit = (-offsetDotDir + det) / dirLength2;
			if (tHit <= 0 || tHit > r.mTmax) {
				return false;
			}
		}

		return true;
	}

	Intersection Sample(real *pdf, const Vec2 &u) const override {
		//*pdf = 1.f / (4.f * PI * rad * rad);
		//Vec3 pHit = UniformSampleSphere(u) * rad + p;
		//Vec3 scaledDir((pHit - p) * rad / Distance(pHit, p));
		//Intersection isect;
		//isect.hit = p + scaledDir;
		//isect.n = (pHit - p).Norm();
		//isect.nl = isect.n;

		//Vec3 pError = Abs(p) * gamma(1) + Abs(scaledDir) * gamma(6);
		//isect.pError = pError;
		//return isect;



		*pdf = 1.f / (4.f * PI * mRad * mRad);
		Vec3 samplePos = UniformSampleSphere(u) * mRad + mCenter;
		Vec3 scaledDir((samplePos - mCenter) * mRad / Distance(samplePos, mCenter));
		Intersection isect;
		Vec3 pos = mCenter + scaledDir;
		Vec3 normal = GetNorm(pos);
		Vec3 pError = Abs(mCenter) * Gamma(1) + Abs(scaledDir) * Gamma(6);

		isect.mPos = mObjectToWorld(pos, pError, &isect.mPointError);
		isect.mNormal = mObjectToWorld.TransformNormal(normal).Norm();
		isect.mGeometryNormal = isect.mNormal;

		isect.mAbsNormal = isect.mNormal;
		isect.mShapeId = mShapeId;
		return isect;
	}

	Intersection Sample(const Intersection &isect, real *pdf, const Vec2 &u) const override {
		/*
		Vec3 sw = p - isect.hit, su = ((fabs(sw.x) > .1 ? Vec3(0, 1) : Vec3(1)) % sw).Norm(), sv = sw % su;
		real cosAMax = sqrt(1 - rad * rad / (isect.hit - p).Dot(isect.hit - p));
		real zeta1 = u.x, zeta2 = u.y;
		real cosA = 1 - zeta1 + zeta1 * cosAMax;
		real sinA = sqrt(1 - cosA * cosA);
		real phi = 2 * PI * zeta2;
		Vec3 dir = su * cos(phi) * sinA + sv * sin(phi) * sinA + sw * cosA;
		real omega = 2 * PI *(1 - cosAMax);
		*pdf = 1.0 / omega;
		return isect.hit + dir.Norm() * (sw.Length() - rad);
		//return dir.Norm();*/


		//if ((p - isect.hit).Length2() <= rad * rad) {
		//	Intersection lightPoint = Sample(pdf, u);
		//	Vec3 wi = lightPoint.hit - isect.hit;
		//	if (wi.Dot(wi) == 0)
		//		*pdf = 0;
		//	else {
		//		real s = wi.Length();
		//		wi.Normalize();
		//		*pdf *= s / std::abs((lightPoint.hit - p).Norm().Dot(-1 * wi));
		//	}
		//	if (std::isinf(*pdf)) *pdf = 0.f;
		//	return lightPoint;
		//}
		//Vec3 localZ = (p - isect.hit);
		//real dis = localZ.Length();
		//localZ.Normalize();
		//Vec3 localX, localY;
		//CoordinateSystem(localZ, &localX, &localY);
		//real sin2ThetaMax = rad * rad / (dis * dis);
		//real cosThetaMax = std::sqrt(std::max(1 - sin2ThetaMax, (real)0));
		//real cosTheta = (1 - u[0]) + u[0] * cosThetaMax;
		//real sinTheta = std::sqrt(std::max(1 - cosTheta * cosTheta, (real)0));
		//real phi = 2 * PI * u[1];
		//real s = dis * cosTheta - std::sqrt(std::max(rad * rad - dis * dis * sinTheta * sinTheta, (real)0));
		//real cosAlpha = (dis * dis + rad * rad - s * s) / (2 * dis * rad);
		//real sinAlpha = std::sqrt(std::max(1 - cosAlpha * cosAlpha, (real)0));
		//Vec3 wi = sinAlpha * std::cos(phi) * localX + sinAlpha * std::sin(phi) * localY + cosTheta * localZ;
		//Vec3 nWorld = -1 * sinAlpha * std::cos(phi) * localX - sinAlpha * std::sin(phi) * localY - cosAlpha * localZ;
		//Vec3 pWorld = p + rad * nWorld + nWorld * rayeps;
		////Projection into surface and more convenient to calculate error
		//Vec3 scaledDir = (pWorld - pCenter) * rad / Distance(pWorld, pCenter);
		//pWorld = pWorld + scaledDir;
		////Vec3 wi = sinTheta * std::cos(phi) * localX + sinTheta * std::sin(phi) * localY + cosTheta * localZ;
		////Vec3 lightPoint = isect.hit + wi * s;
		//*pdf = 1 / (2 * PI * (1 - cosThetaMax));
		//Intersection ret;
		//ret.hit = pWorld;
		//ret.n = nWorld;
		//ret.nl = ret.n;
		//ret.pError = gamma(5) * Abs(pWorld);
		
		Vec3 pCenter;

		pCenter = mObjectToWorld(mCenter);

		if ((pCenter - isect.mPos).Length2() <= mRad * mRad) {
			Intersection lightPoint = Sample(pdf, u);
			Vec3 wi = lightPoint.mPos - isect.mPos;
			if (wi.Dot(wi) == 0)
				*pdf = 0;
			else {
				real s = wi.Length();
				wi.Normalize();
				*pdf *= s * s / std::abs(lightPoint.mGeometryNormal.Dot(-1 * wi));
			}
			if (std::isinf(*pdf)) *pdf = 0.f;
			return lightPoint;
		}
		Vec3 localZ = (pCenter - isect.mPos);
		real dis = localZ.Length();
		localZ.Normalize();
		Vec3 localX, localY;
		CoordinateSystem(localZ, &localX, &localY);
		real sin2ThetaMax = mRad * mRad / (dis * dis);
		real cosThetaMax = std::sqrt(std::max(1 - sin2ThetaMax, (real)0));
		real cosTheta = (1 - u[0]) + u[0] * cosThetaMax;
		real sinTheta = std::sqrt(std::max(1 - cosTheta * cosTheta, (real)0));
		real phi = 2 * PI * u[1];
		real s = dis * cosTheta - std::sqrt(std::max(mRad * mRad - dis * dis * sinTheta * sinTheta, (real)0));
		real cosAlpha = (dis * dis + mRad * mRad - s * s) / (2 * dis * mRad);
		real sinAlpha = std::sqrt(std::max(1 - cosAlpha * cosAlpha, (real)0));
		Vec3 wi = sinAlpha * std::cos(phi) * localX + sinAlpha * std::sin(phi) * localY + cosTheta * localZ;
		Vec3 nWorld = -1 * sinAlpha * std::cos(phi) * localX - sinAlpha * std::sin(phi) * localY - cosAlpha * localZ;
		nWorld.Normalize();
		Vec3 pWorld = pCenter + mRad * nWorld / nWorld.Length();
		//Projection into surface and more convenient to calculate error
		//Vec3 scaledDir = (pWorld - pCenter) * rad / Distance(pWorld, pCenter);
		//pWorld = pWorld + scaledDir;
		//Vec3 wi = sinTheta * std::cos(phi) * localX + sinTheta * std::sin(phi) * localY + cosTheta * localZ;
		//Vec3 lightPoint = isect.hit + wi * s;
		*pdf = ConePdf(sin2ThetaMax);
		Intersection ret;
		ret.mPos = pWorld;
		ret.mNormal = nWorld;
		ret.mGeometryNormal = nWorld;
		ret.mAbsNormal = ret.mNormal;
		//ret.pError = gamma(6) * Abs(pWorld) + gamma(6) * Abs(scaledDir);
		ret.mPointError = Gamma(5) * Abs(pWorld);
		ret.mShapeId = mShapeId;
		return ret;


		//if ((p - isect.hit).Length2() <= rad * rad) {
		//	Intersection lightPoint = Sample(pdf, u);
		//	Vec3 wi = lightPoint.hit - isect.hit;
		//	if (wi.Dot(wi) == 0)
		//		*pdf = 0;
		//	else {
		//		real s = wi.Length();
		//		wi.Normalize();
		//		*pdf *= s / std::abs((lightPoint.hit - p).Norm().Dot(-1 * wi));
		//	}
		//	if (std::isinf(*pdf)) *pdf = 0.f;
		//	return lightPoint;
		//}
		//Vec3 localZ = (p - isect.hit);
		//real dis = localZ.Length();
		//localZ.Normalize();
		//Vec3 localX, localY;
		//CoordinateSystem(localZ, &localX, &localY);
		//real sin2ThetaMax = rad * rad / (dis * dis);
		//real cosThetaMax = std::sqrt(std::max(1 - sin2ThetaMax, (real)0));
		//real cosTheta = (1 - u[0]) + u[0] * cosThetaMax;
		//real sinTheta = std::sqrt(std::max(1 - cosTheta * cosTheta, (real)0));
		//real phi = 2 * PI * u[1];
		//real s = dis * cosTheta - std::sqrt(std::max(rad * rad - dis * dis * sinTheta * sinTheta, (real)0));
		//real cosAlpha = (dis * dis + rad * rad - s * s) / (2 * dis * rad);
		//real sinAlpha = std::sqrt(std::max(1 - cosAlpha * cosAlpha, (real)0));
		//Vec3 wi = sinAlpha * std::cos(phi) * localX + sinAlpha * std::sin(phi) * localY + cosTheta * localZ;
		//Vec3 nWorld = -1 * sinAlpha * std::cos(phi) * localX - sinAlpha * std::sin(phi) * localY - cosAlpha * localZ;
		//Vec3 pWorld = p + rad * nWorld + nWorld * rayeps;
		////Vec3 wi = sinTheta * std::cos(phi) * localX + sinTheta * std::sin(phi) * localY + cosTheta * localZ;
		////Vec3 lightPoint = isect.hit + wi * s;
		//*pdf = 1 / (2 * PI * (1 - cosThetaMax));
		//Intersection ret;
		//ret.hit = pWorld;
		//ret.n = nWorld;
		//ret.nl = ret.n;
		//ret.pError = gamma(5) * Abs(pWorld);
		////std::cout << ret.hit << " " << ret.pError << std::endl;
		//return ret;
	}

	real Pdf(const Intersection& isect, const Vec3& wi) const override {
		const Vec3 center = mObjectToWorld(mCenter);
		const real distance2 = (center - isect.mPos).Length2();
		if (distance2 <= mRad * mRad) return Shape::Pdf(isect, wi);
		return ConePdf(mRad * mRad / distance2);
	}

	Vec3 GetNorm(const Vec3 & point) const override {
		//return (point - p).Norm();
		if (mInside) return (mCenter - point).Norm();
		else return (point - mCenter).Norm();
	}

	real Area() const override {
		return 4.f * PI * mRad * mRad;
	}

	AABB ObjectBound() const override {
		return mBbox;
	}

private:
	static real ConePdf(real sin2ThetaMax) {
		const real cosThetaMax = std::sqrt(std::max(1 - sin2ThetaMax, real(0)));
		// Avoid cancellation in 1 - cosThetaMax for distant lights.
		const real oneMinusCosThetaMax = sin2ThetaMax / (1 + cosThetaMax);
		return 1 / (2 * PI * oneMinusCosThetaMax);
	}
};

GYT_NAMESPACE_END