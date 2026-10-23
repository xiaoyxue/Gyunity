#pragma once
#include "common/Core.h"
#include "Shape.h"

GYT_NAMESPACE_BEGIN

class Triangle : public Shape {
public:
	Triangle(){}

	Triangle(const Triangle& triangle) = default;

	Triangle(const Transform& ObjectToWorld, const Transform& WorldToObject,
		const Vec3& p0, const Vec3& p1, const Vec3& p2, const Vec3& normal, Vec2 *uvs = nullptr) 
		: Shape(ObjectToWorld, WorldToObject), mP0(p0), mP1(p1), mP2(p2), mFaceNormal(normal)
	{
		//faceNormal = Cross(p1 - p0, p2 - p0).Norm();
		mN0 = mN1 = mN2 = mFaceNormal;
		if (uvs) {
			this->mUvs[0] = uvs[0];
			this->mUvs[1] = uvs[1];
			this->mUvs[2] = uvs[2];
		}
		Initialize();

	}
	Triangle(const Transform &ObjectToWorld, const Transform& WorldToObject,
		const Vec3& p0, const Vec3& p1, const Vec3& p2, 
		const Vec3& n0, const Vec3& n1, const Vec3& n2, 
		Vec2* uvs = nullptr) :
		Shape(ObjectToWorld, WorldToObject), 
		mP0(p0), mP1(p1), mP2(p2), mN0(n0), mN1(n1), mN2(n2)
	{
		mFaceNormal = Cross(p1 - p0, p2 - p0).Norm();
		if (uvs) {
			this->mUvs[0] = uvs[0];
			this->mUvs[1] = uvs[1];
			this->mUvs[2] = uvs[2];
		}
		Initialize();
	}

	Triangle(const Transform& ObjectToWorld, const Transform& WorldToObject,
		const Vec3& p0, const Vec3& p1, const Vec3& p2,
		const Vec3& n0, const Vec3& n1, const Vec3& n2,
		const Vec2 &uv0, const Vec2& uv1, const Vec2& uv2) :
		Shape(ObjectToWorld, WorldToObject),
		mP0(p0), mP1(p1), mP2(p2), mN0(n0), mN1(n1), mN2(n2)
	{
		mFaceNormal = Cross(p1 - p0, p2 - p0).Norm();
		this->mUvs[0] = uv0;
		this->mUvs[1] = uv1;
		this->mUvs[2] = uv2;
		Initialize();
	}

	bool Intersect(const Ray& r, Intersection* isect, real* t) const override;

	bool Intersect(const Ray& r) const override;

	Intersection Sample(real* pdf, const Vec2& rand) const override;

	//Intersection Sample(const Intersection& isect, real* pdf, const Vec2& u) const override;

	Vec3 GetNorm(const Vec3& point) const override {
		return mFaceNormal;
	}

	real Area() const override {
		return 0.5 * Cross(mP1 - mP0, mP2 - mP0).Length();
	}

	AABB ObjectBound() const override {
		AABB ret(mP0, mP1);
		return Union(ret, mP2);
	}

	void SetTransform(const Transform& transform) {
		mP0 = transform(mP0);
		mP1 = transform(mP1);
		mP2 = transform(mP2);
		mN0 = transform.TransformNormal(mN0).Norm();
		mN1 = transform.TransformNormal(mN1).Norm();
		mN2 = transform.TransformNormal(mN2).Norm();
		//faceNormal = transform.TransformNormal(faceNormal);
		mFaceNormal = Cross(mP1 - mP0, mP2 - mP1).Norm();
		Initialize();
	}

	void QueryIntersectionInfo(const Ray& ray, Intersection* isct) const override;

private:
	void GetUVs(real uv[3][2]) const {
		if (mUvs == nullptr) {
			uv[0][0] = 0.; uv[0][1] = 0.;
			uv[1][0] = 1.; uv[1][1] = 0.;
			uv[2][0] = 1.; uv[2][1] = 1.;
		}
		else {
			uv[0][0] = mUvs[0][0]; uv[0][1] = mUvs[0][1];
			uv[1][0] = mUvs[1][0]; uv[1][1] = mUvs[1][1];
			uv[2][0] = mUvs[2][0]; uv[2][1] = mUvs[2][1];
		}

	}

	void Initialize() {
		mE1 = mP1 - mP0;
		mE2 = mP2 - mP0;
		real uvs[3][2];
		GetUVs(uvs);
		mDu1 = uvs[0][0] - uvs[2][0];
		mDu2 = uvs[1][0] - uvs[2][0];
		mDv1 = uvs[0][1] - uvs[2][1];
		mDv2 = uvs[1][1] - uvs[2][1];
		mDp1 = mP0 - mP2;
		mDp2 = mP1 - mP2;
	}

	Vec3 mP0, mP1, mP2;
	Vec3 mN0, mN1, mN2;
	Vec3 mFaceNormal;
	Vec2 mUvs[3];
	Vec3 mE1, mE2;

private:
	real mDu1; 
	real mDu2;
	real mDv1;
	real mDv2;
	Vec3 mDp1;
	Vec3 mDp2;
};

GYT_NAMESPACE_END
