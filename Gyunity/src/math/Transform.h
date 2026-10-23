#pragma once
#include "Linalg.h"
#include "Ray.h"
#include "numeric/NumericUtils.h"
#include "AABB.h"
#include "visual/Intersection.h"

GYT_NAMESPACE_BEGIN

class Transform{
public:
	Transform() {
		mMat = Matrix4::Identity();
		mInvMat = Matrix4::Identity();
	}

	Transform(
		real m00, real m01, real m02, real m03,
		real m10, real m11, real m12, real m13,
		real m20, real m21, real m22, real m23,
		real m30, real m31, real m32, real m33)
	{
		mMat = Matrix4(
			m00, m01, m02, m03,
			m10, m11, m12, m13,
			m20, m21, m22, m23,
			m30, m31, m32, m33);

		mInvMat = Inverse(mMat);
	}

	Transform(const real m[4][4]) {
		mMat = Matrix4(
			m[0][0], m[0][1], m[0][2], m[0][3],
			m[1][0], m[1][1], m[1][2], m[1][3],
			m[2][0], m[2][1], m[2][2], m[2][3],
			m[3][0], m[3][1], m[3][2], m[3][3]);

		mInvMat = Inverse(mMat);
	}

	Transform(const Matrix4 &m) {
		mMat = m;
		mInvMat = Inverse(m);
	}

	Transform(const Matrix4 &m, const Matrix4 &invM) {
		mMat = m;
		mInvMat = invM;
	}

	Transform(const Transform &t) {
		mMat = t.mMat;
		mInvMat = t.mInvMat;
	}

	friend Transform Inverse(const Transform &t) {
		return Transform(t.mInvMat, t.mMat);
	}

	friend  Transform Transpose(const Transform &t) {
		return Transform(t.mMat.Transpose(), t.mInvMat.Transpose());
	}

	const Matrix4 &GetMatrix() const {
		return mMat;
	}

	const Matrix4 &GetInverseMatrix() const {
		return mInvMat;
	}

	GYT_FORCE_INLINE Vector3 operator()(const Vector3 &pos) const;
	GYT_FORCE_INLINE Vector3 operator()(const Vector3 &pos, Vector3 *pError) const;
	GYT_FORCE_INLINE Vector3 operator()(const Vector3 &pos, const Vector3 &pError, Vector3 *absError) const;
	GYT_FORCE_INLINE Ray operator()(const Ray &r, Vector3 *originError, Vector3 *directionError) const;
	GYT_FORCE_INLINE Ray operator()(const Ray &r) const;
	GYT_FORCE_INLINE Transform operator*(const Transform &t) const;
	GYT_FORCE_INLINE AABB operator()(const AABB &bound) const;
	GYT_FORCE_INLINE Intersection operator()(const Intersection& isect) const;
	GYT_FORCE_INLINE Vector3 TransformPoint(const Vector3 &pos) const;
	GYT_FORCE_INLINE Vector3 TransformPoint(const Vector3 &pos, Vector3 *pError) const;
	GYT_FORCE_INLINE Vector3 TransformPoint(const Vector3 &pos, const Vector3 &pError, Vector3 *absError) const;
	GYT_FORCE_INLINE Vector3 TransformVector(const Vector3 &v) const;
	GYT_FORCE_INLINE Vector3 TransformVector(const Vector3 &v, Vector3 *absError) const;
	GYT_FORCE_INLINE Vector3 TransformVector(const Vector3 &v, const Vector3 &vError, Vector3 *absError) const;
	GYT_FORCE_INLINE Vector3 TransformNormal(const Vector3 &normal) const;
	GYT_FORCE_INLINE Ray TransformRay(const Ray &r, Vector3 *originError, Vector3 *directionError) const;
	GYT_FORCE_INLINE Ray TransformRay(const Ray &r) const;
	GYT_FORCE_INLINE AABB TransformAABB(const AABB &bound) const;

	static Transform Translate(const Vector3 &v);
	static Transform Scale(real sx, real sy, real sz);
	static Transform RotateX(real theta);
	static Transform RotateY(real theta);
	static Transform RotateZ(real theta);
	static Transform Rotate(real theta, const Vector3 &axis);
	static Transform LookAt(const Vector3 &pos, const Vector3 &look, const Vector3 &up);
	static Transform Orthographic(real n, real f);
	static Transform Orthographic(real width, real height, real n, real f);
	static Transform Perspective(real fovy, real aspect, real dis, real n, real f);
private:
	Matrix4 mMat, mInvMat;
};

GYT_FORCE_INLINE Vector3 Transform::operator()(const Vector3 &pos) const {
	Vector4 pp(pos.x, pos.y, pos.z, 1.0);
	pp = mMat * pp;
	if(pp.w == 1 || pp.w == 0) {
		return Vector3(pp.x, pp.y, pp.z);
	}
	else {
		return Vector3(pp.x, pp.y, pp.z) / pp.w;
	}
}

GYT_FORCE_INLINE Vector3 Transform::operator()(const Vector3 &pos, Vector3 *pError) const {
	Vector4 pp(pos.x, pos.y, pos.z, 1.0);
	pp = mMat * pp;
	real xAbsSum = (std::abs(mMat(0, 0) * pos.x) + std::abs(mMat(0, 1) * pos.y) +
		std::abs(mMat(0, 2) * pos.z) + std::abs(mMat(0, 3)));
	real yAbsSum = (std::abs(mMat(1, 0) * pos.x) + std::abs(mMat(1, 1) * pos.y) +
		std::abs(mMat(1, 2) * pos.z) + std::abs(mMat(1, 3)));
	real zAbsSum = (std::abs(mMat(2, 0) * pos.x) + std::abs(mMat(2, 1) * pos.y) +
		std::abs(mMat(2, 2) * pos.z) + std::abs(mMat(2, 3)));
	*pError = Gamma(3) * Vector3(xAbsSum, yAbsSum, zAbsSum);
	if(pp.w == 1 || pp.w == 0) {
		return Vector3(pp.x, pp.y, pp.z);
	}
	else {
		return Vector3(pp.x, pp.y, pp.z) / pp.w;
	}
}

GYT_FORCE_INLINE Vector3 Transform::operator()(const Vector3 &pos, const Vector3 &pError,
	Vector3 *absError) const {
	real x = pos.x, y = pos.y, z = pos.z;
	Vector4 pp(pos.x, pos.y, pos.z, 1.0);
	pp = mMat * pp;
	absError->x =
		(Gamma(3) + (real)1) *
		(std::abs(mMat(0, 0)) * pError.x + std::abs(mMat(0, 1)) * pError.y +
			std::abs(mMat(0, 2)) * pError.z) +
		Gamma(3) * (std::abs(mMat(0, 0) * x) + std::abs(mMat(0, 1) * y) +
			std::abs(mMat(0, 2) * z) + std::abs(mMat(0, 3)));
	absError->y =
		(Gamma(3) + (real)1) *
		(std::abs(mMat(1, 0)) * pError.x + std::abs(mMat(1, 1)) * pError.y +
			std::abs(mMat(1, 2)) * pError.z) +
		Gamma(3) * (std::abs(mMat(1, 0) * x) + std::abs(mMat(1, 1) * y) +
			std::abs(mMat(1, 2) * z) + std::abs(mMat(1, 3)));
	absError->z =
		(Gamma(3) + (real)1) *
		(std::abs(mMat(2, 0)) * pError.x + std::abs(mMat(2, 1)) * pError.y +
			std::abs(mMat(2, 2)) * pError.z) +
		Gamma(3) * (std::abs(mMat(2, 0) * x) + std::abs(mMat(2, 1) * y) +
			std::abs(mMat(2, 2) * z) + std::abs(mMat(2, 3)));

	if (pp.w == 1.f || pp.w == 0) {
		return Vector3(pp.x, pp.y, pp.z);
	}
	else {
		return Vector3(pp.x, pp.y, pp.z) / pp.w;
	}

}


GYT_FORCE_INLINE Vector3 Transform::TransformVector(const Vector3 &v) const {
	Vector4 vv(v.x, v.y, v.z, 0);
	vv = mMat * vv;
	return Vector3(vv.x, vv.y, vv.z);
}

GYT_FORCE_INLINE Vector3 Transform::TransformVector(const Vector3 &v, Vector3 *absError) const {
	real x = v.x, y = v.y, z = v.z;
	absError->x =
		Gamma(3) * (std::abs(mMat(0, 0) * v.x) + std::abs(mMat(0, 1) * v.y) +
			std::abs(mMat(0, 2) * v.z));
	absError->y =
		Gamma(3) * (std::abs(mMat(1, 0) * v.x) + std::abs(mMat(1, 1) * v.y) +
			std::abs(mMat(1, 2) * v.z));
	absError->z =
		Gamma(3) * (std::abs(mMat(2, 0) * v.x) + std::abs(mMat(2, 1) * v.y) +
			std::abs(mMat(2, 2) * v.z));
	Vector4 vv(v.x, v.y, v.z, 0);
	vv = mMat * vv;
	return Vector3(vv.x, vv.y, vv.z);
}

GYT_FORCE_INLINE Vector3 Transform::TransformVector(const Vector3 &v, const Vector3 &vError,
	Vector3 *absError) const {
	real x = v.x, y = v.y, z = v.z;
	absError->x =
		(Gamma(3) + (real)1) *
		(std::abs(mMat(0, 0)) * vError.x + std::abs(mMat(0, 1)) * vError.y +
			std::abs(mMat(0, 2)) * vError.z) +
		Gamma(3) * (std::abs(mMat(0, 0) * v.x) + std::abs(mMat(0, 1) * v.y) +
			std::abs(mMat(0, 2) * v.z));
	absError->y =
		(Gamma(3) + (real)1) *
		(std::abs(mMat(1, 0)) * vError.x + std::abs(mMat(1, 1)) * vError.y +
			std::abs(mMat(1, 2)) * vError.z) +
		Gamma(3) * (std::abs(mMat(1, 0) * v.x) + std::abs(mMat(1, 1) * v.y) +
			std::abs(mMat(1, 2) * v.z));
	absError->z =
		(Gamma(3) + (real)1) *
		(std::abs(mMat(2, 0)) * vError.x + std::abs(mMat(2, 1)) * vError.y +
			std::abs(mMat(2, 2)) * vError.z) +
		Gamma(3) * (std::abs(mMat(2, 0) * v.x) + std::abs(mMat(2, 1) * v.y) +
			std::abs(mMat(2, 2) * v.z));
	Vector4 vv(v.x, v.y, v.z, 0);
	vv = mMat * vv;
	return Vector3(vv.x, vv.y, vv.z);
}

GYT_FORCE_INLINE Ray Transform::operator()(const Ray& r) const {
	Vector3 originError;
	Vector3 orig = (*this)(r.mOrig, &originError);
	Vector3 dir = this->TransformVector(r.mDir);

	real length2 = dir.Length2();
	real tMax = r.mTmax;
	if(length2 > 0) {
		real dt = Dot(Abs(dir), originError) / length2;
		orig += dir * dt;
		tMax -= dt;
	}
	return Ray(orig, dir, tMax);
}

GYT_FORCE_INLINE Ray Transform::operator()(const Ray &r, Vector3 *originError, Vector3 *directionError) const {
	Vector3 orig = (*this)(r.mOrig, originError);
	Vector3 dir = this->TransformVector(r.mDir, directionError);
	real tMax = r.mTmax;
	real length2 = dir.Length2();
	if (length2 > 0) {
		real dt = Dot(Abs(dir), *originError) / length2;
		orig += dir * dt;
		//        tMax -= dt;
	}
	return Ray(orig, dir, tMax);
}

GYT_FORCE_INLINE Vector3 Transform::TransformNormal(const Vector3 &normal) const {
	Vector4 nn(normal.x, normal.y, normal.z, 0);
	nn = mInvMat.Transpose() * nn;
	return Vector3(nn.x, nn.y, nn.z);
}

GYT_FORCE_INLINE Transform Transform::operator*(const Transform &t) const {
	return Transform(mMat * t.mMat, t.mInvMat * mInvMat);
}

GYT_FORCE_INLINE AABB Transform::operator()(const AABB &bound) const {
	const Transform& transform = *this;
	AABB ret;
	ret = Union(ret, transform(Vec3(bound.minPoint.x, bound.minPoint.y, bound.minPoint.z)));
	ret = Union(ret, transform(Vec3(bound.maxPoint.x, bound.minPoint.y, bound.minPoint.z)));
	ret = Union(ret, transform(Vec3(bound.minPoint.x, bound.maxPoint.y, bound.minPoint.z)));
	ret = Union(ret, transform(Vec3(bound.minPoint.x, bound.minPoint.y, bound.maxPoint.z)));
	ret = Union(ret, transform(Vec3(bound.minPoint.x, bound.maxPoint.y, bound.maxPoint.z)));
	ret = Union(ret, transform(Vec3(bound.maxPoint.x, bound.maxPoint.y, bound.minPoint.z)));
	ret = Union(ret, transform(Vec3(bound.maxPoint.x, bound.minPoint.y, bound.maxPoint.z)));
	ret = Union(ret, transform(Vec3(bound.maxPoint.x, bound.maxPoint.y, bound.maxPoint.z)));
	return ret;
}

GYT_FORCE_INLINE Vector3 Transform::TransformPoint(const Vector3 &pos) const {
	return (*this)(pos);
}

GYT_FORCE_INLINE Vector3 Transform::TransformPoint(const Vector3 &pos, Vector3 *pError) const {
	return (*this)(pos, pError);
}

GYT_FORCE_INLINE Vector3 Transform::TransformPoint(const Vector3 &pos, const Vector3 &pError,
	Vector3 *absError) const {
	return (*this)(pos, pError, absError);
}

GYT_FORCE_INLINE Ray Transform::TransformRay(const Ray &r, Vector3 *originError, Vector3 *directionError) const {
	return (*this)(r, originError, directionError);
}

GYT_FORCE_INLINE Ray Transform::TransformRay(const Ray &r) const {
	return (*this)(r);
}

GYT_FORCE_INLINE AABB Transform::TransformAABB(const AABB &bound) const {
	return (*this)(bound);
}

GYT_FORCE_INLINE Intersection Transform::operator()(const Intersection& isect) const {
	Intersection ret;
	ret.mPos = (*this)(isect.mPos, isect.mPointError, &ret.mPointError);
	ret.mNormal = this->TransformNormal(isect.mNormal).Norm();
	ret.mAbsNormal = this->TransformNormal(isect.mAbsNormal).Norm();
	ret.mOutDir = this->TransformVector(isect.mOutDir).Norm();
	ret.mDpDu = this->TransformVector(isect.mDpDu).Norm();
	ret.mDpDv = this->TransformVector(isect.mDpDv).Norm();
	ret.mGeometryNormal = this->TransformNormal(isect.mGeometryNormal).Norm();
	ret.mShadingDpDu = this->TransformVector(isect.mShadingDpDu).Norm();
	ret.mShadingDpDv = this->TransformVector(isect.mShadingDpDv).Norm();
	return ret;
}

GYT_NAMESPACE_END