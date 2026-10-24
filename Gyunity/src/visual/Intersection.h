#pragma once

#include "math/Linalg.h"
#include "math/Ray.h"
#include "math/GeometryUtils.h"
#include "material/Material.h"
#include "visual/Medium.h"
#include "ForwardDecl.h"

GYT_NAMESPACE_BEGIN

class Intersection {
public:
	Vec3 mPos, mNormal, mAbsNormal, mGeometryNormal, mOutDir;
	Vec3 mDpDu, mDpDv;
	Vec3 mShadingDpDu, mShadingDpDv;
	Vec2 mUv;
	Vec3 mPointError;
	real mB1, mB2;
	int64 mShapeId, mPrimId;
	const Primitive *mPrimitive;
	BSDF* mBsdf;
	MediumInterface mMediumInterface;
	//bool mIsDelta = false;

	Intersection() { 
		mShapeId = -1;
		mNormal = mAbsNormal = mGeometryNormal = Vec3();
	}

	Intersection(const Vec3& pos, const Vec3& wo, const MediumInterface& medium) :
		mPos(pos), mOutDir(wo), mMediumInterface(medium) {
		mNormal = mAbsNormal = mGeometryNormal = Vec3();
	}

	Intersection(const Vec3 &pos, const Vec3 &ng, const Vec3 &absNormal, const Vec3 &wo, const Vec3 &pError):
		mPos(pos), mGeometryNormal(ng), mAbsNormal(absNormal), mOutDir(wo), mPointError(pError), mPrimitive(nullptr){
		mBsdf = nullptr;
		mShapeId = -1;
	}

	Intersection(const Vec3& pos, const Vec3& ng, const Vec3& absNormal, 
		const Vec3& dpdu, const Vec3& dpdv, const Vec3& wo, const Vec3& pError) :
		mPos(pos), mDpDu(dpdu), mDpDv(dpdv), mGeometryNormal(ng), mAbsNormal(absNormal), mOutDir(wo), mPointError(pError), mPrimitive(nullptr) {
		mBsdf = nullptr;
		mShapeId = -1;
	}

	void SetShading(const Vec3& ns, const Vec3& dpdus, Vec3& dpdvs) {
		mNormal = ns;
		this->mShadingDpDu = dpdus;
		this->mShadingDpDv = dpdvs;
		//n = Cross(dpdus, dpdvs);
		//n = (Dot(n, ng) > 0) ? n : -n;
		//this->dpdus = dpdus;
		//this->dpdvs = dpdvs;
	}

	void ComputeScatteringFunction(MemoryPool &arena, TransportMode mode = TransportMode::Radiance);

	Ray SpawnTo(const Intersection &it) const {
		Vec3 orig = OffsetRayOrigin(mPos, mPointError, mAbsNormal, it.mPos - mPos);
		Vec3 target = OffsetRayOrigin(it.mPos, it.mPointError, it.mAbsNormal, orig - it.mPos);
		Vec3 dir = target - orig;
		real dis = dir.Length();
		Normalize(dir);
		return Ray(orig, dir, dis - SHADOW_RAY_EPS, GetMedium(dir));
		//return Ray(orig, dir, 1 - shadowRayEps, 0.f);
	}

	Ray SpawnRay(const Vec3 &dir) const {
		Vec3 orig = OffsetRayOrigin(mPos, mPointError, mAbsNormal, dir);
		//Vec3 orig = hit + dir * 0.0001;
		return Ray(orig, dir, INF, GetMedium(dir));
	}

	const Medium* GetMedium(const Vec3& w) const {
		return Dot(w, mNormal) > 0 ? mMediumInterface.outside : mMediumInterface.inside;
	}

	const Medium* GetMedium() const {
		return mMediumInterface.inside;
	}

	virtual bool IsSurfaceScatter() const {
		return (mNormal != Vec3());
	}

};


class MediumIntersection : public Intersection {
public:
	// MediumInteraction Public Methods
	MediumIntersection() : mPhase(nullptr) {}

	MediumIntersection(const Vec3& pos, const Vec3& wo, 
		const Medium* medium, const PhaseFunction* phase)
		: Intersection(pos, wo, medium), mPhase(phase) {}

	bool IsValid() const { return mPhase != nullptr; }

	// MediumInteraction Public Data
	const PhaseFunction* mPhase;
};

GYT_NAMESPACE_END