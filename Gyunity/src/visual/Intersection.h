#pragma once

#include "math/Linagl.h"
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
	Vec2 mUV;
	real mRayEps = 0;
	Vec3 mPointError;
	real mB1, mB2;
	int64 mShapeId, mPrimId;
	const Primitive *mPrimitive;
	BSDF* mBsdf;
	MediumInterface mMediumInterface;
	//bool mIsDelta = false;

	Intersection() { 
		mRayEps = 0; 
		mShapeId = -1;
		mNormal = mAbsNormal = mGeometryNormal = Vec3();
	}

	Intersection(const Vec3& hit, const Vec3& wo, const MediumInterface& medium) :
		mPos(hit), mOutDir(wo), mMediumInterface(medium) {
		mRayEps = 0;
		mNormal = mAbsNormal = mGeometryNormal = Vec3();
	}

	Intersection(const Vec3 &hit, const Vec3 &ng, const Vec3 &nl, const Vec3 &wo, const Vec3 &pError):
		mPos(hit), mGeometryNormal(ng), mAbsNormal(nl), mOutDir(wo), mPointError(pError), mRayEps(0), mPrimitive(nullptr){
		mBsdf = nullptr;
		mShapeId = -1;
	}

	Intersection(const Vec3& hit, const Vec3& ng, const Vec3& nl, 
		const Vec3& dpdu, const Vec3& dpdv, const Vec3& wo, const Vec3& pError) :
		mPos(hit), mDpDu(dpdu), mDpDv(dpdv), mGeometryNormal(ng), mAbsNormal(nl), mOutDir(wo), mPointError(pError), mRayEps(0), mPrimitive(nullptr) {
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
		Vec3 origin = OffsetRayOrigin(mPos, mPointError, mAbsNormal, it.mPos - mPos);
		Vec3 target = OffsetRayOrigin(it.mPos, it.mPointError, it.mAbsNormal, origin - it.mPos);
		Vec3 d = target - origin;
		real dis = d.Length();
		Normalize(d);
		return Ray(origin, d, dis - ShadowRayEps, mRayEps, GetMedium(d));
		//return Ray(origin, d, 1 - shadowRayEps, 0.f);
	}

	Ray SpawnRay(const Vec3 &d) const {
		Vec3 o = OffsetRayOrigin(mPos, mPointError, mAbsNormal, d);
		//Vec3 o = hit + d * 0.0001;
		return Ray(o, d, Infinity, mRayEps, GetMedium(d));
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

	MediumIntersection(const Vec3& p, const Vec3& wo, 
		const Medium* medium, const PhaseFunction* phase)
		: Intersection(p, wo, medium), mPhase(phase) {}

	bool IsValid() const { return mPhase != nullptr; }

	// MediumInteraction Public Data
	const PhaseFunction* mPhase;
};

GYT_NAMESPACE_END