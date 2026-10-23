#pragma once
#include "common/Core.h"
#include "accelerator/Accelerator.h"
#include "visual/Primitive.h"

GYT_NAMESPACE_BEGIN

class BruteForce : public Accelerator {
public:

	BruteForce() { }

	BruteForce(const std::vector<std::shared_ptr<Primitive>> primitives) :
		mPrimitives(primitives) {
		for (int i = 0; i < primitives.size(); ++i) {
			mBounds = Union(mBounds, primitives[i]->WorldBound());
		}
	}

	bool Intersect(const Ray& r, Intersection* isect) const override {
		int n = (int)mPrimitives.size();
		bool intersected = false;
		for (int i = 0; i < n; ++i) {
			if (mPrimitives[i]->Intersect(r, isect)) {
				intersected = true;
			}
		}
		return  intersected;
	}

	bool Intersect(const Ray& r) const override {
		for (auto primitive : mPrimitives) {
			if (primitive->Intersect(r)) {
				return true;
			}
		}
		return false;
	}

	void SetPrimitives(const std::vector<std::shared_ptr<Primitive>> &pPrimitives) override {
		mPrimitives = pPrimitives;
	}

	AABB WorldBound() const { return mBounds; }
private:
	std::vector<std::shared_ptr<Primitive>> mPrimitives;
	AABB mBounds;
};

GYT_NAMESPACE_END