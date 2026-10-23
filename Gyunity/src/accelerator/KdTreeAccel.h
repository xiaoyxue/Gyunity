#pragma once

#include "common/Core.h"
#include "Accelerator.h"

GYT_NAMESPACE_BEGIN

struct KdAccelNode;
struct BoundEdge;
class KdTreeAccel : public Accelerator{
public:
	// KdTreeAccel Public Methods
	KdTreeAccel(std::vector<std::shared_ptr<Primitive>> p,
		int isectCost = 80, int traversalCost = 1,
		real emptyBonus = 0.5, int maxPrims = 1, int maxDepth = -1);
	AABB WorldBound() const { return mBounds; }
	~KdTreeAccel();
	virtual bool Intersect(const Ray& r, Intersection* isect) const override;
	virtual bool Intersect(const Ray& r) const override;
	//Todo
	void SetPrimitives(const std::vector<std::shared_ptr<Primitive>>& a) override {}
private:
	// KdTreeAccel Private Methods
	void BuildTree(int nodeNum, const AABB& bounds,
		const std::vector<AABB>& primBounds, int* primNums,
		int nprims, int depth,
		const std::unique_ptr<BoundEdge[]> edges[3], int* prims0,
		int* prims1, int badRefines = 0);

	// KdTreeAccel Private Data
	const int mIsectCost, mTraversalCost, mMaxPrims;
	const real mEmptyBonus;
	std::vector<std::shared_ptr<Primitive>> mPrimitives;
	std::vector<int> mPrimitiveIndices;
	KdAccelNode* mNodes;
	int mNAllocedNodes, mNextFreeNode;
	AABB mBounds;
};

struct KdToDo {
	const KdAccelNode* node;
	real tMin, tMax;
};

GYT_NAMESPACE_END