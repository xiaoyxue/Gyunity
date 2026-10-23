#include "KdTreeAccel.h"
#include "system/Memory.h"
#include "math/MathUtils.h"

GYT_NAMESPACE_BEGIN

struct KdAccelNode {
	// KdAccelNode Methods
	void InitLeaf(int* primNums, int np, std::vector<int>* primitiveIndices);
	void InitInterior(int axis, int ac, real s) {
		split = s;
		flags = axis;
		aboveChild |= (ac << 2);
	}
	real SplitPos() const { return split; }
	int nPrimitives() const { return nPrims >> 2; }
	int SplitAxis() const { return flags & 3; }
	bool IsLeaf() const { return (flags & 3) == 3; }
	int AboveChild() const { return aboveChild >> 2; }
	union {
		real split;                 // Interior
		int onePrimitive;            // Leaf
		int primitiveIndicesOffset;  // Leaf
	};

private:
	union {
		int flags;       // Both
		int nPrims;      // Leaf
		int aboveChild;  // Interior
	};
};

enum class EdgeType { Start, End };
struct BoundEdge {
	// BoundEdge Public Methods
	BoundEdge() {}
	BoundEdge(real t, int primNum, bool starting) : t(t), primNum(primNum) {
		type = starting ? EdgeType::Start : EdgeType::End;
	}
	real t;
	int primNum;
	EdgeType type;
};


// KdTreeAccel Method Definitions
KdTreeAccel::KdTreeAccel(std::vector<std::shared_ptr<Primitive>> p,
	int isectCost, int traversalCost, real emptyBonus,
	int maxPrims, int maxDepth)
	: mIsectCost(isectCost),
	mTraversalCost(traversalCost),
	mMaxPrims(maxPrims),
	mEmptyBonus(emptyBonus),
	mPrimitives(std::move(p)) {
	// Build kd-tree for accelerator
	mNextFreeNode = mNAllocedNodes = 0;
	if (maxDepth <= 0)
		maxDepth = std::round(8 + 1.3f * Log2Int(int64_t(mPrimitives.size())));

	// Compute bounds for kd-tree construction
	std::vector<AABB> primBounds;
	primBounds.reserve(mPrimitives.size());
	for (const std::shared_ptr<Primitive>& prim : mPrimitives) {
		AABB b = prim->WorldBound();
		mBounds = Union(mBounds, b);
		primBounds.push_back(b);
	}

	// Allocate working memory for kd-tree construction
	std::unique_ptr<BoundEdge[]> edges[3];
	for (int i = 0; i < 3; ++i)
		edges[i].reset(new BoundEdge[2 * mPrimitives.size()]);
	std::unique_ptr<int[]> prims0(new int[mPrimitives.size()]);
	std::unique_ptr<int[]> prims1(new int[(maxDepth + 1) * mPrimitives.size()]);

	// Initialize _primNums_ for kd-tree construction
	std::unique_ptr<int[]> primNums(new int[mPrimitives.size()]);
	for (size_t i = 0; i < mPrimitives.size(); ++i) primNums[i] = i;

	// Start recursive construction of kd-tree
	BuildTree(0, mBounds, primBounds, primNums.get(), mPrimitives.size(),
		maxDepth, edges, prims0.get(), prims1.get());
}

void KdAccelNode::InitLeaf(int* primNums, int np,
	std::vector<int>* primitiveIndices) {
	flags = 3;
	nPrims |= (np << 2);
	// Store primitive ids for leaf node
	if (np == 0)
		onePrimitive = 0;
	else if (np == 1)
		onePrimitive = primNums[0];
	else {
		primitiveIndicesOffset = primitiveIndices->size();
		for (int i = 0; i < np; ++i) primitiveIndices->push_back(primNums[i]);
	}
}

KdTreeAccel::~KdTreeAccel() { FreeAligned(mNodes); }

void KdTreeAccel::BuildTree(int nodeNum, const AABB& nodeBounds,
	const std::vector<AABB>& allPrimBounds,
	int* primNums, int nPrimitives, int depth,
	const std::unique_ptr<BoundEdge[]> edges[3],
	int* prims0, int* prims1, int badRefines) {
	// Get next free node from _nodes_ array
	if (mNextFreeNode == mNAllocedNodes) {
		int nNewAllocNodes = std::max(2 * mNAllocedNodes, 512);
		KdAccelNode* n = AllocAligned<KdAccelNode>(nNewAllocNodes);
		if (mNAllocedNodes > 0) {
			memcpy(n, mNodes, mNAllocedNodes * sizeof(KdAccelNode));
			FreeAligned(mNodes);
		}
		mNodes = n;
		mNAllocedNodes = nNewAllocNodes;
	}
	++mNextFreeNode;

	// Initialize leaf node if termination criteria met
	if (nPrimitives <= mMaxPrims || depth == 0) {
		mNodes[nodeNum].InitLeaf(primNums, nPrimitives, &mPrimitiveIndices);
		return;
	}

	// Initialize interior node and continue recursion

	// Choose split axis position for interior node
	int bestAxis = -1, bestOffset = -1;
	real bestCost = INF;
	real oldCost = mIsectCost * real(nPrimitives);
	real totalSA = nodeBounds.SurfaceArea();
	real invTotalSA = 1 / totalSA;
	Vector3f d = nodeBounds.maxPoint - nodeBounds.minPoint;

	// Choose which axis to split along
	int axis = nodeBounds.MaximumExtent();
	int retries = 0;
retrySplit:

	// Initialize edges for _axis_
	for (int i = 0; i < nPrimitives; ++i) {
		int pn = primNums[i];
		const AABB& bounds = allPrimBounds[pn];
		edges[axis][2 * i] = BoundEdge(bounds.minPoint[axis], pn, true);
		edges[axis][2 * i + 1] = BoundEdge(bounds.maxPoint[axis], pn, false);
	}

	// Sort _edges_ for _axis_
	std::sort(&edges[axis][0], &edges[axis][2 * nPrimitives],
		[](const BoundEdge& e0, const BoundEdge& e1) -> bool {
			if (e0.t == e1.t)
				return (int)e0.type < (int)e1.type;
			else
				return e0.t < e1.t;
		});

	// Compute cost of all splits for _axis_ to find best
	int nBelow = 0, nAbove = nPrimitives;
	for (int i = 0; i < 2 * nPrimitives; ++i) {
		if (edges[axis][i].type == EdgeType::End) --nAbove;
		real edgeT = edges[axis][i].t;
		if (edgeT > nodeBounds.minPoint[axis] && edgeT < nodeBounds.maxPoint[axis]) {
			// Compute cost for split at _i_th edge

			// Compute child surface areas for split at _edgeT_
			int otherAxis0 = (axis + 1) % 3, otherAxis1 = (axis + 2) % 3;
			real belowSA = 2 * (d[otherAxis0] * d[otherAxis1] +
				(edgeT - nodeBounds.minPoint[axis]) *
				(d[otherAxis0] + d[otherAxis1]));
			real aboveSA = 2 * (d[otherAxis0] * d[otherAxis1] +
				(nodeBounds.maxPoint[axis] - edgeT) *
				(d[otherAxis0] + d[otherAxis1]));
			real pBelow = belowSA * invTotalSA;
			real pAbove = aboveSA * invTotalSA;
			real eb = (nAbove == 0 || nBelow == 0) ? mEmptyBonus : 0;
			real cost =
				mTraversalCost +
				mIsectCost * (1 - eb) * (pBelow * nBelow + pAbove * nAbove);

			// Update best split if this is lowest cost so far
			if (cost < bestCost) {
				bestCost = cost;
				bestAxis = axis;
				bestOffset = i;
			}
		}
		if (edges[axis][i].type == EdgeType::Start) ++nBelow;
	}

	// Create leaf if no good splits were found
	if (bestAxis == -1 && retries < 2) {
		++retries;
		axis = (axis + 1) % 3;
		goto retrySplit;
	}
	if (bestCost > oldCost) ++badRefines;
	if ((bestCost > 4 * oldCost && nPrimitives < 16) || bestAxis == -1 ||
		badRefines == 3) {
		mNodes[nodeNum].InitLeaf(primNums, nPrimitives, &mPrimitiveIndices);
		return;
	}

	// Classify primitives with respect to split
	int n0 = 0, n1 = 0;
	for (int i = 0; i < bestOffset; ++i)
		if (edges[bestAxis][i].type == EdgeType::Start)
			prims0[n0++] = edges[bestAxis][i].primNum;
	for (int i = bestOffset + 1; i < 2 * nPrimitives; ++i)
		if (edges[bestAxis][i].type == EdgeType::End)
			prims1[n1++] = edges[bestAxis][i].primNum;

	// Recursively initialize children nodes
	real tSplit = edges[bestAxis][bestOffset].t;
	AABB bounds0 = nodeBounds, bounds1 = nodeBounds;
	bounds0.maxPoint[bestAxis] = bounds1.minPoint[bestAxis] = tSplit;
	BuildTree(nodeNum + 1, bounds0, allPrimBounds, prims0, n0, depth - 1, edges,
		prims0, prims1 + nPrimitives, badRefines);
	int aboveChild = mNextFreeNode;
	mNodes[nodeNum].InitInterior(bestAxis, aboveChild, tSplit);
	BuildTree(aboveChild, bounds1, allPrimBounds, prims1, n1, depth - 1, edges,
		prims0, prims1 + nPrimitives, badRefines);
}

bool KdTreeAccel::Intersect(const Ray& ray, Intersection* isect) const {
	// Compute initial parametric range of ray inside kd-tree extent
	real tMin, tMax;
	if (!mBounds.Intersect(ray, &tMin, &tMax)) {
		return false;
	}

	// Prepare to traverse kd-tree for ray
	Vector3f invDir(1 / ray.mDir.x, 1 / ray.mDir.y, 1 / ray.mDir.z);
	constexpr int maxTodo = 64;
	KdToDo todo[maxTodo];
	int todoPos = 0;

	// Traverse kd-tree nodes in order for ray
	bool hit = false;
	const KdAccelNode* node = &mNodes[0];
	while (node != nullptr) {
		// Bail out if we found a hit closer than the current node
		if (ray.mTmax < tMin) break;
		if (!node->IsLeaf()) {
			// Process kd-tree interior node

			// Compute parametric distance along ray to split plane
			int axis = node->SplitAxis();
			real tPlane = (node->SplitPos() - ray.mOrig[axis]) * invDir[axis];

			// Get node children pointers for ray
			const KdAccelNode* firstChild, * secondChild;
			int belowFirst =
				(ray.mOrig[axis] < node->SplitPos()) ||
				(ray.mOrig[axis] == node->SplitPos() && ray.mDir[axis] <= 0);
			if (belowFirst) {
				firstChild = node + 1;
				secondChild = &mNodes[node->AboveChild()];
			}
			else {
				firstChild = &mNodes[node->AboveChild()];
				secondChild = node + 1;
			}

			// Advance to next child node, possibly enqueue other child
			if (tPlane > tMax || tPlane <= 0)
				node = firstChild;
			else if (tPlane < tMin)
				node = secondChild;
			else {
				// Enqueue _secondChild_ in todo list
				todo[todoPos].node = secondChild;
				todo[todoPos].tMin = tPlane;
				todo[todoPos].tMax = tMax;
				++todoPos;
				node = firstChild;
				tMax = tPlane;
			}
		}
		else {
			// Check for intersections inside leaf node
			int nPrimitives = node->nPrimitives();
			if (nPrimitives == 1) {
				const std::shared_ptr<Primitive>& p =
					mPrimitives[node->onePrimitive];
				// Check one primitive inside leaf node
				if (p->Intersect(ray, isect)) hit = true;
			}
			else {
				for (int i = 0; i < nPrimitives; ++i) {
					int index =
						mPrimitiveIndices[node->primitiveIndicesOffset + i];
					const std::shared_ptr<Primitive>& p = mPrimitives[index];
					// Check one primitive inside leaf node
					if (p->Intersect(ray, isect)) hit = true;
				}
			}

			// Grab next node to process from todo list
			if (todoPos > 0) {
				--todoPos;
				node = todo[todoPos].node;
				tMin = todo[todoPos].tMin;
				tMax = todo[todoPos].tMax;
			}
			else
				break;
		}
	}
	return hit;
}

bool KdTreeAccel::Intersect(const Ray& ray) const {
	// Compute initial parametric range of ray inside kd-tree extent
	real tMin, tMax;
	if (!mBounds.Intersect(ray, &tMin, &tMax)) {
		return false;
	}

	// Prepare to traverse kd-tree for ray
	Vector3f invDir(1 / ray.mDir.x, 1 / ray.mDir.y, 1 / ray.mDir.z);
	constexpr int maxTodo = 64;
	KdToDo todo[maxTodo];
	int todoPos = 0;
	const KdAccelNode* node = &mNodes[0];
	while (node != nullptr) {
		if (node->IsLeaf()) {
			// Check for shadow ray intersections inside leaf node
			int nPrimitives = node->nPrimitives();
			if (nPrimitives == 1) {
				const std::shared_ptr<Primitive>& p =
					mPrimitives[node->onePrimitive];
				if (p->Intersect(ray)) {
					return true;
				}
			}
			else {
				for (int i = 0; i < nPrimitives; ++i) {
					int primitiveIndex =
						mPrimitiveIndices[node->primitiveIndicesOffset + i];
					const std::shared_ptr<Primitive>& prim =
						mPrimitives[primitiveIndex];
					if (prim->Intersect(ray)) {
						return true;
					}
				}
			}

			// Grab next node to process from todo list
			if (todoPos > 0) {
				--todoPos;
				node = todo[todoPos].node;
				tMin = todo[todoPos].tMin;
				tMax = todo[todoPos].tMax;
			}
			else
				break;
		}
		else {
			// Process kd-tree interior node

			// Compute parametric distance along ray to split plane
			int axis = node->SplitAxis();
			real tPlane = (node->SplitPos() - ray.mOrig[axis]) * invDir[axis];

			// Get node children pointers for ray
			const KdAccelNode* firstChild, * secondChild;
			int belowFirst =
				(ray.mOrig[axis] < node->SplitPos()) ||
				(ray.mOrig[axis] == node->SplitPos() && ray.mDir[axis] <= 0);
			if (belowFirst) {
				firstChild = node + 1;
				secondChild = &mNodes[node->AboveChild()];
			}
			else {
				firstChild = &mNodes[node->AboveChild()];
				secondChild = node + 1;
			}

			// Advance to next child node, possibly enqueue other child
			if (tPlane > tMax || tPlane <= 0)
				node = firstChild;
			else if (tPlane < tMin)
				node = secondChild;
			else {
				// Enqueue _secondChild_ in todo list
				todo[todoPos].node = secondChild;
				todo[todoPos].tMin = tPlane;
				todo[todoPos].tMax = tMax;
				++todoPos;
				node = firstChild;
				tMax = tPlane;
			}
		}
	}
	return false;
}


GYT_NAMESPACE_END