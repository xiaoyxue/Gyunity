
/*
    pbrt source code Copyright(c) 1998-2012 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are
    met:

    - Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

    - Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
    IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
    TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
    PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
    HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
    SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
    LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
    DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
    THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */


#include "BVHAccel.h"


GYT_NAMESPACE_BEGIN

// BVHAccel Local Declarations
struct BVHPrimitiveInfo {
    BVHPrimitiveInfo() { }
    BVHPrimitiveInfo(int pn, const AABB& b)
        : primitiveNumber(pn), bounds(b) {
        centroid = .5f * b.minPoint + .5f * b.maxPoint;
    }
    int primitiveNumber;
    Vec3 centroid;
    AABB bounds;
};


struct BVHBuildNode {
    // BVHBuildNode Public Methods
    BVHBuildNode() { children[0] = children[1] = NULL; }
    void InitLeaf(uint32_t first, uint32_t n, const AABB& b) {
        firstPrimOffset = first;
        nPrimitives = n;
        bounds = b;
    }
    void InitInterior(uint32_t axis, BVHBuildNode* c0, BVHBuildNode* c1) {
        children[0] = c0;
        children[1] = c1;
        bounds = Union(c0->bounds, c1->bounds);
        splitAxis = axis;
        nPrimitives = 0;
    }
    AABB bounds;
    BVHBuildNode* children[2];
    uint32_t splitAxis, firstPrimOffset, nPrimitives;
};


struct CompareToMid {
    CompareToMid(int d, float m) { dim = d; mid = m; }
    int dim;
    float mid;
    bool operator()(const BVHPrimitiveInfo& a) const {
        return a.centroid[dim] < mid;
    }
};


struct ComparePoints {
    ComparePoints(int d) { dim = d; }
    int dim;
    bool operator()(const BVHPrimitiveInfo& a,
        const BVHPrimitiveInfo& b) const {
        return a.centroid[dim] < b.centroid[dim];
    }
};


struct CompareToBucket {
    CompareToBucket(int split, int num, int d, const AABB& b)
        : centroidBounds(b)
    {
        splitBucket = split; nBuckets = num; dim = d;
    }
    bool operator()(const BVHPrimitiveInfo& p) const;

    int splitBucket, nBuckets, dim;
    const AABB& centroidBounds;
};


bool CompareToBucket::operator()(const BVHPrimitiveInfo& p) const {
    int b = nBuckets * ((p.centroid[dim] - centroidBounds.minPoint[dim]) /
        (centroidBounds.maxPoint[dim] - centroidBounds.minPoint[dim]));
    if (b == nBuckets) b = nBuckets - 1;
    assert(b >= 0 && b < nBuckets);
    return b <= splitBucket;
}


struct LinearBVHNode {
    AABB bounds;
    union {
        uint32_t primitivesOffset;    // leaf
        uint32_t secondChildOffset;   // interior
    };

    uint8_t nPrimitives;  // 0 -> interior node
    uint8_t axis;         // interior node: xyz
    uint8_t pad[2];       // ensure 32 byte total size
};


static inline bool IntersectP(const AABB& bounds, const Ray& ray,
    const Vec3& invDir, const uint32_t dirIsNeg[3]) {
    // Check for ray intersection against $x$ and $y$ slabs
    float tmin = (bounds[dirIsNeg[0]].x - ray.mOrig.x) * invDir.x;
    float tmax = (bounds[1 - dirIsNeg[0]].x - ray.mOrig.x) * invDir.x;
    float tymin = (bounds[dirIsNeg[1]].y - ray.mOrig.y) * invDir.y;
    float tymax = (bounds[1 - dirIsNeg[1]].y - ray.mOrig.y) * invDir.y;
    if ((tmin > tymax) || (tymin > tmax))
        return false;
    if (tymin > tmin) tmin = tymin;
    if (tymax < tmax) tmax = tymax;

    // Check for ray intersection against $z$ slab
    float tzmin = (bounds[dirIsNeg[2]].z - ray.mOrig.z) * invDir.z;
    float tzmax = (bounds[1 - dirIsNeg[2]].z - ray.mOrig.z) * invDir.z;
    if ((tmin > tzmax) || (tzmin > tmax))
        return false;
    if (tzmin > tmin)
        tmin = tzmin;
    if (tzmax < tmax)
        tmax = tzmax;
    return (tmin < ray.mTMax) && (tmax > ray.mTMin);
}



// BVHAccel Method Definitions
BVHAccel::BVHAccel(const std::vector<std::shared_ptr<Primitive> >& p,
    uint32_t mp, const std::string& sm) {
    maxPrimsInNode = std::min(255u, mp);
    primitives = p;
    if (sm == "sah")         splitMethod = SPLIT_SAH;
    else if (sm == "middle") splitMethod = SPLIT_MIDDLE;
    else if (sm == "equal")  splitMethod = SPLIT_EQUAL_COUNTS;
    else {
        //Warning("BVH split method \"%s\" unknown.  Using \"sah\".",
        //    sm.c_str());
        splitMethod = SPLIT_SAH;
    }

    if (primitives.size() == 0) {
        nodes = NULL;
        return;
    }
    // Build BVH from _primitives_

    // Initialize _buildData_ array for primitives
    std::vector<BVHPrimitiveInfo> buildData;
    buildData.reserve(primitives.size());
    for (uint32_t i = 0; i < primitives.size(); ++i) {
        AABB AABB = primitives[i]->WorldBound();
        buildData.push_back(BVHPrimitiveInfo(i, AABB));
    }

    // Recursively build BVH tree for primitives
    MemoryPool buildArena;
    uint32_t totalNodes = 0;
    std::vector<std::shared_ptr<Primitive> > orderedPrims;
    orderedPrims.reserve(primitives.size());
    BVHBuildNode* root = RecursiveBuild(buildArena, buildData, 0,
        primitives.size(), &totalNodes,
        orderedPrims);
    primitives.swap(orderedPrims);
    //Info("BVH created with %d nodes for %d primitives (%.2f MB)", totalNodes,
    //    (int)primitives.size(), float(totalNodes * sizeof(LinearBVHNode)) / (1024.f * 1024.f));

    // Compute representation of depth-first traversal of BVH tree
    nodes = AllocAligned<LinearBVHNode>(totalNodes);
    for (uint32_t i = 0; i < totalNodes; ++i)
        new (&nodes[i]) LinearBVHNode;
    uint32_t offset = 0;
    FlattenBVHTree(root, &offset);
    assert(offset == totalNodes);
}


AABB BVHAccel::WorldBound() const {
    return nodes ? nodes[0].bounds : AABB();
}


BVHBuildNode* BVHAccel::RecursiveBuild(MemoryPool& buildArena,
    std::vector<BVHPrimitiveInfo>& buildData, uint32_t start,
    uint32_t end, uint32_t* totalNodes,
    std::vector<std::shared_ptr<Primitive> >& orderedPrims) {
    assert(start != end);
    (*totalNodes)++;
    BVHBuildNode* node = buildArena.Alloc<BVHBuildNode>();
    // Compute bounds of all primitives in BVH node
    AABB boundBox;
    for (uint32_t i = start; i < end; ++i)
        boundBox = Union(boundBox, buildData[i].bounds);
    uint32_t nPrimitives = end - start;
    if (nPrimitives == 1) {
        // Create leaf _BVHBuildNode_
        uint32_t firstPrimOffset = orderedPrims.size();
        for (uint32_t i = start; i < end; ++i) {
            uint32_t primNum = buildData[i].primitiveNumber;
            orderedPrims.push_back(primitives[primNum]);
        }
        node->InitLeaf(firstPrimOffset, nPrimitives, boundBox);
    }
    else {
        // Compute bound of primitive centroids, choose split dimension _dim_
        AABB centroidBounds;
        for (uint32_t i = start; i < end; ++i)
            centroidBounds = Union(centroidBounds, buildData[i].centroid);
        int dim = centroidBounds.MaximumExtent();

        // Partition primitives into two sets and build children
        uint32_t mid = (start + end) / 2;
        if (centroidBounds.maxPoint[dim] == centroidBounds.minPoint[dim]) {
            // If nPrimitives is no greater than maxPrimsInNode,
            // then all the nodes can be stored in a compact bvh node.
            if (nPrimitives <= maxPrimsInNode) {
                // Create leaf _BVHBuildNode_
                uint32_t firstPrimOffset = orderedPrims.size();
                for (uint32_t i = start; i < end; ++i) {
                    uint32_t primNum = buildData[i].primitiveNumber;
                    orderedPrims.push_back(primitives[primNum]);
                }
                node->InitLeaf(firstPrimOffset, nPrimitives, boundBox);
                return node;
            }
            else {
                // else if nPrimitives is greater than maxPrimsInNode, we
                // need to split it further to guarantee each node contains
                // no more than maxPrimsInNode primitives.
                node->InitInterior(dim,
                    RecursiveBuild(buildArena, buildData, start, mid,
                        totalNodes, orderedPrims),
                    RecursiveBuild(buildArena, buildData, mid, end,
                        totalNodes, orderedPrims));
                return node;
            }
        }

        // Partition primitives based on _splitMethod_
        switch (splitMethod) {
        case SPLIT_MIDDLE: {
            // Partition primitives through node's midpoint
            float pmid = .5f * (centroidBounds.minPoint[dim] + centroidBounds.maxPoint[dim]);
            BVHPrimitiveInfo* midPtr = std::partition(&buildData[start],
                &buildData[end - 1] + 1,
                CompareToMid(dim, pmid));
            mid = midPtr - &buildData[0];
            if (mid != start && mid != end)
                // for lots of prims with large overlapping bounding boxes, this
                // may fail to partition; in that case don't break and fall through
                // to SPLIT_EQUAL_COUNTS
                break;
        }
        case SPLIT_EQUAL_COUNTS: {
            // Partition primitives into equally-sized subsets
            mid = (start + end) / 2;
            std::nth_element(&buildData[start], &buildData[mid],
                &buildData[end - 1] + 1, ComparePoints(dim));
            break;
        }
        case SPLIT_SAH: default: {
            // Partition primitives using approximate SAH
            if (nPrimitives <= 4) {
                // Partition primitives into equally-sized subsets
                mid = (start + end) / 2;
                std::nth_element(&buildData[start], &buildData[mid],
                    &buildData[end - 1] + 1, ComparePoints(dim));
            }
            else {
                // Allocate _BucketInfo_ for SAH partition buckets
                const int nBuckets = 12;
                struct BucketInfo {
                    BucketInfo() { count = 0; }
                    int count;
                    AABB bounds;
                };
                BucketInfo buckets[nBuckets];

                // Initialize _BucketInfo_ for SAH partition buckets
                for (uint32_t i = start; i < end; ++i) {
                    int b = nBuckets *
                        ((buildData[i].centroid[dim] - centroidBounds.minPoint[dim]) /
                        (centroidBounds.maxPoint[dim] - centroidBounds.minPoint[dim]));
                    if (b == nBuckets) b = nBuckets - 1;
                    assert(b >= 0 && b < nBuckets);
                    buckets[b].count++;
                    buckets[b].bounds = Union(buckets[b].bounds, buildData[i].bounds);
                }

                // Compute costs for splitting after each bucket
                float cost[nBuckets - 1];
                for (int i = 0; i < nBuckets - 1; ++i) {
                    AABB b0, b1;
                    int count0 = 0, count1 = 0;
                    for (int j = 0; j <= i; ++j) {
                        b0 = Union(b0, buckets[j].bounds);
                        count0 += buckets[j].count;
                    }
                    for (int j = i + 1; j < nBuckets; ++j) {
                        b1 = Union(b1, buckets[j].bounds);
                        count1 += buckets[j].count;
                    }
                    cost[i] = .125f + (count0 * b0.SurfaceArea() + count1 * b1.SurfaceArea()) /
                        boundBox.SurfaceArea();
                }

                // Find bucket to split at that minimizes SAH metric
                float minCost = cost[0];
                uint32_t minCostSplit = 0;
                for (int i = 1; i < nBuckets - 1; ++i) {
                    if (cost[i] < minCost) {
                        minCost = cost[i];
                        minCostSplit = i;
                    }
                }

                // Either create leaf or split primitives at selected SAH bucket
                if (nPrimitives > maxPrimsInNode ||
                    minCost < nPrimitives) {
                    BVHPrimitiveInfo* pmid = std::partition(&buildData[start],
                        &buildData[end - 1] + 1,
                        CompareToBucket(minCostSplit, nBuckets, dim, centroidBounds));
                    mid = pmid - &buildData[0];
                }

                else {
                    // Create leaf _BVHBuildNode_
                    uint32_t firstPrimOffset = orderedPrims.size();
                    for (uint32_t i = start; i < end; ++i) {
                        uint32_t primNum = buildData[i].primitiveNumber;
                        orderedPrims.push_back(primitives[primNum]);
                    }
                    node->InitLeaf(firstPrimOffset, nPrimitives, boundBox);
                    return node;
                }
            }
            break;
        }
        }
        node->InitInterior(dim,
            RecursiveBuild(buildArena, buildData, start, mid,
                totalNodes, orderedPrims),
            RecursiveBuild(buildArena, buildData, mid, end,
                totalNodes, orderedPrims));
    }
    return node;
}


uint32_t BVHAccel::FlattenBVHTree(BVHBuildNode* node, uint32_t* offset) {
    LinearBVHNode* linearNode = &nodes[*offset];
    linearNode->bounds = node->bounds;
    uint32_t myOffset = (*offset)++;
    if (node->nPrimitives > 0) {
        assert(!node->children[0] && !node->children[1]);
        linearNode->primitivesOffset = node->firstPrimOffset;
        linearNode->nPrimitives = node->nPrimitives;
    }
    else {
        // Creater interior flattened BVH node
        linearNode->axis = node->splitAxis;
        linearNode->nPrimitives = 0;
        FlattenBVHTree(node->children[0], offset);
        linearNode->secondChildOffset = FlattenBVHTree(node->children[1],
            offset);
    }
    return myOffset;
}


BVHAccel::~BVHAccel() {
    FreeAligned(nodes);
}


bool BVHAccel::Intersect(const Ray& ray, Intersection* isect) const {
    if (!nodes) return false;
    bool hit = false;
    Vec3 invDir(1.f / ray.mDir.x, 1.f / ray.mDir.y, 1.f / ray.mDir.z);
    uint32_t dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    // Follow ray through BVH nodes to find primitive intersections
    uint32_t todoOffset = 0, nodeNum = 0;
    uint32_t todo[64];
    while (true) {
        const LinearBVHNode* node = &nodes[nodeNum];
        // Check ray against BVH node
        if (IntersectP(node->bounds, ray, invDir, dirIsNeg)) {
            if (node->nPrimitives > 0) {
                // Intersect ray with primitives in leaf BVH node
                for (uint32_t i = 0; i < node->nPrimitives; ++i)
                {
                    if (primitives[node->primitivesOffset + i]->Intersect(ray, isect))
                    {
                        hit = true;
                    }
                    else {
                    }
                }
                if (todoOffset == 0) break;
                nodeNum = todo[--todoOffset];
            }
            else {
                // Put far BVH node on _todo_ stack, advance to near node
                if (dirIsNeg[node->axis]) {
                    todo[todoOffset++] = nodeNum + 1;
                    nodeNum = node->secondChildOffset;
                }
                else {
                    todo[todoOffset++] = node->secondChildOffset;
                    nodeNum = nodeNum + 1;
                }
            }
        }
        else {
            if (todoOffset == 0) break;
            nodeNum = todo[--todoOffset];
        }
    }
    return hit;
}


bool BVHAccel::Intersect(const Ray& ray) const {
    if (!nodes) return false;
    Vec3 invDir(1.f / ray.mDir.x, 1.f / ray.mDir.y, 1.f / ray.mDir.z);
    uint32_t dirIsNeg[3] = { invDir.x < 0, invDir.y < 0, invDir.z < 0 };
    uint32_t todo[64];
    uint32_t todoOffset = 0, nodeNum = 0;
    while (true) {
        const LinearBVHNode* node = &nodes[nodeNum];
        if (IntersectP(node->bounds, ray, invDir, dirIsNeg)) {
            // Process BVH node _node_ for traversal
            if (node->nPrimitives > 0) {
                for (uint32_t i = 0; i < node->nPrimitives; ++i) {
                    if (primitives[node->primitivesOffset + i]->Intersect(ray)) {
                        return true;
                    }
                    else {
                    }
                }
                if (todoOffset == 0) break;
                nodeNum = todo[--todoOffset];
            }
            else {
                if (dirIsNeg[node->axis]) {
                    /// second child first
                    todo[todoOffset++] = nodeNum + 1;
                    nodeNum = node->secondChildOffset;
                }
                else {
                    todo[todoOffset++] = node->secondChildOffset;
                    nodeNum = nodeNum + 1;
                }
            }
        }
        else {
            if (todoOffset == 0) break;
            nodeNum = todo[--todoOffset];
        }
    }
    return false;
}

GYT_NAMESPACE_END