#pragma once

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

#include "Accelerator.h"
#include "visual/Primitive.h"
#include "system/Memory.h"
#include "common/Core.h"

GYT_NAMESPACE_BEGIN

struct BVHBuildNode;

// BVHAccel Forward Declarations
struct BVHPrimitiveInfo;
struct LinearBVHNode;

// BVHAccel Declarations
class BVHAccel : public Accelerator {
public:
    // BVHAccel Public Methods
    BVHAccel(const std::vector<std::shared_ptr<Primitive> >& p, uint32_t maxPrims = 1,
        const std::string& sm = "sah");
    AABB WorldBound() const;
    bool CanIntersect() const { return true; }
    ~BVHAccel();
    bool Intersect(const Ray& ray, Intersection* isect) const override;
    bool Intersect(const Ray& ray) const override;
private:
    // BVHAccel Private Methods
    BVHBuildNode* RecursiveBuild(MemoryPool& buildArena,
        std::vector<BVHPrimitiveInfo>& buildData, uint32_t start, uint32_t end,
        uint32_t* totalNodes, std::vector<std::shared_ptr<Primitive> >& orderedPrims);
    uint32_t FlattenBVHTree(BVHBuildNode* node, uint32_t* offset);

    // BVHAccel Private Data
    uint32_t mMaxPrimsInNode;
    enum class SplitMethod { Middle, EqualCounts, Sah };
    SplitMethod mSplitMethod;
    std::vector<std::shared_ptr<Primitive> > mPrimitives;
    LinearBVHNode* mNodes;
};

GYT_NAMESPACE_END
