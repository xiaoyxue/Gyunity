#pragma once

#include <utility>
#include <cassert>
#include "math/LowDiscrepancy.h"

GYT_NAMESPACE_BEGIN


extern const unsigned long long vdc_sobol_matrices[][52];
extern const unsigned long long vdc_sobol_matrices_inv[][52];


class SamplerEnum {
public:
	SamplerEnum(){}
	SamplerEnum(uint32 width, uint32 height): mResX(width), mResY(height) {}
	virtual uint64 GetIndex(uint32 sampleNum, uint32 x, uint32 y) const {
		return sampleNum * y;
	}
	virtual real SampleX(uint32 x, real u) const {
		return u;
	}
	virtual real SampleY(uint32 y, real v) const {
		return v;
	}
protected:
	uint32 mResX, mResY;
};

GYT_NAMESPACE_END