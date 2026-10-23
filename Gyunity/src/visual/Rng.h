#pragma once
#include "common/Core.h"
#include <random>

GYT_NAMESPACE_BEGIN

class Rng {
private:
	std::mt19937_64 mRng;
	std::uniform_real_distribution<real> mUniform;
public:
	Rng(int seed = 123) : mRng(seed){}

	real operator()() {
		return mUniform(mRng);
	}

	double operator()(unsigned long long i) {
		return mUniform(mRng);
	}

	double GetDouble() {
		return mUniform(mRng);
	}

	std::mt19937_64 GetRngEngine() {
		return mRng;
	}
};



// Random Number Declarations

static const double DoubleOneMinusEpsilon = 0.99999999999999989;
static const float FloatOneMinusEpsilon = 0.99999994;


#ifdef USING_DOUBLE
static const real OneMinusEpsilon = DoubleOneMinusEpsilon;
#else
static const real OneMinusEpsilon = FloatOneMinusEpsilon;
#endif

#define PCG32_DEFAULT_STATE 0x853c49e6748fea9bULL
#define PCG32_DEFAULT_STREAM 0xda3e39cb94b95bdbULL
#define PCG32_MULT 0x5851f42d4c957f2dULL
class RNG {
  public:
    // RNG Public Methods
    RNG();
    RNG(uint64 sequenceIndex) { SetSequence(sequenceIndex); }
    void SetSequence(uint64 sequenceIndex);
    uint32_t UniformUInt32();
    uint32_t UniformUInt32(uint32_t b) {
        uint32_t threshold = (~b + 1u) % b;
        while (true) {
            uint32_t r = UniformUInt32();
            if (r >= threshold) return r % b;
        }
    }
    real UniformFloat() {
#ifndef PBRT_HAVE_HEX_FP_CONSTANTS
        return std::min(OneMinusEpsilon,
                        real(UniformUInt32() * 2.3283064365386963e-10f));
#else
        return std::min(OneMinusEpsilon, Float(UniformUInt32() * 0x1p-32f));
#endif
    }
    template <typename Iterator>
    void Shuffle(Iterator begin, Iterator end) {
        for (Iterator it = end - 1; it > begin; --it)
            std::iter_swap(it,
                           begin + UniformUInt32((uint32_t)(it - begin + 1)));
    }
    void Advance(int64_t idelta) {
        uint64 cur_mult = PCG32_MULT, cur_plus = inc, acc_mult = 1u,
                 acc_plus = 0u, delta = (uint64)idelta;
        while (delta > 0) {
            if (delta & 1) {
                acc_mult *= cur_mult;
                acc_plus = acc_plus * cur_mult + cur_plus;
            }
            cur_plus = (cur_mult + 1) * cur_plus;
            cur_mult *= cur_mult;
            delta /= 2;
        }
        state = acc_mult * state + acc_plus;
    }
    int64_t operator-(const RNG &other) const {
        uint64 cur_mult = PCG32_MULT, cur_plus = inc, cur_state = other.state,
                 the_bit = 1u, distance = 0u;
        while (state != cur_state) {
            if ((state & the_bit) != (cur_state & the_bit)) {
                cur_state = cur_state * cur_mult + cur_plus;
                distance |= the_bit;
            }
            the_bit <<= 1;
            cur_plus = (cur_mult + 1ULL) * cur_plus;
            cur_mult *= cur_mult;
        }
        return (int64_t)distance;
    }

  private:
    // RNG Private Data
    uint64 state, inc;
};

// RNG Inline Method Definitions
inline RNG::RNG() : state(PCG32_DEFAULT_STATE), inc(PCG32_DEFAULT_STREAM) {}
inline void RNG::SetSequence(uint64 initseq) {
    state = 0u;
    inc = (initseq << 1u) | 1u;
    UniformUInt32();
    state += PCG32_DEFAULT_STATE;
    UniformUInt32();
}

inline uint32_t RNG::UniformUInt32() {
    uint64 oldstate = state;
    state = oldstate * PCG32_MULT + inc;
    uint32_t xorshifted = (uint32_t)(((oldstate >> 18u) ^ oldstate) >> 27u);
    uint32_t rot = (uint32_t)(oldstate >> 59u);
    return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31));
}



GYT_NAMESPACE_END