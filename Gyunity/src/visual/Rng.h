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
class Pcg32 {
  public:
    // Pcg32 Public Methods
    Pcg32();
    Pcg32(uint64 sequenceIndex) { SetSequence(sequenceIndex); }
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
        uint64 cur_mult = PCG32_MULT, cur_plus = mInc, acc_mult = 1u,
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
        mState = acc_mult * mState + acc_plus;
    }
    int64_t operator-(const Pcg32 &other) const {
        uint64 cur_mult = PCG32_MULT, cur_plus = mInc, cur_state = other.mState,
                 the_bit = 1u, distance = 0u;
        while (mState != cur_state) {
            if ((mState & the_bit) != (cur_state & the_bit)) {
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
    // Pcg32 Private Data
    uint64 mState, mInc;
};

// Pcg32 Inline Method Definitions
inline Pcg32::Pcg32() : mState(PCG32_DEFAULT_STATE), mInc(PCG32_DEFAULT_STREAM) {}
inline void Pcg32::SetSequence(uint64 initseq) {
    mState = 0u;
    mInc = (initseq << 1u) | 1u;
    UniformUInt32();
    mState += PCG32_DEFAULT_STATE;
    UniformUInt32();
}

inline uint32_t Pcg32::UniformUInt32() {
    uint64 oldstate = mState;
    mState = oldstate * PCG32_MULT + mInc;
    uint32_t xorshifted = (uint32_t)(((oldstate >> 18u) ^ oldstate) >> 27u);
    uint32_t rot = (uint32_t)(oldstate >> 59u);
    return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31));
}



GYT_NAMESPACE_END