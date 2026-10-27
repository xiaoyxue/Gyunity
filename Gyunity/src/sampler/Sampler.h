#pragma once

#include <memory>
#include <random>
#include <limits>
#include <typeinfo>
#include "math/LowDiscrepancy.h"
#include "visual/Rng.h"

GYT_NAMESPACE_BEGIN

class Sampler {
public:
	virtual real Sample(unsigned int d, unsigned long long i) = 0;
	virtual std::shared_ptr<Sampler> Clone(uint32 seed) = 0;
	// Legacy custom samplers can keep Clone; wide path keys require an explicit implementation.
	virtual std::shared_ptr<Sampler> CloneForPath(uint64 pathKey) {
		if (pathKey > (std::numeric_limits<uint32>::max)()) {
			GetDefaultLogger()->Error("Sampler does not support 64-bit path-stream cloning.", false);
			return nullptr;
		}
		return Clone(static_cast<uint32>(pathKey));
	}
};

class StateSequence {
protected:
	int mCursor = 0;

public:
	virtual real Sample() = 0;

	virtual real operator()() {
		return Sample();
	}

	int GetCursor() const {
		return mCursor;
	}
	/*
	void AssertCursorPos(int cursor) const {
		assert_info(
			this->cursor == cursor,
			std::string("Cursor position should be " + std::to_string(cursor) +
				" instead of " + std::to_string(this->cursor)));
	}*/



};

class RandomStateSequence : public StateSequence {
private:
	std::shared_ptr<Sampler> mSampler;
	int64 mInstance;

public:
	RandomStateSequence(std::shared_ptr<Sampler> sampler, int64 instance)
		: mSampler(sampler), mInstance(instance) 
	{
	}

	real Sample() override {
		//assert_info(sampler != nullptr, "null sampler");
		real ret = mSampler->Sample(mCursor++, mInstance);
		//assert_info(ret >= 0, "sampler output should be non-neg");
		if (ret > 1 + 1e-5f) {
			printf("Warning: sampler returns value > 1: [%f]", ret);
		}
		if (ret >= 1) {
			ret = 0;
		}
		return ret;
	}

};

class RandomSampler : public Sampler {
public:
	RandomSampler(int seed) : mSeed(static_cast<uint64>(seed)), mRng(seed), mPcg32(seed) {

	}

	real Sample(unsigned int d, unsigned long long i) {
		return mUniform(mRng);
		//return pbrtRng.UniformFloat();
	}

	std::shared_ptr<Sampler> Clone(uint32 seed) override {
		RandomSampler* newSampler = new RandomSampler(seed);
		newSampler->mPcg32.SetSequence(seed);
		return std::shared_ptr<Sampler>(newSampler);
	}
	std::shared_ptr<Sampler> CloneForPath(uint64 pathKey) override {
		// Subclasses retain their virtual Clone behavior unless they opt into wide path keys.
		if (typeid(*this) != typeid(RandomSampler)) return Sampler::CloneForPath(pathKey);
		return std::shared_ptr<Sampler>(new RandomSampler(mSeed ^ pathKey, PathSeed{}));
	}
private:
	struct PathSeed {};
	RandomSampler(uint64 seed, PathSeed) : mSeed(seed), mRng(seed), mPcg32(seed) {}
	uint64 mSeed;
	std::mt19937_64 mRng;
	std::uniform_real_distribution<real> mUniform;
	Pcg32 mPcg32;
};

class PrimeList {
public:
	PrimeList() {
		for (int i = 2; i <= 10000; i++) {
			if (IsPrime(i)) {
				mPrimes.push_back(i);
			}
		}
		GYT_DEBUG_ASSERT(mPrimes.size() == 1229);
	}

	int GetPrime(int i) {
		return mPrimes[i];
	}

	int GetPrimesNum() {
		return (int)mPrimes.size();
	}

private:
	std::vector<int> mPrimes;
};


class RegularHaltonSampler : public Sampler {
public:
	real Sample(unsigned int d, unsigned long long i) override {
		GYT_DEBUG_ASSERT(d < (unsigned)primeList.GetPrimesNum());
		real val = ReversedRadicalInverse(d, i + 1);  // The first one is evil...
		return val;
	}

	std::shared_ptr<Sampler> Clone(uint32 seed) override {
		return std::shared_ptr<Sampler>(new RegularHaltonSampler(*this));
	}

private:
	inline int ReverseDigit(const int i, const int p) const {
		return i == 0 ? i : p - i;
	}

	real ReversedRadicalInverse(const int d, long long j) const {
		const int p = primeList.GetPrime(d);
		real h = 0.f, f = 1.f / p, fct = f;
		while (j > 0) {
			h += ReverseDigit(j % p, p) * fct;
			j /= p;
			fct *= f;
		}
		return h;
	}
	static PrimeList primeList;
};


GYT_NAMESPACE_END