#pragma once

#include "common/Core.h"
#include "NumericUtils.h"
#include <iostream>

GYT_NAMESPACE_BEGIN

class EReal {
public:
	// EReal Public Methods
	EReal() {}
	EReal(real v, real err = 0.0) : mV(v) {
		if (err == 0.)
			mLow = mHigh = v;
		else {
			// Compute conservative bounds by rounding the endpoints away
			// from the middle. Note that this will be over-conservative in
			// cases where v-err or v+err are exactly representable in
			// floating-point, but it's probably not worth the trouble of
			// checking this case.
			mLow = NextFloatDown(v - err);
			mHigh = NextFloatUp(v + err);
		}
		// Store high precision reference value in _EReal_
#ifndef NDEBUG
		mVPrecise = v;
		//Check();
#endif  // NDEBUG
		}
#ifndef NDEBUG
	EReal(real v, long double lD, real err) : EReal(v, err) {
		mVPrecise = lD;
	}
#endif  // DEBUG

	EReal operator+(EReal ef) const {
		EReal r;
		r.mV = mV + ef.mV;
#ifndef NDEBUG
		r.mVPrecise = mVPrecise + ef.mVPrecise;
#endif  // DEBUG
		// Interval arithemetic addition, with the result rounded away from
		// the value r.v in order to be conservative.
		r.mLow = NextFloatDown(LowerBound() + ef.LowerBound());
		r.mHigh = NextFloatUp(UpperBound() + ef.UpperBound());
		//r.Check();
		return r;
	}

	explicit operator real() const { return mV; }
	real GetAbsoluteError() const { return mHigh - mLow; }
	real UpperBound() const { return mHigh; }
	real LowerBound() const { return mLow; }

#ifndef NDEBUG
	real GetRelativeError() const {
		return std::abs((mVPrecise - mV) / mVPrecise);
	}
	long double PreciseValue() const { return mVPrecise; }
#endif

	EReal operator-(EReal ef) const {
		EReal r;
		r.mV = mV - ef.mV;
#ifndef NDEBUG
		r.mVPrecise = mVPrecise - ef.mVPrecise;
#endif
		r.mLow = NextFloatDown(LowerBound() - ef.UpperBound());
		r.mHigh = NextFloatUp(UpperBound() - ef.LowerBound());
		//r.Check();
		return r;
	}
	EReal operator*(EReal ef) const {
		EReal r;
		r.mV = mV * ef.mV;
#ifndef NDEBUG
		r.mVPrecise = mVPrecise * ef.mVPrecise;
#endif
		real prod[4] = {
			LowerBound() * ef.LowerBound(), UpperBound() * ef.LowerBound(),
			LowerBound() * ef.UpperBound(), UpperBound() * ef.UpperBound() };
		r.mLow = NextFloatDown(
			std::min(std::min(prod[0], prod[1]), std::min(prod[2], prod[3])));
		r.mHigh = NextFloatUp(
			std::max(std::max(prod[0], prod[1]), std::max(prod[2], prod[3])));
		//r.Check();
		return r;
	}

	EReal operator/(EReal ef) const {
		EReal r;
		r.mV = mV / ef.mV;
#ifndef NDEBUG
		r.mVPrecise = mVPrecise / ef.mVPrecise;
#endif
		if (ef.mLow < 0 && ef.mHigh > 0) {
			// Bah. The interval we're dividing by straddles zero, so just
			// return an interval of everything.
			r.mLow = -Infinity;
			r.mHigh = Infinity;
		}
		else {
			real div[4] = {
				LowerBound() / ef.LowerBound(), UpperBound() / ef.LowerBound(),
				LowerBound() / ef.UpperBound(), UpperBound() / ef.UpperBound() };
			r.mLow = NextFloatDown(
				std::min(std::min(div[0], div[1]), std::min(div[2], div[3])));
			r.mHigh = NextFloatUp(
				std::max(std::max(div[0], div[1]), std::max(div[2], div[3])));
		}
		//r.Check();
		return r;
	}
	EReal operator-() const {
		EReal r;
		r.mV = -mV;
#ifndef NDEBUG
		r.mVPrecise = -mVPrecise;
#endif
		r.mLow = -mHigh;
		r.mHigh = -mLow;
		//r.Check();
		return r;
	}
	GYT_FORCE_INLINE bool operator==(EReal fe) const { return mV == fe.mV; }
//	FORCE_INLINE void Check() const {
//		if (!std::isinf(low) && !std::isnan(low) && !std::isinf(high) &&
//			!std::isnan(high))
//			CHECK_LE(low, high);
//#ifndef NDEBUG
//		if (!std::isinf(v) && !std::isnan(v)) {
//			CHECK_LE(LowerBound(), vPrecise);
//			CHECK_LE(vPrecise, UpperBound());
//		}
//#endif
//	}
	EReal(const EReal &ef) {
		//ef.Check();
		mV = ef.mV;
		mLow = ef.mLow;
		mHigh = ef.mHigh;
#ifndef NDEBUG
		mVPrecise = ef.mVPrecise;
#endif
	}
	EReal &operator=(const EReal &ef) {
		//ef.Check();
		if (&ef != this) {
			mV = ef.mV;
			mLow = ef.mLow;
			mHigh = ef.mHigh;
#ifndef NDEBUG
			mVPrecise = ef.mVPrecise;
#endif
		}
		return *this;
	}

	friend std::ostream &operator<<(std::ostream &os, const EReal &ef) {
		printf("v=%f (%a) - [%f, %f]",	ef.mV, ef.mV, ef.mLow, ef.mHigh);
		//os << "v=" << ef.v << " - "
		//	<< "[" << ef.low << ", " << ef.high << "] ";
#ifndef NDEBUG
		printf(", precise=%.30Lf", ef.mVPrecise);
		//os << ", precise=" << ef.vPrecise;
#endif // !NDEBUG
		return os;
	}

private:
	// EReal Private Data
	real mV, mLow, mHigh;
#ifndef NDEBUG
	long double mVPrecise;
#endif  // NDEBUG
	friend GYT_FORCE_INLINE EReal Sqrt(EReal fe);
	friend GYT_FORCE_INLINE EReal Abs(EReal fe);
	friend GYT_FORCE_INLINE bool Quadratic(EReal A, EReal B, EReal C, EReal *t0, EReal *t1);
};

// EReal FORCE_INLINE Functions
GYT_FORCE_INLINE EReal operator*(float f, EReal fe) { return EReal(f) * fe; }

GYT_FORCE_INLINE EReal operator/(float f, EReal fe) { return EReal(f) / fe; }

GYT_FORCE_INLINE EReal operator+(float f, EReal fe) { return EReal(f) + fe; }

GYT_FORCE_INLINE EReal operator-(float f, EReal fe) { return EReal(f) - fe; }

GYT_FORCE_INLINE EReal Sqrt(EReal fe) {
	EReal r;
	r.mV = std::sqrt(fe.mV);
#ifndef NDEBUG
	r.mVPrecise = std::sqrt(fe.mVPrecise);
#endif
	r.mLow = NextFloatDown(std::sqrt(fe.mLow));
	r.mHigh = NextFloatUp(std::sqrt(fe.mHigh));
	//r.Check();
	return r;
}

GYT_FORCE_INLINE EReal Abs(EReal fe) {
	if (fe.mLow >= 0)
		// The entire interval is greater than zero, so we're all set.
		return fe;
	else if (fe.mHigh <= 0) {
		// The entire interval is less than zero.
		EReal r;
		r.mV = -fe.mV;
#ifndef NDEBUG
		r.mVPrecise = -fe.mVPrecise;
#endif
		r.mLow = -fe.mHigh;
		r.mHigh = -fe.mLow;
		//r.Check();
		return r;
	}
	else {
		// The interval straddles zero.
		EReal r;
		r.mV = std::abs(fe.mV);
#ifndef NDEBUG
		r.mVPrecise = std::abs(fe.mVPrecise);
#endif
		r.mLow = 0;
		r.mHigh = std::max(-fe.mLow, fe.mHigh);
		//r.Check();
		return r;
	}
}

GYT_FORCE_INLINE bool Quadratic(EReal A, EReal B, EReal C, EReal *t0, EReal *t1);
GYT_FORCE_INLINE bool Quadratic(EReal A, EReal B, EReal C, EReal *t0, EReal *t1) {
	// Find quadratic discriminant
	double discrim = (double)B.mV * (double)B.mV - 4. * (double)A.mV * (double)C.mV;
	if (discrim < 0.) return false;
	double rootDiscrim = std::sqrt(discrim);

	EReal floatRootDiscrim(rootDiscrim, MachineEps * rootDiscrim);

	// Compute quadratic _t_ values
	EReal q;
	if ((real)B < 0)
		q = -.5 * (B - floatRootDiscrim);
	else
		q = -.5 * (B + floatRootDiscrim);
	*t0 = q / A;
	*t1 = C / q;
	if ((real)*t0 > (real)*t1) std::swap(*t0, *t1);
	return true;
}

GYT_NAMESPACE_END