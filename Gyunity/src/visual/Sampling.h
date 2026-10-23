#pragma once
#include "common/Core.h"
#include "math/MathUtils.h"
#include "math/Linalg.h"

GYT_NAMESPACE_BEGIN

struct Distribution1D
{
private:
	friend struct Distribution2D;
	//real* mFunc, * mCdf;
	std::vector<real> mFunc, mCdf;
	real mFuncInt;
	int mCount;

public:
	Distribution1D(const real *f, int n)
	{
		mCount = n;
		//mFunc = new real[n];
		//memcpy(mFunc, f, n * sizeof(real));
		//mCdf = new real[n + 1];
		mFunc.resize(n);
		memcpy(&mFunc[0], f, n * sizeof(real));
		mCdf.resize(n + 1);

		// Compute integral of step function at $x_i$
		mCdf[0] = 0.;
		for (int i = 1; i < mCount + 1; ++i)
			mCdf[i] = mCdf[i - 1] + mFunc[i - 1] / n;

		// Transform step function integral into CDF
		mFuncInt = mCdf[mCount];
		if (mFuncInt == 0.f)
		{
			for (int i = 1; i < n + 1; ++i)
				mCdf[i] = real(i) / real(n);
		}
		else
		{
			for (int i = 1; i < n + 1; ++i)
				mCdf[i] /= mFuncInt;
		}
	}

	~Distribution1D()
	{
		//delete[] mFunc;
		//delete[] mCdf;
	}

	real SampleContinuous(real u, real *pdf, int *off = NULL) const
	{
		// Find surrounding CDF segments and _offset_
		//real *ptr = std::upper_bound(mCdf, mCdf + count + 1, u);
		//int offset = Clamp(int(ptr - mCdf - 1), 0, count - 1);
		auto it = std::upper_bound(mCdf.begin(), mCdf.begin() + mCount + 1, u);
		int offset = Clamp(int(std::distance(mCdf.begin(), it) - 1), 0, mCount - 1);
		if (off) *off = offset;
		ASSERT(offset < mCount);
		ASSERT(u >= mCdf[offset] && (u < mCdf[offset + 1] || u == 1));

		// Fix the case when func ends with zeros
		if (mCdf[offset] == mCdf[offset + 1])
		{
			ASSERT(u == 1.0f);

			do { offset--; } while (mCdf[offset] == mCdf[offset + 1] && offset > 0);

			ASSERT(mCdf[offset] != mCdf[offset + 1]);
		}

		// Compute offset along CDF segment
		real du = (u - mCdf[offset]) / (mCdf[offset + 1] - mCdf[offset]);
		ASSERT(!std::isnan(du));

		// Compute PDF for sampled offset
		if (pdf) *pdf = mFunc[offset] / mFuncInt;
		ASSERT(mFunc[offset] > 0);

		// Return $x\in{}[0,1]$ corresponding to sample
		return (offset + du) / mCount;
	}

	int SampleDiscrete(real u, real *pdf) const
	{
		// Find surrounding CDF segments and _offset_
		//real *ptr = std::upper_bound(mCdf, mCdf + count + 1, u);
		//int offset = std::max(0, int(ptr - mCdf - 1));
		auto it = std::upper_bound(mCdf.begin(), mCdf.begin() + mCount + 1, u);
		int offset = std::max(0, int(std::distance(mCdf.begin(), it)) - 1);
		ASSERT(offset < mCount);
		ASSERT(u >= mCdf[offset] && u < mCdf[offset + 1]);
		if (pdf) *pdf = mFunc[offset] / (mFuncInt * mCount);
		return offset;
	}

};

struct Distribution2D
{
private:
	std::vector<Distribution1D*> mConditionalV;
	Distribution1D* mMarginal;
public:
	Distribution2D(const real *func, int nu, int nv)
	{
		mConditionalV.reserve(nv);
		for (int v = 0; v < nv; ++v)
		{
			// Compute conditional sampling distribution for $\tilde{v}$
			mConditionalV.push_back(new Distribution1D(&func[v*nu], nu));
		}

		// Compute marginal sampling distribution $p[\tilde{v}]$
		std::vector<real> marginalFunc;
		marginalFunc.reserve(nv);
		for (int v = 0; v < nv; ++v)
			marginalFunc.push_back(mConditionalV[v]->mFuncInt);
		mMarginal = new Distribution1D(&marginalFunc[0], nv);
	}

	~Distribution2D()
	{
		delete mMarginal;
		for (uint32_t i = 0; i < mConditionalV.size(); ++i)
			delete mConditionalV[i];
	}

	void SampleContinuous(real u0, real u1, real uv[2], real *pdf) const
	{
		real pdfs[2];
		int v;
		uv[1] = mMarginal->SampleContinuous(u1, &pdfs[1], &v);
		uv[0] = mConditionalV[v]->SampleContinuous(u0, &pdfs[0]);
		*pdf = pdfs[0] * pdfs[1];
	}

	real Pdf(real u, real v) const
	{
		int iu = Clamp((int)(u * mConditionalV[0]->mCount), 0,
			mConditionalV[0]->mCount - 1);
		int iv = Clamp((int)(v * mMarginal->mCount), 0,
			mMarginal->mCount - 1);
		if (mConditionalV[iv]->mFuncInt * mMarginal->mFuncInt == 0.f) return 0.f;
		return (mConditionalV[iv]->mFunc[iu] * mMarginal->mFunc[iv]) /
			(mConditionalV[iv]->mFuncInt * mMarginal->mFuncInt);
	}
};



Vec2 ConcentricSampleDisk(const Vec2 &u);

Vec3 UniformSampleSphere(const Vec2 &u);

Vec3 CosineSampleHemisphere(const Vec2 &u);

real UniformSpherePdf();

Vec2 UniformSampleTriangle(const Vec2 &u);

real BalanceHeuristic(int nf, real fPdf, int ng, real gPdf);

real CosineHemispherePdf(real cosTheta);

Vec3 UniformSampleHemisphere(const Vec2& u);

real UniformSampleHemispherePdf();
GYT_NAMESPACE_END