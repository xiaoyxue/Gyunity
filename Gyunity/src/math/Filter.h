#pragma once

#include "common/Core.h"

GYT_NAMESPACE_BEGIN

class Filter
{
protected:
	real mRadius;

public:
	Filter(const real rad)
		: mRadius(rad)
	{
	}
	virtual ~Filter() {}

	const real GetRadius() const
	{
		return mRadius;
	}
	virtual real Evaluate(const real dx, const real dy) const = 0;
};

class BoxFilter : public Filter
{
public:
	BoxFilter()
		: Filter(0.5)
	{
	}

public:
	real Evaluate(const real dx, const real dy) const override {
		return 1.0;
	}
};


class TentFilter : public Filter {
public:
	TentFilter() : Filter(1) {}
	real Evaluate(real dx, real dy) const override {
		return std::max(real(0), 1 - std::abs(dx)) * std::max(real(0), 1 - std::abs(dy));
	}
};

class GaussianFilter : public Filter
{
private:
	real mStdDev;
	real mAlpha;
	real mExpR;

public:
	GaussianFilter(const real stdDiv = 0.5f)
		: Filter(4 * stdDiv)
		, mStdDev(stdDiv)
	{
		mAlpha = -1.0f / (2.0f * mStdDev * mStdDev);
		mExpR = std::exp(mAlpha * mRadius * mRadius);
	}

public:
	real Evaluate(const real dx, const real dy) const override
	{
		auto Gaussian = [this](const real d)
		{
			return std::max(0.0f, std::exp(mAlpha * d * d) - mExpR);
		};

		return Gaussian(dx) * Gaussian(dy);
	}
};

GYT_NAMESPACE_END