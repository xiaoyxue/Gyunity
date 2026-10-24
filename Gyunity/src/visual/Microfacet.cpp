#include "Microfacet.h"
#include "bsdf/BSDF.h"

GYT_NAMESPACE_BEGIN

static void TrowbridgeReitzSample11(real cosTheta, real U1, real U2,
	real* slopeX, real* slope_y) {
	// special case (normal incidence)
	if (cosTheta > .9999) {
		real r = sqrt(U1 / (1 - U1));
		real phi = 6.28318530718 * U2;
		*slopeX = r * cos(phi);
		*slope_y = r * sin(phi);
		return;
	}

	real sinTheta =
		std::sqrt(std::max((real)0, (real)1 - cosTheta * cosTheta));
	real tanTheta = sinTheta / cosTheta;
	real a = 1 / tanTheta;
	real G1 = 2 / (1 + std::sqrt(1.f + 1.f / (a * a)));

	// sample slopeX
	real A = 2 * U1 / G1 - 1;
	real tmp = 1.f / (A * A - 1.f);
	if (tmp > 1e10) tmp = 1e10;
	real B = tanTheta;
	real D = std::sqrt(
		std::max(real(B * B * tmp * tmp - (A * A - B * B) * tmp), real(0)));
	real slopeX1 = B * tmp - D;
	real slopeX2 = B * tmp + D;
	*slopeX = (A < 0 || slopeX2 > 1.f / tanTheta) ? slopeX1 : slopeX2;

	// sample slope_y
	real S;
	if (U2 > 0.5f) {
		S = 1.f;
		U2 = 2.f * (U2 - .5f);
	}
	else {
		S = -1.f;
		U2 = 2.f * (.5f - U2);
	}
	real z =
		(U2 * (U2 * (U2 * 0.27385f - 0.73369f) + 0.46341f)) /
		(U2 * (U2 * (U2 * 0.093073f + 0.309420f) - 1.000000f) + 0.597999f);
	*slope_y = S * z * std::sqrt(1.f + *slopeX * *slopeX);

}

static Vec3 TrowbridgeReitzSample(const Vec3& wi, real alphaX,
	real alphaY, real U1, real U2) {
	// 1. stretch wi
	Vec3 wiStretched =
		Vec3(alphaX * wi.x, alphaY * wi.y, wi.z).Norm();

	// 2. simulate P22_{wi}(x_slope, y_slope, 1, 1)
	real slopeX, slope_y;
	TrowbridgeReitzSample11(BSDFCoordinate::CosTheta(wiStretched), U1, U2, &slopeX, &slope_y);

	// 3. rotate
	real tmp = BSDFCoordinate::CosPhi(wiStretched) * slopeX - BSDFCoordinate::SinPhi(wiStretched) * slope_y;
	slope_y = BSDFCoordinate::SinPhi(wiStretched) * slopeX + BSDFCoordinate::CosPhi(wiStretched) * slope_y;
	slopeX = tmp;

	// 4. unstretch
	slopeX = alphaX * slopeX;
	slope_y = alphaY * slope_y;

	// 5. compute normal
	return Vec3(-slopeX, -slope_y, 1.).Norm();
}


static Vec2 GGXSampleVisible11(real thetaI, Vec2 sample)  {
	const real SQRT_PI_INV = 1 / std::sqrt(PI);
	Vector2 slope;

	/* Special case (normal incidence) */
	if (thetaI < 1e-4f) {
		real sinPhi = std::sin(2 * PI * sample.y), cosPhi = std::cos(2 * PI * sample.y);
		real r = SafeSqrt(sample.x / (1 - sample.x));
		return Vec2(r * cosPhi, r * sinPhi);
	}

	/* Precomputations */
	real tanThetaI = std::tan(thetaI);
	real a = 1 / tanThetaI;
	real G1 = 2.0f / (1.0f + SafeSqrt(1.0f + 1.0f / (a * a)));

	/* Simulate X component */
	real A = 2.0f * sample.x / G1 - 1.0f;
	if (std::abs(A) == 1)
		A -= Sgn(A) * 1e-4;
	real tmp = 1.0f / (A * A - 1.0f);
	real B = tanThetaI;
	real D = SafeSqrt(B * B * tmp * tmp - (A * A - B * B) * tmp);
	real slopeX1 = B * tmp - D;
	real slopeX2 = B * tmp + D;
	slope.x = (A < 0.0f || slopeX2 > 1.0f / tanThetaI) ? slopeX1 : slopeX2;

	/* Simulate Y component */
	real S;
	if (sample.y > 0.5f) {
		S = 1.0f;
		sample.y = 2.0f * (sample.y - 0.5f);
	}
	else {
		S = -1.0f;
		sample.y = 2.0f * (0.5f - sample.y);
	}

	/* Improved fit */
	real z =
		(sample.y * (sample.y * (sample.y * (-(real)0.365728915865723) + (real)0.790235037209296) -
		(real)0.424965825137544) + (real)0.000152998850436920) /
			(sample.y * (sample.y * (sample.y * (sample.y * (real)0.169507819808272 - (real)0.397203533833404) -
		(real)0.232500544458471) + (real)1) - (real)0.539825872510702);

	slope.y = S * z * std::sqrt(1.0f + slope.x * slope.x);

	return slope;
}


static Vec3 GGXSampleVisible(const Vec3& _wi, const Vec2& sample, real alphaU, real alphaV) {
	/* Step 1: stretch wi */
	Vec3 wi = Vec3(alphaU * _wi.x, alphaV * _wi.y, _wi.z).Norm();

	/* Get polar coordinates */
	real theta = 0, phi = 0;
	if (wi.z < (real)0.99999) {
		theta = std::acos(wi.z);
		phi = std::atan2(wi.y, wi.x);
	}
	real sinPhi = std::sin(phi), cosPhi = std::cos(phi);

	/* Step 2: simulate P22_{wi}(slope.x, slope.y, 1, 1) */
	Vec2 slope = GGXSampleVisible11(theta, sample);

	/* Step 3: rotate */
	slope = Vec2(
		cosPhi * slope.x - sinPhi * slope.y,
		sinPhi * slope.x + cosPhi * slope.y);

	/* Step 4: unstretch */
	slope.x *= alphaU;
	slope.y *= alphaV;

	/* Step 5: compute normal */
	real normalization = (real)1 / std::sqrt(slope.x * slope.x + slope.y * slope.y + (real)1.0);

	return Vec3(-slope.x * normalization, -slope.y * normalization, normalization);
}



real MicrofacetDistribution::Pdf(const Vec3& wo, const Vec3& wh) const {
	if (mSampleVisibleArea) {
		if (IsPixelDebugEnabled()) {
			std::cout << "D: " << D(wh) << " G1: " << G1(wo, wh) << std::endl;
		}
		return D(wh) * G1(wo, wh) * std::abs(Dot(wo, wh)) / BSDFCoordinate::AbsCosTheta(wo);
	}
        
    else
        return D(wh) * BSDFCoordinate::AbsCosTheta(wh);
}

real GGXDistribution::Lambda(const Vec3& w) const {
	real tanTheta = BSDFCoordinate::TanTheta(w);
	if (std::isinf(std::abs(tanTheta))) return 0.f;
	real invA2;
	if (mAlphax == mAlphay) {
		invA2 = mAlphax * mAlphax * tanTheta * tanTheta;
	}
	else {
		real cos2Phi = BSDFCoordinate::Cos2Phi(w);
		real sin2Phi = BSDFCoordinate::Sin2Phi(w);
		real alphaO = std::sqrt(cos2Phi * mAlphax * mAlphax + sin2Phi * mAlphay * mAlphay);
		invA2 = alphaO * alphaO * tanTheta * tanTheta;
	}
	return (-1 + std::sqrt(1 + invA2)) / 2;
}


//real GGXDistribution::D(const Vec3& wh) const {
//	if (alphax == alphay) {
//		real alpha = alphax;
//		const real alpha2 = alpha * alpha;
//		const real cos2ThetaM = BSDFCoordinate::Cos2Theta(wh);
//		const real cos4ThetaM = cos2ThetaM * cos2ThetaM;
//		const real tan2ThetaM = BSDFCoordinate::Tan2Theta(wh);
//
//		if (std::isinf(tan2ThetaM)) return 0.;
//		const real root = alpha2 + tan2ThetaM;
//
//		return alpha2 / (PI * cos4ThetaM * root * root);
//	}
//	else {
//		//const real cos2PhiM = BSDFCoordinate::Cos2Phi(wh);
//		//const real sin2PhiM = BSDFCoordinate::Sin2Phi(wh);
//		//const real tan2ThetaM = BSDFCoordinate::Tan2Theta(wh);
//		//const real cosThetaM = BSDFCoordinate::CosTheta(wh);
//		//const real cos2ThetaM = BSDFCoordinate::Cos2Theta(wh);
//
//		//if (std::isinf(tan2ThetaM)) return 0.;
//		//const real alphax2 = alphax * alphax;
//		//const real alphay2 = alphay * alphay;
//		//const real root = 1 + tan2ThetaM * (cos2PhiM / alphax2 + sin2PhiM / alphay2);
//		//return 1 / (PI * alphax * alphay * cos2ThetaM * cos2ThetaM * root * root);
//
//		real tan2Theta = BSDFCoordinate::Tan2Theta(wh);
//		if (std::isinf(tan2Theta)) return 0.;
//		const real cos4Theta = BSDFCoordinate::Cos2Theta(wh) * BSDFCoordinate::Cos2Theta(wh);
//		real e =
//			(BSDFCoordinate::Cos2Phi(wh) / (alphax * alphax) + BSDFCoordinate::Sin2Phi(wh) / (alphay * alphay)) *
//			tan2Theta;
//		return 1 / (PI * alphax * alphay * cos4Theta * (1 + e) * (1 + e));
//
//		//real tan2Theta = BSDFCoordinate::Tan2Theta(wh);
//		//if (std::isinf(tan2Theta)) return 0.;
//		//const real cos4Theta = BSDFCoordinate::Cos2Theta(wh) * BSDFCoordinate::Cos2Theta(wh);
//		//real e =
//		//	(BSDFCoordinate::Cos2Phi(wh) / (alphax * alphax) + BSDFCoordinate::Sin2Phi(wh) / (alphay * alphay)) *
//		//	tan2Theta;
//		//return 1 / (PI * alphax * alphay * cos4Theta * (1 + e) * (1 + e));
//	}
//}


real GGXDistribution::D(const Vec3& m) const {
	if (BSDFCoordinate::CosTheta(m) <= 0)
		return 0.0f;

	real cosTheta2 = BSDFCoordinate::Cos2Theta(m);
	real beckmannExponent = ((m.x * m.x) / (mAlphax * mAlphax)
		+ (m.y * m.y) / (mAlphay * mAlphay)) / cosTheta2;

	real result;
	
	/* GGX / Trowbridge-Reitz distribution function for rough surfaces */
	real root = ((real)1 + beckmannExponent) * cosTheta2;
	result = (real)1 / (PI * mAlphax * mAlphay * root * root);
	

	/* Prevent potential numerical issues in other stages of the model */
	if (result * BSDFCoordinate::CosTheta(m) < 1e-20f)
		result = 0;

	return result;
}


real GGXDistribution::G1(const Vec3& v, const Vec3& wh) const {
	return 1 / (1 + Lambda(v));
}

real GGXDistribution::G(const Vec3& wo, const Vec3& wi, const Vec3& wh) const {
    //return G1(wo, wh) * G1(wi, wh);
	//return 1 / (1 + Lambda(wo) + Lambda(wi));
	return SmithG1(wo, wh) * SmithG1(wi, wh);
}

Vec3 GGXDistribution::SampleWh(const Vec3& wo, const Vec2& u) const {
	if (!mSampleVisibleArea) {
		Vec3 wh;
		if (mAlphax == mAlphay) {
			real alpha = mAlphax;
			real alpha2 = alpha * alpha;
			real tan2Theta = alpha2 * u[0] / (1 - u[0]);
			real cosTheta = 1.f / std::sqrt(1 + tan2Theta);
			real cos2Theta = cosTheta * cosTheta;
			real sinTheta = std::sqrt(std::max(real(0), 1 - cos2Theta));
			real phi = 2 * PI * u[1];
			wh = SphericalDirection(sinTheta, cosTheta, phi);
		}
		else {
			real tanPhi = mAlphay / mAlphax * std::tan(2 * PI * u[1]);
			real phi;
			if (u[1] >= 0 && u[1] <= 0.25) {
				phi = std::atan(tanPhi);
			}
			else if (u[1] > 0.25 && u[1] < 0.75) {
				phi = std::atan(tanPhi) + PI;
			}
			else {
				phi = std::atan(tanPhi) + 2 * PI;
			}
			real cosPhi = std::cos(phi);
			real cos2Phi = cosPhi * cosPhi;
			real sinPhi = std::sin(phi);
			real sin2Phi = sinPhi * sinPhi;
			real alphax2 = mAlphax * mAlphax;
			real alphay2 = mAlphay * mAlphay;
			real tanTheta = u[0] / ((1 - u[0]) * (cos2Phi / alphax2 + sin2Phi / alphay2));
			real theta = std::atan(tanTheta);
			real cosTheta = std::cos(theta);
			real sinTheta = std::cos(theta);
			wh = SphericalDirection(sinTheta, cosTheta, phi);
		}
		if (!SameHemisphere(wo, wh)) wh = -wh;
		return wh;
	}
	else {

		Vec3 wh = SampleGGXVNDF(wo, u);
		return wh;

		//Vec3 wh = GGXSampleVisible(wo, u, alphax, alphay);
		//return wh;
	}
	

}

//https://hal.archives-ouvertes.fr/hal-01509746
Vec3 GGXDistribution::SampleGGXVNDF(const Vec3& v_, const Vec2& u) const {
	//stretch view
	Vec3 v = Vec3(mAlphax * v_.x, mAlphay * v_.y, v_.z).Norm();

	//orthonormal basis
	Vec3 T1 = (v.z < 0.9999) ? Cross(v, Vec3(0, 0, 1)).Norm() : Vec3(1, 0, 0);
	Vec3 T2 = Cross(T1, v);

	//sample point with polar coordinates
	real a = 1 / (1.f + v.z);
	real r = std::sqrt(u[0]);
	real phi = (u[1] < a) ? u[1] / a * PI : PI + (u[1] - a) / (1.f - a) * PI;
	real P1 = r * std::cos(phi);
	real P2 = r * std::sin(phi) * ((u[1] < a) ? 1.0f : v.z);

	//compute normal
	Vec3 dir = P1 * T1 + P2 * T2 + std::sqrt(std::max(real(0), 1 - P1 * P1 - P2 * P2)) * v;

	//unstretch
	dir = Vec3(mAlphax * dir.x, mAlphay * dir.y, std::max(real(0), dir.z)).Norm();
	return dir;
}





real GGXDistribution::ProjectRoughness(const Vec3& v) const {
	real invSinTheta2 = 1 / BSDFCoordinate::Sin2Theta(v);

	if (mAlphax == mAlphay || invSinTheta2 <= 0)
		return mAlphax;

	real cosPhi2 = v.x * v.x * invSinTheta2;
	real sinPhi2 = v.y * v.y * invSinTheta2;

	return std::sqrt(cosPhi2 * mAlphax * mAlphax + sinPhi2 * mAlphay * mAlphay);
}

real GGXDistribution::SmithG1(const Vec3& v, const Vec3& m) const {
	/* Ensure consistent orientation (can't see the back
	   of the microfacet from the front and vice versa) */
	if (Dot(v, m) * BSDFCoordinate::CosTheta(v) <= 0)
		return 0.0f;

	/* Perpendicular incidence -- no shadowing/masking */
	real tanTheta = std::abs(BSDFCoordinate::TanTheta(v));
	if (tanTheta == 0.0f)
		return 1.0f;

	real alpha = ProjectRoughness(v);

	real root = alpha * tanTheta;
	return 2.0f / (1.0f + std::sqrt(1 + root * root));

}

real GGXDistribution::Pdf(const Vec3& wo, const Vec3& wh) const {
	if (BSDFCoordinate::CosTheta(wo) == 0)
		return 0.0f;
	return SmithG1(wo, wh) * std::abs(Dot(wo, wh)) * D(wh) / std::abs(BSDFCoordinate::CosTheta(wo));
}

GYT_NAMESPACE_END