#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

// TEST ONLY. Implementing VCM, revision 2, sections 4.1/4.2 and 5.4.
// Inputs are independently evaluated vertex densities/conversion factors, never
// production strategy scores. No production MIS/PDF or BDPT ratio-walk calls.
namespace VcmReference {

struct Edges {
	std::vector<double> f, r, gf, gr;
	// Pure-delta INTERIOR scattering vertices, not emitter/camera event flags.
	// Delta arrival factors in f/r are branch mass times the report geometry.
	std::vector<bool> delta;
	bool pointOrigin = false;
};

inline bool Delta(const Edges& e, int i) {
	return i > 0 && i + 1 < static_cast<int>(e.f.size()) && !e.delta.empty() && e.delta[i];
}

inline bool Feasible(const Edges& e, bool merging, int s) {
	const int n = static_cast<int>(e.f.size());
	if (merging) return s >= 2 && s < n && !Delta(e, s - 1);
	return s >= 0 && s < n && !(s == 0 && e.pointOrigin) && !(n == 2 && s == 1) &&
		(s == 0 || (!Delta(e, s - 1) && !Delta(e, s)));
}

inline bool SpecularEmission(const Edges& e, int s) {
	if (s != 0 || e.pointOrigin) return false;
	for (int i = 1; i + 1 < static_cast<int>(e.f.size()); ++i) if (!Delta(e, i)) return false;
	return true;
}

struct State {
	double dVCM = 0, dVC = 0, dVM = 0;
};

inline State Trace(const Edges& e, int count, bool eye, double eta, bool vc, bool vm, double pixels) {
	const int n = static_cast<int>(e.f.size());
	auto pdf = [&](int i) { return eye ? e.r[n - 1 - i] : e.f[i]; };
	auto geometry = [&](int i) { return eye ? e.gf[n - 1 - i] : e.gr[i]; };
	auto reverse = [&](int i) { return eye ? e.f[n - 1 - i] : e.r[i]; };
	State d;
	if (count < 2) return d;
	// Eq. (31)-(33). The report uses a conditional-pixel camera PDF and
	// n_light=pixels. Gyunity uses a global-film PDF: q_pixel=pixels*q_film.
	d.dVCM = eye ? pixels / (pixels * pdf(1)) : 1 / pdf(1);
	d.dVC = eye || e.pointOrigin ? 0 : geometry(0) / (pdf(0) * pdf(1)); // (49)
	d.dVM = vc ? d.dVC / eta : 0;
	for (int i = 2; i < count; ++i) {
		const double pdfRevDir = reverse(i - 2) / geometry(i - 2);
		const double conversion = geometry(i - 1) / pdf(i);
		const State previous = d;
		if (Delta(e, eye ? n - i : i - 1)) {
			d.dVCM = 0; // Eq. (53), remove the two adjacent VC cuts and the delta merge.
			d.dVC = conversion * pdfRevDir * previous.dVC; // (54)
			d.dVM = conversion * pdfRevDir * previous.dVM; // (55)
			continue;
		}
		d.dVCM = 1 / pdf(i); // Eq. (34)
		d.dVC = conversion * ((vm ? eta : 0) + previous.dVCM + pdfRevDir * previous.dVC); // (35)
		d.dVM = conversion * (1 + (vc ? previous.dVCM / eta : 0) + pdfRevDir * previous.dVM); // (36)
	}
	return d;
}

inline double Weight(const Edges& e, bool merging, int s, double area, double count, bool vc, bool vm) {
	const int n = static_cast<int>(e.f.size());
	const int t = n - s + (merging ? 1 : 0);
	if ((merging && !vm) || (!merging && !vc && !SpecularEmission(e, s))) return 0;
	if (!Feasible(e, merging, s)) return 0;
	if (n == 2) return !merging && s == 0 ? 1 : 0;
	const double eta = count * area;
	const State light = Trace(e, s, false, eta, vc, vm, count);
	const State eye = Trace(e, t, true, eta, vc, vm, count);
	double wLight = 0, wEye = 0;
	if (merging) {
		const int j = s - 1;
		wLight = (vc ? light.dVCM / eta : 0) + e.r[j - 1] / e.gr[j - 1] * light.dVM; // (38)
		wEye = (vc ? eye.dVCM / eta : 0) + e.f[j + 1] / e.gf[j + 1] * eye.dVM; // (39)
	}
	else if (s == 0) {
		wEye = e.f[0] * eye.dVCM + e.f[0] * e.f[1] / e.gf[1] * eye.dVC; // (42)-(43)
	}
	else if (s == 1) {
		wLight = e.pointOrigin ? 0 : e.r[0] / e.f[0]; // (44), section 5.3
		wEye = e.f[1] * ((vm ? eta : 0) + eye.dVCM + e.f[2] / e.gf[2] * eye.dVC); // (45)
	}
	else {
		wLight = e.r[s - 1] * ((vm ? eta : 0) + light.dVCM +
			e.r[s - 2] / e.gr[s - 2] * light.dVC); // (40)
		if (t == 1) {
			// Eq. (46)-(47): conditional camera directional PDF / n_light.
			wLight = (count * e.r[s - 1]) / count * ((vm ? eta : 0) + light.dVCM +
				e.r[s - 2] / e.gr[s - 2] * light.dVC);
		}
		else {
			wEye = e.f[s] * ((vm ? eta : 0) + eye.dVCM + e.f[s + 1] / e.gf[s + 1] * eye.dVC); // (41)
		}
	}
	return 1 / (1 + wLight + wEye); // (37), balance (not power-two)
}

// A third computation: direct Eq. (10)-(12) products with a binary exponent,
// rather than logs or recurrences, also usable when naive products underflow.
struct Product {
	double mantissa = 1;
	int exponent = 0;
	void Multiply(double value) {
		int shift = 0;
		mantissa *= std::frexp(value, &shift);
		exponent += shift;
		mantissa = std::frexp(mantissa, &shift);
		exponent += shift;
	}
};

inline std::vector<double> DirectWeights(const Edges& e, double area, double count, bool vc, bool vm) {
	const int n = static_cast<int>(e.f.size());
	std::vector<Product> products;
	for (int kind = 0; kind < 2; ++kind) {
		for (int s = kind == 0 ? 0 : 2; s < n; ++s) {
			Product p;
			if ((kind == 0 && !vc && !SpecularEmission(e, s)) ||
				(kind == 1 && !vm) || !Feasible(e, kind == 1, s)) p.Multiply(0);
			for (int i = 0; i < s; ++i) p.Multiply(e.f[i]);
			for (int i = s - kind; i < n; ++i) p.Multiply(e.r[i]);
			if (kind == 1) { p.Multiply(area); p.Multiply(count); }
			products.push_back(p);
		}
	}
	int maximum = std::numeric_limits<int>::min();
	for (const Product& p : products) if (p.mantissa != 0) maximum = std::max(maximum, p.exponent);
	std::vector<double> weights;
	double sum = 0;
	for (const Product& p : products) {
		const double value = std::ldexp(p.mantissa, p.exponent - maximum);
		weights.push_back(value);
		sum += value;
	}
	for (double& weight : weights) weight /= sum;
	return weights;
}

}
