#pragma once

#include <functional>
#include "common/Core.h"
#include "simd/SSE.h"
#include "simd/AVX.h"

GYT_NAMESPACE_BEGIN

enum class IntrinsicSet {
	None,
	SSE,
	AVX
};

enum class TensorType {
	VECTOR,
	MATRIX
};

#if defined ISE_NONE
constexpr IntrinsicSet defaultInstructionSet = IntrinsicSet::None;
#elif defined ISE_SSE
constexpr IntrinsicSet defaultInstructionSet = IntrinsicSet::SSE;
#define USING_SIMD
#elif defined ISE_AVX
#define USING_SIMD
#endif

template<int dim, typename T, IntrinsicSet ISE, class Enable = void>
struct VectorBase {
	static constexpr int elements = dim;
	static constexpr bool simd = false;
	T d[dim];
};

template<typename T, IntrinsicSet ISE>
struct VectorBase<1, T, ISE, void> {
	static constexpr int elements = 1;
	static constexpr bool simd = false;
	union {
		T d[1];
		struct {
			T x;
		};
	};
};

template<typename T, IntrinsicSet ISE>
struct VectorBase<2, T, ISE, void> {
	static constexpr int elements = 2;
	static constexpr int simd = false;
	union {
		T d[2];
		struct {
			T x, y;
		};
	};
};


template<typename T, IntrinsicSet ISE>
struct VectorBase<3, T, ISE, typename std::enable_if_t<ISE < IntrinsicSet::SSE>> {
	static constexpr int elements = 3;
	static constexpr bool simd = false;
	union {
		T d[3];
		struct {
			T x, y, z;
		};
	};
};

template<IntrinsicSet ISE>
struct GYT_ALIGNED(16) VectorBase<3, float32, ISE, typename std::enable_if_t<ISE == IntrinsicSet::SSE> > {
	static constexpr int elements = 3;
	static constexpr bool simd = true;
	union {
		__m128 v;
		float32 d[4];
		struct {
			float32 x, y, z, _w;
		};
	};

	GYT_FORCE_INLINE VectorBase(float32 a = 0.f): v(_mm_set_ps1(a)) {}

	explicit VectorBase(__m128 v) : v(v) {}
};

template<typename T, IntrinsicSet ISE>
struct VectorBase<4, T, ISE, std::enable_if_t<ISE < IntrinsicSet::SSE>> {
	static constexpr int elements = 4;
	static constexpr bool simd = false;
	union {
		T d[4];
		struct {
			T x, y, z, w;
		};
	};
};

template<IntrinsicSet ISE>
struct GYT_ALIGNED(16) VectorBase < 4, float32, ISE, std::enable_if_t<ISE >= IntrinsicSet::SSE> > {
	static constexpr int elements = 4;
	static constexpr bool simd = true;
	union {
		__m128 v;
		float32 d[4];
		struct {
			float32 x, y, z, w;
		};
	};

	GYT_FORCE_INLINE VectorBase(float32 a = 0.f) : v(_mm_set_ps1(a)) {}

	explicit VectorBase(__m128 v) : v(v) {}
};


template<int dim_, typename T, IntrinsicSet ISE = defaultInstructionSet>
struct Vector : public VectorBase<dim_, T, ISE> {
	static constexpr int dim = dim_;
	static constexpr int elements = dim_;
	static constexpr TensorType tensorType = TensorType::VECTOR;

	using type = T;

	template <int dim__, typename T_, IntrinsicSet ISE_>
	static constexpr bool SIMD_4_32F = (dim__ == 3 || dim__ == 4) &&
		std::is_same<T_, float32>::value && ISE_ >= IntrinsicSet::SSE;

	template <int dim__, typename T_, IntrinsicSet ISE_>
	static constexpr bool SIMD_4_64F = (dim__ == 3 || dim__ == 4) &&
		std::is_same<T_, float32>::value && ISE_ >= IntrinsicSet::SSE;

	template <int dim__, typename T_, IntrinsicSet ISE_>
	static constexpr bool NOT_SIMD_4_32F = !SIMD_4_32F<dim__, T_, ISE_>;

	template <int dim__, typename T_, IntrinsicSet ISE_>
	static constexpr bool SIMD_NONE = !SIMD_4_32F<dim__, T_, ISE_>;

	template<int dim__, typename T_, IntrinsicSet ISE_>
	static constexpr bool SSE_4_32F = (dim__ == 3 || dim__ == 4) &&
		std::is_same<T_, float32>::value && ISE_ == IntrinsicSet::SSE;

	GYT_FORCE_INLINE Vector() {
		for (int i = 0; i < dim; ++i) {
			this->d[i] = T(0);
		}
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
	explicit GYT_FORCE_INLINE Vector() : VectorBase(_mm_set_ps1(0.f)) { }

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0 >
	explicit GYT_FORCE_INLINE Vector(__m128 v) : VectorBase<dim__, T_, ISE_>(v) { }

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
	explicit GYT_FORCE_INLINE Vector(real x, real y, real z, real w = 0.f) :
		VectorBase<dim__, T_, ISE_>(_mm_set_ps(w, z, y, x)) { }


	//FORCE_INLINE Vector(T *a) {
	//	memcpy(&(this->d[0]), a, sizeof(T) * dim);
	//}

	
	template<typename T_>
	GYT_FORCE_INLINE Vector(const std::vector<T_> &a) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] = a[i];
		}
	}

	GYT_FORCE_INLINE Vector(const Vector &a) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] = a.d[i];
		}
	}

	GYT_FORCE_INLINE Vector(Vector &&a) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] = std::move(a.d[i]);
		}
	}

	GYT_FORCE_INLINE Vector(T a) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] = a;
		}
	}

	template<int dim__ = dim, typename T_ = T, typename std::enable_if_t<dim__ == 1, int> = 0>
	GYT_FORCE_INLINE Vector(T x) {
		this->d[0] = x;
	}

	template<int dim__ = dim, typename T_ = T, typename std::enable_if_t<dim__ == 2, int> = 0>
	GYT_FORCE_INLINE Vector(T x, T y) {
		this->d[0] = x;
		this->d[1] = y;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE, 
		typename std::enable_if_t<dim__ == 3 && ((ISE_ == IntrinsicSet::None) || (ISE_ >= IntrinsicSet::SSE && !std::is_same<T_, float32>::value))
			, int> = 0>
	GYT_FORCE_INLINE Vector(T x, T y, T z) {
		this->d[0] = x;
		this->d[1] = y;
		this->d[2] = z;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<dim__ == 4 && ((ISE_ == IntrinsicSet::None) || (ISE_ >= IntrinsicSet::SSE && !std::is_same<T_, float32>::value))
		, int> = 0>
	GYT_FORCE_INLINE Vector(T x, T y, T z, T w) {
		this->d[0] = x;
		this->d[1] = y;
		this->d[2] = z;
		this->d[3] = w;
	}

	GYT_FORCE_INLINE T& operator[](int i) {
		return this->d[i];
	}

	GYT_FORCE_INLINE const T& operator[](int i) const {
		return this->d[i];
	}

	GYT_FORCE_INLINE T& operator()(int i) {
		return this->d[i];
	}

	GYT_FORCE_INLINE const T& operator()(int i) const {
		return this->d[i];
	}

	GYT_FORCE_INLINE Vector& operator=(const Vector &a) {
		//memcpy(&(this->d[0]), &(a.d[0]), sizeof(T) * dim);
		memcpy(this, &a, sizeof(*this));
		return *this;
	}

	GYT_FORCE_INLINE Vector& operator=(Vector &&a) {
		//memcpy(&(this->d[0]), &(a.d[0]), sizeof(T) * dim);
		memcpy(this, &a, sizeof(*this));
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator+(const Vector &a) const {
		Vector ret;
		for (int i = 0; i < dim; ++i) {
			ret.d[i] = this->d[i] + a.d[i];
		}
		return ret;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector operator+(const Vector &a) const {
		return Vector(_mm_add_ps(this->v, a.v));
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator-(const Vector &a) const {
		Vector ret;
		for (int i = 0; i < dim; ++i) {
			ret.d[i] = this->d[i] - a.d[i];
		}
		return ret;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector operator-(const Vector &a) const {
		return Vector(_mm_sub_ps(this->v, a.v));
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator-() const {
		Vector ret;
		for (int i = 0; i < dim; ++i) {
			ret.d[i] = -this->d[i];
		}
		return ret;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator-() const {
		return Vector(_mm_sub_ps(_mm_set_ps1(0.f), this->v));
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator*(const Vector &a) const {
		Vector ret;
		for (int i = 0; i < dim; ++i) {
			ret.d[i] = this->d[i] * a.d[i];
		}
		return ret;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector operator*(const Vector &a) const {
		return Vector(_mm_mul_ps(this->v, a.v));
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator*(real a) const {
		Vector ret;
		for (int i = 0; i < dim; ++i) {
			ret.d[i] = this->d[i] * a;
		}
		return ret;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator*(float32 a) const {
		return Vector(_mm_mul_ps(this->v, _mm_set_ps1(a)));
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator/(const Vector &a) const {
		Vector ret;
		for (int i = 0; i < dim; ++i) {
			ret.d[i] = this->d[i] / a.d[i];
		}
		return ret;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator/(const Vector &a) const {
		return Vector(_mm_div_ps(this->v, a.v));
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator/(real a) const {
		Vector ret;
		if (a == 0) {
			return ret;
		}
		for (int i = 0; i < dim; ++i) {
			ret.d[i] = this->d[i] / a;
		}
		return ret;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector operator/(float32 a) const {
		if(a == 0) return Vector(_mm_set_ps1(0.f));
		return Vector(_mm_div_ps(this->v, _mm_set_ps1(a)));
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector& operator+=(const Vector &a) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] += a.d[i];
		}
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector& operator+=(const Vector &a) {
		this->v = _mm_add_ps(this->v, a.v);
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector& operator-=(const Vector &a) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] -= a.d[i];
		}
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
		GYT_FORCE_INLINE Vector& operator-=(const Vector &a) {
		this->v = _mm_sub_ps(this->v, a.v);
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector& operator*=(const Vector &a) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] *= a.d[i];
		}
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector& operator*=(const Vector &a) {
		this->v = _mm_mul_ps(this->v, a.v);
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector& operator*=(real a) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] *= a;
		}
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector& operator*=(real a) {
		this->v = _mm_mul_ps(this->v, _mm_set_ps1(a));
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector& operator/=(const Vector &a) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] /= a.d[i];
		}
		return *this;
	}


	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector& operator/=(const Vector &a) {
		this->v = _mm_div_ps(this->v, a.v);
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector& operator/=(real a) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] /= a;
		}
		return *this;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector& operator/=(real a) {
		this->v = _mm_div_ps(this->v, _mm_set_ps1(a));
		return *this;
	}

	GYT_FORCE_INLINE bool operator==(const Vector &a) const {
		for (int i = 0; i < dim; ++i) {
			if (this->d[i] != a.d[i]) {
				return false;
			}
		}
		return true;
	}

	GYT_FORCE_INLINE bool operator!=(const Vector &a) const {
		for (int i = 0; i < dim; ++i) {
			if (this->d[i] != a.d[i]) {
				return true;
			}
		}
		return false;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE T Dot(const Vector &a) const {
		T ret = T(0);
		for (int i = 0; i < dim; ++i) {
			ret += this->d[i] * a.d[i];
		}
		return ret;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_> && dim__ == 3, int> = 0>
	GYT_FORCE_INLINE real Dot(const Vector &a) const {
		return _mm_cvtss_f32(_mm_dp_ps(this->v, a.v, 0x71));
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_> && dim__ == 4, int> = 0>
	GYT_FORCE_INLINE real Dot(const Vector &a) const {
		return _mm_cvtss_f32(_mm_dp_ps(this->v, a.v, 0xf1));
	}

	GYT_FORCE_INLINE real Length2() const {
		return this->Dot(*this);
	}

	GYT_FORCE_INLINE real Length() const {
		return std::sqrt(Length2());
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE Vector Norm() const {
		real invLen = (real)(1.0) / Length();
		Vector ret(*this);
		for (int i = 0; i < dim; ++i) {
			ret.d[i] *= invLen;
		}
		return ret;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector Norm() const {
		return Vector(_mm_div_ps(this->v, _mm_set_ps1(this->Length())));
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_NONE<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE void Normalize() {
		real invLen = (real)(1.0) / Length();
		for (int i = 0; i < dim; ++i) {
			this->d[i] *= invLen;
		}
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SSE_4_32F<dim__, T_, ISE_>, int> = 0 >
	GYT_FORCE_INLINE void Normalize() {
		this->v = _mm_div_ps(this->v, _mm_set_ps1(this->Length()));
	}

	GYT_FORCE_INLINE T MaxVal() const {
		T ret = this->d[0];
		for (int i = 1; i < dim; ++i) {
			ret = std::max(ret, this->d[i]);
		}
		return ret;
	}

	GYT_FORCE_INLINE T MinVal() const {
		T ret = this->d[0];
		for (int i = 1; i < dim; ++i) {
			ret = std::min(ret, this->d[i]);
		}
		return ret;
	}

	template<int dim__ = dim, typename T, typename std::enable_if_t<dim__ == 3, int> = 0>
	GYT_FORCE_INLINE Vector Cross(const Vector<dim__, T> &a) const {
		return Vector<dim__, T>(this->y * a.z - this->z * a.y, this->z * a.x - this->x * a.z, this->x * a.y - this->y * a.x);
	}

	GYT_FORCE_INLINE real Y()  const {
		const real YWeight[3] = { (real)0.212671, (real)0.715160, (real)0.072169f };
		return YWeight[0] * this->x + YWeight[1] * this->y + YWeight[2] * this->z;
	}

	template <int a,
		int b,
		int c,
		int d,
		int dim__ = dim,
		typename T_ = T,
		IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<!SIMD_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector Permute() const {
		return Vector(this->d[a], this->d[b], this->d[c], this->d[d]);
	}

	template <int a,
		int b,
		int c,
		int d,
		int dim__ = dim,
		typename T_ = T,
		IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector Permute() const {
		return Vector(_mm_permute_ps(this->v, _MM_SHUFFLE(a, b, c, d)));
	}

	template <int a, int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE>
	GYT_FORCE_INLINE Vector Broadcast() const {
		return Permute<a, a, a, a>();
	}
};

template<int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE std::ostream& operator<<(std::ostream &os, const Vector<dim, T, ISE> &a) {
	os << "[ ";
	for (int i = 0; i < dim; ++i) {
		if (i < dim - 1) {
			os << a.d[i] << ", ";
		}
		else {
			os << a.d[i] << " ";
		}

	}
	os << "]";
	return os;
}

template<int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE Vector<dim, T, ISE> operator*(real a, const Vector<dim, T, ISE> &b) {
	return b * a;
}


template<int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE real Dot(const Vector<dim, T, ISE> &a, const Vector<dim, T, ISE> &b) {
	return a.Dot(b);
}

template<typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE Vector<3, T, ISE> Cross(const Vector<3, T, ISE> &a, const Vector<3, T, ISE> &b) {
	return Vector<3, T, ISE>(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

template<int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE void Normalize(Vector<dim, T, ISE> &a) {
	a.Normalize();
}

template<int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE real Distance2(const Vector<dim, T, ISE> &a, const Vector<dim, T, ISE> &b) {
	return (b - a).Length2();
}

template<int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE real Distance(const Vector<dim, T, ISE> &a, const Vector<dim, T, ISE> &b) {
	return (b - a).Length();
}

template<int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE Vector<dim, T, ISE> Abs(const Vector<dim, T, ISE> &a) {
	Vector<dim, T, ISE> ret;
	for(int i = 0; i < dim; ++i) {
		ret.d[i] = std::abs(a.d[i]);
	}
	return ret;
}

template<int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE Vector<dim, T, ISE> Sqrt(const Vector<dim, T, ISE>& a) {
	Vector<dim, T, ISE> ret;
	for (int i = 0; i < dim; ++i) {
		ret.d[i] = std::sqrt(a.d[i]);
	}
	return ret;
}

template<int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE Vector<dim, T, ISE> Exp(const Vector<dim, T, ISE>& a) {
	Vector<dim, T, ISE> ret;
	for (int i = 0; i < dim; ++i) {
		ret.d[i] = std::exp(a.d[i]);
	}
	return ret;
}

using Vector2 = Vector<2, real, defaultInstructionSet>;
using Vector2i = Vector<2, int, defaultInstructionSet>;
using Vector2f = Vector<2, float, defaultInstructionSet>;
using Vector2d = Vector<2, double, defaultInstructionSet>;

using Vector3 = Vector<3, real, defaultInstructionSet>;
using Vector3i = Vector<3, int, defaultInstructionSet>;
using Vector3f = Vector<3, float, defaultInstructionSet>;
using Vector3d = Vector<3, double, defaultInstructionSet>;

using Vector4 = Vector<4, real, defaultInstructionSet>;
using Vector4i = Vector<4, int, defaultInstructionSet>;
using Vector4f = Vector<4, float, defaultInstructionSet>;
using Vector4d = Vector<4, double, defaultInstructionSet>;

using Vec3 = Vector3;
using Vec2 = Vector2;
using Vec2i = Vector2i;
using Vec2f = Vector2f;


// FMA: a * b + c
template <typename T>
GYT_FORCE_INLINE typename std::enable_if<T::simd && (defaultInstructionSet >= IntrinsicSet::AVX), T>::type
fused_mul_add(const T &a, const T &b, const T &c) {
	return T(_mm_fmadd_ps(a, b, c));
}

template <typename T>
GYT_FORCE_INLINE typename std::enable_if<!T::simd || (defaultInstructionSet < IntrinsicSet::AVX), T>::type
fused_mul_add(const T &a, const T &b, const T &c) {
	return a * b + c;
}


template<int dim_, typename T, IntrinsicSet ISE>
struct Matrix {
	static constexpr int dim = dim_;
	static constexpr TensorType tensorType = TensorType::MATRIX;
	using type = T;
	Vector<dim, T, ISE> d[dim];

	template<int dim__, typename T_, IntrinsicSet ISE_>
	static constexpr bool SIMD_4_32F =
		(dim__ == 3 || dim__ == 4) && std::is_same<T_, float32>::value &&
		(ISE_ >= IntrinsicSet::SSE);


	template <
		typename F,
		std::enable_if_t<std::is_convertible<
		F,
		std::function<Vector<dim_, T, ISE>(int)>>::value,
		int> = 0>
	GYT_FORCE_INLINE Matrix& Set(const F &f) {
		for (int i = 0; i < dim; i++)
			this->d[i] = f(i);
		return *this;
	}

	GYT_FORCE_INLINE Matrix() {
		for (int i = 0; i < dim; ++i) {
			d[i] = Vector<dim, T, ISE>();
			//d[i][i] = 1;
		}
	}

	GYT_FORCE_INLINE Matrix(const Matrix &m) {
		*this = m;
	}

	GYT_FORCE_INLINE Matrix(T a) {
		for (int i = 0; i < dim; ++i) {
			d[i][i] = a;
		}
	}

	GYT_FORCE_INLINE Matrix(const Vector<dim, T, ISE> &v) {
		for (int i = 0; i < dim; i++)
			this->d[i][i] = v[i];
	}

	template<int dim__ = dim, typename T_, 
		typename std::enable_if_t<dim__ == 2> = 0>
	GYT_FORCE_INLINE Matrix(const Vector<dim__, T_, ISE> &v0, const Vector<dim__, T_, ISE> &v1) {
		this->d[0] = v0;
		this->d[1] = v1;
	}

	template<int dim__ = dim, typename T_,
		typename std::enable_if_t<dim__ == 3> = 0>
	GYT_FORCE_INLINE Matrix(const Vector<dim__, T_, ISE> &v0, const Vector<dim__, T_, ISE> &v1, const Vector<dim__, T_, ISE> &v2) {
		this->d[0] = v0;
		this->d[1] = v1;
		this->d[2] = v2;
	}

	template<int dim__ = dim, typename T_,
		typename std::enable_if_t<dim__ == 4> = 0>
	GYT_FORCE_INLINE Matrix(const Vector<dim__, T_, ISE> &v0, const Vector<dim__, T_, ISE> &v1,
		const Vector<dim__, T_, ISE> &v2, const Vector<dim__, T_, ISE> &v3) {
		this->d[0] = v0;
		this->d[1] = v1;
		this->d[2] = v2;
		this->d[3] = v3;
	}

	template <typename F, 
		typename std::enable_if_t<
		std::is_convertible<F, std::function<Vector<dim, T, ISE>(int)>>::value,
		int> = 0>
	GYT_FORCE_INLINE Matrix(const F &f) {
		for (int i = 0; i < dim; i++)
			this->d[i] = f(i);
	}

	template<int dim__ = dim, typename std::enable_if_t<dim__ == 2, int> = 0>
	GYT_FORCE_INLINE Matrix(T m00, T m01,
						T m10, T m11) {
		d[0][0] = m00; d[1][0] = m01;
		d[0][1] = m10; d[1][1] = m11;
	}


	template<int dim__ = dim, typename std::enable_if_t<dim__ == 3, int> = 0>
	GYT_FORCE_INLINE Matrix(T m00, T m01, T m02,
						T m10, T m11, T m12,
						T m20, T m21, T m22) {
		d[0][0] = m00; d[1][0] = m01; d[2][0] = m02;
		d[0][1] = m10; d[1][1] = m11; d[2][1] = m12;
		d[0][2] = m20; d[1][2] = m21; d[2][2] = m22;
	}

	template<int dim__ = dim, typename std::enable_if_t<dim__ == 4, int> = 0>
	GYT_FORCE_INLINE Matrix(T m00, T m01, T m02, T m03,
						T m10, T m11, T m12, T m13,
						T m20, T m21, T m22, T m23,
						T m30, T m31, T m32, T m33) {
		d[0][0] = m00; d[1][0] = m01; d[2][0] = m02; d[3][0] = m03;
		d[0][1] = m10; d[1][1] = m11; d[2][1] = m12; d[3][1] = m13;
		d[0][2] = m20; d[1][2] = m21; d[2][2] = m22; d[3][2] = m23;
		d[0][3] = m30; d[1][3] = m31; d[2][3] = m32; d[3][3] = m33;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<dim__ == 2, int> = 0>
	GYT_FORCE_INLINE explicit Matrix(const Vector<dim, T, ISE> &v0, const Vector<dim, T, ISE>  &v1) {
		static_assert(dim == 2, "Matrix dim must be 2");
		this->d[0] = v0;
		this->d[1] = v1;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<dim__ == 3, int> = 0>
	GYT_FORCE_INLINE explicit Matrix(const Vector<dim, T, ISE> &v0, const Vector<dim, T, ISE> &v1, 
		const Vector<dim, T, ISE> &v2) {
		static_assert(dim == 3, "Matrix dim must be 3");
		this->d[0] = v0;
		this->d[1] = v1;
		this->d[2] = v2;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<dim__ == 4, int> = 0>
	GYT_FORCE_INLINE explicit Matrix(const Vector<dim, T, ISE> &v0, const Vector<dim, T, ISE> &v1,
		const Vector<dim, T, ISE> &v2, const Vector<dim, T, ISE> &v3) {
		static_assert(dim == 4, "Matrix dim must be 4");
		this->d[0] = v0;
		this->d[1] = v1;
		this->d[2] = v2;
		this->d[3] = v3;
	}

	GYT_FORCE_INLINE Matrix& operator=(const Matrix &m) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] = m.d[i];
		}
		return *this;
	}

	GYT_FORCE_INLINE Matrix& operator=(Matrix &&m) {
		for (int i = 0; i < dim; ++i) {
			this->d[i] = m.d[i];
		}
		return *this;
	}

	GYT_FORCE_INLINE const Vector<dim, T, ISE>& operator[](int i) const {
		return d[i];
	}

	GYT_FORCE_INLINE Vector<dim, T, ISE>& operator[](int i) {
		return d[i];
	}

	GYT_FORCE_INLINE const T& operator()(int i, int j) const {
		return d[j][i];
	}

	GYT_FORCE_INLINE T& operator()(int i, int j) {
		return d[j][i];
	}

	GYT_FORCE_INLINE Matrix operator+(const Matrix &a) const {
		return Matrix([=](int i) {return this->d[i] + a.d[i]; });
	}

	GYT_FORCE_INLINE Matrix& operator+=(const Matrix &a) {
		return this->Set([&](int i) {return this->d[i] + a[i]; });
	}

	GYT_FORCE_INLINE Matrix operator-(const Matrix &a) const {
		return Matrix([=](int i) {return this->d[i] - a.d[i]; });
	}

	GYT_FORCE_INLINE Matrix& operator-=(const Matrix &a) {
		return this->Set([&](int i) {return this->d[i] - a[i]; });
	}

	GYT_FORCE_INLINE bool operator==(const Matrix &a) const {
		for (int i = 0; i < dim; i++)
			for (int j = 0; j < dim; j++)
				if (d[i][j] != a[i][j])
					return false;
		return true;
	}

	GYT_FORCE_INLINE bool operator!=(const Matrix &o) const {
		for (int i = 0; i < dim; i++)
			for (int j = 0; j < dim; j++)
				if (d[i][j] != o[i][j])
					return true;
		return false;
	}

	template<int dim__ = dim, typename T_ = T, IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<!SIMD_4_32F<dim__, T_, ISE_>, int> = 0>
	GYT_FORCE_INLINE Vector<dim, T, ISE> operator*(const Vector<dim, T, ISE> &a) const {
		Vector<dim, T, ISE> ret(d[0] * a.d[0]);
		for (int i = 1; i < dim; ++i) {
			ret += d[i] * a.d[i];
		}
		return ret;
	}

	// Matrix3
	template <int dim__ = dim,
		typename T_ = T,
		IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_4_32F<dim__, T_, ISE_> && dim__ == 3,
		int> = 0>
	GYT_FORCE_INLINE Vector<dim, T, ISE> operator*(const Vector<dim, T, ISE> &a) const {
		Vector<dim, T, ISE> ret = a.template Broadcast<2>() * d[2];
		ret = fused_mul_add(d[1], a.template Broadcast<1>(), ret);
		ret = fused_mul_add(d[0], a.template Broadcast<0>(), ret);
		return ret;
	}

	// Matrix4
	template <int dim__ = dim,
		typename T_ = T,
		IntrinsicSet ISE_ = ISE,
		typename std::enable_if_t<SIMD_4_32F<dim__, T_, ISE_> && dim__ == 4,
		int> = 0>
	GYT_FORCE_INLINE Vector<dim, T, ISE> operator*(const Vector<dim, T, ISE> &a) const {
		Vector<dim, T, ISE> ret = a.template Broadcast<3>() * d[3];
		ret = fused_mul_add(d[2], a.template Broadcast<2>(), ret);
		ret = fused_mul_add(d[1], a.template Broadcast<1>(), ret);
		ret = fused_mul_add(d[0], a.template Broadcast<0>(), ret);
		return ret;
	}

	GYT_FORCE_INLINE Matrix operator*(const Matrix &a) const {
		return Matrix([&](int i) {return (*this) * a[i]; });
	}

	GYT_FORCE_INLINE Matrix operator*(real a) const {
		return Matrix([&](int i) {return (*this)[i] * a; });
	}

	GYT_FORCE_INLINE Matrix& operator*=(const Matrix &a) {
		Matrix mat([&](int i) {return (*this) * a[i]; });
		*this = std::move(mat);
		return *this;
	}

	GYT_FORCE_INLINE Matrix& operator*=(real a) {
		for (int i = 0; i < dim; ++i) {
			d[i] *= a;
		}
		return *this;
	}

	GYT_FORCE_INLINE Matrix operator/(real a) const {
		return Matrix([=](int i) {return (*this)[i] / a; });
	}

	GYT_FORCE_INLINE Matrix& operator/=(real a) {
		for (int i = 0; i < dim; ++i) {
			d[i] /= a;
		}
		return *this;
	}

	GYT_FORCE_INLINE T FrobeniusNorm2() const {
		T sum = d[0].Length2();
		for (int i = 1; i < dim; i++) {
			sum += d[i].Length2();
		}
		return sum;
	}

	GYT_FORCE_INLINE auto FrobeniusNorm() const {
		return std::sqrt(FrobeniusNorm2());
	}

	GYT_FORCE_INLINE Matrix Transpose() const {
		Matrix ret;
		for (int i = 0; i < dim; i++) {
			for (int j = 0; j < dim; j++) {
				ret[i][j] = d[j][i];
			}
		}
		return ret;
	}

	GYT_FORCE_INLINE static Matrix Identity() {
		return Matrix(1.f);
	}

	GYT_FORCE_INLINE static Matrix Zeros() {
		return Matrix();
	}

	GYT_FORCE_INLINE T MaxVal() const {
		T ret = d[0].MaxVal();
		for (int i = 1; i < dim; ++i) {
			ret = std::max(ret, d[i].MaxVal());
		}
		return ret;
	}

	GYT_FORCE_INLINE T MinVal() const {
		T ret = d[0].MinVal();
		for (int i = 1; i < dim; ++i) {
			ret = std::min(ret, d[i].MinVal());
		}
		return ret;
	}
};

template<int dim, typename T, IntrinsicSet ISE>
std::ostream& operator<<(std::ostream &os, const Matrix<dim, T, ISE> &mat) {
	os << "[ ";
	for (int i = 0; i < dim; ++i) {
		if (i != 0) {
			os << "  ";
		}
		for (int j = 0; j < dim; ++j) {
			os << mat(i, j);
			if (j < dim - 1) {
				os << ", ";
			}
			else {
				os << " ";
			}
			
		}
		if (i != dim - 1) {
			os << std::endl;
		}
	}
	os << "]";
	return os;
}

template<int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE Matrix<dim, T, ISE> operator*(real a, const Matrix<dim, T, ISE> & b) {
	return b * a;
}

template <typename T, IntrinsicSet ISE>
T Determinant(const Matrix<4, T, ISE> &m) {
	// This function is adopted from GLM
	/*
	================================================================================
	OpenGL Mathematics (GLM)
	--------------------------------------------------------------------------------
	GLM is licensed under The Happy Bunny License and MIT License

	================================================================================
	The Happy Bunny License (Modified MIT License)
	--------------------------------------------------------------------------------
	Copyright (c) 2005 - 2014 G-Truc Creation

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	Restrictions:
	 By making use of the Software for military purposes, you choose to make a
	 Bunny unhappy.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.

	================================================================================
	The MIT License
	--------------------------------------------------------------------------------
	Copyright (c) 2005 - 2014 G-Truc Creation

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
	 */

	T Coef00 = m[2][2] * m[3][3] - m[3][2] * m[2][3];
	T Coef02 = m[1][2] * m[3][3] - m[3][2] * m[1][3];
	T Coef03 = m[1][2] * m[2][3] - m[2][2] * m[1][3];

	T Coef04 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
	T Coef06 = m[1][1] * m[3][3] - m[3][1] * m[1][3];
	T Coef07 = m[1][1] * m[2][3] - m[2][1] * m[1][3];

	T Coef08 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
	T Coef10 = m[1][1] * m[3][2] - m[3][1] * m[1][2];
	T Coef11 = m[1][1] * m[2][2] - m[2][1] * m[1][2];

	T Coef12 = m[2][0] * m[3][3] - m[3][0] * m[2][3];
	T Coef14 = m[1][0] * m[3][3] - m[3][0] * m[1][3];
	T Coef15 = m[1][0] * m[2][3] - m[2][0] * m[1][3];

	T Coef16 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
	T Coef18 = m[1][0] * m[3][2] - m[3][0] * m[1][2];
	T Coef19 = m[1][0] * m[2][2] - m[2][0] * m[1][2];

	T Coef20 = m[2][0] * m[3][1] - m[3][0] * m[2][1];
	T Coef22 = m[1][0] * m[3][1] - m[3][0] * m[1][1];
	T Coef23 = m[1][0] * m[2][1] - m[2][0] * m[1][1];

	Vector<4, T, ISE> Fac0(Coef00, Coef00, Coef02, Coef03);
	Vector<4, T, ISE> Fac1(Coef04, Coef04, Coef06, Coef07);
	Vector<4, T, ISE> Fac2(Coef08, Coef08, Coef10, Coef11);
	Vector<4, T, ISE> Fac3(Coef12, Coef12, Coef14, Coef15);
	Vector<4, T, ISE> Fac4(Coef16, Coef16, Coef18, Coef19);
	Vector<4, T, ISE> Fac5(Coef20, Coef20, Coef22, Coef23);

	Vector<4, T, ISE> Vec0(m[1][0], m[0][0], m[0][0], m[0][0]);
	Vector<4, T, ISE> Vec1(m[1][1], m[0][1], m[0][1], m[0][1]);
	Vector<4, T, ISE> Vec2(m[1][2], m[0][2], m[0][2], m[0][2]);
	Vector<4, T, ISE> Vec3(m[1][3], m[0][3], m[0][3], m[0][3]);

	Vector<4, T, ISE> Inv0(Vec1 * Fac0 - Vec2 * Fac1 + Vec3 * Fac2);
	Vector<4, T, ISE> Inv1(Vec0 * Fac0 - Vec2 * Fac3 + Vec3 * Fac4);
	Vector<4, T, ISE> Inv2(Vec0 * Fac1 - Vec1 * Fac3 + Vec3 * Fac5);
	Vector<4, T, ISE> Inv3(Vec0 * Fac2 - Vec1 * Fac4 + Vec2 * Fac5);

	Vector<4, T, ISE> SignA(+1, -1, +1, -1);
	Vector<4, T, ISE> SignB(-1, +1, -1, +1);
	Matrix<4, T, ISE> Inverse(Inv0 * SignA, Inv1 * SignB, Inv2 * SignA,
		Inv3 * SignB);

	Vector<4, T, ISE> Row0(Inverse[0][0], Inverse[1][0], Inverse[2][0], Inverse[3][0]);

	Vector<4, T, ISE> Dot0(m[0] * Row0);
	T Dot1 = (Dot0.x + Dot0.y) + (Dot0.z + Dot0.w);

	return Dot1;
}

template<int dim, typename T, IntrinsicSet ISE>
Matrix<dim, T, ISE> Transpose(const Matrix<dim, T, ISE> &m) {
	return m.Transpose();
}

template <typename T, IntrinsicSet ISE>
Matrix<4, T, ISE> Inversed(const Matrix<4, T, ISE> &m) {
	// This function is copied from GLM
	/*
	================================================================================
	OpenGL Mathematics (GLM)
	--------------------------------------------------------------------------------
	GLM is licensed under The Happy Bunny License and MIT License

	================================================================================
	The Happy Bunny License (Modified MIT License)
	--------------------------------------------------------------------------------
	Copyright (c) 2005 - 2014 G-Truc Creation

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	Restrictions:
	 By making use of the Software for military purposes, you choose to make a
	 Bunny unhappy.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.

	================================================================================
	The MIT License
	--------------------------------------------------------------------------------
	Copyright (c) 2005 - 2014 G-Truc Creation

	Permission is hereby granted, free of charge, to any person obtaining a copy
	of this software and associated documentation files (the "Software"), to deal
	in the Software without restriction, including without limitation the rights
	to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
	copies of the Software, and to permit persons to whom the Software is
	furnished to do so, subject to the following conditions:

	The above copyright notice and this permission notice shall be included in
	all copies or substantial portions of the Software.

	THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
	IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
	FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
	AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
	LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
	OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
	THE SOFTWARE.
	 */

	T Coef00 = m[2][2] * m[3][3] - m[3][2] * m[2][3];
	T Coef02 = m[1][2] * m[3][3] - m[3][2] * m[1][3];
	T Coef03 = m[1][2] * m[2][3] - m[2][2] * m[1][3];

	T Coef04 = m[2][1] * m[3][3] - m[3][1] * m[2][3];
	T Coef06 = m[1][1] * m[3][3] - m[3][1] * m[1][3];
	T Coef07 = m[1][1] * m[2][3] - m[2][1] * m[1][3];

	T Coef08 = m[2][1] * m[3][2] - m[3][1] * m[2][2];
	T Coef10 = m[1][1] * m[3][2] - m[3][1] * m[1][2];
	T Coef11 = m[1][1] * m[2][2] - m[2][1] * m[1][2];

	T Coef12 = m[2][0] * m[3][3] - m[3][0] * m[2][3];
	T Coef14 = m[1][0] * m[3][3] - m[3][0] * m[1][3];
	T Coef15 = m[1][0] * m[2][3] - m[2][0] * m[1][3];

	T Coef16 = m[2][0] * m[3][2] - m[3][0] * m[2][2];
	T Coef18 = m[1][0] * m[3][2] - m[3][0] * m[1][2];
	T Coef19 = m[1][0] * m[2][2] - m[2][0] * m[1][2];

	T Coef20 = m[2][0] * m[3][1] - m[3][0] * m[2][1];
	T Coef22 = m[1][0] * m[3][1] - m[3][0] * m[1][1];
	T Coef23 = m[1][0] * m[2][1] - m[2][0] * m[1][1];


	Vector<4, T, ISE> Fac0(Coef00, Coef00, Coef02, Coef03);
	Vector<4, T, ISE> Fac1(Coef04, Coef04, Coef06, Coef07);
	Vector<4, T, ISE> Fac2(Coef08, Coef08, Coef10, Coef11);
	Vector<4, T, ISE> Fac3(Coef12, Coef12, Coef14, Coef15);
	Vector<4, T, ISE> Fac4(Coef16, Coef16, Coef18, Coef19);
	Vector<4, T, ISE> Fac5(Coef20, Coef20, Coef22, Coef23);

	Vector<4, T, ISE> Vec0(m[1][0], m[0][0], m[0][0], m[0][0]);
	Vector<4, T, ISE> Vec1(m[1][1], m[0][1], m[0][1], m[0][1]);
	Vector<4, T, ISE> Vec2(m[1][2], m[0][2], m[0][2], m[0][2]);
	Vector<4, T, ISE> Vec3(m[1][3], m[0][3], m[0][3], m[0][3]);

	Vector<4, T, ISE> Inv0(Vec1 * Fac0 - Vec2 * Fac1 + Vec3 * Fac2);
	Vector<4, T, ISE> Inv1(Vec0 * Fac0 - Vec2 * Fac3 + Vec3 * Fac4);
	Vector<4, T, ISE> Inv2(Vec0 * Fac1 - Vec1 * Fac3 + Vec3 * Fac5);
	Vector<4, T, ISE> Inv3(Vec0 * Fac2 - Vec1 * Fac4 + Vec2 * Fac5);

	Vector<4, T, ISE> SignA(+1, -1, +1, -1);
	Vector<4, T, ISE> SignB(-1, +1, -1, +1);
	Matrix<4, T, ISE> Inverse(Inv0 * SignA, Inv1 * SignB, Inv2 * SignA,
		Inv3 * SignB);

	Vector<4, T, ISE> Row0(Inverse[0][0], Inverse[1][0], Inverse[2][0], Inverse[3][0]);

	Vector<4, T, ISE> Dot0(m[0] * Row0);
	T Dot1 = (Dot0.x + Dot0.y) + (Dot0.z + Dot0.w);

	T OneOverDeterminant = static_cast<T>(1) / Dot1;

	return Inverse * OneOverDeterminant;
}


template <typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE Matrix<2, T, ISE> Inversed(const Matrix<2, T, ISE> &mat) {
	T det = Determinant(mat);
	return static_cast<T>(1) / det *
		Matrix<2, T, ISE>(Vector<2, T, ISE>(mat[1][1], -mat[0][1]),
			Vector<2, T, ISE>(-mat[1][0], mat[0][0]));
}

template <typename T, IntrinsicSet ISE>
Matrix<3, T, ISE> Inversed(const Matrix<3, T, ISE> &mat) {
	T det = Determinant(mat);
	return T(1.0) / det *
		Matrix<3, T, ISE>(
			Vector<3, T, ISE>(mat[1][1] * mat[2][2] - mat[2][1] * mat[1][2],
				mat[2][1] * mat[0][2] - mat[0][1] * mat[2][2],
				mat[0][1] * mat[1][2] - mat[1][1] * mat[0][2]),
			Vector<3, T, ISE>(mat[2][0] * mat[1][2] - mat[1][0] * mat[2][2],
				mat[0][0] * mat[2][2] - mat[2][0] * mat[0][2],
				mat[1][0] * mat[0][2] - mat[0][0] * mat[1][2]),
			Vector<3, T, ISE>(
				mat[1][0] * mat[2][1] - mat[2][0] * mat[1][1],
				mat[2][0] * mat[0][1] - mat[0][0] * mat[2][1],
				mat[0][0] * mat[1][1] - mat[1][0] * mat[0][1]));
}

template <int dim, typename T, IntrinsicSet ISE>
GYT_FORCE_INLINE Matrix<dim, T, ISE> Inverse(const Matrix<dim, T, ISE> &m) {
	return Inversed(m);
}

using Matrix2 = Matrix<2, real, defaultInstructionSet>;
using Matrix3 = Matrix<3, real, defaultInstructionSet>;
using Matrix4 = Matrix<4, real, defaultInstructionSet>;

using Matrix2f = Matrix<2, float32, defaultInstructionSet>;
using Matrix3f = Matrix<3, float32, defaultInstructionSet>;
using Matrix4f = Matrix<4, float32, defaultInstructionSet>;

using Matrix2d = Matrix<2, float64, defaultInstructionSet>;
using Matrix3d = Matrix<3, float64, defaultInstructionSet>;
using Matrix4d = Matrix<4, float64, defaultInstructionSet>;

namespace type
{
	template <typename>
	struct is_Vector : public std::false_type {};

	template <int N, typename T, IntrinsicSet ISE>
	struct is_Vector<Vector<N, T, ISE>> : public std::true_type {};

	template <typename>
	struct is_Matrix : public std::false_type {};

	template <int N, typename T, IntrinsicSet ISE>
	struct is_Matrix<Matrix<N, T, ISE>> : public std::true_type {};
}

GYT_NAMESPACE_END

// fmt (v9+) no longer formats types via their std::ostream operator<< implicitly.
// Opt Vector types in explicitly so fmt::print / GYT_Print can format them.
FMT_BEGIN_NAMESPACE
template <int dim, typename T, Gyunity::IntrinsicSet ISE>
struct formatter<Gyunity::Vector<dim, T, ISE>> : ostream_formatter {};
FMT_END_NAMESPACE