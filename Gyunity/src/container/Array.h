#pragma once

#include "common/Core.h"
#include "math/Linalg.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <type_traits>
#include <vector>

GYT_NAMESPACE_BEGIN

namespace ArrayDetail {

inline bool Error(const char* message) {
	GetDefaultLogger()->Error(message, false);
	return false;
}

[[noreturn]] inline void InvalidAccess() {
	Error("Array access is out of bounds or uses an invalid array. Use TryAt for checked access.");
	std::abort();
}

template<typename... Types> struct AllIntegral : std::true_type {};
template<typename First, typename... Rest>
struct AllIntegral<First, Rest...> : std::integral_constant<bool,
	std::is_integral<First>::value && AllIntegral<Rest...>::value> {};

template<int rank, typename T>
class Slice {
public:
	Slice(T* data, const int* shape, const size_t* strides) : mData(data), mShape(shape), mStrides(strides) {}
	Slice<rank - 1, T> operator[](size_t index) const {
		if (index >= static_cast<size_t>(mShape[0]) || !mData) InvalidAccess();
		return Slice<rank - 1, T>(mData + index * mStrides[0], mShape + 1, mStrides + 1);
	}
private:
	T* mData;
	const int* mShape;
	const size_t* mStrides;
};

template<typename T>
class Slice<2, T> {
public:
	Slice(T* data, const int* shape, const size_t* strides) : mData(data), mShape(shape), mStrides(strides) {}
	T* operator[](size_t index) const {
		if (index >= static_cast<size_t>(mShape[0]) || mShape[1] == 0 || !mData) InvalidAccess();
		return mData + index * mStrides[0];
	}
private:
	T* mData;
	const int* mShape;
	const size_t* mStrides;
};

template<typename T>
class Slice<1, T> {
public:
	Slice(T* data, const int* shape, const size_t*) : mData(data), mSize(shape[0]) {}
	T& operator[](size_t index) const {
		if (index >= static_cast<size_t>(mSize) || !mData) InvalidAccess();
		return mData[index];
	}
private:
	T* mData;
	int mSize;
};
}

// Fixed rank, runtime extents, last axis contiguous. mRes remains a compatibility
// view of the extents; change dimensions through Resize/Assign, not by writing it.
template<int dim, typename T>
class ArrayND {
	static_assert(dim > 0, "Array rank must be positive.");
	static_assert(!std::is_same<T, bool>::value, "Use uint8_t for contiguous boolean storage.");
public:
	using Extents = std::array<int, dim>;
	using Index = Vector<dim, int>;
	using StrideArray = std::array<size_t, dim>;
	using iterator = typename std::vector<T>::iterator;
	using const_iterator = typename std::vector<T>::const_iterator;

	Index mRes = Index(0);

	ArrayND() = default;
	explicit ArrayND(const Extents& shape, const T& initial = T()) { mValid = Resize(shape, initial); }
	explicit ArrayND(const Index& shape, const T& initial = T()) { mValid = Resize(ToExtents(shape), initial); }

	template<typename... Dimensions, typename std::enable_if<
		sizeof...(Dimensions) == dim && ArrayDetail::AllIntegral<Dimensions...>::value, int>::type = 0>
	explicit ArrayND(Dimensions... dimensions) {
		Extents shape{};
		mValid = MakeExtents(shape, dimensions...) && Resize(shape);
	}

	ArrayND(const std::vector<T>& data, const Extents& shape) { mValid = Assign(data, shape); }
	ArrayND(const std::vector<T>& data, const Index& shape) { mValid = Assign(data, ToExtents(shape)); }
	template<typename... Dimensions, typename std::enable_if<
		sizeof...(Dimensions) == dim && ArrayDetail::AllIntegral<Dimensions...>::value, int>::type = 0>
	ArrayND(const std::vector<T>& data, Dimensions... dimensions) {
		Extents shape{};
		mValid = MakeExtents(shape, dimensions...) && Assign(data, shape);
	}

	ArrayND(const ArrayND&) = default;
	ArrayND& operator=(const ArrayND&) = default;
	ArrayND(ArrayND&& other) noexcept { MoveFrom(std::move(other)); }
	ArrayND& operator=(ArrayND&& other) noexcept {
		if (this != &other) MoveFrom(std::move(other));
		return *this;
	}

	bool Resize(const Extents& shape, const T& initial = T()) {
		size_t count;
		StrideArray strides{};
		if (!Layout(shape, count, strides)) return false;
		std::vector<T> replacement(count, initial);
		mData.swap(replacement);
		SetShape(shape, strides);
		return true;
	}
	bool Resize(const Index& shape, const T& initial = T()) { return Resize(ToExtents(shape), initial); }

	bool Assign(const std::vector<T>& data, const Extents& shape) {
		size_t count;
		StrideArray strides{};
		if (!Layout(shape, count, strides)) return false;
		if (count != data.size()) return ArrayDetail::Error("Array source size does not match its extents.");
		std::vector<T> replacement(data);
		mData.swap(replacement);
		SetShape(shape, strides);
		return true;
	}
	bool Assign(const std::vector<T>& data, const Index& shape) { return Assign(data, ToExtents(shape)); }

	bool Fill(const T& value) {
		if (!IsValid()) return ArrayDetail::Error("Cannot fill an invalid array.");
		std::fill(mData.begin(), mData.end(), value);
		return true;
	}
	void Clear() noexcept {
		mData.clear();
		mShape.fill(0);
		mStrides.fill(0);
		for (int i = 0; i < dim; ++i) mRes[i] = 0;
		mValid = true;
	}

	bool IsValid() const {
		if (!mValid) return false;
		for (int i = 0; i < dim; ++i) if (mRes[i] != mShape[i]) return false;
		return true;
	}
	size_t Size() const { return mData.size(); }
	bool Empty() const { return mData.empty(); }
	static constexpr int Rank() { return dim; }
	const Extents& GetShape() const { return mShape; }
	const StrideArray& GetStrides() const { return mStrides; }
	T* Data() { return IsValid() ? mData.data() : nullptr; }
	const T* Data() const { return IsValid() ? mData.data() : nullptr; }
	iterator begin() { return mData.begin(); }
	iterator end() { return mData.end(); }
	const_iterator begin() const { return mData.begin(); }
	const_iterator end() const { return mData.end(); }

	bool Contains(const Extents& index) const {
		if (!IsValid()) return false;
		for (int i = 0; i < dim; ++i) if (index[i] < 0 || index[i] >= mShape[i]) return false;
		return true;
	}
	bool Contains(const Index& index) const { return Contains(ToExtents(index)); }

	bool TryOffset(const Extents& index, size_t& offset) const {
		if (!Contains(index)) return ArrayDetail::Error("Array coordinate is out of bounds.");
		offset = Offset(index);
		return true;
	}
	bool TryCoordinates(size_t offset, Extents& index) const {
		if (!IsValid() || offset >= Size()) return ArrayDetail::Error("Array linear index is out of bounds.");
		for (int i = 0; i < dim; ++i) {
			index[i] = static_cast<int>(offset / mStrides[i]);
			offset %= mStrides[i];
		}
		return true;
	}
	bool TryAt(const Extents& index, T*& value) {
		value = nullptr;
		size_t offset;
		if (!TryOffset(index, offset)) return false;
		value = mData.data() + offset;
		return true;
	}
	bool TryAt(const Extents& index, const T*& value) const {
		value = nullptr;
		size_t offset;
		if (!TryOffset(index, offset)) return false;
		value = mData.data() + offset;
		return true;
	}

	T& operator()(const Extents& index) {
		if (!Contains(index)) ArrayDetail::InvalidAccess();
		return mData[Offset(index)];
	}
	const T& operator()(const Extents& index) const {
		if (!Contains(index)) ArrayDetail::InvalidAccess();
		return mData[Offset(index)];
	}
	T& operator()(const Index& index) { return (*this)(ToExtents(index)); }
	const T& operator()(const Index& index) const { return (*this)(ToExtents(index)); }
	T& operator[](const Index& index) { return (*this)(index); }
	const T& operator[](const Index& index) const { return (*this)(index); }

	template<typename... Coordinates, typename std::enable_if<
		sizeof...(Coordinates) == dim && ArrayDetail::AllIntegral<Coordinates...>::value, int>::type = 0>
	T& operator()(Coordinates... coordinates) {
		Extents index{};
		if (!MakeExtents(index, coordinates...)) ArrayDetail::InvalidAccess();
		return (*this)(index);
	}
	template<typename... Coordinates, typename std::enable_if<
		sizeof...(Coordinates) == dim && ArrayDetail::AllIntegral<Coordinates...>::value, int>::type = 0>
	const T& operator()(Coordinates... coordinates) const {
		Extents index{};
		if (!MakeExtents(index, coordinates...)) ArrayDetail::InvalidAccess();
		return (*this)(index);
	}

	auto operator[](size_t index) -> decltype(std::declval<ArrayDetail::Slice<dim,T>>()[index]) {
		if (!IsValid()) ArrayDetail::InvalidAccess();
		return ArrayDetail::Slice<dim,T>(mData.data(), mShape.data(), mStrides.data())[index];
	}
	auto operator[](size_t index) const -> decltype(std::declval<ArrayDetail::Slice<dim,const T>>()[index]) {
		if (!IsValid()) ArrayDetail::InvalidAccess();
		return ArrayDetail::Slice<dim,const T>(mData.data(), mShape.data(), mStrides.data())[index];
	}

	template<typename Function>
	bool ForEachIndex(Function function) const {
		if (!IsValid()) return ArrayDetail::Error("Cannot iterate an invalid array.");
		Extents index{};
		for (size_t n = 0; n < Size(); ++n) {
			function(static_cast<const Extents&>(index));
			for (int axis = dim - 1; axis >= 0; --axis) {
				if (++index[axis] < mShape[axis]) break;
				index[axis] = 0;
			}
		}
		return true;
	}

private:
	Extents mShape{};
	StrideArray mStrides{};
	std::vector<T> mData;
	bool mValid = true;

	static Extents ToExtents(const Index& index) {
		Extents result{};
		for (int i = 0; i < dim; ++i) result[i] = index[i];
		return result;
	}
	template<typename Integer>
	static bool Extent(Integer value, int& result) {
		if ((std::is_signed<Integer>::value && value < 0) ||
			static_cast<uintmax_t>(value) > static_cast<uintmax_t>(std::numeric_limits<int>::max()))
			return ArrayDetail::Error("Array extent/index is outside the supported integer range.");
		result = static_cast<int>(value);
		return true;
	}
	template<typename... Values>
	static bool MakeExtents(Extents& result, Values... values) {
		bool valid = true;
		int axis = 0;
		using Expand = int[];
		(void)Expand{0, (valid = Extent(values, result[axis++]) && valid, 0)...};
		return valid;
	}
	bool Layout(const Extents& shape, size_t& count, StrideArray& strides) const {
		bool empty = false;
		for (int value : shape) {
			if (value < 0) return ArrayDetail::Error("Array extents cannot be negative.");
			empty = empty || value == 0;
		}
		if (empty) { count = 0; strides.fill(0); return true; }
		count = 1;
		for (int i = dim - 1; i >= 0; --i) {
			strides[i] = count;
			if (count > mData.max_size() / static_cast<size_t>(shape[i]))
				return ArrayDetail::Error("Array extent product exceeds the supported storage range.");
			count *= static_cast<size_t>(shape[i]);
		}
		return true;
	}
	void SetShape(const Extents& shape, const StrideArray& strides) {
		mShape = shape;
		mStrides = strides;
		for (int i = 0; i < dim; ++i) mRes[i] = shape[i];
		mValid = true;
	}
	size_t Offset(const Extents& index) const {
		size_t result = 0;
		for (int i = 0; i < dim; ++i) result += static_cast<size_t>(index[i]) * mStrides[i];
		return result;
	}
	void MoveFrom(ArrayND&& other) noexcept {
		mRes = other.mRes;
		mShape = other.mShape;
		mStrides = other.mStrides;
		mValid = other.mValid;
		mData = std::move(other.mData);
		other.Clear();
	}
};

template<typename T> using Array1D = ArrayND<1, T>;
template<typename T> using Array2D = ArrayND<2, T>;
template<typename T> using Array3D = ArrayND<3, T>;

GYT_NAMESPACE_END
