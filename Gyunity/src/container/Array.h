#pragma once

#include "common/Core.h"

GYT_NAMESPACE_BEGIN


template<int dim, typename T>
class ArrayND {

};

template<typename T>
class ArrayND<2, T> {
public:
	ArrayND() {}
	ArrayND(const Vec2i &res) : mRes(res) {}
	ArrayND(int row, int col) : mData(row* col) {
		mRes[0] = row;
		mRes[1] = col;
	}
	ArrayND(const std::vector<T>& data, int row, int col) : mData(data) {
		mRes[0] = row;
		mRes[1] = col;
	}
	T& operator()(int x, int y) {
		return mData[x * mRes[1] + y];
	}
	const T* operator[](int i) const {
		return &mData[0] + i * mRes[1];
	}
	T* operator[](int i) {
		return &mData[0] + i * mRes[1];
	}
	const T& operator[](Vec2i i) const {
		return mData[i.x * mRes[1] + i.y];
	}
	T& operator[](Vec2i i) {
		return mData[i.x * mRes[1] + i.y];
	}
public:
	Vec2i mRes;
private:
	std::vector<T> mData;
};

template <typename T>
using Array2D = ArrayND<2, T>;

GYT_NAMESPACE_END