#pragma once
#include "Texture.h"

GYT_NAMESPACE_BEGIN

template<typename T>
class ConstantTexture : public Texture<T> {
public:
	ConstantTexture(T value) : mValue(value) {}

	T Sample(const Vec3 &coord) const override {
		return mValue;
	}

	T Sample(const Vec2 &coord) const override {
		return mValue;
	}

	T Sample(const Intersection &coord) const override {
		return mValue;
	}

private:
	T mValue;
};

GYT_NAMESPACE_END