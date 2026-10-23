#pragma once

#include "Shape.h"

GYT_NAMESPACE_BEGIN

class HeartSurface : public Shape {
public:
	HeartSurface(){
		Initialize();
	}

	HeartSurface(const Transform& ObjectToWorld, const Transform& WorldToObject) : Shape(ObjectToWorld, WorldToObject){
		Initialize();
	}

	bool Intersect(const Ray& r, Intersection* isect, real* t) const override;

	bool Intersect(const Ray& r) const override;

	Intersection Sample(real* pdf, const Vec2& rand) const override {
		return Intersection();
	}

	Vec3 GetNorm(const Vec3& point) const override {
		return Gradient(point);
	}

	AABB ObjectBound() const override {
		return mBounding;
	}

	void QueryIntersectionInfo(const Ray& ray, Intersection* isct) const override;

	real Area() const override {
		return 1;
	}
private:

	AABB mBounding;

	void Initialize() {
		real val0[2] = { 1.5, -1.5 };
		real val1[2] = { 1.5, -1.5 };
		for (int i = 0; i < 2; ++i) {
			for (int j = 0; j < 2; ++j) {
				for (int k = 0; k < 2; ++k) {
					mBounding = Union(mBounding, Vec3(val0[i], val0[j], val1[k]));
				}
			}
		}
		std::cout << mBounding << std::endl;
	}

	real F(const Vec3 &p) const {
		real scale = 1.f;
		real x = p.x * scale, y = p.y * scale, z = p.z * scale;
		real t = x * x + y * y + (real)(9) / 4 * z * z - 1;
		return t * t * t - x * x * y * y * y - (real)(9) / 80 * y * y * y * z * z;
	}

	Vec3 Gradient(const Vec3& p) const {
		real scale = 1.f;
		real x = p.x * scale, y = p.y * scale, z = p.z * scale;
		real x2 = x * x, y2 = y * y, z2 = z * z;
		real t = x * x + y * y + (real)(9) / 4 * z * z - 1;
		real t2 = t * t;
		real dF_dx = 3 * t2 * 2 * x - 2 * x * y2 * y;
		real dF_dy = 3 * t2 * 2 * y - 3 * x2 * y2 - (real)(27) / 80 * y2 * z2;
		real dF_dz = 3 * t2 * (real)(9) / 2 * z - (real)(9) / 40 * y2 * y * z;
		Vec3 gradient(dF_dx, dF_dy, dF_dz);
		return gradient.Norm();
	}

	Vec3 Gradient2(const Vec3& p) const {
		real delta = 0.00005;
		real x = (F(Vec3(p.x + delta, p.y, p.z)) - F(Vec3(p.x - delta, p.y, p.z))) / (2.f * delta);
		real y = (F(Vec3(p.x, p.y + delta, p.z)) - F(Vec3(p.x, p.y - delta, p.z))) / (2.f * delta);
		real z = (F(Vec3(p.x, p.y, p.z + delta)) - F(Vec3(p.x, p.y, p.z - delta))) / (2.f * delta);
		return Vec3(x, y, z).Norm();
	}

	bool BinarySearch(real left, real right, const Ray& ray, real *t) const;
};

GYT_NAMESPACE_END