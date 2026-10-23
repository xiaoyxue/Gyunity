#include "Ray.h"

GYT_NAMESPACE_BEGIN

std::ostream& operator<<(std::ostream &os, const Ray &ray) {
	os << "ray: " << "orig: " << ray.mOrig << " dir: " << ray.mDir << " tMin: " << ray.mTMin << " tMax: " << ray.mTMax;
	return os;
}

GYT_NAMESPACE_END