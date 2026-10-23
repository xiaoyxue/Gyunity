#include "Ray.h"

GYT_NAMESPACE_BEGIN

std::ostream& operator<<(std::ostream &os, const Ray &ray) {
	os << "ray: " << "orig: " << ray.mOrig << " dir: " << ray.mDir << " tMax: " << ray.mTmax;
	return os;
}

GYT_NAMESPACE_END