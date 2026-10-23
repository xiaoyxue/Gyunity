#include "Scene.h"

GYT_NAMESPACE_BEGIN

void Scene::QueryIntersectionInfo(const Ray& ray, Intersection* isect) const {
	mPrimitives[isect->mPrimId]->QueryIntersectionInfo(ray, isect);
}

void Scene::GetBoundingSphere(Vec3* center, real* radius) const {
	mAccelerator->WorldBound().GetBoundingSphere(center, radius);
}


bool Scene::IntersectTr(Ray& ray, StateSequence& rand, Intersection* isect, Vec3* Tr) const {
    *Tr = Vec3(1.f);
    while (true) {
        bool hitSurface = Intersect(ray, isect);
        // Accumulate beam transmittance for ray segment
        if (ray.mMedium) *Tr *= ray.mMedium->Tr(ray, rand);

        // Initialize next ray segment or terminate transmittance computation
        if (!hitSurface) return false;
        if (isect->mPrimitive->GetMaterial() != nullptr) return true;
        ray = isect->SpawnRay(ray.mDir);
    }
}

GYT_NAMESPACE_END