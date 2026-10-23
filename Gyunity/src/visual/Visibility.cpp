#include "Visibility.h"
#include "Scene.h"
#include "Medium.h"

GYT_NAMESPACE_BEGIN

bool VisibilityTester::Unoccluded(const Scene& scene) const {
	Ray ray = mP0.SpawnTo(mP1);
	return !scene.Intersect(ray);
}


Vec3 VisibilityTester::Tr(const Scene& scene, StateSequence& rand) const {
    Ray ray(mP0.SpawnTo(mP1));
    Vec3 Tr(1.f);
    while (true) {
        Intersection isect;

        bool hitSurface = scene.Intersect(ray, &isect);
        if (hitSurface && isect.mPrimitive->GetMaterial() != nullptr)
            return Vec3();

        //DEBUG_PIXEL_IF(ThreadIndex()) {
        //    std::cout << "Hit: "<< isect.hit 
        //        << ", Tr: " << Tr << std::endl;
        //}

        if (ray.mMedium) Tr *= ray.mMedium->Tr(ray, rand);

        if (!hitSurface) break;
        scene.QueryIntersectionInfo(ray, &isect);
        ray = isect.SpawnTo(mP1);
    }
    return Tr;
}

GYT_NAMESPACE_END