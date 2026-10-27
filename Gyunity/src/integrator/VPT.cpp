#include "VPT.h"
#include "sampler/Sampler.h"
#include "light/AreaLight.h"
#include "light/PointLight.h"
#include <typeinfo>


GYT_NAMESPACE_BEGIN

Vec3 VolPathTracing::Li(const Ray& r, const Scene& scene, StateSequence& rand, MemoryPool& arena) const 
{
	Ray ray = r;
	bool specularBounce = false;
	Vec3 throughput = Vec3(1, 1, 1);
	Vec3 L;
	// Select one estimator for the entire scene, not conditionally per light.
	// Only these concrete light implementations provide finite point sampling.
	bool useEquiAngular = mUseEquiAngularSample && !scene.GetLights().empty();
	for (const auto& light : scene.GetLights()) {
		if (typeid(*light) != typeid(AreaLight) && typeid(*light) != typeid(PointLight)) {
			useEquiAngular = false;
			break;
		}
	}
	for (int i = 0; ; ++i) {
		Intersection isect;
		bool intersect = scene.Intersect(ray, &isect);
		MediumIntersection mi;

		Intersection eqLightPoint;
		real eqLightProb = 0;
		real eqLightPdf = 0;
		Vec3 eqLightLe;
		if (i >= mMaxDepth) {
			// No scattering remains, but attenuated emission is still visible.
			if (ray.mMedium) throughput *= ray.mMedium->Tr(ray, rand);
		}
		else if (useEquiAngular) {
			const Light* light = scene.SampleOneLight(&eqLightProb, rand());
			if (!light || !std::isfinite(eqLightProb) || eqLightProb <= 0) break;
			eqLightLe = light->SampleOnePoint(&eqLightPoint, &eqLightPdf, Vec2(rand(), rand()));
			if (!std::isfinite(eqLightPdf) || eqLightPdf <= 0) {
				GetDefaultLogger()->Error("Equi-angular light sampling requires a positive finite point PDF.", false);
				break;
			}
			if (ray.mMedium) throughput *= ray.mMedium->EquiAngularSampling(ray, rand, arena, eqLightPoint, &mi);

		}
		else {
			if (ray.mMedium) throughput *= ray.mMedium->Sample(ray, rand, arena, &mi);
		}

		if (throughput == Vec3()) break;

		if (mi.IsValid()) {
			if (useEquiAngular) {
				L += throughput * ConnectToLight(scene, rand, mi, eqLightPoint, eqLightLe, eqLightPdf) / eqLightProb;
				Vec3 wi;
				mi.mPhase->SampleP(-ray.mDir, &wi, Vec2(rand(), rand()));
				ray = mi.SpawnRay(wi);
				specularBounce = false;
				
				//L += throughput * DirectIllumination(scene, mi, rand(), Vec2(rand(), rand()), Vec3(rand(), rand(), rand()), rand, true);
				//Vec3 wi;
				//mi.phase->SampleP(-ray.d, &wi, Vec2(rand(), rand()));
				//ray = mi.SpawnRay(wi);
				//specularBounce = false;

			}
			else {
				//if (intersect) scene.QueryIntersectionInfo(ray, &isect);
				if (!scene.GetLights().empty())
					L += throughput * DirectIllumination(scene, mi, rand(), Vec2(rand(), rand()), Vec3(rand(), rand(), rand()), rand, true);
				Vec3 wi;
				mi.mPhase->SampleP(-ray.mDir, &wi, Vec2(rand(), rand()));
				ray = mi.SpawnRay(wi);
				specularBounce = false;
			}

		}
		else {
			if (!intersect) {
				if ((i == 0 || specularBounce) && scene.GetEnvironmentLight()) {
					L += throughput * scene.GetEnvironmentLight()->Emission(ray);
				}
				break;
			}
			scene.QueryIntersectionInfo(ray, &isect);
			isect.ComputeScatteringFunction(arena);
			BSDF* bsdf = isect.mBsdf;
			if ((i == 0 || specularBounce) && isect.mPrimitive->IsLight()) {
				const Light* emitter = isect.mPrimitive->GetLight();
				L += throughput * emitter->Emission(isect, isect.mOutDir);
			}
			if (!bsdf) {
				ray = isect.SpawnRay(ray.mDir);
				i--;
				continue;
			}

			if (i >= mMaxDepth) break;
			if (bsdf->HasNonDeltaLobes() && !scene.GetLights().empty()) {
				L += throughput * DirectIllumination(scene, isect, rand(), Vec2(rand(), rand()), Vec3(rand(), rand(), rand()), rand, true);

			}

			Vec3 wi;
			real pdf;
			ScatterEventType sampledType = ScatterEventType(0);
			Vec3 f = bsdf->Sample(-ray.mDir, &wi, &pdf, Vec3(rand(), rand(), rand()), BSDF_ALL, &sampledType);
			if (f == Vec3() || pdf == 0) break;
			Vec3 estimation = f * std::abs(Dot(isect.mNormal, wi)) / pdf;
			specularBounce = SurfaceEventClassifier::IsDelta(sampledType);

			throughput = throughput * estimation;
			ray = isect.SpawnRay(wi);
		}
		real survivalProb = std::min((real)1.0, (throughput).Y() / throughput.Y());
		if (survivalProb < 1 && i > 5) {
			if (rand() < survivalProb) {
				throughput = throughput / survivalProb;
			}
			else {
				break;
			}
		}
	}
	return L;
}


Vec3 VolPathTracing::ConnectToLight(const Scene &scene, StateSequence &rand,
	const Intersection& isect, const Intersection& lightPoint, const Vec3 &Le, real lightPdf) const {

	Vec3 L1, L2;
	real weight1, weight2 = 0;
	//Vec3 f;
	//real cosTheta = 1;
	//Vec3 dir = isect.hit - lightPoint.hit;
	//Vec3 wi = -dir.Norm();
	//if (lightPoint.n != Vec3()) {
	//	cosTheta = Dot(dir.Norm(), lightPoint.n);
	//	if (cosTheta < 0) return Vec3();
	//}
	//VisibilityTester visibilityTester(isect, lightPoint);
	//const MediumIntersection& mi = (const MediumIntersection&)isect;
	//real p = mi.phase->Evaluate(mi.wo, wi);
	//f = Vec3(p);
	//return Le * f * visibilityTester.Tr(scene, rand) * cosTheta / dir.Length2() / lightPdf;



	{
		Vec3 f;
		real cosTheta = 1;
		Vec3 dir = isect.mPos - lightPoint.mPos;
		Vec3 wi = -dir.Norm();
		if (lightPoint.mNormal != Vec3()) {
			cosTheta = Dot(dir.Norm(), lightPoint.mNormal);
			if (cosTheta < 0) return Vec3();
		}
		VisibilityTester visibilityTester(isect, lightPoint);
		const MediumIntersection& mi = (const MediumIntersection&)isect;
		real phase = mi.mPhase->Evaluate(mi.mOutDir, wi);
		f = Vec3(phase);
		real geomTerm = std::abs(cosTheta) / dir.Length2();
		real pdfPhase = phase * geomTerm;
		weight1 = (lightPoint.mNormal != Vec3()) ? PowerHeuristic(1, lightPdf, 1, pdfPhase) : 1;
		L1 = Le * f * visibilityTester.Tr(scene, rand) * cosTheta / dir.Length2() / lightPdf;
	}
	//{
	//	const MediumIntersection& mi = (const MediumIntersection&)isect;
	//	Vec3 wi;
	//	real p = mi.phase->SampleP(mi.wo, &wi, Vec2(rand(), rand()));
	//	real pdf = p;
	//	Vec3 f = Vec3(p);
	//	Ray shadowRay = isect.SpawnRay(wi);
	//	Intersection lightIt;
	//	Vec3 Tr;
	//	if (scene.IntersectTr(shadowRay, rand, &lightIt, &Tr)) {
	//		if (lightIt.primitive->IsLight()) {
	//			scene.QueryIntersectionInfo(shadowRay, &lightIt);
	//			lightPdf = lightIt.primitive->light->PdfLi(isect, wi);
	//			Vec3 dir = isect.hit - lightPoint.hit;
	//			real geomTerm = std::abs(Dot(dir.Norm(), lightIt.n)) / dir.Length2();
	//			real scatteringPdfA = pdf * geomTerm;
	//			weight2 = PowerHeuristic(1, scatteringPdfA, 1, lightPdf * geomTerm);
	//			L2 = lightIt.primitive->light->Emission(lightIt, -wi) * f * Tr / pdf;
	//		}
	//	}
	//}
	//return weight1 * L1 + weight2 * L2;
	return L1;
	//return L2;
}
GYT_NAMESPACE_END