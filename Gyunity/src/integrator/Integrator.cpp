#include "Integrator.h"
#include "visual/Medium.h"
#include "visual/Visibility.h"
#include "visual/Scene.h"
#include "bsdf/BSDF.h"

GYT_NAMESPACE_BEGIN

Vec3 Integrator::DirectIllumination(const Scene& scene, const Intersection& isect, 
	real uLight, const Vec2& u, const Vec3& v, StateSequence& rand, bool handleMedia) 
{
	Vec3 L;
	//Sample light
	real lightSamplingPdf;
	real weight1 = 1, weight2 = 1;
	Vec3 L1, L2;
	Light* light = scene.SampleOneLight(&lightSamplingPdf, uLight);
	{
		Vec3 wi;
		Intersection lightPoint;
		real pdf;
		Vec3 Li = light->Sample_Li(isect, &wi, &pdf, &lightPoint, u);
		real scatteringPdf;
		Vec3 f;
		real cosTheta = 1;
		if (pdf > 0 && Li != Vec3(0)) {
			VisibilityTester visibilityTester(isect, lightPoint);
			if (isect.IsSurfaceScatter()) {
				BSDF* bsdf = isect.mBsdf;
				scatteringPdf = bsdf->Pdf(isect.mOutDir, wi);
				f = bsdf->Evaluate(isect.mOutDir, wi);
				cosTheta = std::abs(isect.mNormal.Dot(wi));
			}
			else {
				const MediumIntersection& mi = (const MediumIntersection&)isect;
				real p = mi.mPhase->p(mi.mOutDir, wi);
				f = Vec3(p);
				scatteringPdf = p;
			}
			
			if (f != Vec3()) {
				Vec3 Tr(1.f);
				if (handleMedia) {
					Tr = visibilityTester.Tr(scene, rand);
				}
				else {
					if (!visibilityTester.Unoccluded(scene))
						Li = Vec3();
				}
				if (light->IsDeltaLight())
					weight1 = 1;
				else
					weight1 = PowerHeuristic(1, pdf, 1, scatteringPdf);
				L1 = Li * Tr * f * cosTheta / pdf;
			}
		}

	}
	//Sample BSDF
	if (!light->IsDeltaLight()) {
		Vec3 wi;
		real pdf;
		Vec3 f;
		real cosTheta = 1;
		if (isect.IsSurfaceScatter()) {
			BSDF* bsdf = isect.mBsdf;
			f = bsdf->Sample(isect.mOutDir, &wi, &pdf, v);
			cosTheta = std::abs(isect.mNormal.Dot(wi));
		}
		else {
			const MediumIntersection& mi = (const MediumIntersection&)isect;
			real p = mi.mPhase->Sample_p(mi.mOutDir, &wi, Vec2(v[0], v[1]));
			f = Vec3(p);
			pdf = p;
		}

		Intersection intersection;

		if (pdf != 0 && f != Vec3()) {
			real lightPdf = light->Pdf_Li(isect, wi);
			if (lightPdf == 0) {
				weight2 = 0;
			}
			else {
				weight2 = PowerHeuristic(1, pdf, 1, lightPdf);
				Ray ray = isect.SpawnRay(wi);
				Vec3 Tr(1.f);
				bool surfaceIntersection = handleMedia ? scene.IntersectTr(ray, rand, &intersection, &Tr) : scene.Intersect(ray, &intersection);
				if (surfaceIntersection) {
					if (intersection.mPrimitive->IsLight() && intersection.mPrimitive->GetLight() == light) {
						//std::cout << lightPdf << std::endl;
						scene.QueryIntersectionInfo(ray, &intersection);
						const Light* emissionShape = intersection.mPrimitive->GetLight();
						L2 = emissionShape->Emission(intersection, -wi) * f * Tr * cosTheta / pdf;
					}
				}
				else if (scene.GetEnvironmentLight() == light) {
					L2 = light->Emission(wi) * f * Tr * cosTheta / pdf;
				}
			}
		}
		
	}
	//return L1 / lightSamplingPdf;
	//return L2 / lightSamplingPdf;
	return (L1 * weight1 + L2 * weight2) / lightSamplingPdf;
}

GYT_NAMESPACE_END