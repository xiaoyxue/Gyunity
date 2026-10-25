#pragma once

#include "camera/Camera.h"
#include "shape/Shape.h"
#include "shape/Triangle.h"
#include "light/Light.h"
#include "Sampling.h"
#include "accelerator/Accelerator.h"
#include "common/DebugUtils.h"
#include "mesh/Mesh.h"


GYT_NAMESPACE_BEGIN

class Scene
{
private:
	std::vector<std::shared_ptr<Primitive>> mPrimitives;
	std::vector<std::shared_ptr<Light>> mLights;
	std::shared_ptr<Light> mEnvLight = nullptr;
	std::unique_ptr<Distribution1D> mLightPowerDistribution;
	std::shared_ptr<Accelerator> mAccelerator;
	int64 mShapeNum;
	int64 mPrimitiveNum;
public:
	void Initialize() {
		for (auto light : mLights) {
			light->Initialize(*this);
		}
		mLightPowerDistribution = ComputeLightPowerDistribution();
		mAccelerator->SetPrimitives(mPrimitives);
		mShapeNum = 0;
	}

	void SetAccelerator(const std::shared_ptr<Accelerator>& accelerator) {
		mAccelerator = accelerator;
	}

	void AddPrimitive(std::shared_ptr<Shape> shape, std::shared_ptr<Material> material, MediumInterface mi = MediumInterface()) {
		shape->mShapeId = mShapeNum;
		std::shared_ptr<Primitive> primitive =
			std::make_shared<GeometryPrimitive>(shape, material, nullptr, mi);
		primitive->mPrimId = mPrimitiveNum;
		mPrimitives.push_back(primitive);
		++mShapeNum;
		++mPrimitiveNum;
	}

	void AddPrimitive(std::shared_ptr<Primitive> primitive) {
		Shape* shape = primitive->GetShape();
		if (shape) {
			shape->mShapeId = mShapeNum;
			++mShapeNum;
		}
		primitive->mPrimId = mPrimitiveNum;
		mPrimitives.push_back(primitive);
		++mPrimitiveNum;
	}

	void AddLight(std::shared_ptr<Light> light) {
		mLights.push_back(light);
		if (light->IsEnvironmentLight()) {
			mEnvLight = light;
		}
	}


	void AddMesh(Mesh &mesh, const Transform &transform) {
		for (int i = 0; i < mesh.mUntransformedTriangles.size(); ++i) {
			Triangle* triangle = new Triangle();
			//*triangle = mesh.untransformedTriangles[i];
			auto pTriangle = std::make_shared<Triangle>(mesh.mUntransformedTriangles[i]);
			pTriangle->SetTransform(transform);
			std::shared_ptr<Shape> shape = std::shared_ptr<Shape>(pTriangle);
			AddPrimitive(shape, mesh.mMaterial, mesh.mMediumInterface);
		}
	}

	const Light* GetEnvironmentLight() const {
		return mEnvLight.get();
	}

	bool IsInitialized() const {
		return mAccelerator && (mLights.empty() || mLightPowerDistribution);
	}

	bool Intersect(const Ray& r, Intersection* isect) const {
		return mAccelerator->Intersect(r, isect);
	}

	bool Intersect(const Ray& r) const {
		return mAccelerator->Intersect(r);
	}

	bool IntersectTr(Ray& ray, StateSequence& rand, Intersection* isect, Vec3* Tr) const;

	const std::vector<std::shared_ptr<Light>>& GetLights() const {
		return mLights;
	}

	Light* SampleOneLight(real *lightPdf, real u) const {
		int nLights = (int)(mLights.size());
		if (nLights == 0) {
			*lightPdf = 0;
			return nullptr;
		}
		int lightNum;
		if (mLightPowerDistribution != nullptr) {
			lightNum = mLightPowerDistribution->SampleDiscrete(u, lightPdf);
			if (lightNum < 0 || *lightPdf == 0) return nullptr;
		}
		else {
			lightNum = std::min((int)(u * nLights), nLights - 1);
			*lightPdf = 1.f / nLights;
		}
		return mLights[lightNum].get();
	}

	// A PMF for a registered light slot, not a solid-angle or area density.
	bool LightSelectionPdf(size_t lightIndex, real* pdf) const {
		if (lightIndex >= mLights.size()) {
			*pdf = 0;
			GetDefaultLogger()->Error("Light index is outside the scene.", false);
			return false;
		}
		if (mLightPowerDistribution) {
			return mLightPowerDistribution->DiscretePdf(static_cast<int>(lightIndex), pdf);
		}
		*pdf = 1.f / static_cast<real>(mLights.size());
		return true;
	}

	const std::vector<std::shared_ptr<Primitive>>& GetPrimitives() const {
		return mPrimitives;
	}

	void QueryIntersectionInfo(const Ray& ray, Intersection* isect) const;

	void GetBoundingSphere(Vec3* center, real* radius) const;

private:

	std::unique_ptr<Distribution1D> ComputeLightPowerDistribution() {
		if (mLights.size() == 0) return nullptr;
		std::vector<real> lightPower;
		for (const auto &light : mLights)
			lightPower.push_back(light->Power().Y());
		return std::unique_ptr<Distribution1D>(
			new Distribution1D(&lightPower[0], (int)lightPower.size()));
	}
};

GYT_NAMESPACE_END