#pragma once
#include "common/Core.h"
#include "visual/Scene.h"
#include "shape/Sphere.h"
#include "shape/Triangle.h"
#include "material/Diffuse.h"
#include "material/Mirror.h"
#include "material/Glass.h"
#include "material/Roughness.h"
#include "material/Plastic.h"
#include "material/RoughDielectric.h"
#include "light/AreaLight.h"
#include "light/EnvLight.h"
#include "texture/ConstantTexture.h"
#include "texture/ImageTexture.h"
#include <string>

using namespace Gyunity;

// Builds a scene with one call per object. Objects are added to the scene in
// call order, which matters: the BVH and kd-tree builds depend on it.
class SceneBuilder {
public:
	explicit SceneBuilder(const std::shared_ptr<Scene>& scene) : mScene(scene) {}

	// Paths in scene descriptions are relative to the resources folder.
	static std::string Resource(const std::string& path) { return "..\\resources\\" + path; }

	static std::shared_ptr<Texture<Vec3>> ConstantColor(const Vec3& value) {
		return std::make_shared<ConstantTexture<Vec3>>(value);
	}
	static std::shared_ptr<Texture<real>> ConstantValue(real value) {
		return std::make_shared<ConstantTexture<real>>(value);
	}

	static std::shared_ptr<Texture<Vec3>> ImageColor(const std::string& path) {
		return std::make_shared<ImageTexture<Vec3>>(Resource(path));
	}

	static std::shared_ptr<Material> Diffuse(const std::shared_ptr<Texture<Vec3>>& reflectance) {
		return std::make_shared<DiffuseMaterial>(reflectance);
	}
	static std::shared_ptr<Material> Diffuse(const Vec3& kd) {
		return Diffuse(ConstantColor(kd));
	}
	static std::shared_ptr<Material> Mirror(const Vec3& kr) {
		return std::make_shared<MirrorMaterial>(ConstantColor(kr));
	}
	static std::shared_ptr<Material> Glass(const Vec3& kr, const Vec3& kt, real etaOutside = 1.f, real etaInside = 1.5f) {
		return std::make_shared<GlassMaterial>(ConstantColor(kr), ConstantColor(kt), etaOutside, etaInside);
	}
	static std::shared_ptr<Material> RoughConductor(const Vec3& reflectance, real roughnessX, real roughnessY, const Vec3& eta, const Vec3& k) {
		return std::make_shared<RoughnessMaterial>(ConstantColor(reflectance), ConstantValue(roughnessX), ConstantValue(roughnessY),
			ConstantColor(eta), ConstantColor(k));
	}
	static std::shared_ptr<Material> RoughDielectric(const Vec3& kr, const Vec3& kt, real roughnessX, real roughnessY, real ior) {
		return std::make_shared<RoughDielectricMaterial>(ConstantColor(kr), ConstantColor(kt), ConstantValue(roughnessX),
			ConstantValue(roughnessY), ConstantValue(ior));
	}
	static std::shared_ptr<Material> Plastic(const Vec3& kd, const Vec3& ks, real roughness) {
		return std::make_shared<PlasticMaterial>(ConstantColor(kd), ConstantColor(ks), ConstantValue(roughness));
	}

	// MediumInterface is non-owning; its media must outlive the scene.
	void AddTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& normal,
		const std::shared_ptr<Material>& material, MediumInterface medium = MediumInterface()) {
		mScene->AddPrimitive(std::make_shared<Primitive>(MakeTriangle(a, b, c, normal), material, nullptr, medium));
	}

	// Two triangles: (a, b, c) and (a, c, d).
	void AddQuad(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Vec3& normal,
		const std::shared_ptr<Material>& material, MediumInterface medium = MediumInterface()) {
		AddTriangle(a, b, c, normal, material, medium);
		AddTriangle(a, c, d, normal, material, medium);
	}

	// An emitting quad: triangles (a, b, c) and (a, c, d), each with its own area light.
	void AddQuadLight(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& d, const Vec3& normal, const Vec3& emission) {
		std::shared_ptr<Shape> shape0 = MakeTriangle(a, b, c, normal);
		std::shared_ptr<Shape> shape1 = MakeTriangle(a, c, d, normal);
		std::shared_ptr<Light> light0 = std::make_shared<AreaLight>(shape0, emission);
		std::shared_ptr<Light> light1 = std::make_shared<AreaLight>(shape1, emission);
		std::shared_ptr<Material> black = Diffuse(Vec3(0, 0, 0));
		mScene->AddLight(light0);
		mScene->AddLight(light1);
		mScene->AddPrimitive(std::make_shared<Primitive>(shape0, black, light0));
		mScene->AddPrimitive(std::make_shared<Primitive>(shape1, black, light1));
	}

	// inside: the sphere is seen from inside (e.g. a giant sphere used as a wall).
	void AddSphere(const Vec3& center, real radius, const std::shared_ptr<Material>& material, bool inside = false) {
		mScene->AddPrimitive(std::make_shared<Primitive>(MakeSphere(center, radius, inside), material));
	}

	void AddSphere(const Vec3& center, real radius, const std::shared_ptr<Material>& material,
		const Transform& objectToWorld, bool inside = false) {
		auto shape = std::make_shared<Sphere>(objectToWorld, Inverse(objectToWorld), radius, center, inside);
		mScene->AddPrimitive(std::make_shared<Primitive>(shape, material));
	}

	void AddSphereLight(const Vec3& center, real radius, const Vec3& emission) {
		std::shared_ptr<Shape> shape = MakeSphere(center, radius, false);
		std::shared_ptr<Light> light = std::make_shared<AreaLight>(shape, emission);
		mScene->AddLight(light);
		mScene->AddPrimitive(std::make_shared<Primitive>(shape, Diffuse(Vec3(0, 0, 0)), light));
	}

	// path is relative to the resources folder.
	void AddMesh(const std::string& path, const std::shared_ptr<Material>& material, const Transform& transform) {
		Mesh mesh;
		mesh.LoadFromFile(Resource(path));
		mesh.SetMaterial(material);
		mScene->AddMesh(mesh, transform);
	}

	// path is relative to the resources folder.
	void AddEnvironmentLight(const std::string& path, real rotate = 0) {
		mScene->AddLight(std::make_shared<EnvironmentLight>(Resource(path), rotate));
	}

private:
	static std::shared_ptr<Shape> MakeTriangle(const Vec3& a, const Vec3& b, const Vec3& c, const Vec3& normal) {
		return std::make_shared<Triangle>(Transform(), Transform(), a, b, c, normal);
	}
	static std::shared_ptr<Shape> MakeSphere(const Vec3& center, real radius, bool inside) {
		return std::make_shared<Sphere>(Transform(), Transform(), radius, center, inside);
	}

	std::shared_ptr<Scene> mScene;
};

// The Cornell box spanning [-1, 1]^3: red left wall, blue right wall, white back
// wall, floor and ceiling. Triangle vertex orders match the original scenes, so
// renders stay byte-identical.
inline void AddCornellBoxWalls(SceneBuilder& b, bool includeBackWall = true) {
	const Vec3 p0(-1, -1, 1), p1(-1, -1, -1), p2(-1, 1, -1), p3(-1, 1, 1);
	const Vec3 p4(1, -1, 1), p5(1, -1, -1), p6(1, 1, -1), p7(1, 1, 1);
	std::shared_ptr<Material> red = SceneBuilder::Diffuse(Vec3(.75f, .25f, .25f));
	std::shared_ptr<Material> blue = SceneBuilder::Diffuse(Vec3(.25f, .25f, .75f));
	std::shared_ptr<Material> white = SceneBuilder::Diffuse(Vec3(.75f, .75f, .75f));
	// Preserve the left wall's cyclic vertex order, not just its diagonal.
	b.AddTriangle(p0, p1, p2, Vec3(1, 0, 0), red);
	b.AddTriangle(p2, p3, p0, Vec3(1, 0, 0), red);
	b.AddQuad(p4, p5, p6, p7, Vec3(-1, 0, 0), blue);
	if (includeBackWall) {
		b.AddQuad(p1, p5, p6, p2, Vec3(0, 0, 1), white);
	}
	b.AddQuad(p0, p4, p5, p1, Vec3(0, 1, 0), white);    // floor
	b.AddQuad(p3, p7, p6, p2, Vec3(0, -1, 0), white);   // ceiling
}

inline void AddCornellSphereWalls(SceneBuilder& builder) {
	auto red = builder.Diffuse(Vec3(.75f, .25f, .25f));
	auto blue = builder.Diffuse(Vec3(.25f, .25f, .75f));
	auto white = builder.Diffuse(Vec3(.75f, .75f, .75f));
	// Giant inward-facing spheres form the left, right, back, floor and ceiling.
	builder.AddSphere(Vec3(1e5f + 1, 40.8f, 81.6f), 1e5f, red, true);
	builder.AddSphere(Vec3(-1e5f + 99, 40.8f, 81.6f), 1e5f, blue, true);
	builder.AddSphere(Vec3(50, 40.8f, 1e5), 1e5f, white, true);
	builder.AddSphere(Vec3(50, 1e5, 81.6f), 1e5f, white, true);
	builder.AddSphere(Vec3(50, -1e5f + 81.6f, 81.6f), 1e5f, white, true);
}

inline void AddCornellBoxLight(SceneBuilder& b, real power = 85, real halfSize = 0.25f) {
	b.AddQuadLight(Vec3(halfSize, 0.965, halfSize), Vec3(halfSize, 0.965, -halfSize),
		Vec3(-halfSize, 0.965, -halfSize), Vec3(-halfSize, 0.965, halfSize),
		Vec3(0, -1, 0), Vec3(0.3f, 0.3f, 0.3f) * power);
}

struct CornellBoxOptions {
	bool includeBackWall = true;
	bool includeLight = true;
	real lightPower = 85;
	real lightHalfSize = 0.25f;
};

// Use the separate wall/light helpers when scene objects must precede the light.
inline void AddCornellBox(SceneBuilder& b, const CornellBoxOptions& options = CornellBoxOptions()) {
	AddCornellBoxWalls(b, options.includeBackWall);
	if (options.includeLight) {
		AddCornellBoxLight(b, options.lightPower, options.lightHalfSize);
	}
}
