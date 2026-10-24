#pragma once
#include "SceneBuilder.h"
#include "volume/Homogeneous.h"

// Parallel versions of CornellBox.h scenes; the original classes remain available.
namespace CornellBoxScenes {

inline void GlassBunny(SceneBuilder& builder) {
	AddCornellBoxWalls(builder);
	auto glass = builder.Glass(Vec3(1, 1, 1), Vec3(1, 1, 1));
	const Transform bunnyTransform =
		Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(0.28, 0.28, 0.28);
	builder.AddMesh("meshs\\bunny3.obj", glass, bunnyTransform);
	AddCornellBoxLight(builder, 85);
}

inline void Slope(SceneBuilder& builder) {
	AddCornellBoxWalls(builder);
	auto white = builder.Diffuse(Vec3(.75f, .75f, .75f));
	const Vec3 backLeft(-1, 0, -1);
	const Vec3 backRight(1, 0, -1);
	const Vec3 frontRight(1, -1, 1);
	const Vec3 frontLeft(-1, -1, 1);
	const Vec3 slopeNormal = Cross(backLeft - backRight, frontRight - backRight).Norm();
	builder.AddQuad(backLeft, backRight, frontRight, frontLeft, slopeNormal, white);
	AddCornellBoxLight(builder, 85);
}

inline void SphereBox(SceneBuilder& builder) {
	AddCornellSphereWalls(builder);
	auto fullWhite = builder.Diffuse(Vec3(1, 1, 1));
	auto glass = builder.Glass(Vec3(1, 1, 1), Vec3(1, 1, 1));

	builder.AddSphere(Vec3(23, 0.0f, 98), 9.5f, fullWhite);
	builder.AddSphere(Vec3(73, 26.5f, 78), 16.5f, glass);
	builder.AddSphere(Vec3(27, 16.5f, 47), 16.5f, glass);
	builder.AddSphere(Vec3(53, 9.5f, 88), 9.5f, glass);
	builder.AddSphereLight(Vec3(50, 81.6f - 16.5f, 81.6f), 8.f, Vec3(0.3f, 0.3f, 0.3f) * 100);
}

inline void Environment(SceneBuilder& builder) {
	auto floor = builder.Diffuse(Vec3(.40f, .40f, .40f));
	builder.AddQuad(Vec3(-1, -1, 1), Vec3(1, -1, 1), Vec3(1, -1, -1), Vec3(-1, -1, -1),
		Vec3(0, 1, 0), floor);

	const Vec3 reflectance(1, 1, 1);
	const Vec3 refractiveIndex(1.5, 1.5, 1.5);
	const Vec3 extinction(1.5, 1.5, 1.5);
	auto roughConductor = builder.RoughConductor(reflectance, 0.02, 0.02, refractiveIndex, extinction);
	const Transform bunnyTransform =
		Transform::Translate(Vec3(0.52, -0.92, 0.3)) * Transform::Scale(0.22, 0.22, 0.22);
	builder.AddMesh("meshs\\bunny3.obj", roughConductor, bunnyTransform);

	auto roughGlass = builder.RoughDielectric(Vec3(1.0, 1.0, 1.0), Vec3(1.0, 1.0, 1.0), 0.02, 0.02, 1.5);
	// Keep the original multiplication order; combining transforms changes rounding.
	const Transform lucyTransform =
		Transform::Translate(Vec3(-0.4, 0, 0)) *
		Transform::Translate(Vec3(0, -1, 0)) *
		Transform::RotateY(15) *
		Transform::RotateY(60) *
		Transform::Scale(0.0019, 0.0019, 0.0019);
	builder.AddMesh("meshs\\Alucy.obj", roughGlass, lucyTransform);
	builder.AddEnvironmentLight("media\\pictures\\uffizi-large.hdr", 0);
}

inline void TriangleBox(SceneBuilder& builder) {
	AddCornellSphereWalls(builder);
	const Transform triangleTransform = Transform::Translate(Vec3(27, 16.5, 47)) * Transform::RotateY(30);
	const Vec3 top = triangleTransform(Vec3(10, 20, 0));
	const Vec3 bottomLeft = triangleTransform(Vec3(-10, 0, 0));
	const Vec3 bottomRight = triangleTransform(Vec3(10, 0, 0));
	const Vec3 normal = Cross(bottomLeft - top, bottomRight - top).Norm();
	builder.AddTriangle(top, bottomLeft, bottomRight, normal, builder.Diffuse(Vec3(.25f, .25f, .75f)));
	builder.AddSphere(Vec3(23, 0.0f, 98), 9.5f, builder.Diffuse(Vec3(1, 1, 1)));
	builder.AddSphereLight(Vec3(50, 81.6f - 16.5f, 81.6f), 8.f, Vec3(0.3f, 0.3f, 0.3f) * 100);
}

inline void TexturedSpheres(SceneBuilder& builder) {
	AddCornellBoxWalls(builder);
	builder.AddSphere(Vec3(0.5f, -0.6f, 0.3f), 0.35f, builder.Glass(Vec3(1, 1, 1), Vec3(1, 1, 1)));
	auto mappedDiffuse = builder.Diffuse(builder.ImageColor("texture_images\\sphere_mapping_8k.jpg"));
	const Transform sphereTransform =
		Transform::Translate(Vec3(-0.5f, -0.6f, -0.3f)) * Transform::RotateY(220) * Transform::Scale(1, 1, 1);
	builder.AddSphere(Vec3(0, 0, 0), 0.35f, mappedDiffuse, sphereTransform);
	AddCornellBoxLight(builder, 100);
}

inline void Empty(SceneBuilder& builder) {
	AddCornellBox(builder);
}

inline void DiffuseBunny(SceneBuilder& builder) {
	AddCornellBoxWalls(builder);
	const Transform bunnyTransform =
		Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(0.28, 0.28, 0.28);
	builder.AddMesh("meshs\\bunny3.obj", builder.Diffuse(Vec3(.5f, .5f, .5f)), bunnyTransform);
	AddCornellBoxLight(builder, 85);
}

inline void Water(SceneBuilder& builder) {
	AddCornellBoxWalls(builder);
	auto water = builder.Glass(Vec3(1, 1, 1), Vec3(1, 1, 1), 1.f, 1.33);
	const Transform waterTransform =
		Transform::Translate(Vec3(0, -0.35, 0)) *
		Transform::Scale(0.02, 0.02, 0.02) *
		Transform::Translate(Vec3(-50, 0, 50)) *
		Transform::RotateX(-90) *
		Transform::Scale(1, 2, 1);
	builder.AddMesh("meshs\\water.obj", water, waterTransform);

	const Transform lucyTransform =
		Transform::Translate(Vec3(-0.3, 0, -0.2)) *
		Transform::Translate(Vec3(0, -1, 0)) *
		Transform::RotateY(70) *
		Transform::Scale(0.0017, 0.0017, 0.0017);
	builder.AddMesh("meshs\\Alucy.obj", builder.Diffuse(Vec3(.5f, .5f, .5f)), lucyTransform);
	AddCornellBoxLight(builder, 350, 0.15f);
}

inline void FogBunny(SceneBuilder& builder) {
	AddCornellBoxWalls(builder);
	// MediumInterface borrows its pointers, so this immutable medium outlives every scene.
	static const HomogeneousMedium fog(Vec3(0, 0, 0), Vec3(1, 1, 1), 0);
	builder.AddQuad(Vec3(-1, -1, 1), Vec3(1, -1, 1), Vec3(1, 1, 1), Vec3(-1, 1, 1),
		Vec3(0, 0, 1), nullptr, MediumInterface(&fog, nullptr));
	const Transform bunnyTransform =
		Transform::Translate(Vec3(0, -0.5, 0)) * Transform::Scale(0.28, 0.28, 0.28);
	builder.AddMesh("meshs\\bunny3.obj", builder.Diffuse(Vec3(.5f, .5f, .5f)), bunnyTransform);
	AddCornellBoxLight(builder, 85);
}

}
