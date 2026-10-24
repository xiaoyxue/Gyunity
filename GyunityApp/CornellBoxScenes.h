#pragma once
#include "SceneBuilder.h"

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
	auto red = builder.Diffuse(Vec3(.75f, .25f, .25f));
	auto blue = builder.Diffuse(Vec3(.25f, .25f, .75f));
	auto white = builder.Diffuse(Vec3(.75f, .75f, .75f));
	auto fullWhite = builder.Diffuse(Vec3(1, 1, 1));
	auto glass = builder.Glass(Vec3(1, 1, 1), Vec3(1, 1, 1));

	// Giant inward-facing spheres form the left, right, back, floor and ceiling.
	builder.AddSphere(Vec3(1e5f + 1, 40.8f, 81.6f), 1e5f, red, true);
	builder.AddSphere(Vec3(-1e5f + 99, 40.8f, 81.6f), 1e5f, blue, true);
	builder.AddSphere(Vec3(50, 40.8f, 1e5), 1e5f, white, true);
	builder.AddSphere(Vec3(50, 1e5, 81.6f), 1e5f, white, true);
	builder.AddSphere(Vec3(50, -1e5f + 81.6f, 81.6f), 1e5f, white, true);

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

}
