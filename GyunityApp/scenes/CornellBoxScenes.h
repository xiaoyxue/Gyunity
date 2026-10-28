#pragma once
#include "SceneBuilder.h"
#include "volume/Homogeneous.h"
#include <array>

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

inline void RoughGlassLucyAndPurpleBunny(SceneBuilder& builder) {
	AddCornellBoxWalls(builder);
	auto roughGlass = builder.RoughDielectric(Vec3(1), Vec3(1), 0.02, 0.02, 1.5);
	const Transform lucyTransform =
		Transform::Translate(Vec3(-0.4, 0, 0)) *
		Transform::Translate(Vec3(0, -1, 0)) *
		Transform::RotateY(75) *
		Transform::Scale(0.0017, 0.0017, 0.0017);
	builder.AddMesh("meshs\\Alucy.obj", roughGlass, lucyTransform);

	const Vec3 purple(0.9999f, 0.75f, 0.9999f);
	auto purpleGlass = builder.Glass(purple, purple);
	const Transform bunnyTransform =
		Transform::Translate(Vec3(0.52, -0.92, 0.3)) * Transform::Scale(0.22, 0.22, 0.22);
	builder.AddMesh("meshs\\bunny3.obj", purpleGlass, bunnyTransform);
	AddCornellBoxLight(builder, 85);
}

// SmallVCM's Cornell box (scene.hxx, LoadCornellBox) with its glossy floor, in its z-up
// coordinates. Triangles keep SmallVCM's vertex order; normals are cross(b - a, c - a),
// facing into the box. A ceiling light uses SmallVCM's emitting light box.
inline std::array<Vec3, 8> SmallVcmCorners() {
	return { Vec3(-1.27029f, 1.30455f, -1.28002f), Vec3(1.28975f, 1.30455f, -1.28002f),
		Vec3(1.28975f, 1.30455f, 1.28002f), Vec3(-1.27029f, 1.30455f, 1.28002f),
		Vec3(-1.27029f, -1.25549f, -1.28002f), Vec3(1.28975f, -1.25549f, -1.28002f),
		Vec3(1.28975f, -1.25549f, 1.28002f), Vec3(-1.27029f, -1.25549f, 1.28002f) };
}

inline void AddSmallVcmBox(SceneBuilder& builder, bool lightBox) {
	const auto cb = SmallVcmCorners();
	auto triangle = [&](const Vec3& a, const Vec3& b, const Vec3& c, const std::shared_ptr<Material>& material) {
		builder.AddTriangle(a, b, c, Cross(b - a, c - a).Norm(), material);
	};
	auto glossy = builder.Phong(Vec3(0.1f), Vec3(0.7f), 90);
	auto green = builder.Diffuse(Vec3(0.156863f, 0.803922f, 0.172549f));
	auto red = builder.Diffuse(Vec3(0.803922f, 0.152941f, 0.152941f));
	auto white = builder.Diffuse(Vec3(0.803922f));
	auto blue = builder.Diffuse(Vec3(0.156863f, 0.172549f, 0.803922f));
	triangle(cb[0], cb[4], cb[5], glossy);  // floor
	triangle(cb[5], cb[1], cb[0], glossy);
	triangle(cb[0], cb[1], cb[2], blue);    // back wall
	triangle(cb[2], cb[3], cb[0], blue);
	triangle(cb[2], cb[6], cb[7], white);   // ceiling
	triangle(cb[7], cb[3], cb[2], white);
	triangle(cb[3], cb[7], cb[4], green);   // left wall
	triangle(cb[4], cb[0], cb[3], green);
	triangle(cb[1], cb[5], cb[6], red);     // right wall
	triangle(cb[6], cb[2], cb[1], red);
	if (!lightBox) return;
	const Vec3 lb[8] = {
		Vec3(-0.25f, 0.25f, 1.26002f), Vec3(0.25f, 0.25f, 1.26002f), Vec3(0.25f, 0.25f, 1.28002f),
		Vec3(-0.25f, 0.25f, 1.28002f), Vec3(-0.25f, -0.25f, 1.26002f), Vec3(0.25f, -0.25f, 1.26002f),
		Vec3(0.25f, -0.25f, 1.28002f), Vec3(-0.25f, -0.25f, 1.28002f) };
	triangle(lb[0], lb[2], lb[1], white);
	triangle(lb[2], lb[0], lb[3], white);
	triangle(lb[3], lb[4], lb[7], white);
	triangle(lb[4], lb[3], lb[0], white);
	triangle(lb[1], lb[6], lb[5], white);
	triangle(lb[6], lb[1], lb[2], white);
	triangle(lb[4], lb[5], lb[6], white);
	triangle(lb[6], lb[7], lb[4], white);
	// The light box's floor emits radiance 25.03 downwards.
	const Vec3 emission(25.03329895614464f);
	builder.AddTriangleLight(lb[0], lb[5], lb[4], Cross(lb[5] - lb[0], lb[4] - lb[0]).Norm(), emission);
	builder.AddTriangleLight(lb[5], lb[0], lb[1], Cross(lb[0] - lb[5], lb[1] - lb[5]).Norm(), emission);
}

// SmallVCM scene 1: glossy floor, large mirror sphere, ceiling (area) light.
inline void SmallVcmMirrorSphereCeiling(SceneBuilder& builder) {
	AddSmallVcmBox(builder, true);
	const auto cb = SmallVcmCorners();
	const real radius = 0.8f;
	const Vec3 center = (cb[0] + cb[1] + cb[4] + cb[5]) * (1.f / 4.f) + Vec3(0, 0, radius);
	builder.AddSphere(center, radius, builder.Mirror(Vec3(1)));
}

// SmallVCM scene 2: glossy floor, small mirror and glass spheres, point light.
inline void SmallVcmSpheresPoint(SceneBuilder& builder) {
	AddSmallVcmBox(builder, false);
	const auto cb = SmallVcmCorners();
	const real radius = 0.5f;
	const Vec3 leftWallCenter = (cb[0] + cb[4]) * (1.f / 2.f) + Vec3(0, 0, radius);
	const Vec3 rightWallCenter = (cb[1] + cb[5]) * (1.f / 2.f) + Vec3(0, 0, radius);
	const real xlen = rightWallCenter.x - leftWallCenter.x;
	builder.AddSphere(leftWallCenter + Vec3(2.f * xlen / 7.f, 0, 0), radius, builder.Mirror(Vec3(1)));
	builder.AddSphere(rightWallCenter - Vec3(2.f * xlen / 7.f, 0, 0), radius, builder.Glass(Vec3(1), Vec3(1), 1.f, 1.6f));
	builder.AddPointLight(Vec3(0, -0.5f, 1), Vec3(70 * INV_PI * 0.25f));
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
