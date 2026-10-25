#pragma once
#include "SceneBuilder.h"

namespace BdptRoomScenes {

inline void AddBlock(SceneBuilder& builder, const Vec3& lower, const Vec3& upper,
	const std::shared_ptr<Material>& material) {
	const real x0 = lower.x, y0 = lower.y, z0 = lower.z;
	const real x1 = upper.x, y1 = upper.y, z1 = upper.z;
	builder.AddQuad(Vec3(x0, y0, z0), Vec3(x0, y0, z1), Vec3(x0, y1, z1), Vec3(x0, y1, z0), Vec3(-1, 0, 0), material);
	builder.AddQuad(Vec3(x1, y0, z1), Vec3(x1, y0, z0), Vec3(x1, y1, z0), Vec3(x1, y1, z1), Vec3(1, 0, 0), material);
	builder.AddQuad(Vec3(x0, y0, z0), Vec3(x1, y0, z0), Vec3(x1, y0, z1), Vec3(x0, y0, z1), Vec3(0, -1, 0), material);
	builder.AddQuad(Vec3(x0, y1, z1), Vec3(x1, y1, z1), Vec3(x1, y1, z0), Vec3(x0, y1, z0), Vec3(0, 1, 0), material);
	builder.AddQuad(Vec3(x1, y0, z0), Vec3(x0, y0, z0), Vec3(x0, y1, z0), Vec3(x1, y1, z0), Vec3(0, 0, -1), material);
	builder.AddQuad(Vec3(x0, y0, z1), Vec3(x1, y0, z1), Vec3(x1, y1, z1), Vec3(x0, y1, z1), Vec3(0, 0, 1), material);
}

inline void Build(SceneBuilder& builder, bool glossy) {
	auto white = builder.Diffuse(Vec3(0.72f));
	auto floor = builder.Diffuse(Vec3(0.6f));
	auto wood = builder.Diffuse(Vec3(0.48f, 0.3f, 0.14f));
	auto shelf = builder.Diffuse(Vec3(0.25f, 0.32f, 0.38f));
	builder.AddQuad(Vec3(-2, 0, 2), Vec3(2, 0, 2), Vec3(2, 0, -2), Vec3(-2, 0, -2), Vec3(0, 1, 0), floor);
	builder.AddQuad(Vec3(-4, 0, 2), Vec3(-2, 0, 2), Vec3(-2, 0, -2), Vec3(-4, 0, -2),
		Vec3(0, 1, 0), builder.Diffuse(Vec3(0.8f)));
	builder.AddQuad(Vec3(-4, 0, -2), Vec3(2, 0, -2), Vec3(2, 3, -2), Vec3(-4, 3, -2), Vec3(0, 0, 1), white);
	builder.AddQuad(Vec3(-4, 3, -2), Vec3(2, 3, -2), Vec3(2, 3, 2), Vec3(-4, 3, 2), Vec3(0, -1, 0), white);
	builder.AddQuad(Vec3(2, 0, 2), Vec3(2, 3, 2), Vec3(2, 3, -2), Vec3(2, 0, -2),
		Vec3(-1, 0, 0), builder.Diffuse(Vec3(0.38f, 0.5f, 0.68f)));
	builder.AddQuad(Vec3(-4, 0, -2), Vec3(-4, 3, -2), Vec3(-4, 3, 2), Vec3(-4, 0, 2), Vec3(1, 0, 0), white);
	builder.AddQuad(Vec3(-4, 0, 2), Vec3(-4, 3, 2), Vec3(-2, 3, 2), Vec3(-2, 0, 2), Vec3(0, 0, -1), white);

	// A thick partition leaves a finite doorway; the chamber baffle hides the emitter.
	AddBlock(builder, Vec3(-2.08f, 0, -2), Vec3(-1.92f, 3, -0.8f), white);
	AddBlock(builder, Vec3(-2.08f, 0, 0.6f), Vec3(-1.92f, 3, 2), white);
	AddBlock(builder, Vec3(-2.08f, 0, -0.8f), Vec3(-1.92f, 0.1f, 0.6f), white);
	AddBlock(builder, Vec3(-2.08f, 2.15f, -0.8f), Vec3(-1.92f, 3, 0.6f), white);
	AddBlock(builder, Vec3(-2.75f, 1.25f, -2), Vec3(-2.65f, 3, 2), white);

	AddBlock(builder, Vec3(-0.45f, 0.82f, 0.1f), Vec3(0.65f, 0.98f, 0.9f), wood);
	for (real x : { -0.4f, 0.5f }) {
		for (real z : { 0.15f, 0.75f })
			AddBlock(builder, Vec3(x, 0, z), Vec3(x + 0.1f, 0.82f, z + 0.1f), wood);
	}
	for (real z : { -1.6f, 0.7f })
		AddBlock(builder, Vec3(1.55f, 0, z), Vec3(1.9f, 1.75f, z + 0.1f), shelf);
	for (int i = 0; i < 3; ++i) {
		const real y = 0.35f + 0.5f * i;
		AddBlock(builder, Vec3(1.55f, y, -1.5f), Vec3(1.9f, y + 0.08f, 0.7f), shelf);
	}
	AddBlock(builder, Vec3(0.7f, 0, 0.6f), Vec3(1.3f, 0.45f, 1.2f), white);

	auto panel = glossy ? builder.RoughConductor(Vec3(1), 0.18f, 0.18f, Vec3(0.2f), Vec3(3))
		: builder.Diffuse(Vec3(0.5f));
	builder.AddQuad(Vec3(-0.1f, 0.7f, -1.5f), Vec3(1, 0.7f, -1.2f), Vec3(1, 1.9f, -1.2f), Vec3(-0.1f, 1.9f, -1.5f),
		Vec3(-0.3f, 0, 1.1f).Norm(), panel);
	auto ball = glossy ? builder.RoughConductor(Vec3(1), 0.08f, 0.08f, Vec3(0.2f), Vec3(3))
		: builder.Diffuse(Vec3(0.2f, 0.55f, 0.3f));
	builder.AddSphere(Vec3(-0.1f, 1.18f, 0.5f), 0.2f, ball);
	builder.AddSphere(Vec3(1, 0.73f, 0.9f), 0.28f, builder.Diffuse(Vec3(0.65f, 0.25f, 0.12f)));
	builder.AddQuadLight(Vec3(-3.8f, 2.85f, -1.6f), Vec3(-3, 2.85f, -1.6f),
		Vec3(-3, 2.85f, -0.7f), Vec3(-3.8f, 2.85f, -0.7f), Vec3(0, -1, 0), Vec3(45));
}

inline void Diffuse(SceneBuilder& builder) { Build(builder, false); }
inline void Glossy(SceneBuilder& builder) { Build(builder, true); }

}
