#pragma once
#include "SceneBuilder.h"

namespace MisScenes {

inline void Glossy(SceneBuilder& builder) {
	builder.AddQuad(Vec3(-4, 0, 3), Vec3(4, 0, 3), Vec3(4, 0, -3.2f), Vec3(-4, 0, -3.2f),
		Vec3(0, 1, 0), builder.Diffuse(Vec3(0.08f)));
	builder.AddQuad(Vec3(-4, 0, -3.2f), Vec3(4, 0, -3.2f), Vec3(4, 4.2f, -3.2f), Vec3(-4, 4.2f, -3.2f),
		Vec3(0, 0, 1), builder.Diffuse(Vec3(0.015f)));

	const Vec3 camera(0, 4, 8);
	const Vec3 lightRow(0, 2.6f, -2.3f);
	const real roughness[] = { 0.025f, 0.07f, 0.18f, 0.4f };
	for (int i = 0; i < 4; ++i) {
		const Vec3 center(0, 0.4f, -1.35f + 0.9f * i);
		// Aim each strip's specular reflection at the emitter row from the default camera.
		const Vec3 normal = ((camera - center).Norm() + (lightRow - center).Norm()).Norm();
		const Vec3 halfWidth(3.1f, 0, 0);
		const Vec3 halfDepth = Vec3(0, -normal.z, normal.y) * 0.34f;
		auto material = builder.RoughConductor(Vec3(1), roughness[i], roughness[i], Vec3(0.2f), Vec3(3));
		builder.AddQuad(center - halfWidth - halfDepth, center - halfWidth + halfDepth,
			center + halfWidth + halfDepth, center + halfWidth - halfDepth, normal, material);
	}

	const real radii[] = { 0.06f, 0.12f, 0.24f, 0.48f };
	for (int i = 0; i < 4; ++i) {
		// Equal emitted power: sphere area grows as radius squared.
		builder.AddSphereLight(Vec3(-2.25f + 1.5f * i, lightRow.y, lightRow.z),
			radii[i], Vec3(1.5f) / (radii[i] * radii[i]));
	}
}

}
