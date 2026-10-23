#pragma once
#include "common/Core.h"
#include "math/Linalg.h"
#include "math/Transform.h"
#include "shape/Triangle.h"

GYT_NAMESPACE_BEGIN

struct Face {
	Face() {}
	Face(int v0, int v1, int v2) {
		vertIdx[0] = v0;
		vertIdx[1] = v1;
		vertIdx[2] = v2;
	}
	int vertIdx[3];
};

class Mesh {
public:
	Mesh(){
		mMaterial = nullptr;
	}
	void LoadFromFile(std::string finename, bool reverseVertices = false);
	void SetMaterial(const std::shared_ptr<Material> material);
	void SetEmission(const Vec3 emission);
	void SetMedium(const MediumInterface& mi);
	std::vector<Vec3> mVertices;
	std::vector<Vec3> mNormals;
	std::vector<Vec2> mUvs;
	std::vector<Face> mFaces;
	Transform mTransform;
	Vec3 mEmission;
	bool mIsEmission = false;
	std::shared_ptr<Material> mMaterial;
	std::vector<Triangle> mUntransformedTriangles;
	MediumInterface mMediumInterface;
};

GYT_NAMESPACE_END