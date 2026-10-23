#include "Mesh.h"
#define TINYOBJLOADER_IMPLEMENTATION
#include "tinyobjloader/tiny_obj_loader.h"

GYT_NAMESPACE_BEGIN

void Mesh::LoadFromFile(std::string inputfile, bool reverseVertices) {
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::string warn;
	std::string err;

	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inputfile.c_str());

	if (!warn.empty()) {
		std::cout << warn << std::endl;
	}
	if (!err.empty()) {
		std::cerr << err << std::endl;
	}
	if (!ret) {
		exit(1);
	}
	// Loop over shapes
	for (size_t s = 0; s < shapes.size(); s++) {
		// Loop over faces(polygon)
		size_t indexOffset = 0;
		for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
			int fv = shapes[s].mesh.num_face_vertices[f];

			// Loop over vertices in the face.
			int i = (int)mVertices.size(), j = i + 1, k = i + 2;
			if (reverseVertices) {
				std::swap(j, k);
			}
			bool hasNormal = false;
			for (int v = 0; v < fv; v++) {
				// access to vertex
				tinyobj::index_t idx = shapes[s].mesh.indices[indexOffset + v];
				float vx = attrib.vertices[3 * idx.vertex_index + 0];
				float vy = attrib.vertices[3 * idx.vertex_index + 1];
				float vz = attrib.vertices[3 * idx.vertex_index + 2];
				mVertices.push_back(Vec3(vx, vy, vz));
				if (idx.normal_index != -1) {
					float nx = attrib.normals[3 * idx.normal_index + 0];
					float ny = attrib.normals[3 * idx.normal_index + 1];
					float nz = attrib.normals[3 * idx.normal_index + 2];
					hasNormal = true;
					mNormals.push_back(Vec3(nx, ny, nz));
				}
				else {
					// should be consistent
					assert(!hasNormal);
					hasNormal = false;
				}
				float tx = 0.0f, ty = 0.0f;
				if (idx.texcoord_index != -1) {
					tx = attrib.texcoords[2 * idx.texcoord_index + 0];
					ty = attrib.texcoords[2 * idx.texcoord_index + 1];
				}
				mUvs.push_back(Vector2(tx, ty));
			}
			if (!hasNormal) {
				Vec3* a = &mVertices[mVertices.size() - 1] - 2;
				Vec3 generatedNormal = Cross(a[1] - a[0], a[2] - a[0]);
				if (generatedNormal.Length() > 1e-6f) {
					generatedNormal = generatedNormal.Norm();
				}
				for (int v = 0; v < fv; v++) {
					mNormals.push_back(generatedNormal);
				}
			}

			mUntransformedTriangles.push_back(
				Triangle(Transform(), Transform(),
					mVertices[i], mVertices[j], mVertices[k],
					mNormals[i], mNormals[j], mNormals[k],
					mUvs[i], mUvs[j], mUvs[k]));
			indexOffset += fv;
		}
	}
}

void Mesh::SetMaterial(const std::shared_ptr<Material> material) {
	this->mMaterial = material;
}
void Mesh::SetEmission(const Vec3 emission) {
	this->mIsEmission = true;
	this->mEmission = emission;
}

void Mesh::SetMedium(const MediumInterface& mi) {
	this->mMediumInterface = mi;
}

GYT_NAMESPACE_END