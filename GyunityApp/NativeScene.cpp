#include "NativeScene.h"
#include "json/single_include/nlohmann/json.hpp"
#include <climits>
#include <filesystem>
#include <map>
#include <set>

namespace {
using Json = nlohmann::json;

void Fields(const Json& value, std::initializer_list<const char*> names) {
	if (!value.is_object()) throw std::invalid_argument("Expected a native scene object.");
	for (const auto& item : value.items()) {
		if (std::none_of(names.begin(), names.end(), [&](const char* name) { return item.key() == name; }))
			throw std::invalid_argument("Unknown native scene field: " + item.key());
	}
}

real Number(const Json& value) {
	if (!value.is_number()) throw std::invalid_argument("Expected a finite scene number.");
	const double number = value.get<double>();
	if (!std::isfinite(number) || std::abs(number) > std::numeric_limits<real>::max())
		throw std::invalid_argument("Scene number is outside the finite renderer range.");
	return static_cast<real>(number);
}

int PositiveInteger(const Json& value) {
	if (!value.is_number_integer() || value.get<double>() < 1 || value.get<double>() > INT_MAX)
		throw std::invalid_argument("Expected a positive scene integer in the renderer range.");
	return value.get<int>();
}

std::string Text(const Json& value) {
	if (!value.is_string() || value.get_ref<const std::string&>().empty())
		throw std::invalid_argument("Expected a nonempty scene string.");
	return value.get<std::string>();
}

Vec3 ReadVector(const Json& value) {
	if (!value.is_array() || value.size() != 3) throw std::invalid_argument("Expected a three-component vector.");
	return Vec3(Number(value[0]), Number(value[1]), Number(value[2]));
}

Vec3 Color(const Json& value, bool reflectance = false) {
	const Vec3 result = ReadVector(value);
	for (int i = 0; i < 3; ++i) {
		if (result[i] < 0 || (reflectance && result[i] > 1))
			throw std::invalid_argument("Color is outside its physical parameter range.");
	}
	return result;
}

Json Identity() { return Json::array({ 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 }); }

std::array<real, 16> ReadMatrix(const Json& value) {
	if (!value.is_array() || value.size() != 16) throw std::invalid_argument("Expected a row-major 4x4 matrix.");
	std::array<real, 16> m;
	for (int i = 0; i < 16; ++i) m[i] = Number(value[i]);
	if (m[12] != 0 || m[13] != 0 || m[14] != 0 || m[15] != 1)
		throw std::invalid_argument("Native transforms must be affine.");
	const double determinant = double(m[0]) * (double(m[5])*m[10] - double(m[6])*m[9]) -
		double(m[1]) * (double(m[4])*m[10] - double(m[6])*m[8]) +
		double(m[2]) * (double(m[4])*m[9] - double(m[5])*m[8]);
	if (std::abs(determinant) < 1e-12) throw std::invalid_argument("Native transform is singular.");
	return m;
}

Transform ObjectTransform(const Json& value) {
	const auto m = ReadMatrix(value);
	return Transform(m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7],
		m[8], m[9], m[10], m[11], m[12], m[13], m[14], m[15]);
}

class NativeConductor : public Material {
public:
	NativeConductor(Vec3 reflectance, Vec3 eta, Vec3 k, real alphaX, real alphaY, bool beckmann)
		: mReflectance(reflectance), mEta(eta), mK(k), mAlphaX(alphaX), mAlphaY(alphaY), mBeckmann(beckmann) {}
	void ComputeScatteringFunction(Intersection* hit, MemoryPool& arena, TransportMode) const override {
		MicrofacetDistribution* distribution;
		if (mBeckmann)
			distribution = GYT_MEMORY_POOL_ALLOC(arena, BeckmannDistribution)(mAlphaX, mAlphaY);
		else
			distribution = GYT_MEMORY_POOL_ALLOC(arena, GGXDistribution)(mAlphaX, mAlphaY, true);
		auto fresnel = GYT_MEMORY_POOL_ALLOC(arena, FresnelConductor)(Vec3(1), mEta, mK);
		hit->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*hit);
		hit->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, MicrofacetReflectionBSDF)(distribution, fresnel, mReflectance));
	}
private:
	Vec3 mReflectance, mEta, mK;
	real mAlphaX, mAlphaY;
	bool mBeckmann;
};

std::shared_ptr<Material> ReadMaterial(const Json& value) {
	const auto type = Text(value.at("type"));
	if (type == "diffuse" || type == "conductor") {
		const auto twoSided = value.value("two_sided", Json(true));
		if (!twoSided.is_boolean() || !twoSided.get<bool>())
			throw std::invalid_argument("Version 1 requires two-sided diffuse/conductor materials.");
	}
	if (type == "diffuse") {
		Fields(value, { "type", "reflectance", "two_sided" });
		return SceneBuilder::Diffuse(Color(value.at("reflectance"), true));
	}
	if (type == "dielectric") {
		Fields(value, { "type", "eta_inside", "eta_outside", "reflection", "transmission" });
		const real inside = Number(value.value("eta_inside", Json(1.5)));
		const real outside = Number(value.value("eta_outside", Json(1)));
		if (inside <= 0 || outside <= 0) throw std::invalid_argument("Dielectric IORs must be positive.");
		return SceneBuilder::Glass(Color(value.value("reflection", Json::array({ 1, 1, 1 })), true),
			Color(value.value("transmission", Json::array({ 1, 1, 1 })), true), outside, inside);
	}
	if (type == "conductor") {
		Fields(value, { "type", "distribution", "alpha", "reflectance", "eta", "k", "two_sided" });
		const auto distribution = Text(value.at("distribution"));
		if (distribution != "beckmann" && distribution != "ggx")
			throw std::invalid_argument("Unknown microfacet distribution: " + distribution);
		const auto& alpha = value.at("alpha");
		if (!alpha.is_array() || alpha.size() != 2 || Number(alpha[0]) <= 0 || Number(alpha[1]) <= 0)
			throw std::invalid_argument("Conductor alpha must contain two positive values.");
		return std::make_shared<NativeConductor>(Color(value.value("reflectance", Json::array({ 1, 1, 1 })), true),
			Color(value.at("eta")), Color(value.at("k")), Number(alpha[0]), Number(alpha[1]), distribution == "beckmann");
	}
	throw std::invalid_argument("Unsupported native material: " + type);
}

void AddShape(Scene& scene, const std::shared_ptr<Shape>& shape, const std::shared_ptr<Material>& material, const Json& emission) {
	if (!std::isfinite(shape->Area()) || shape->Area() <= 0) throw std::invalid_argument("Degenerate native shape.");
	std::shared_ptr<Light> light;
	if (!emission.is_null()) {
		light = std::make_shared<AreaLight>(shape, Color(emission));
		scene.AddLight(light);
	}
	scene.AddPrimitive(std::make_shared<Primitive>(shape, material, light));
}

void AddQuad(Scene& scene, const std::array<Vec3, 4>& vertices, const Transform& transform,
	const std::shared_ptr<Material>& material, const Json& emission) {
	const Vec3 cross = Cross(vertices[1] - vertices[0], vertices[2] - vertices[0]);
	const Vec3 second = Cross(vertices[2] - vertices[0], vertices[3] - vertices[0]);
	if (cross.Length2() == 0 || Dot(cross, second) <= 0 ||
		std::abs(Dot(cross.Norm(), vertices[3] - vertices[0])) > 1e-6f)
		throw std::invalid_argument("Quad must be planar with nondegenerate, consistently wound triangles.");
	const Vec3 normal = cross.Norm();
	for (int i : { 1, 2 }) {
		Vec2 uv[3] = { Vec2(0, 0), Vec2(1, 0), Vec2(1, 1) };
		auto triangle = std::make_shared<Triangle>(Transform(), Transform(), vertices[0], vertices[i], vertices[i + 1], normal, uv);
		triangle->SetTransform(transform);
		AddShape(scene, triangle, material, emission);
	}
}

std::filesystem::path MeshPath(const std::filesystem::path& directory, const std::string& filename) {
	const std::filesystem::path relative(filename);
	if (relative.is_absolute() || relative.has_root_name() || relative.has_root_directory())
		throw std::invalid_argument("Native mesh paths must be relative to the scene JSON.");
	for (const auto& part : relative) {
		if (part == "..") throw std::invalid_argument("Native mesh paths cannot traverse parent directories.");
	}
	const auto result = directory / relative;
	if (!std::filesystem::is_regular_file(result)) throw std::invalid_argument("Missing native mesh: " + result.string());
	return result;
}
}

std::shared_ptr<Camera> NativeScene::CreateCamera(const std::shared_ptr<Film>& film) const {
	const auto& m = cameraToWorld;
	return std::make_shared<PinHoleCamera>(film, Vec3(m[3], m[7], m[11]), Vec3(m[2], m[6], m[10]),
		Vec3(m[0], m[4], m[8]), Vec3(m[1], m[5], m[9]), fovY, 1);
}

std::shared_ptr<NativeScene> LoadNativeScene(const std::string& filename) {
	std::ifstream input(filename);
	if (!input) throw std::invalid_argument("Cannot read native scene: " + filename);
	std::vector<std::set<std::string>> objectKeys;
	const auto root = Json::parse(input, [&](int, Json::parse_event_t event, Json& value) {
		if (event == Json::parse_event_t::object_start) objectKeys.emplace_back();
		else if (event == Json::parse_event_t::object_end) objectKeys.pop_back();
		else if (event == Json::parse_event_t::key && !objectKeys.back().insert(value.get<std::string>()).second)
			throw std::invalid_argument("Duplicate JSON object key: " + value.get<std::string>());
		return true;
	});
	Fields(root, { "schema_version", "name", "render", "camera", "materials", "shapes", "metadata" });
	if (PositiveInteger(root.value("schema_version", Json(1))) != 1) throw std::invalid_argument("Unsupported native scene version.");
	if (root.contains("metadata") && !root.at("metadata").is_object()) throw std::invalid_argument("Scene metadata must be an object.");
	auto result = std::make_shared<NativeScene>();
	result->name = Text(root.at("name"));
	const auto render = root.value("render", Json::object());
	Fields(render, { "width", "height", "spp", "max_depth", "seed", "integrator", "accelerator", "filter" });
	result->width = PositiveInteger(render.value("width", Json(512)));
	result->height = PositiveInteger(render.value("height", Json(512)));
	if (int64(result->width) * result->height > INT_MAX / 3) throw std::invalid_argument("Native film is too large.");
	result->spp = PositiveInteger(render.value("spp", Json(64)));
	result->maxDepth = PositiveInteger(render.value("max_depth", Json(8)));
	const auto seed = render.value("seed", Json(123));
	if (!seed.is_number_integer() || seed.get<double>() < 0 || seed.get<double>() > UINT32_MAX)
		throw std::invalid_argument("Seed must be a 32-bit unsigned integer.");
	result->seed = seed.get<uint32>();
	result->integrator = Text(render.value("integrator", Json("path")));
	result->accelerator = Text(render.value("accelerator", Json("bvh")));
	result->filter = Text(render.value("filter", Json("box")));
	if ((result->integrator != "path" && result->integrator != "bdpt") ||
		(result->accelerator != "bvh" && result->accelerator != "kd") ||
		(result->filter != "box" && result->filter != "tent"))
		throw std::invalid_argument("Unsupported native integrator, accelerator or filter.");
	const auto& camera = root.at("camera");
	Fields(camera, { "type", "to_world", "fov_y" });
	if (Text(camera.value("type", Json("pinhole"))) != "pinhole") throw std::invalid_argument("Unsupported native camera.");
	result->cameraToWorld = ReadMatrix(camera.at("to_world"));
	result->fovY = Number(camera.at("fov_y"));
	if (result->fovY <= 0 || result->fovY >= 179) throw std::invalid_argument("Invalid vertical FOV.");
	const auto& m = result->cameraToWorld;
	const Vec3 axes[] = { Vec3(m[0], m[4], m[8]), Vec3(m[1], m[5], m[9]), Vec3(m[2], m[6], m[10]) };
	for (int i = 0; i < 3; ++i) {
		if (std::abs(axes[i].Length2() - 1) > 1e-3f) throw std::invalid_argument("Camera axes must be unit length.");
		for (int j = 0; j < i; ++j)
			if (std::abs(Dot(axes[i], axes[j])) > 1e-3f) throw std::invalid_argument("Camera axes must be orthogonal.");
	}
	const auto& materials = root.at("materials");
	if (!materials.is_object()) throw std::invalid_argument("Native materials must be named objects.");
	std::map<std::string, std::shared_ptr<Material>> materialMap;
	for (const auto& item : materials.items()) {
		if (item.key().empty()) throw std::invalid_argument("Empty material ID.");
		materialMap.emplace(item.key(), ReadMaterial(item.value()));
	}
	result->scene = std::make_shared<Scene>();
	const auto& shapes = root.at("shapes");
	if (!shapes.is_array() || shapes.empty()) throw std::invalid_argument("Native scene needs an ordered shape array.");
	std::set<std::string> ids;
	const auto directory = std::filesystem::absolute(filename).parent_path();
	for (const auto& shape : shapes) {
		const auto type = Text(shape.at("type"));
		const auto id = Text(shape.at("id"));
		if (!ids.insert(id).second) throw std::invalid_argument("Duplicate shape ID: " + id);
		const auto material = materialMap.find(Text(shape.at("material")));
		if (material == materialMap.end()) throw std::invalid_argument(id + ": unknown material.");
		const auto transform = ObjectTransform(shape.value("to_world", Identity()));
		const auto emission = shape.value("emission", Json());
		if (type == "quad") {
			Fields(shape, { "type", "id", "material", "to_world", "emission", "vertices" });
			const auto& v = shape.at("vertices");
			if (!v.is_array() || v.size() != 4) throw std::invalid_argument(id + ": quad needs four vertices.");
			AddQuad(*result->scene, { ReadVector(v[0]), ReadVector(v[1]), ReadVector(v[2]), ReadVector(v[3]) }, transform, material->second, emission);
		}
		else if (type == "box") {
			Fields(shape, { "type", "id", "material", "to_world", "emission", "lower", "upper" });
			const Vec3 a = ReadVector(shape.value("lower", Json::array({ -1, -1, -1 })));
			const Vec3 b = ReadVector(shape.value("upper", Json::array({ 1, 1, 1 })));
			for (int axis = 0; axis < 3; ++axis) if (a[axis] >= b[axis]) throw std::invalid_argument(id + ": invalid box bounds.");
			const Vec3 v[] = { Vec3(a.x,a.y,a.z), Vec3(b.x,a.y,a.z), Vec3(b.x,b.y,a.z), Vec3(a.x,b.y,a.z),
				Vec3(a.x,a.y,b.z), Vec3(b.x,a.y,b.z), Vec3(b.x,b.y,b.z), Vec3(a.x,b.y,b.z) };
			for (const auto& face : { std::array<int,4>{0,4,7,3}, {5,1,2,6}, {0,1,5,4}, {7,6,2,3}, {1,0,3,2}, {4,5,6,7} })
				AddQuad(*result->scene, { v[face[0]], v[face[1]], v[face[2]], v[face[3]] }, transform, material->second, emission);
		}
		else if (type == "sphere") {
			Fields(shape, { "type", "id", "material", "to_world", "emission", "center", "radius" });
			if (ReadMatrix(shape.value("to_world", Identity())) != ReadMatrix(Identity()))
				throw std::invalid_argument(id + ": version 1 spheres use world-space center/radius.");
			const real radius = Number(shape.value("radius", Json(1)));
			if (radius <= 0) throw std::invalid_argument(id + ": sphere radius must be positive.");
			AddShape(*result->scene, std::make_shared<Sphere>(Transform(), Transform(), radius,
				ReadVector(shape.value("center", Json::array({ 0, 0, 0 })))), material->second, emission);
		}
		else if (type == "mesh") {
			Fields(shape, { "type", "id", "material", "to_world", "emission", "file", "face_normals" });
			const auto faceNormals = shape.value("face_normals", Json(false));
			if (!faceNormals.is_boolean()) throw std::invalid_argument(id + ": face_normals must be boolean.");
			Mesh mesh;
			mesh.LoadFromFile(MeshPath(directory, Text(shape.at("file"))).string(), false, faceNormals.get<bool>());
			for (const auto& source : mesh.mUntransformedTriangles) {
				auto triangle = std::make_shared<Triangle>(source);
				triangle->SetTransform(transform);
				AddShape(*result->scene, triangle, material->second, emission);
			}
		}
		else throw std::invalid_argument(id + ": unsupported native shape type " + type);
	}
	bool lit = false;
	for (const auto& light : result->scene->GetLights()) lit = lit || light->Power().Y() > 0;
	if (!lit) throw std::invalid_argument("Native scene requires a nonzero area emitter.");
	return result;
}
