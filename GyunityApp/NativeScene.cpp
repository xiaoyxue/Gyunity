#include "NativeScene.h"
#include "json/single_include/nlohmann/json.hpp"
#include <climits>
#include <filesystem>
#include <map>
#include <optional>
#include <set>

namespace {
using Json = nlohmann::json;

bool Fail(const std::string& message) {
	GetDefaultLogger()->Error("Native scene: " + message, false);
	return false;
}

template<typename T>
std::optional<T> Invalid(const std::string& message) {
	Fail(message);
	return std::nullopt;
}

bool Fields(const Json& object, std::initializer_list<const char*> allowed,
	std::initializer_list<const char*> required = {}) {
	if (!object.is_object()) return Fail("Expected an object.");
	for (const auto* name : required)
		if (!object.contains(name)) return Fail(std::string("Missing field: ") + name);
	for (const auto& item : object.items())
		if (std::none_of(allowed.begin(), allowed.end(), [&](const char* name) { return item.key() == name; }))
			return Fail("Unknown field: " + item.key());
	return true;
}

std::optional<real> Number(const Json& value) {
	if (!value.is_number()) return Invalid<real>("Expected a number.");
	const double number = value.get<double>();
	if (!std::isfinite(number) || std::abs(number) > std::numeric_limits<real>::max())
		return Invalid<real>("Number is outside the finite renderer range.");
	return static_cast<real>(number);
}

std::optional<int> PositiveInteger(const Json& value) {
	if (!value.is_number_integer() || value.get<double>() < 1 || value.get<double>() > INT_MAX)
		return Invalid<int>("Expected a positive integer in the renderer range.");
	return value.get<int>();
}

std::optional<std::string> Text(const Json& value) {
	if (!value.is_string() || value.get_ref<const std::string&>().empty())
		return Invalid<std::string>("Expected a nonempty string.");
	return value.get<std::string>();
}

std::optional<Vec3> ReadVector(const Json& value) {
	if (!value.is_array() || value.size() != 3) return Invalid<Vec3>("Expected a three-component vector.");
	const auto x = Number(value[0]), y = Number(value[1]), z = Number(value[2]);
	if (!x || !y || !z) return std::nullopt;
	return Vec3(*x, *y, *z);
}

std::optional<Vec3> Color(const Json& value, bool reflectance = false) {
	auto color = ReadVector(value);
	if (!color) return std::nullopt;
	for (int i = 0; i < 3; ++i)
		if ((*color)[i] < 0 || (reflectance && (*color)[i] > 1))
			return Invalid<Vec3>("Color is outside its physical parameter range.");
	return color;
}

Json Identity() { return Json::array({ 1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1 }); }

std::optional<std::array<real, 16>> ReadMatrix(const Json& value) {
	if (!value.is_array() || value.size() != 16) return Invalid<std::array<real,16>>("Expected a row-major 4x4 matrix.");
	std::array<real,16> m;
	for (int i = 0; i < 16; ++i) {
		const auto number = Number(value[i]);
		if (!number) return std::nullopt;
		m[i] = *number;
	}
	if (m[12] != 0 || m[13] != 0 || m[14] != 0 || m[15] != 1)
		return Invalid<std::array<real,16>>("Transforms must be affine.");
	const double determinant = double(m[0]) * (double(m[5])*m[10] - double(m[6])*m[9]) -
		double(m[1]) * (double(m[4])*m[10] - double(m[6])*m[8]) +
		double(m[2]) * (double(m[4])*m[9] - double(m[5])*m[8]);
	if (std::abs(determinant) < 1e-12) return Invalid<std::array<real,16>>("Transform is singular.");
	return m;
}

Transform MakeTransform(const std::array<real,16>& m) {
	return Transform(m[0],m[1],m[2],m[3], m[4],m[5],m[6],m[7],
		m[8],m[9],m[10],m[11], m[12],m[13],m[14],m[15]);
}

bool Orthonormal(const std::array<real,16>& m) {
	const Vec3 axes[] = { Vec3(m[0],m[4],m[8]), Vec3(m[1],m[5],m[9]), Vec3(m[2],m[6],m[10]) };
	for (int i = 0; i < 3; ++i) {
		if (std::abs(axes[i].Length2() - 1) > 1e-3f) return Fail("Direction axes must be unit length.");
		for (int j = 0; j < i; ++j)
			if (std::abs(Dot(axes[i], axes[j])) > 1e-3f) return Fail("Direction axes must be orthogonal.");
	}
	return true;
}

class NativeCheckerboard : public Texture<Vec3> {
public:
	NativeCheckerboard(Vec3 color0, Vec3 color1, real scaleU, real scaleV)
		: mColor0(color0), mColor1(color1), mScaleU(scaleU), mScaleV(scaleV) {}
	Vec3 Sample(const Vec2& uv) const override {
		const double u = double(uv.x) * mScaleU, v = double(uv.y) * mScaleV;
		return (u - std::floor(u) > 0.5) == (v - std::floor(v) > 0.5) ? mColor0 : mColor1;
	}
	Vec3 Sample(const Intersection& hit) const override { return Sample(hit.mUv); }
private:
	Vec3 mColor0, mColor1;
	real mScaleU, mScaleV;
};

std::shared_ptr<Texture<Vec3>> ReflectanceTexture(const Json& value) {
	if (value.is_array()) {
		const auto color = Color(value, true);
		return color ? SceneBuilder::ConstantColor(*color) : nullptr;
	}
	if (!Fields(value, {"type","color0","color1","scale"}, {"color0","color1"})) return nullptr;
	const auto type = Text(value.value("type", Json("checkerboard")));
	if (!type) return nullptr;
	if (*type != "checkerboard") { Fail("Unsupported reflectance texture: " + *type); return nullptr; }
	const auto scale = value.value("scale", Json::array({1,1}));
	if (!scale.is_array() || scale.size() != 2) { Fail("Checker scale requires two values."); return nullptr; }
	const auto u = Number(scale[0]), v = Number(scale[1]);
	const auto a = Color(value.at("color0"), true), b = Color(value.at("color1"), true);
	if (!u || !v || !a || !b) return nullptr;
	if (*u <= 0 || *v <= 0) { Fail("Checker scale must be positive."); return nullptr; }
	return std::make_shared<NativeCheckerboard>(*a, *b, *u, *v);
}

class FrontDiffuseBsdf : public DiffuseBSDF {
public:
	explicit FrontDiffuseBsdf(const Vec3& reflectance) : DiffuseBSDF(reflectance) {}
	Vec3 Evaluate(const Vec3& wo, const Vec3& wi) const override {
		return wo.z > 0 && wi.z > 0 ? DiffuseBSDF::Evaluate(wo, wi) : Vec3();
	}
	real Pdf(const Vec3& wo, const Vec3& wi) const override {
		return wo.z > 0 && wi.z > 0 ? DiffuseBSDF::Pdf(wo, wi) : 0;
	}
	Vec3 Sample(const Vec3& wo, Vec3* wi, real* pdf, const Vec3& sample) const override {
		if (wo.z <= 0) { *wi = Vec3(); *pdf = 0; return Vec3(); }
		return DiffuseBSDF::Sample(wo, wi, pdf, sample);
	}
};

class FrontDiffuseMaterial : public Material {
public:
	explicit FrontDiffuseMaterial(std::shared_ptr<Texture<Vec3>> reflectance) : mReflectance(std::move(reflectance)) {}
	void ComputeScatteringFunction(Intersection* hit, MemoryPool& arena, TransportMode) const override {
		hit->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*hit);
		hit->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, FrontDiffuseBsdf)(mReflectance->Sample(*hit)));
	}
private:
	std::shared_ptr<Texture<Vec3>> mReflectance;
};

class NativeConductor : public Material {
public:
	NativeConductor(Vec3 reflectance, Vec3 eta, Vec3 k, real alphaX, real alphaY, bool beckmann)
		: mReflectance(reflectance), mEta(eta), mK(k), mAlphaX(alphaX), mAlphaY(alphaY), mBeckmann(beckmann) {}
	void ComputeScatteringFunction(Intersection* hit, MemoryPool& arena, TransportMode) const override {
		MicrofacetDistribution* distribution;
		if (mBeckmann) distribution = GYT_MEMORY_POOL_ALLOC(arena, BeckmannDistribution)(mAlphaX, mAlphaY);
		else distribution = GYT_MEMORY_POOL_ALLOC(arena, GGXDistribution)(mAlphaX, mAlphaY, true);
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
	if (!value.is_object() || !value.contains("type")) { Fail("Material needs a type."); return nullptr; }
	const auto type = Text(value.at("type"));
	if (!type) return nullptr;
	if (*type == "diffuse" || *type == "conductor") {
		const auto side = value.value("two_sided", Json(true));
		if (!side.is_boolean() || (*type == "conductor" && !side.get<bool>())) {
			Fail("two_sided must be boolean; conductors currently require true.");
			return nullptr;
		}
	}
	if (*type == "diffuse") {
		if (!Fields(value, {"type","reflectance","two_sided"}, {"reflectance"})) return nullptr;
		const auto texture = ReflectanceTexture(value.at("reflectance"));
		if (!texture) return nullptr;
		if (!value.value("two_sided", true)) return std::make_shared<FrontDiffuseMaterial>(texture);
		return SceneBuilder::Diffuse(texture);
	}
	if (*type == "dielectric") {
		if (!Fields(value, {"type","eta_inside","eta_outside","reflection","transmission"})) return nullptr;
		const auto inside = Number(value.value("eta_inside", Json(1.5))), outside = Number(value.value("eta_outside", Json(1)));
		const auto reflection = Color(value.value("reflection", Json::array({1,1,1})), true);
		const auto transmission = Color(value.value("transmission", Json::array({1,1,1})), true);
		if (!inside || !outside || !reflection || !transmission) return nullptr;
		if (*inside <= 0 || *outside <= 0) { Fail("Dielectric IORs must be positive."); return nullptr; }
		return SceneBuilder::Glass(*reflection, *transmission, *outside, *inside);
	}
	if (*type == "mirror") {
		if (!Fields(value, {"type","reflection"})) return nullptr;
		const auto reflection = Color(value.value("reflection", Json::array({1,1,1})), true);
		return reflection ? SceneBuilder::Mirror(*reflection) : nullptr;
	}
	if (*type == "rough_dielectric") {
		if (!Fields(value, {"type","roughness","eta_inside","reflection","transmission"}, {"roughness"})) return nullptr;
		const auto& roughness = value.at("roughness");
		if (!roughness.is_array() || roughness.size() != 2) { Fail("Roughness requires two values."); return nullptr; }
		const auto x = Number(roughness[0]), y = Number(roughness[1]);
		const auto inside = Number(value.value("eta_inside", Json(1.5)));
		const auto reflection = Color(value.value("reflection", Json::array({1,1,1})), true);
		const auto transmission = Color(value.value("transmission", Json::array({1,1,1})), true);
		if (!x || !y || !inside || !reflection || !transmission) return nullptr;
		if (*x <= 0 || *x > 1 || *y <= 0 || *y > 1 || *inside <= 0) {
			Fail("Rough dielectric requires roughness in (0,1] and a positive interior IOR.");
			return nullptr;
		}
		return SceneBuilder::RoughDielectric(*reflection, *transmission, *x, *y, *inside);
	}
	if (*type == "plastic") {
		if (!Fields(value, {"type","diffuse","specular","roughness"}, {"diffuse","roughness"})) return nullptr;
		const auto diffuse = Color(value.at("diffuse"), true);
		const auto specular = Color(value.value("specular", Json::array({1,1,1})), true);
		const auto roughness = Number(value.at("roughness"));
		if (!diffuse || !specular || !roughness) return nullptr;
		if (*roughness <= 0 || *roughness > 1) { Fail("Plastic roughness must be in (0,1]."); return nullptr; }
		return SceneBuilder::Plastic(*diffuse, *specular, *roughness);
	}
	if (*type == "conductor") {
		if (!Fields(value, {"type","distribution","alpha","reflectance","eta","k","two_sided"}, {"distribution","alpha","eta","k"})) return nullptr;
		const auto distribution = Text(value.at("distribution"));
		if (!distribution) return nullptr;
		if (*distribution != "beckmann" && *distribution != "ggx") { Fail("Unknown microfacet distribution."); return nullptr; }
		const auto& alpha = value.at("alpha");
		if (!alpha.is_array() || alpha.size() != 2) { Fail("Alpha requires two values."); return nullptr; }
		const auto x = Number(alpha[0]), y = Number(alpha[1]);
		const auto eta = Color(value.at("eta")), k = Color(value.at("k"));
		const auto reflectance = Color(value.value("reflectance", Json::array({1,1,1})), true);
		if (!x || !y || !eta || !k || !reflectance) return nullptr;
		if (*x <= 0 || *y <= 0) { Fail("Conductor alpha must be positive."); return nullptr; }
		return std::make_shared<NativeConductor>(*reflectance, *eta, *k, *x, *y, *distribution == "beckmann");
	}
	Fail("Unsupported material: " + *type);
	return nullptr;
}

bool AddShape(Scene& scene, const std::shared_ptr<Shape>& shape, const std::shared_ptr<Material>& material, const Json& emission) {
	if (!std::isfinite(shape->Area()) || shape->Area() <= 0) return Fail("Degenerate shape.");
	std::shared_ptr<Light> light;
	if (!emission.is_null()) {
		const auto color = Color(emission);
		if (!color) return false;
		light = std::make_shared<AreaLight>(shape, *color);
		scene.AddLight(light);
	}
	scene.AddPrimitive(std::make_shared<Primitive>(shape, material, light));
	return true;
}

bool AddQuad(Scene& scene, const std::array<Vec3,4>& vertices, const Transform& transform,
	const std::shared_ptr<Material>& material, const Json& emission) {
	const Vec3 normal = Cross(vertices[1]-vertices[0], vertices[2]-vertices[0]);
	const Vec3 second = Cross(vertices[2]-vertices[0], vertices[3]-vertices[0]);
	if (normal.Length2() == 0 || Dot(normal, second) <= 0 ||
		std::abs(Dot(normal.Norm(), vertices[3]-vertices[0])) > 1e-6f)
		return Fail("Quad must be planar with consistently wound, nondegenerate triangles.");
	for (int i : {1,2}) {
		Vec2 uv[3] = {Vec2(0,0), Vec2(1,0), Vec2(1,1)};
		auto triangle = std::make_shared<Triangle>(Transform(), Transform(), vertices[0], vertices[i], vertices[i+1], normal.Norm(), uv);
		triangle->SetTransform(transform);
		if (!AddShape(scene, triangle, material, emission)) return false;
	}
	return true;
}

std::optional<std::filesystem::path> AssetPath(const std::filesystem::path& directory, const std::string& filename) {
	const std::filesystem::path relative(filename);
	if (relative.is_absolute() || relative.has_root_name() || relative.has_root_directory())
		return Invalid<std::filesystem::path>("Asset paths must be relative.");
	for (const auto& part : relative)
		if (part == "..") return Invalid<std::filesystem::path>("Asset paths cannot traverse parents.");
	const auto result = directory / relative;
	std::error_code error;
	if (!std::filesystem::is_regular_file(result, error) || error)
		return Invalid<std::filesystem::path>("Cannot read asset: " + result.string());
	return result;
}
}

std::shared_ptr<Camera> NativeScene::CreateCamera(const std::shared_ptr<Film>& film) const {
	const auto& m = cameraToWorld;
	return std::make_shared<PinHoleCamera>(film, Vec3(m[3],m[7],m[11]), Vec3(m[2],m[6],m[10]),
		Vec3(m[0],m[4],m[8]), Vec3(m[1],m[5],m[9]), fovY, 1);
}

std::shared_ptr<NativeScene> LoadNativeScene(const std::string& filename) {
	std::ifstream input(filename);
	if (!input) { Fail("Cannot read file: " + filename); return nullptr; }
	std::vector<std::set<std::string>> keys;
	bool duplicate = false;
	const auto root = Json::parse(input, [&](int, Json::parse_event_t event, Json& value) {
		if (event == Json::parse_event_t::object_start) keys.emplace_back();
		else if (event == Json::parse_event_t::object_end) keys.pop_back();
		else if (event == Json::parse_event_t::key && !keys.back().insert(value.get<std::string>()).second) duplicate = true;
		return true;
	}, false);
	if (root.is_discarded() || duplicate) { Fail("Malformed JSON or duplicate keys in " + filename); return nullptr; }
	if (!Fields(root, {"schema_version","name","render","camera","materials","shapes","lights","metadata"}, {"name","camera","materials","shapes"})) return nullptr;
	const auto version = PositiveInteger(root.value("schema_version", Json(1)));
	if (!version) return nullptr;
	if (*version != 1) { Fail("Unsupported schema version."); return nullptr; }
	if (root.contains("metadata") && !root.at("metadata").is_object()) { Fail("Metadata must be an object."); return nullptr; }
	const auto name = Text(root.at("name"));
	if (!name) return nullptr;
	const auto render = root.value("render", Json::object());
	if (!Fields(render, {"width","height","spp","max_depth","seed","integrator","accelerator","filter","sppm"})) return nullptr;
	const auto width = PositiveInteger(render.value("width", Json(512))), height = PositiveInteger(render.value("height", Json(512)));
	const auto spp = PositiveInteger(render.value("spp", Json(64))), depth = PositiveInteger(render.value("max_depth", Json(8)));
	const auto integrator = Text(render.value("integrator", Json("path"))), accelerator = Text(render.value("accelerator", Json("bvh")));
	const auto filter = Text(render.value("filter", Json("box")));
	if (!width || !height || !spp || !depth || !integrator || !accelerator || !filter) return nullptr;
	const auto seed = render.value("seed", Json(123));
	if (!seed.is_number_integer() || seed.get<double>() < 0 || seed.get<double>() > UINT32_MAX) { Fail("Seed must be an unsigned 32-bit integer."); return nullptr; }
	if (int64(*width) * *height > INT_MAX / 3) { Fail("Film dimensions exceed the supported range."); return nullptr; }
	if ((*integrator != "path" && *integrator != "bdpt" && *integrator != "sppm") || (*accelerator != "bvh" && *accelerator != "kd") ||
		(*filter != "box" && *filter != "tent" && *filter != "gaussian")) { Fail("Unsupported renderer, accelerator or filter."); return nullptr; }
	auto sppm = render.value("sppm", Json::object());
	if (sppm.is_null()) sppm = Json::object();
	if (!Fields(sppm, {"iterations","photons_per_stage"})) return nullptr;
	const auto iterations = PositiveInteger(sppm.value("iterations", Json(256)));
	const auto photons = PositiveInteger(sppm.value("photons_per_stage", Json(300000)));
	if (!iterations || !photons) return nullptr;
	const auto& camera = root.at("camera");
	if (!Fields(camera, {"type","to_world","fov_y"}, {"to_world","fov_y"})) return nullptr;
	const auto cameraType = Text(camera.value("type", Json("pinhole")));
	const auto cameraMatrix = ReadMatrix(camera.at("to_world"));
	const auto fov = Number(camera.at("fov_y"));
	if (!cameraType || !cameraMatrix || !fov) return nullptr;
	if (*cameraType != "pinhole" || *fov <= 0 || *fov >= 179) { Fail("Unsupported camera or vertical FOV."); return nullptr; }
	if (!Orthonormal(*cameraMatrix)) return nullptr;
	auto result = std::make_shared<NativeScene>();
	result->name = *name; result->width = *width; result->height = *height;
	result->spp = *spp; result->maxDepth = *depth; result->seed = seed.get<uint32>();
	result->sppm.iterations = *iterations; result->sppm.photonsPerStage = *photons;
	result->integrator = *integrator; result->accelerator = *accelerator; result->filter = *filter;
	result->cameraToWorld = *cameraMatrix; result->fovY = *fov;
	const auto& materials = root.at("materials");
	if (!materials.is_object()) { Fail("Materials must be named objects."); return nullptr; }
	std::map<std::string, std::shared_ptr<Material>> materialMap;
	for (const auto& item : materials.items()) {
		if (item.key().empty()) { Fail("Empty material ID."); return nullptr; }
		auto material = ReadMaterial(item.value());
		if (!material) return nullptr;
		materialMap.emplace(item.key(), std::move(material));
	}
	const auto& shapes = root.at("shapes");
	if (!shapes.is_array() || shapes.empty()) { Fail("An ordered, nonempty shape array is required."); return nullptr; }
	std::error_code error;
	const auto directory = std::filesystem::absolute(filename, error).parent_path();
	if (error) { Fail("Cannot resolve scene path: " + filename); return nullptr; }
	result->scene = std::make_shared<Scene>();
	std::set<std::string> ids;
	for (const auto& shape : shapes) {
		if (!shape.is_object() || !shape.contains("type") || !shape.contains("id") || !shape.contains("material")) { Fail("Shape needs type, id and material."); return nullptr; }
		const auto type = Text(shape.at("type")), id = Text(shape.at("id")), materialName = Text(shape.at("material"));
		const auto matrix = ReadMatrix(shape.value("to_world", Identity()));
		if (!type || !id || !materialName || !matrix) return nullptr;
		if (!ids.insert(*id).second) { Fail("Duplicate shape ID: " + *id); return nullptr; }
		const auto material = materialMap.find(*materialName);
		if (material == materialMap.end()) { Fail(*id + ": unknown material " + *materialName); return nullptr; }
		const auto transform = MakeTransform(*matrix);
		const auto emission = shape.value("emission", Json());
		if (*type == "quad") {
			if (!Fields(shape, {"type","id","material","to_world","emission","vertices"}, {"vertices"})) return nullptr;
			const auto& v = shape.at("vertices");
			if (!v.is_array() || v.size() != 4) { Fail(*id + ": expected four vertices."); return nullptr; }
			const auto a=ReadVector(v[0]), b=ReadVector(v[1]), c=ReadVector(v[2]), d=ReadVector(v[3]);
			if (!a || !b || !c || !d || !AddQuad(*result->scene, {*a,*b,*c,*d}, transform, material->second, emission)) return nullptr;
		}
		else if (*type == "box") {
			if (!Fields(shape, {"type","id","material","to_world","emission","lower","upper"})) return nullptr;
			const auto lower=ReadVector(shape.value("lower", Json::array({-1,-1,-1}))), upper=ReadVector(shape.value("upper", Json::array({1,1,1})));
			if (!lower || !upper) return nullptr;
			const Vec3 a=*lower, b=*upper;
			for (int axis=0; axis<3; ++axis) if (a[axis]>=b[axis]) { Fail(*id + ": invalid box bounds."); return nullptr; }
			const Vec3 v[] = {Vec3(a.x,a.y,a.z),Vec3(b.x,a.y,a.z),Vec3(b.x,b.y,a.z),Vec3(a.x,b.y,a.z),
				Vec3(a.x,a.y,b.z),Vec3(b.x,a.y,b.z),Vec3(b.x,b.y,b.z),Vec3(a.x,b.y,b.z)};
			for (const auto& f : {std::array<int,4>{0,4,7,3},{5,1,2,6},{0,1,5,4},{7,6,2,3},{1,0,3,2},{4,5,6,7}})
				if (!AddQuad(*result->scene, {v[f[0]],v[f[1]],v[f[2]],v[f[3]]}, transform, material->second, emission)) return nullptr;
		}
		else if (*type == "sphere") {
			if (!Fields(shape, {"type","id","material","to_world","emission","center","radius"})) return nullptr;
			const auto identity = ReadMatrix(Identity());
			if (*matrix != *identity) { Fail(*id + ": transformed spheres are not supported."); return nullptr; }
			const auto radius=Number(shape.value("radius", Json(1)));
			const auto center=ReadVector(shape.value("center", Json::array({0,0,0})));
			if (!radius || !center) return nullptr;
			if (*radius<=0) { Fail(*id + ": radius must be positive."); return nullptr; }
			if (!AddShape(*result->scene, std::make_shared<Sphere>(Transform(),Transform(),*radius,*center), material->second, emission)) return nullptr;
		}
		else if (*type == "mesh") {
			if (!Fields(shape, {"type","id","material","to_world","emission","file","face_normals"}, {"file"})) return nullptr;
			const auto file=Text(shape.at("file"));
			const auto faceNormals=shape.value("face_normals", Json(false));
			if (!file) return nullptr;
			if (!faceNormals.is_boolean()) { Fail(*id + ": face_normals must be boolean."); return nullptr; }
			const auto path=AssetPath(directory,*file);
			if (!path) return nullptr;
			Mesh mesh;
			if (!mesh.TryLoadFromFile(path->string(),false,faceNormals.get<bool>())) return nullptr;
			for (const auto& source : mesh.mUntransformedTriangles) {
				auto triangle=std::make_shared<Triangle>(source);
				triangle->SetTransform(transform);
				if (!AddShape(*result->scene,triangle,material->second,emission)) return nullptr;
			}
		}
		else { Fail(*id + ": unsupported shape type " + *type); return nullptr; }
	}
	const auto lights=root.value("lights",Json::array());
	if (!lights.is_array() || lights.size()>1) { Fail("At most one environment is supported."); return nullptr; }
	for (const auto& light : lights) {
		if (!Fields(light,{"type","id","file","scale","to_world"},{"file"})) return nullptr;
		const auto type=Text(light.value("type",Json("environment"))), id=Text(light.value("id",Json("environment")));
		const auto file=Text(light.at("file"));
		const auto scale=Number(light.value("scale",Json(1)));
		const auto matrix=ReadMatrix(light.value("to_world",Identity()));
		if (!type || !id || !file || !scale || !matrix) return nullptr;
		if (*type!="environment" || *scale<=0) { Fail("Unsupported light or nonpositive environment scale."); return nullptr; }
		if ((*matrix)[3]!=0 || (*matrix)[7]!=0 || (*matrix)[11]!=0) { Fail("Environment direction transforms cannot translate."); return nullptr; }
		if (!Orthonormal(*matrix)) return nullptr;
		const auto path=AssetPath(directory,*file);
		if (!path) return nullptr;
		auto environment=std::make_shared<EnvironmentLight>(path->string(),0,*scale,MakeTransform(*matrix),true);
		if (!environment->IsValid()) return nullptr;
		result->scene->AddLight(environment);
	}
	bool lit=false;
	for (const auto& light : result->scene->GetLights()) lit=lit || light->IsEnvironmentLight() || light->Power().Y()>0;
	if (!lit) { Fail("An area or environment emitter is required."); return nullptr; }
	return result;
}
