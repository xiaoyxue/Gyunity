#include "gtest/gtest.h"
#include "integrator/VCMPath.h"
#include "integrator/BDPT.h"
#include "VCMReference.h"
#include "accelerator/BruteForceAccel.h"
#include "camera/PinholeCamera.h"
#include "light/AreaLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Glass.h"
#include "material/Mirror.h"
#include "texture/ConstantTexture.h"
#include "visual/Rng.h"
#include <array>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

using namespace Gyunity;

namespace {

const double Pi = std::acos(-1.0);
using Vcm::Technique;
using Vcm::Mode;
using Vcm::Strategy;
using VcmReference::Edges;

class TestCamera : public PinHoleCamera {
public:
	bool fixed = false;
	real pdf = 0.5f;
	TestCamera(const Vec3& position, const Vec3& target)
		: PinHoleCamera(std::make_shared<Film>(4, 4), position, target, Vec3(0, 1, 0), 70, 1) {}
	real PdfDir(const Ray& ray) const override { return fixed ? pdf : PinHoleCamera::PdfDir(ray); }
};

class TestLight : public AreaLight {
public:
	bool fixed = false;
	real pdfPos = 0.5f, pdfDir = 0.25f;
	TestLight(const std::shared_ptr<Shape>& shape, real power = 1) : AreaLight(shape, Vec3(power)) {}
	bool PdfLight(const Intersection& point, const Vec3& dir, real* position, real* direction) const override {
		if (!fixed) return AreaLight::PdfLight(point, dir, position, direction);
		*position = pdfPos;
		*direction = pdfDir;
		return true;
	}
};

class DirectionalLobe : public BxDF {
public:
	explicit DirectionalLobe(real coefficient)
		: BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_GLOSSY)), coefficient(coefficient) {}
	real Pdf(const Vec3& wo, const Vec3& wi) const override {
		// Normalized on the sphere, strictly positive, and deliberately asymmetric.
		return (1 + coefficient * wo.x * wi.z + 0.3f * wo.z * wi.x) / (4 * PI);
	}
	Vec3 Sample(const Vec3&, Vec3*, real*, const Vec3&) const override { return Vec3(0); }
	Vec3 Evaluate(const Vec3& wo, const Vec3& wi) const override { return Vec3(Pdf(wo, wi)); }
	real coefficient;
};

class FixedLobe : public BxDF {
public:
	explicit FixedLobe(real value)
		: BxDF(ScatterEventType(BSDF_REFLECTION | BSDF_TRANSMISSION | BSDF_GLOSSY)), value(value) {}
	real Pdf(const Vec3&, const Vec3&) const override { return value; }
	Vec3 Sample(const Vec3&, Vec3*, real*, const Vec3&) const override { return Vec3(0); }
	Vec3 Evaluate(const Vec3&, const Vec3&) const override { return Vec3(value); }
	real value;
};

// Independent oracle conversion: no Bidirectional helpers or ConvertDensity.
double Jacobian(const PathVertex& source, const PathVertex& destination) {
	const double x = double(destination.mIsect.mPos.x) - source.mIsect.mPos.x;
	const double y = double(destination.mIsect.mPos.y) - source.mIsect.mPos.y;
	const double z = double(destination.mIsect.mPos.z) - source.mIsect.mPos.z;
	const double distance2 = x * x + y * y + z * z;
	if (destination.mType == PathVertexType::Camera) return 1 / distance2;
	const Vec3& normal = destination.mIsect.mGeometryNormal;
	return std::abs(normal.x * x + normal.y * y + normal.z * z) / (distance2 * std::sqrt(distance2));
}

double DirectionPdf(const PathVertex& source, const PathVertex* previous, const PathVertex& destination) {
	const Vec3 wi = (destination.mIsect.mPos - source.mIsect.mPos).Norm();
	if (source.mType == PathVertexType::Camera) return source.mCamera->PdfDir(Ray(source.mIsect.mPos, wi));
	if (!previous) {
		real position, direction;
		EXPECT_TRUE(source.mLight->PdfLight(source.mIsect, wi, &position, &direction));
		return direction;
	}
	return source.mIsect.mBsdf->Pdf((previous->mIsect.mPos - source.mIsect.mPos).Norm(), wi);
}

double OriginPdf(const Scene& scene, const PathVertex& origin, const PathVertex& next) {
	real position, direction;
	EXPECT_TRUE(origin.mLight->PdfLight(origin.mIsect, (next.mIsect.mPos - origin.mIsect.mPos).Norm(), &position, &direction));
	double selection = 0;
	for (size_t i = 0; i < scene.GetLights().size(); ++i) {
		if (scene.GetLights()[i].get() != origin.mLight) continue;
		real pdf;
		EXPECT_TRUE(scene.LightSelectionPdf(i, &pdf));
		selection += pdf;
	}
	return position * selection;
}

// Independently re-evaluate an unchanged reverse event from its ORIGINAL
// sampled directions, not the cached mPdfPrev or a reconstructed polyline.
double ReverseEventPdf(const std::vector<PathVertex>& path, int i) {
	const auto& source = path[i + 1];
	return source.mIsect.mBsdf->Pdf(-path[i + 2].mIsect.mOutDir, source.mIsect.mOutDir) *
		Jacobian(source, path[i]);
}

// Report input extraction is separate from the production evaluator. Two
// geometric chains supply conversions; ONLY the merge scattering comes from x.
Edges ReportEdges(const Scene& scene, const Vcm::PathContext& path) {
	const bool merge = path.selected.technique == Technique::Merging;
	const int n = path.selected.s + path.selected.t - (merge ? 1 : 0);
	std::vector<const PathVertex*> left, right;
	for (int i = 0; i < path.selected.s; ++i) left.push_back(&path.light[i]);
	for (int i = path.selected.t - 1 - (merge ? 1 : 0); i >= 0; --i) left.push_back(&path.camera[i]);
	right = left;
	if (merge) right[path.selected.s - 1] = &path.camera[path.selected.t - 1];
	Edges e;
	e.f.resize(n); e.r.resize(n); e.gf.resize(n, 1); e.gr.resize(n, 1);
	for (int i = 1; i < n; ++i) {
		const auto& chain = merge && i >= path.selected.s ? right : left;
		e.gf[i] = Jacobian(*chain[i - 1], *chain[i]);
	}
	for (int i = 0; i + 1 < n; ++i) {
		const auto& chain = merge && i >= path.selected.s - 1 ? right : left;
		e.gr[i] = Jacobian(*chain[i + 1], *chain[i]);
	}
	e.f[0] = path.selected.s ? path.light[0].mPdfFwd : OriginPdf(scene, *left[0], *left[1]);
	e.r[n - 1] = path.camera[0].mPdfFwd;
	for (int i = 1; i < n; ++i) {
		if (i < path.selected.s) e.f[i] = path.light[i].mPdfFwd;
		else if (n - 1 - i + 2 < path.selected.t) e.f[i] = ReverseEventPdf(path.camera, n - 1 - i);
		else if (merge && i == path.selected.s) {
			const auto& x = path.camera[path.selected.t - 1];
			e.f[i] = x.mIsect.mBsdf->Pdf(path.light[path.selected.s - 1].mIsect.mOutDir, x.mIsect.mOutDir) * e.gf[i];
		}
		else e.f[i] = DirectionPdf(*right[i - 1], i > 1 ? right[i - 2] : nullptr, *right[i]) * e.gf[i];
	}
	const int eyeStart = path.selected.s - (merge ? 1 : 0);
	for (int i = 0; i + 1 < n; ++i) {
		if (i >= eyeStart) e.r[i] = path.camera[n - 1 - i].mPdfFwd;
		else if (i + 2 < path.selected.s) e.r[i] = ReverseEventPdf(path.light, i);
		else if (merge && i == eyeStart - 1) {
			const auto& x = path.camera[path.selected.t - 1];
			e.r[i] = x.mIsect.mBsdf->Pdf(x.mIsect.mOutDir, path.light[path.selected.s - 1].mIsect.mOutDir) * e.gr[i];
		}
		else e.r[i] = DirectionPdf(*left[i + 1], i + 2 < n ? left[i + 2] : nullptr, *left[i]) * e.gr[i];
	}
	return e;
}

struct Fixture {
	Scene scene;
	MemoryPool arena;
	std::shared_ptr<TestCamera> camera;
	std::shared_ptr<TestLight> light;
	std::vector<PathVertex> full;
	Edges direct;

	explicit Fixture(int n = 3, bool exact = false, bool directional = false) : full(n) {
		auto shape = std::make_shared<Triangle>(Transform(), Transform(),
			Vec3(-1, -1, 2), Vec3(1, -1, 2), Vec3(0, 1, 2), Vec3(0, 0, -1));
		light = std::make_shared<TestLight>(shape);
		light->fixed = exact;
		scene.AddLight(light);
		scene.SetAccelerator(std::make_shared<BruteForce>());
		scene.Initialize();
		for (int i = 0; i < n; ++i) {
			auto& v = full[i];
			v.mIsect.mPos = exact ? Vec3(0, 0, i % 2 == 0 ? 1.f : 0.f) :
				Vec3(0.23f * i, 0.08f * (i % 3), i % 2 == 0 ? 2.f : 0.f);
			v.mIsect.mNormal = v.mIsect.mGeometryNormal = Vec3(0, 0, i == 0 ? -1.f : 1.f);
			v.mIsect.mPrimitive = nullptr;
			CoordinateSystem(v.mIsect.mNormal, &v.mIsect.mShadingDpDu, &v.mIsect.mShadingDpDv);
			v.mThroughput = Vec3(real(i + 1));
			if (i > 0 && i < n - 1) {
				v.mIsect.mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(v.mIsect);
				if (exact) v.mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, FixedLobe)(0.125f));
				else if (directional) v.mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, DirectionalLobe)(0.5f));
				else v.mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(Vec3(1)));
			}
		}
		full[0].mType = PathVertexType::Light;
		full[0].mLight = light.get();
		camera = std::make_shared<TestCamera>(full.back().mIsect.mPos, full[n - 2].mIsect.mPos);
		camera->fixed = exact;
		full.back().mType = PathVertexType::Camera;
		full.back().mCamera = camera.get();
		Refresh();
	}

	void Refresh() {
		const int n = static_cast<int>(full.size());
		direct.f.resize(n); direct.r.resize(n); direct.gf.resize(n, 1); direct.gr.resize(n, 1);
		direct.f[0] = OriginPdf(scene, full[0], full[1]);
		direct.r[n - 1] = 1;
		for (int i = 1; i < n; ++i) {
			direct.gf[i] = Jacobian(full[i - 1], full[i]);
			direct.f[i] = DirectionPdf(full[i - 1], i > 1 ? &full[i - 2] : nullptr, full[i]) * direct.gf[i];
		}
		for (int i = 0; i + 1 < n; ++i) {
			direct.gr[i] = Jacobian(full[i + 1], full[i]);
			direct.r[i] = DirectionPdf(full[i + 1], i + 2 < n ? &full[i + 2] : nullptr, full[i]) * direct.gr[i];
		}
	}

	void Split(int s, bool merge, std::vector<PathVertex>& lightPath, std::vector<PathVertex>& eyePath) const {
		lightPath.clear(); eyePath.clear();
		const int n = static_cast<int>(full.size());
		for (int i = 0; i < s; ++i) {
			auto v = full[i];
			v.mPdfFwd = real(direct.f[i]); v.mPdfPrev = real(direct.r[i]);
			if (i > 0) v.mIsect.mOutDir = (full[i - 1].mIsect.mPos - v.mIsect.mPos).Norm();
			lightPath.push_back(v);
		}
		for (int i = n - 1; i >= s - (merge ? 1 : 0); --i) {
			auto v = full[i];
			v.mPdfFwd = real(direct.r[i]); v.mPdfPrev = real(direct.f[i]);
			if (i < n - 1) v.mIsect.mOutDir = (full[i + 1].mIsect.mPos - v.mIsect.mPos).Norm();
			eyePath.push_back(v);
		}
	}
};

void SamePaths(const std::vector<PathVertex>& a, const std::vector<PathVertex>& b) {
	ASSERT_EQ(a.size(), b.size());
	for (size_t i = 0; i < a.size(); ++i) {
		EXPECT_EQ(a[i].mIsect.mPos, b[i].mIsect.mPos);
		EXPECT_EQ(a[i].mIsect.mOutDir, b[i].mIsect.mOutDir);
		EXPECT_EQ(a[i].mIsect.mNormal, b[i].mIsect.mNormal);
		EXPECT_EQ(a[i].mIsect.mGeometryNormal, b[i].mIsect.mGeometryNormal);
		EXPECT_EQ(a[i].mIsect.mPointError, b[i].mIsect.mPointError);
		EXPECT_EQ(a[i].mIsect.mBsdf, b[i].mIsect.mBsdf);
		EXPECT_EQ(a[i].mType, b[i].mType);
		EXPECT_EQ(a[i].mDirection, b[i].mDirection);
		EXPECT_EQ(a[i].mThroughput, b[i].mThroughput);
		EXPECT_EQ(a[i].mLight, b[i].mLight);
		EXPECT_EQ(a[i].mCamera, b[i].mCamera);
		EXPECT_EQ(a[i].mIsDelta, b[i].mIsDelta);
		if (std::isnan(b[i].mPdfFwd)) EXPECT_TRUE(std::isnan(a[i].mPdfFwd));
		else EXPECT_EQ(a[i].mPdfFwd, b[i].mPdfFwd);
		if (std::isnan(b[i].mPdfPrev)) EXPECT_TRUE(std::isnan(a[i].mPdfPrev));
		else EXPECT_EQ(a[i].mPdfPrev, b[i].mPdfPrev);
		EXPECT_EQ(a[i].mPdfPos, b[i].mPdfPos);
		EXPECT_EQ(a[i].mPdfDir, b[i].mPdfDir);
		EXPECT_EQ(a[i].mLightSelectionPdf, b[i].mLightSelectionPdf);
	}
}

std::ofstream* Report() {
	static std::ofstream stream;
	static bool initialized = false;
	if (!initialized) {
		initialized = true;
		if (const char* output = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY")) {
			if (!*output) {
				ADD_FAILURE() << "Requested VCM CSV report directory is empty (GYUNITY_TEST_OUTPUT_DIRECTORY).";
				return nullptr;
			}
#ifdef _DEBUG
			const std::string filename = std::string(output) + "\\vcm-weights-debug.csv";
#else
			const std::string filename = std::string(output) + "\\vcm-weights-release.csv";
#endif
			stream.open(filename);
			if (!stream.is_open()) {
				ADD_FAILURE() << "Cannot open requested VCM CSV report: " << filename;
				return nullptr;
			}
			stream << "case,selected_kind,selected_s,selected_t,mode,radius,N_L,kind,s,t,feasible,production,report,direct,abs_report,abs_direct,log_pdf,direct_log_pdf\n";
			stream << std::setprecision(17);
		}
	}
	return stream.is_open() ? &stream : nullptr;
}

void Close(double actual, double expected) {
	EXPECT_TRUE(std::isfinite(actual));
	EXPECT_GE(actual, 0);
	EXPECT_LE(actual, 1);
	const double difference = std::abs(actual - expected);
	EXPECT_TRUE(difference <= 1e-6 || difference <= 1e-5 * std::abs(expected))
		<< std::setprecision(17) << actual << " versus " << expected;
}

void Compare(const std::string& name, const Scene& scene, const Vcm::PathContext& path, const Edges& direct) {
	SCOPED_TRACE(name);
	const auto beforeLight = path.light, beforeEye = path.camera;
	const Edges report = ReportEdges(scene, path);
	const int n = static_cast<int>(direct.f.size());
	const bool vc = path.mode != Mode::MergingOnly || n == 2;
	const bool vm = path.mode != Mode::ConnectionOnly;
	const double area = Pi * path.radius * path.radius;
	const auto expected = VcmReference::DirectWeights(direct, area, double(path.lightPathCount), vc, vm);
	std::ofstream* reportFile = Report();
	double sum = 0;
	size_t index = 0;
	for (Technique kind : { Technique::Connection, Technique::Merging }) {
		for (int s = kind == Technique::Connection ? 0 : 2; s < n; ++s, ++index) {
			const Strategy strategy{ kind, s, n - s + (kind == Technique::Merging ? 1 : 0) };
			SCOPED_TRACE(testing::Message() << "strategy " << int(kind) << " " << s << "," << strategy.t);
			Vcm::StrategyPdf pdf;
			ASSERT_TRUE(Vcm::ComputePathPdf(scene, path, strategy, &pdf));
			EXPECT_EQ(pdf.feasible, expected[index] > 0);
			real weight = 0;
			double expectedLog = 0;
			if (pdf.feasible) {
				EXPECT_TRUE(std::isfinite(pdf.logPdf));
				VcmReference::Product product;
				for (int i = 0; i < s; ++i) product.Multiply(direct.f[i]);
				for (int i = s - (kind == Technique::Merging ? 1 : 0); i < n; ++i) product.Multiply(direct.r[i]);
				if (kind == Technique::Merging) product.Multiply(area);
				expectedLog = std::log(product.mantissa) + product.exponent * std::log(2.0);
				EXPECT_NEAR(pdf.logPdf, expectedLog, 1e-5);
				ASSERT_TRUE(Vcm::ComputeMisWeight(scene, path, strategy, &weight));
			}
			const double oracle = VcmReference::Weight(report, kind == Technique::Merging, s, area,
				double(path.lightPathCount), vc, vm);
			Close(weight, expected[index]);
			Close(weight, oracle);
			sum += weight;
			if (reportFile) {
				*reportFile << name << ',' << int(path.selected.technique) << ',' << path.selected.s << ',' << path.selected.t <<
					',' << int(path.mode) << ',' << path.radius << ',' << path.lightPathCount << ',' << int(kind) <<
					',' << s << ',' << strategy.t << ',' << pdf.feasible << ',' << weight << ',' << oracle << ',' <<
					expected[index] << ',' << std::abs(weight - oracle) << ',' << std::abs(weight - expected[index]) << ',';
				if (pdf.feasible) *reportFile << pdf.logPdf << ',' << expectedLog;
				else *reportFile << ',';
				*reportFile << '\n';
			}
		}
	}
	if (reportFile) {
		reportFile->flush();
		EXPECT_TRUE(reportFile->good()) << "Cannot write requested VCM CSV report under "
			<< std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
	}
	EXPECT_NEAR(sum, 1, 1e-6);
	SamePaths(path.light, beforeLight);
	SamePaths(path.camera, beforeEye);
}

}

TEST(VCMPath, ExactDepthOneFixtureAllWeightsAndScaling) {
	Fixture fixture(3, true);
	std::vector<PathVertex> light, eye;
	fixture.Split(2, true, light, eye);
	const double radius = std::sqrt(0.25 / Pi);
	const std::vector<std::vector<double>> manual = {
		{ 1. / 11, 4. / 11, 2. / 11, 4. / 11 }, { 1. / 23, 4. / 23, 2. / 23, 16. / 23 },
		{ 1. / 15, 4. / 15, 2. / 15, 8. / 15 }, { 1. / 7, 4. / 7, 2. / 7, 0 }, { 0, 0, 0, 1 }
	};
	for (int variant = 0; variant < 5; ++variant) {
		const Vcm::PathContext path(light, eye, { Technique::Merging, 2, 2 }, radius * (variant == 1 ? 2 : 1),
			variant == 2 ? 32 : 16, variant == 3 ? Mode::ConnectionOnly : variant == 4 ? Mode::MergingOnly : Mode::Combined);
		Compare("exact-" + std::to_string(variant), fixture.scene, path, fixture.direct);
		int index = 0;
		for (Strategy strategy : { Strategy{Technique::Connection, 0, 3}, {Technique::Connection, 1, 2},
			{Technique::Connection, 2, 1}, {Technique::Merging, 2, 2} }) {
			Vcm::StrategyPdf pdf;
			ASSERT_TRUE(Vcm::ComputePathPdf(fixture.scene, path, strategy, &pdf));
			real weight = 0;
			if (pdf.feasible) ASSERT_TRUE(Vcm::ComputeMisWeight(fixture.scene, path, strategy, &weight));
			Close(weight, manual[variant][index++]);
		}
	}
	const Vcm::PathContext path(light, eye, { Technique::Merging, 2, 2 }, radius, 16);
	const double raw[] = { 1. / 16, 4. / 16, 2. / 16, 1. / 64 };
	for (int i = 0; i < 4; ++i) {
		Vcm::StrategyPdf pdf;
		ASSERT_TRUE(Vcm::ComputePathPdf(fixture.scene, path,
			i == 3 ? Strategy{Technique::Merging, 2, 2} : Strategy{Technique::Connection, i, 3 - i}, &pdf));
		// exp(sum(log(pdf))) incurs a few double ULPs even for binary fractions.
		EXPECT_NEAR(std::exp(pdf.logPdf), raw[i], 1e-14 * raw[i]);
	}
}

TEST(VCMPath, RegularAndCoincidentMatrixIncludesAllEndpointFamilies) {
	for (bool directional : { false, true }) {
		for (int n : { 3, 4, 7 }) {
			Fixture fixture(n, false, directional);
			for (Technique kind : { Technique::Connection, Technique::Merging }) {
				for (int s = kind == Technique::Connection ? 0 : 2; s < n; ++s) {
					std::vector<PathVertex> light, eye;
					fixture.Split(s, kind == Technique::Merging, light, eye);
					const Strategy selected{ kind, s, int(eye.size()) };
					for (Mode mode : { Mode::Combined, Mode::ConnectionOnly, Mode::MergingOnly }) {
						const Vcm::PathContext path(light, eye, selected, 0.23, 16, mode);
						Compare(std::string(directional ? "directional-" : "diffuse-") + std::to_string(n),
							fixture.scene, path, fixture.direct);
						if (mode == Mode::ConnectionOnly && kind == Technique::Connection) {
							real oldWeight, weight;
							ASSERT_TRUE(MisWeight(fixture.scene, light, eye, s, int(eye.size()), &oldWeight));
							ASSERT_TRUE(Vcm::ComputeMisWeight(fixture.scene, path, selected, &weight));
							Close(weight, oldWeight);
						}
					}
				}
			}
		}
	}
}

TEST(VCMPath, DepthZeroEmitterHitSurvivesEveryMode) {
	Fixture fixture(2);
	std::vector<PathVertex> light, eye;
	fixture.Split(0, false, light, eye);
	for (Mode mode : { Mode::Combined, Mode::ConnectionOnly, Mode::MergingOnly }) {
		const Vcm::PathContext path(light, eye, { Technique::Connection, 0, 2 }, 0.2, 16, mode);
		Compare("direct-emitter", fixture.scene, path, fixture.direct);
	}
}

TEST(VCMPath, CameraHitSurfaceEmitterUsesItsLightEndpointMeasure) {
	Fixture fixture;
	std::vector<PathVertex> light, eye;
	fixture.Split(0, false, light, eye);
	eye.back().mType = PathVertexType::Surface;
	Compare("surface-emitter-hit", fixture.scene,
		Vcm::PathContext(light, eye, { Technique::Connection, 0, 3 }, 0.2, 16), fixture.direct);
}

TEST(VCMPath, GeneratedContinuousPathsMatchConnectionWeights) {
	class Samples : public StateSequence {
	public:
		explicit Samples(uint64 seed) : rng(seed) {}
		real Sample() override { ++mCursor; return rng.UniformFloat(); }
	private:
		Pcg32 rng;
	};
	Scene scene{};
	auto material = std::make_shared<DiffuseMaterial>(std::make_shared<ConstantTexture<Vec3>>(Vec3(0.8f)));
	for (int axis = 0; axis < 3; ++axis) {
		for (int sign : { -1, 1 }) {
			Vec3 center(0), u(0), v(0), normal(0);
			center[axis] = real(2 * sign);
			normal[axis] = real(-sign);
			u[(axis + 1) % 3] = 2;
			v[(axis + 2) % 3] = 2;
			for (int triangle = 0; triangle < 2; ++triangle) {
				auto shape = std::make_shared<Triangle>(Transform(), Transform(), center - u - v,
					triangle == 0 ? center + u - v : center + u + v,
					triangle == 0 ? center + u + v : center - u + v, normal);
				std::shared_ptr<AreaLight> emitter;
				if (axis == 2 && sign == 1) {
					emitter = std::make_shared<AreaLight>(shape, Vec3(2));
					scene.AddLight(emitter);
				}
				scene.AddPrimitive(std::make_shared<Primitive>(shape, material, emitter));
			}
		}
	}
	scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
	scene.Initialize();
	PinHoleCamera camera(std::make_shared<Film>(4, 4), Vec3(0), Vec3(0, 0, 1), Vec3(0, 1, 0), 120, 1);
	constexpr int pairs = 16, maxDepth = 5;
	std::array<int, 4> families{}; // emitter hit, light origin, camera splat, interior connection
	std::array<int, maxDepth + 1> depths{};
	std::array<bool, 4> checkedSuffix{};
	int candidates = 0, positive = 0, offsetVertices = 0, reconstructedDirectionsDiffer = 0;
	double maxBdptDifference = 0, maxDirectionDifference = 0;
	double maxReverseOriginalDifference = 0, maxReverseGeometricDifference = 0;
	double maxReverseOriginalRelative = 0, maxReverseGeometricRelative = 0;
	for (int sample = 0; sample < pairs; ++sample) {
		SCOPED_TRACE(sample);
		MemoryPool lightArena, eyeArena;
		Samples lightSamples(0x1234u + sample), eyeSamples(0x5678u + sample);
		std::vector<PathVertex> light, eye;
		const Ray ray = camera.GenerateRay(sample % 4, sample / 4, Vec2(0.37f, 0.61f), 0);
		ASSERT_EQ(GenerateLightPath(scene, lightSamples, lightArena, maxDepth + 1, light), maxDepth + 1);
		ASSERT_EQ(GenerateCameraPath(scene, camera, eyeSamples, eyeArena, ray, maxDepth + 2, eye), maxDepth + 2);
		const auto originalLight = light, originalEye = eye;
		for (const auto* subpath : { &light, &eye }) {
			for (size_t i = 1; i < subpath->size(); ++i) {
				const auto& vertex = (*subpath)[i];
				if (vertex.mIsect.mPointError.Length2() > 0) ++offsetVertices;
				const Vec3 reconstructed = ((*subpath)[i - 1].mIsect.mPos - vertex.mIsect.mPos).Norm();
				const double difference = (reconstructed - vertex.mIsect.mOutDir).Length();
				if (difference > 0) ++reconstructedDirectionsDiffer;
				maxDirectionDifference = std::max(maxDirectionDifference, difference);
			}
			for (int i = 0; i + 2 < int(subpath->size()); ++i) {
				const auto& source = (*subpath)[i + 1];
				const double original = ReverseEventPdf(*subpath, i);
				const double reconstructed = DirectionPdf(source, &(*subpath)[i + 2], (*subpath)[i]) * Jacobian(source, (*subpath)[i]);
				const double cached = (*subpath)[i].mPdfPrev;
				const double difference = std::abs(original - cached);
				EXPECT_TRUE(difference <= 1e-6 || difference <= 1e-5 * std::abs(cached));
				maxReverseOriginalDifference = std::max(maxReverseOriginalDifference, difference);
				maxReverseGeometricDifference = std::max(maxReverseGeometricDifference, std::abs(reconstructed - cached));
				if (cached > 0) {
					maxReverseOriginalRelative = std::max(maxReverseOriginalRelative, difference / cached);
					maxReverseGeometricRelative = std::max(maxReverseGeometricRelative, std::abs(reconstructed - cached) / cached);
				}
			}
		}
		for (int s = 0; s <= int(light.size()); ++s) {
			for (int t = 1; t <= int(eye.size()); ++t) {
				const int depth = s + t - 2;
				if (depth < 0 || depth > maxDepth || (s == 1 && t == 1)) continue;
				SCOPED_TRACE(testing::Message() << "generated split " << s << "," << t);
				++candidates;
				Vec3 primary;
				Vec2 raster;
				ASSERT_TRUE(EvaluateConnection(scene, camera, light, eye, s, t, &primary, &raster));
				if (primary == Vec3(0)) continue;
				ASSERT_GT(primary.x, 0);
				const int family = s == 0 ? 0 : s == 1 ? 1 : t == 1 ? 2 : 3;
				++positive;
				++families[family];
				++depths[depth];
				const Vcm::PathContext path(light, eye, { Technique::Connection, s, t }, 0.2, pairs, Mode::ConnectionOnly);
				real weight, bdptWeight;
				ASSERT_TRUE(Vcm::ComputeMisWeight(scene, path, path.selected, &weight));
				ASSERT_TRUE(MisWeight(scene, light, eye, s, t, &bdptWeight));
				Close(weight, bdptWeight);
				maxBdptDifference = std::max(maxBdptDifference, std::abs(double(weight) - bdptWeight));
				Compare("generated-" + std::to_string(sample), scene, path, ReportEdges(scene, path));
				if (!checkedSuffix[family] && s < int(light.size()) && t < int(eye.size())) {
					auto extendedLight = light, extendedEye = eye;
					for (auto pair : { std::make_pair(&extendedLight, s), std::make_pair(&extendedEye, t) }) {
						for (size_t i = pair.second; i < pair.first->size(); ++i) {
							(*pair.first)[i].mType = PathVertexType::Environment;
							(*pair.first)[i].mPdfFwd = std::numeric_limits<real>::quiet_NaN();
							(*pair.first)[i].mIsDelta = true;
						}
					}
					real repeated;
					ASSERT_TRUE(Vcm::ComputeMisWeight(scene,
						Vcm::PathContext(extendedLight, extendedEye, path.selected, 0.2, pairs, Mode::ConnectionOnly),
						path.selected, &repeated));
					EXPECT_EQ(repeated, weight);
					checkedSuffix[family] = true;
				}
			}
		}
		SamePaths(light, originalLight);
		SamePaths(eye, originalEye);
	}
	EXPECT_GE(positive, 100);
	EXPECT_GE(offsetVertices, 100);
	EXPECT_GE(reconstructedDirectionsDiffer, 100);
	for (size_t i = 0; i < families.size(); ++i) {
		EXPECT_GE(families[i], 4) << "Missing meaningful positive endpoint-family coverage: " << i;
		EXPECT_TRUE(checkedSuffix[i]) << "Missing generated unused-suffix check: " << i;
		RecordProperty("positive_family_" + std::to_string(i), families[i]);
	}
	for (size_t i = 0; i < depths.size(); ++i) {
		EXPECT_GE(depths[i], 4) << "Missing positive depth coverage: " << i;
		RecordProperty("positive_depth_" + std::to_string(i), depths[i]);
	}
	RecordProperty("generated_pairs", pairs);
	RecordProperty("candidate_connections", candidates);
	RecordProperty("positive_connections", positive);
	RecordProperty("vertices_with_point_error", offsetVertices);
	RecordProperty("original_directions_differing_from_reconstruction", reconstructedDirectionsDiffer);
	for (const auto& metric : { std::make_pair("max_bdpt_weight_difference", maxBdptDifference),
		std::make_pair("max_original_direction_difference", maxDirectionDifference),
		std::make_pair("max_cached_reverse_vs_original_difference", maxReverseOriginalDifference),
		std::make_pair("max_cached_reverse_vs_geometric_difference", maxReverseGeometricDifference),
		std::make_pair("max_cached_reverse_vs_original_relative", maxReverseOriginalRelative),
		std::make_pair("max_cached_reverse_vs_geometric_relative", maxReverseGeometricRelative) }) {
		std::ostringstream value;
		value << std::setprecision(17) << metric.second;
		RecordProperty(metric.first, value.str());
	}
}

TEST(VCMPath, PositiveCountsAndRadiusAreSharedByEveryAlternative) {
	Fixture fixture(5, false, true);
	std::vector<PathVertex> light, eye;
	fixture.Split(3, true, light, eye);
	for (int64 count : { int64(1), int64(16), int64(1024), std::numeric_limits<int64>::max() }) {
		for (double radius : { 1e-8, 0.2, 1e8 }) {
			for (Mode mode : { Mode::Combined, Mode::ConnectionOnly, Mode::MergingOnly }) {
				Compare("count-radius", fixture.scene,
					Vcm::PathContext(light, eye, { Technique::Merging, 3, 3 }, radius, count, mode), fixture.direct);
			}
		}
	}
}

TEST(VCMPath, UnequalSelectionIsIncludedExactlyOnceWithOriginReuse) {
	Fixture fixture;
	fixture.scene.AddLight(std::make_shared<TestLight>(fixture.light->GetShapePtr(), 3));
	fixture.scene.Initialize();
	fixture.Refresh();
	EXPECT_NEAR(fixture.direct.f[0], 0.125, 1e-7);
	for (int s : { 0, 1, 2 }) {
		std::vector<PathVertex> light, eye;
		fixture.Split(s, false, light, eye);
		Compare("unequal-light-selection", fixture.scene,
			Vcm::PathContext(light, eye, { Technique::Connection, s, 3 - s }, 0.1, 16), fixture.direct);
	}
}

TEST(VCMPath, SeparatedMergeUsesCameraScatteringAndOriginalSideJacobians) {
	for (int n : { 3, 5, 7 }) {
			for (int s = 2; s < n; ++s) {
				Fixture fixture(n, false, true);
				std::vector<PathVertex> light, eye;
				fixture.Split(s, true, light, eye);
				const int j = s - 1;
				auto& y = light.back();
				const auto& x = eye.back();
				const auto& a = light[s - 2];
				const auto& b = eye[eye.size() - 2];
				y.mIsect.mPos += Vec3(0.17f, -0.11f, 0.07f);
				y.mIsect.mNormal = y.mIsect.mGeometryNormal = Vec3(0.2f, 0, std::sqrt(0.96f));
				CoordinateSystem(y.mIsect.mNormal, &y.mIsect.mShadingDpDu, &y.mIsect.mShadingDpDv);
				y.mIsect.mBsdf = GYT_MEMORY_POOL_ALLOC(fixture.arena, BSDF)(y.mIsect);
				y.mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(fixture.arena, DirectionalLobe)(-0.6f));
				y.mIsect.mOutDir = (a.mIsect.mPos - y.mIsect.mPos).Norm();
				y.mPdfFwd = real(DirectionPdf(a, s > 2 ? &light[s - 3] : nullptr, y) * Jacobian(a, y));

				// Hand mapping of the affected Eq. (10) factors. All other sampled
				// events stay unchanged. Adjacent VC cuts are (a,x) and (y,b).
				Edges direct = fixture.direct;
				direct.f[j] = y.mPdfFwd;
				direct.r[j - 1] = x.mIsect.mBsdf->Pdf(x.mIsect.mOutDir, y.mIsect.mOutDir) * Jacobian(y, a);
				direct.f[j + 1] = x.mIsect.mBsdf->Pdf(y.mIsect.mOutDir, x.mIsect.mOutDir) * Jacobian(x, b);
				if (j >= 2) {
					direct.r[j - 2] = DirectionPdf(a, &y, light[j - 2]) * Jacobian(a, light[j - 2]);
					light[j - 2].mPdfPrev = real(direct.r[j - 2]);
				}
				const Vcm::PathContext path(light, eye, { Technique::Merging, s, int(eye.size()) }, 0.35, 16);
				Compare("separated-directional-" + std::to_string(n), fixture.scene, path, direct);

				for (int cut : { j, j + 1 }) {
					double manual = 1;
					for (int i = 0; i < cut; ++i) manual *= direct.f[i];
					for (int i = cut; i < n; ++i) manual *= direct.r[i];
					Vcm::StrategyPdf pdf;
					ASSERT_TRUE(Vcm::ComputePathPdf(fixture.scene, path, { Technique::Connection, cut, n - cut }, &pdf));
					EXPECT_NEAR(std::exp(pdf.logPdf), manual, manual * 1e-5);
				}
				// Both common "move y to x" mistakes must be numerically observable.
				EXPECT_GT(std::abs(Jacobian(y, a) - Jacobian(x, a)), 1e-4);
				EXPECT_GT(std::abs(Jacobian(x, b) - Jacobian(y, b)), 1e-4);
				const double correct = x.mIsect.mBsdf->Pdf(x.mIsect.mOutDir, y.mIsect.mOutDir);
				const double moved = x.mIsect.mBsdf->Pdf(x.mIsect.mOutDir, (a.mIsect.mPos - x.mIsect.mPos).Norm());
				const double photonBsdf = y.mIsect.mBsdf->Pdf(x.mIsect.mOutDir, y.mIsect.mOutDir);
				EXPECT_GT(std::abs(correct - moved), 1e-5);
				EXPECT_GT(std::abs(correct - photonBsdf), 1e-5);
			}
	}
}

TEST(VCMPath, RealGlossyReflectionAndTransmissionUseContinuousDensities) {
	for (bool transmission : { false, true }) {
			Fixture fixture(3);
			auto& surface = fixture.full[1];
			surface.mIsect.mBsdf = GYT_MEMORY_POOL_ALLOC(fixture.arena, BSDF)(surface.mIsect);
			auto distribution = GYT_MEMORY_POOL_ALLOC(fixture.arena, GGXDistribution)(0.3f, 0.5f, true);
			if (transmission) {
				fixture.full[0].mIsect.mPos.z = -2;
				fixture.full[0].mIsect.mNormal = fixture.full[0].mIsect.mGeometryNormal = Vec3(0, 0, 1);
				surface.mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(fixture.arena, MicrofacetTransmissionBSDF)(
					distribution, Vec3(1), 1.f, 1.5f, TransportMode::Radiance));
			}
			else {
				auto fresnel = GYT_MEMORY_POOL_ALLOC(fixture.arena, FresnelConductor)(Vec3(1), Vec3(0.2f), Vec3(3));
				surface.mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(fixture.arena, MicrofacetReflectionBSDF)(
					distribution, fresnel, Vec3(1)));
			}
			fixture.Refresh();
			for (int s : { 0, 1, 2 }) {
				std::vector<PathVertex> light, eye;
				fixture.Split(s, false, light, eye);
				Compare(transmission ? "rough-transmission" : "rough-reflection", fixture.scene,
					Vcm::PathContext(light, eye, { Technique::Connection, s, 3 - s }, 0.2, 16), fixture.direct);
			}
	}
}

TEST(VCMPath, LongTinyDensityProductsDoNotUnderflow) {
	Fixture fixture(64, true);
	const real tiny = 1e-30f;
	fixture.light->pdfDir = fixture.camera->pdf = tiny;
	for (size_t i = 1; i + 1 < fixture.full.size(); ++i) {
			auto& v = fixture.full[i];
			v.mIsect.mBsdf = GYT_MEMORY_POOL_ALLOC(fixture.arena, BSDF)(v.mIsect);
			v.mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(fixture.arena, FixedLobe)(tiny));
	}
	fixture.Refresh();
	double naive = 1;
	for (double pdf : fixture.direct.f) naive *= pdf;
	EXPECT_EQ(naive, 0);
	for (bool merge : { false, true }) {
			std::vector<PathVertex> light, eye;
			fixture.Split(32, merge, light, eye);
			for (Mode mode : { Mode::Combined, Mode::ConnectionOnly, Mode::MergingOnly }) {
				const Vcm::PathContext path(light, eye, { merge ? Technique::Merging : Technique::Connection, 32, int(eye.size()) },
					std::sqrt(1e30 / (16 * Pi)), 16, mode);
				Compare("tiny-64", fixture.scene, path, fixture.direct);
				Vcm::StrategyPdf pdf;
				ASSERT_TRUE(Vcm::ComputePathPdf(fixture.scene, path, path.selected, &pdf));
				EXPECT_EQ(pdf.feasible, mode == Mode::Combined || mode == (merge ? Mode::MergingOnly : Mode::ConnectionOnly));
				if (pdf.feasible) {
					EXPECT_LT(pdf.logPdf, -1000);
					EXPECT_EQ(std::exp(pdf.logPdf), 0);
				}
			}
	}
}

TEST(VCMPath, CommonScoreScaleLeavesWeightsUnchanged) {
	for (real scale : { 1e-25f, 1.f, 1e20f }) {
			Fixture fixture(3, true);
			fixture.light->pdfPos *= scale;
			auto& v = fixture.full[1];
			v.mIsect.mBsdf = GYT_MEMORY_POOL_ALLOC(fixture.arena, BSDF)(v.mIsect);
			v.mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(fixture.arena, FixedLobe)(0.125f * scale));
			fixture.Refresh();
			std::vector<PathVertex> light, eye;
			fixture.Split(2, true, light, eye);
			const Vcm::PathContext path(light, eye, { Technique::Merging, 2, 2 }, std::sqrt(0.25 / Pi), 16);
			Compare("common-scale-" + std::to_string(scale), fixture.scene, path, fixture.direct);
			real weight;
			ASSERT_TRUE(Vcm::ComputeMisWeight(fixture.scene, path, path.selected, &weight));
			Close(weight, 4. / 11);
	}
}

TEST(VCMPath, UnselectedSuffixesNeverAffectValidationOrProbabilities) {
	for (bool merge : { false, true }) {
			Fixture fixture;
			std::vector<PathVertex> light, eye;
			fixture.Split(merge ? 2 : 1, merge, light, eye);
			const Strategy selected{ merge ? Technique::Merging : Technique::Connection, int(light.size()), int(eye.size()) };
			PathVertex unused;
			unused.mType = PathVertexType::Environment;
			unused.mPdfFwd = std::numeric_limits<real>::quiet_NaN();
			unused.mIsDelta = true;
			light.push_back(unused);
			eye.push_back(unused);
			Compare("unused-suffix", fixture.scene, Vcm::PathContext(light, eye, selected, 0.2, 16), fixture.direct);
	}
	Fixture fixture;
	std::vector<PathVertex> light, eye;
	fixture.Split(0, false, light, eye);
	light.push_back(PathVertex());
	light.back().mPdfFwd = -1;
	Compare("unused-entire-light-path", fixture.scene,
			Vcm::PathContext(light, eye, { Technique::Connection, 0, 3 }, 0.2, 16), fixture.direct);
}

TEST(VCMPath, SampledForwardEventsArePreservedAndJoinAdjacentReverseCachesIgnored) {
	Fixture fixture(5);
	std::vector<PathVertex> light, eye;
	fixture.Split(3, false, light, eye);
	Edges direct = fixture.direct;
	light[2].mPdfFwd *= 0.7f;
	direct.f[2] = light[2].mPdfFwd;
	eye[1].mPdfFwd *= 0.6f;
	direct.r[3] = eye[1].mPdfFwd;
	for (size_t i = light.size() - 2; i < light.size(); ++i) light[i].mPdfPrev = 123;
	for (size_t i = eye.size() - 2; i < eye.size(); ++i) eye[i].mPdfPrev = 456;
	Compare("sampled-events", fixture.scene,
			Vcm::PathContext(light, eye, { Technique::Connection, 3, 2 }, 0.2, 16), direct);
}

TEST(VCMPath, ZeroSupportIsNotAnErrorUntilRequestedAsCurrent) {
	Fixture fixture(3, true);
	std::vector<PathVertex> light, eye;
	fixture.Split(1, false, light, eye);
	eye.back().mPdfFwd = 0;
	const auto beforeLight = light, beforeEye = eye;
	const Vcm::PathContext path(light, eye, { Technique::Connection, 1, 2 }, 0.2, 16);
	for (Strategy strategy : { Strategy{Technique::Connection, 0, 3}, {Technique::Connection, 1, 2}, {Technique::Merging, 2, 2} }) {
			Vcm::StrategyPdf pdf{ true, 17 };
			ASSERT_TRUE(Vcm::ComputePathPdf(fixture.scene, path, strategy, &pdf));
			EXPECT_FALSE(pdf.feasible);
			real weight = 7;
			EXPECT_FALSE(Vcm::ComputeMisWeight(fixture.scene, path, strategy, &weight));
			EXPECT_EQ(weight, 0);
	}
	real weight;
	ASSERT_TRUE(Vcm::ComputeMisWeight(fixture.scene, path, { Technique::Connection, 2, 1 }, &weight));
	EXPECT_EQ(weight, 1);
	SamePaths(light, beforeLight);
	SamePaths(eye, beforeEye);
	eye.back().mPdfPrev = -1; // A legitimate zero must not conceal invalid input elsewhere.
	Vcm::StrategyPdf pdf;
	EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, path, path.selected, &pdf));
}

TEST(VCMPath, ZeroScatteringPdfStaysZeroRatherThanBecomingADeltaMarker) {
	Fixture fixture(3, true);
	auto& v = fixture.full[1];
	v.mIsect.mBsdf = GYT_MEMORY_POOL_ALLOC(fixture.arena, BSDF)(v.mIsect);
	v.mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(fixture.arena, FixedLobe)(0));
	fixture.Refresh();
	std::vector<PathVertex> light, eye;
	fixture.Split(1, false, light, eye);
	const Vcm::PathContext path(light, eye, { Technique::Connection, 1, 2 }, 0.2, 16);
	Vcm::StrategyPdf pdf;
	ASSERT_TRUE(Vcm::ComputePathPdf(fixture.scene, path, { Technique::Connection, 0, 3 }, &pdf));
	EXPECT_FALSE(pdf.feasible);
	real weight;
	ASSERT_TRUE(Vcm::ComputeMisWeight(fixture.scene, path, path.selected, &weight));
	const auto direct = VcmReference::DirectWeights(fixture.direct, Pi * 0.04, 16, true, true);
	Close(weight, direct[1]);
}

TEST(VCMPath, SingularSelectedPrefixesAreExplicitlyUnsupported) {
	for (int variant = 0; variant < 5; ++variant) {
			Fixture fixture(5);
			std::vector<PathVertex> light, eye;
			fixture.Split(3, true, light, eye);
			if (variant == 0) light[1].mIsDelta = true;
			else if (variant == 1) {
				MirrorMaterial(std::make_shared<ConstantTexture<Vec3>>(Vec3(1))).ComputeScatteringFunction(&light.back().mIsect, fixture.arena);
				EXPECT_FALSE(light.back().mIsDelta); // Terminal delta vertices have not sampled a lobe.
			}
			else if (variant == 2) {
				GlassMaterial(std::make_shared<ConstantTexture<Vec3>>(Vec3(1)),
					std::make_shared<ConstantTexture<Vec3>>(Vec3(1))).ComputeScatteringFunction(&eye.back().mIsect, fixture.arena);
			}
			else if (variant == 3) light[1].mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(fixture.arena, SpecularBSDF)(Vec3(1)));
			else eye[1].mType = PathVertexType::Environment;
			const auto beforeLight = light, beforeEye = eye;
			const Vcm::PathContext path(light, eye, { Technique::Merging, 3, 3 }, 0.2, 16);
			Vcm::StrategyPdf pdf;
			real weight = 7;
			EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, path, path.selected, &pdf));
			EXPECT_FALSE(pdf.feasible);
			EXPECT_FALSE(Vcm::ComputeMisWeight(fixture.scene, path, path.selected, &weight));
			EXPECT_EQ(weight, 0);
			SamePaths(light, beforeLight);
			SamePaths(eye, beforeEye);
	}
}

TEST(VCMPath, PointAndEnvironmentOriginsAreRejectedWithoutChangingBDPT) {
	Fixture fixture;
	std::vector<PathVertex> light, eye;
	fixture.Split(1, false, light, eye);
	PointLight point(Vec3(0, 0, 2), Vec3(1));
	for (bool environment : { false, true }) {
			light[0].mLight = environment ? static_cast<const Light*>(fixture.light.get()) : &point;
			light[0].mType = environment ? PathVertexType::Environment : PathVertexType::Light;
			const Vcm::PathContext path(light, eye, { Technique::Connection, 1, 2 }, 0.2, 16);
			Vcm::StrategyPdf pdf;
			EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, path, path.selected, &pdf));
	}
}

TEST(VCMPath, InvalidCountsModesRadiusAndCurrentMasksFailChecked) {
	Fixture fixture;
	std::vector<PathVertex> light, eye;
	fixture.Split(2, true, light, eye);
	const Strategy selected{ Technique::Merging, 2, 2 };
	for (double radius : { 0., -1., std::numeric_limits<double>::infinity(),
			std::numeric_limits<double>::quiet_NaN(), 1e-300, 1e300 }) {
			Vcm::StrategyPdf pdf;
			EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, Vcm::PathContext(light, eye, selected, radius, 16), selected, &pdf));
	}
	for (int64 count : { int64(0), int64(-1) }) {
			Vcm::StrategyPdf pdf;
			EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, Vcm::PathContext(light, eye, selected, 0.2, count), selected, &pdf));
	}
	for (Strategy invalid : { Strategy{Technique::Connection, -1, 3}, {Technique::Connection, 1, 1},
			{Technique::Connection, 2, 0}, {Technique::Merging, 1, 2}, {Technique::Merging, 3, 2},
			{Technique::Merging, std::numeric_limits<int>::max(), std::numeric_limits<int>::max()},
			{static_cast<Technique>(100), 2, 2} }) {
			Vcm::StrategyPdf pdf;
			EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, Vcm::PathContext(light, eye, invalid, 0.2, 16), selected, &pdf));
	}
	Vcm::StrategyPdf pdf;
	EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, Vcm::PathContext(light, eye, selected, 0.2, 16, static_cast<Mode>(100)), selected, &pdf));
	const Vcm::PathContext path(light, eye, selected, 0.2, 16);
	EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, path, { Technique::Connection, 0, 4 }, &pdf));
	EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, path, { Technique::Connection, -1, 4 }, &pdf));
	for (Strategy excluded : { Strategy{Technique::Connection, 3, 0}, {Technique::Merging, 1, 3}, {Technique::Merging, 3, 1} }) {
			ASSERT_TRUE(Vcm::ComputePathPdf(fixture.scene, path, excluded, &pdf));
			EXPECT_FALSE(pdf.feasible);
	}
	real weight = 7;
	EXPECT_FALSE(Vcm::ComputeMisWeight(fixture.scene,
			Vcm::PathContext(light, eye, selected, 0.2, 16, Mode::ConnectionOnly), selected, &weight));
	EXPECT_EQ(weight, 0);
	EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, path, selected, nullptr));
	EXPECT_FALSE(Vcm::ComputeMisWeight(fixture.scene, path, selected, nullptr));
}

TEST(VCMPath, InvalidDensitiesAndGeometryLeaveInputsUnchanged) {
	for (int variant = 0; variant < 12; ++variant) {
			Fixture fixture;
			std::vector<PathVertex> light, eye;
			fixture.Split(2, true, light, eye);
			if (variant == 0) light.back().mPdfFwd = -1;
			if (variant == 1) eye.back().mPdfPrev = std::numeric_limits<real>::quiet_NaN();
			if (variant == 2) light[0].mPdfPos = std::numeric_limits<real>::infinity();
			if (variant == 3) eye.back().mIsect.mPos.x = std::numeric_limits<real>::infinity();
			if (variant == 4) light.back().mIsect.mOutDir = Vec3(0);
			if (variant == 5) eye.back().mIsect.mGeometryNormal = Vec3(0);
			if (variant == 6) light[0].mIsect.mPos = light[1].mIsect.mPos;
			if (variant == 7) light.back().mIsect.mPos.x += 1;
			if (variant == 8) eye[0].mCamera = nullptr;
			if (variant == 9) eye.back().mIsect.mBsdf = nullptr;
			if (variant == 10) eye.back().mIsect.mNormal = Vec3(0, 1, 0);
			if (variant == 11) eye[0].mPdfDir = -1;
			const auto beforeLight = light, beforeEye = eye;
			const Vcm::PathContext path(light, eye, { Technique::Merging, 2, 2 }, 0.2, 16);
			Vcm::StrategyPdf pdf{ true, 17 };
			EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, path, path.selected, &pdf));
			EXPECT_FALSE(pdf.feasible);
			real weight = 7;
			EXPECT_FALSE(Vcm::ComputeMisWeight(fixture.scene, path, path.selected, &weight));
			EXPECT_EQ(weight, 0);
			SamePaths(light, beforeLight);
			SamePaths(eye, beforeEye);
	}
}

TEST(VCMPath, EvaluatedNegativeAndNonfiniteScatteringDensitiesFail) {
	for (real value : { -1.f, std::numeric_limits<real>::infinity(), std::numeric_limits<real>::quiet_NaN() }) {
			Fixture fixture;
			std::vector<PathVertex> light, eye;
			fixture.Split(1, false, light, eye);
			auto& v = eye.back();
			v.mIsect.mBsdf = GYT_MEMORY_POOL_ALLOC(fixture.arena, BSDF)(v.mIsect);
			v.mIsect.mBsdf->Add(GYT_MEMORY_POOL_ALLOC(fixture.arena, FixedLobe)(value));
			const Vcm::PathContext path(light, eye, { Technique::Connection, 1, 2 }, 0.2, 16);
			Vcm::StrategyPdf pdf;
			EXPECT_FALSE(Vcm::ComputePathPdf(fixture.scene, path, { Technique::Connection, 0, 3 }, &pdf));
			real weight = 7;
			EXPECT_FALSE(Vcm::ComputeMisWeight(fixture.scene, path, path.selected, &weight));
			EXPECT_EQ(weight, 0);
	}
}
