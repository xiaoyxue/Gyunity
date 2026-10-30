#include "stb/stb_image.h"
#include "gtest/gtest.h"
#include "integrator/VCM.h"
#include "integrator/BDPT.h"
#include "accelerator/BruteForceAccel.h"
#include "camera/PinholeCamera.h"
#include "light/AreaLight.h"
#include "light/EnvLight.h"
#include "light/PointLight.h"
#include "material/Diffuse.h"
#include "material/Glass.h"
#include "material/Mirror.h"
#include "texture/ConstantTexture.h"
#include "volume/Homogeneous.h"
#include "image/ImageIO.h"
#include "sampler/Halton.h"
#include "sampler/Sobol.h"
#include "tbb/task_arena.h"
#include <array>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <map>
#include <mutex>
#include <regex>
#include <set>
#include <sstream>
#include <tuple>

using namespace Gyunity;

namespace {

using Vcm::Mode;
using Renderer = VertexConnectionMerging;
const std::array<Mode, 3> Modes{ Mode::Combined, Mode::ConnectionOnly, Mode::MergingOnly };
const double Pi = std::acos(-1.0);

std::shared_ptr<ConstantTexture<Vec3>> Constant(real value) {
	return std::make_shared<ConstantTexture<Vec3>>(Vec3(value));
}
std::shared_ptr<Material> Diffuse(real value = 1) {
	return std::make_shared<DiffuseMaterial>(Constant(value));
}
std::shared_ptr<Triangle> Plane(real z, real size = 10, real x = 0, real normal = 1) {
	return std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(x - size, -size, z), Vec3(x + size, -size, z), Vec3(x, size, z), Vec3(0, 0, normal));
}
void Initialize(Scene& scene) {
	scene.SetAccelerator(std::make_shared<BruteForce>(scene.GetPrimitives()));
	scene.Initialize();
}
std::shared_ptr<PinHoleCamera> CameraFor(int w, int h, real fov = 53.130102f) {
	return std::make_shared<PinHoleCamera>(std::make_shared<Film>(w, h),
		Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), fov, 1);
}
class FixedSampler : public Sampler {
public:
	real Sample(unsigned int, unsigned long long) override { return 0.5f; }
	std::shared_ptr<Sampler> Clone(uint32) override { return std::make_shared<FixedSampler>(); }
};

TEST(VCMPathSampling, FullWidthPathKeysAreDeterministicAndDoNotChangeLegacySampling) {
	RandomSampler sampler(123), legacyReference(123);
	std::vector<real> previous;
	for (uint64 key : { uint64(7), (uint64(1) << 32) + 7, (uint64(1) << 48) + 7 }) {
		auto first = sampler.CloneForPath(key), second = sampler.CloneForPath(key);
		ASSERT_NE(first, nullptr);
		ASSERT_NE(second, nullptr);
		EXPECT_NE(first.get(), second.get());
		std::vector<real> values;
		for (unsigned d = 0; d < 16; ++d) {
			const real value = first->Sample(d, key);
			EXPECT_EQ(value, second->Sample(d, key));
			values.push_back(value);
		}
		if (!previous.empty()) EXPECT_NE(values, previous);
		previous = values;
	}
	for (unsigned d = 0; d < 16; ++d)
		EXPECT_EQ(sampler.Sample(d, 1), legacyReference.Sample(d, 1));
	for (uint32 seed : { 0u, 123u, 0xfedcba98u }) {
		auto clone = sampler.Clone(seed);
		RandomSampler reference(static_cast<int>(seed));
		for (unsigned d = 0; d < 16; ++d)
			EXPECT_EQ(clone->Sample(d, 0), reference.Sample(d, 0));
	}
	RandomSampler otherSeed(124);
	auto first = sampler.CloneForPath(7), other = otherSeed.CloneForPath(7);
	std::vector<real> firstValues, otherValues;
	for (unsigned d = 0; d < 16; ++d) {
		firstValues.push_back(first->Sample(d, 7));
		otherValues.push_back(other->Sample(d, 7));
	}
	EXPECT_NE(firstValues, otherValues);
	RandomSampler seed0(0), seed2(2);
	std::vector<real> batch0, batch2;
	for (uint64 key = 0; key < 16; ++key) {
		batch0.push_back(seed0.CloneForPath(key)->Sample(0, key));
		batch2.push_back(seed2.CloneForPath(key)->Sample(0, key));
	}
	std::sort(batch0.begin(), batch0.end());
	std::sort(batch2.begin(), batch2.end());
	EXPECT_NE(batch0, batch2) << "Changing the root seed must not merely permute the same path samples.";
}

TEST(VCMPathSampling, LegacyCustomClonesKeepTheirBehaviorAndRejectUnsupportedWideKeys) {
	class CustomRandom : public RandomSampler {
	public:
		explicit CustomRandom(int seed = 0) : RandomSampler(seed) {}
		real Sample(unsigned, unsigned long long) override { return 0.25f; }
		std::shared_ptr<Sampler> Clone(uint32 seed) override { return std::make_shared<CustomRandom>(int(seed)); }
	} custom;
	auto supported = custom.CloneForPath(17);
	ASSERT_NE(supported, nullptr);
	EXPECT_EQ(supported->Sample(0, 17), 0.25f);
	EXPECT_EQ(custom.CloneForPath(uint64(1) << 32), nullptr);
	FixedSampler legacy;
	EXPECT_NE(legacy.CloneForPath((std::numeric_limits<uint32>::max)()), nullptr);
	EXPECT_EQ(legacy.CloneForPath(uint64(1) << 32), nullptr);
}

class DownwardSampler : public FixedSampler {
public:
	real Sample(unsigned int d, unsigned long long i) override { return i % 2 == 0 && d == 3 ? 0.999f : 0.5f; }
	std::shared_ptr<Sampler> Clone(uint32) override { return std::make_shared<DownwardSampler>(); }
};

class FixedLight : public AreaLight {
public:
	FixedLight() : AreaLight(Plane(2, 1, 0, -1), Vec3(1)) {}
	Vec3 SampleLight(Intersection* hit, Vec3* dir, real* pos, real* pdf,
		const Vec2&, const Vec2&) const override {
		*hit = Intersection();
		hit->mPos = Vec3(0, 0, 2);
		hit->mNormal = hit->mGeometryNormal = Vec3(0, 0, -1);
		*dir = Vec3(0, 0, -1);
		*pos = 0.5f;
		*pdf = INV_PI;
		return Vec3(1);
	}
};

struct Event {
	uint32 seed;
	unsigned dimension;
	uint64 instance, ordinal;
	real value;
};
struct Recording {
	std::mutex mutex;
	std::vector<uint32> seeds;
	std::vector<Event> events;
	uint64 pixels = 1;
	bool violation = false, stateful = false;
	std::map<std::pair<uint64, int>, std::set<uint64>> starts;
	Renderer* cancel = nullptr;
	int cancelPhase = -1, failPhase = -1;
};
class RecordingSampler : public Sampler {
public:
	explicit RecordingSampler(std::shared_ptr<Recording> recording, uint32 seed = 0)
		: mRecording(recording), mSeed(seed), mRandom(seed) {}
	real Sample(unsigned d, unsigned long long i) override {
		auto& r = *mRecording;
		std::lock_guard<std::mutex> lock(r.mutex);
		const int phase = int(i % 2);
		const uint64 iteration = i / 2 / r.pixels, pixel = i / 2 % r.pixels;
		if (d == 0) {
			if (!r.starts[{ iteration, phase }].insert(pixel).second) r.violation = true;
			if (phase == 1 && r.starts[{ iteration, 0 }].size() != r.pixels) r.violation = true;
			if (iteration && phase == 0 && r.starts[{ iteration - 1, 1 }].size() != r.pixels) r.violation = true;
		}
		const real value = r.stateful ? mRandom.Sample(d, i) : 0.5f;
		r.events.push_back({ mSeed, d, i, mOrdinal++, value });
		if (r.cancel && r.cancelPhase == phase) r.cancel->RequestCancellation();
		return value;
	}
	std::shared_ptr<Sampler> Clone(uint32 seed) override {
		std::lock_guard<std::mutex> lock(mRecording->mutex);
		if (mRecording->failPhase == int(seed % 2)) return nullptr; // A path stream cannot be created.
		mRecording->seeds.push_back(seed);
		return std::make_shared<RecordingSampler>(mRecording, seed);
	}
private:
	std::shared_ptr<Recording> mRecording;
	uint32 mSeed;
	uint64 mOrdinal = 0;
	RandomSampler mRandom;
};

class VCMRenderer : public testing::Test {
protected:
	std::string output;
	int environments = 0;
	void SetUp() override {
		const char* directory = std::getenv("GYUNITY_TEST_OUTPUT_DIRECTORY");
		ASSERT_NE(directory, nullptr) << "Renderer acceptance must not skip artifact output.";
		output = directory;
	}
	std::vector<Vec3> Image(const Camera& camera, const std::string& name) {
		const auto film = camera.GetFilm();
		const std::string filename = output + "\\vcm-" + name + ".hdr";
		film->SetFileName(filename);
		const bool saved = film->SaveImage();
		EXPECT_TRUE(saved);
		if (!saved) return {};
		int width, height, channels;
		std::unique_ptr<float, decltype(&stbi_image_free)> pixels(
			stbi_loadf(filename.c_str(), &width, &height, &channels, 3), stbi_image_free);
		EXPECT_NE(pixels, nullptr);
		if (!pixels) return {};
		EXPECT_EQ(width, film->mResX);
		EXPECT_EQ(height, film->mResY);
		std::vector<Vec3> values;
		for (int i = 0; i < width * height; ++i)
			values.emplace_back(pixels.get()[3 * i], pixels.get()[3 * i + 1], pixels.get()[3 * i + 2]);
		for (const auto& v : values) for (int c = 0; c < 3; ++c) EXPECT_TRUE(std::isfinite(v[c]));
		return values;
	}
	double Mean(const std::vector<Vec3>& image) {
		double sum = 0;
		for (const auto& p : image) sum += p.x;
		return sum / image.size();
	}
	void Summary(const Renderer& render, const std::string& name, double mean) {
		const auto& s = render.GetStatistics();
		std::ofstream csv(output + "\\renderer-summary.csv", std::ios::app);
		ASSERT_TRUE(csv.good());
		csv << std::setprecision(17) << name << ',' << s.mIterations << ',' << s.mLightAttempts << ','
			<< s.mCameraSamples << ',' << s.mLastRadius << ',' << s.mPeakPhotons << ',' << mean << '\n';
		RecordProperty(name + "_mean", std::to_string(mean));
	}
	std::shared_ptr<EnvironmentLight> Environment(real value = 2, real rotation = 0) {
		const std::string filename = output + "\\vcm-constant-" +
			testing::UnitTest::GetInstance()->current_test_info()->name() + "-" + std::to_string(environments++) + ".hdr";
		EXPECT_TRUE(ImageIO::WriteImage(filename, std::vector<Vec3>(32, Vec3(value)), 8, 4));
		return std::make_shared<EnvironmentLight>(filename, rotation);
	}
};

TEST_F(VCMRenderer, BlackAndVisibleEmissionAllModesAndPartialImageSizes) {
	for (Mode mode : Modes) for (int iterations : { 1, 4 }) for (bool emission : { false, true }) {
		Scene scene{};
		auto shape = Plane(0, 100);
		auto emitter = std::make_shared<AreaLight>(shape, Vec3(3));
		if (emission) scene.AddLight(emitter);
		scene.AddPrimitive(std::make_shared<Primitive>(shape, Diffuse(0), emission ? emitter : nullptr));
		Initialize(scene);
		auto camera = CameraFor(33, 2);
		Renderer render(std::make_shared<FixedSampler>(), 0, iterations, 0.5f, real(2.0 / 3), mode, 1);
		ASSERT_TRUE(render.RenderChecked(scene, *camera));
		const auto& stats = render.GetStatistics();
		EXPECT_EQ(stats.mLightAttempts, 66 * iterations);
		EXPECT_EQ(stats.mCameraSamples, 66 * iterations);
		const std::string name = "emission-" + std::to_string(int(mode)) + "-" +
			std::to_string(iterations) + "-" + std::to_string(emission);
		for (auto pixel : Image(*camera, name)) EXPECT_EQ(pixel.x, emission ? 3 : 0);
	}
}

TEST_F(VCMRenderer, ExplicitFalseMatchesDefaultAndTrueUsesShadingNormals) {
	Scene scene{};
	// Interpolated shading normals differ from ng = +z; false must still use geometric transport.
	const Vec3 tilted(0.6f, 0, 0.8f);
	auto surface = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-10, -10, 0), Vec3(10, -10, 0), Vec3(0, 10, 0), tilted, tilted, tilted);
	scene.AddPrimitive(std::make_shared<Primitive>(surface, Diffuse(0.5f)));
	scene.AddLight(std::make_shared<FixedLight>());
	Initialize(scene);
	for (Mode mode : Modes) {
		auto defaultCamera = CameraFor(4, 4), falseCamera = CameraFor(4, 4), shadingCamera = CameraFor(4, 4);
		Renderer defaults(std::make_shared<FixedSampler>(), 2, 2, 1, real(2.0 / 3), mode, 1);
		Renderer geometric(std::make_shared<FixedSampler>(), 2, 2, 1, real(2.0 / 3), mode, 1, false);
		ASSERT_TRUE(defaults.RenderChecked(scene, *defaultCamera));
		ASSERT_TRUE(geometric.RenderChecked(scene, *falseCamera));
		const auto suffix = std::to_string(int(mode));
		EXPECT_EQ(Image(*defaultCamera, "normal-default-" + suffix), Image(*falseCamera, "normal-false-" + suffix));
		Renderer shading(std::make_shared<FixedSampler>(), 2, 2, 1, real(2.0 / 3), mode, 1, true);
		ASSERT_TRUE(shading.RenderChecked(scene, *shadingCamera));
		EXPECT_TRUE(shading.GetStatistics().mSucceeded);
		EXPECT_NE(Image(*shadingCamera, "normal-true-" + suffix), Image(*falseCamera, "normal-false-again-" + suffix));
	}
}

// Receiver whose interpolated normal is tilted 36.87 degrees toward +x, a small downward emitter
// and a wall at x = 3 that catches light scattered along the shading normal.
std::shared_ptr<Scene> TiltedShadingScene() {
	const Vec3 tilted(0.6f, 0, 0.8f);
	auto scene = std::make_shared<Scene>();
	scene->AddPrimitive(std::make_shared<Primitive>(std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-10, -10, 0), Vec3(10, -10, 0), Vec3(0, 10, 0), tilted, tilted, tilted), Diffuse(0.8f)));
	scene->AddPrimitive(std::make_shared<Primitive>(std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(3, -20, -1), Vec3(3, 20, -1), Vec3(3, 0, 30), Vec3(-1, 0, 0)), Diffuse(0.8f)));
	auto emitter = std::make_shared<Triangle>(Transform(), Transform(),
		Vec3(-1, -1, 2), Vec3(1, -1, 2), Vec3(0, 1, 2), Vec3(0, 0, -1));
	auto light = std::make_shared<AreaLight>(emitter, Vec3(1));
	scene->AddLight(light);
	scene->AddPrimitive(std::make_shared<Primitive>(emitter, Diffuse(0), light));
	Initialize(*scene);
	return scene;
}

TEST_F(VCMRenderer, ShadingNormalsConvergeToBdptInEveryMode) {
	// Eq. (13): without the merge factor MergingOnly would keep the geometric cosine at x.
	auto scene = TiltedShadingScene();
	const auto camera = [] {
		return std::make_shared<PinHoleCamera>(std::make_shared<Film>(32, 32),
			Vec3(0, 0, 1), Vec3(0), Vec3(0, 1, 0), 53.130102f, 1);
	};
	double reference[2];
	for (bool shading : { false, true }) {
		auto view = camera();
		BidirectionalPathTracing bdpt(std::make_shared<RandomSampler>(7), 3, 256, shading);
		ASSERT_TRUE(bdpt.RenderChecked(*scene, *view));
		reference[shading] = Mean(Image(*view, std::string("tilted-bdpt-") + (shading ? "shading" : "geometric")));
	}
	EXPECT_GT(std::abs(reference[1] / reference[0] - 1), 0.05) << "the scene must distinguish the normal modes";
	for (Mode mode : Modes) {
		auto view = camera();
		Renderer render(std::make_shared<RandomSampler>(11), 3, 256, real(0.25), real(0.75), mode, 1, true);
		ASSERT_TRUE(render.RenderChecked(*scene, *view));
		const std::string name = "tilted-vcm-" + std::to_string(int(mode));
		const double mean = Mean(Image(*view, name));
		Summary(render, name, mean);
		EXPECT_NEAR(mean / reference[1], 1, 0.03) << "mode " << int(mode) << ", bdpt " << reference[1];
	}
}

TEST_F(VCMRenderer, FrozenRadiusAndAttemptNormalizationAreUsedByActualGather) {
	// Deterministic repeated samples, not an unbiased scene-radiance oracle.
	// All P light attempts land inside every query disk. Sole VM weight = 1.
	// betaL = 2*pi, fE = .5/pi, hence each pixel = 1/(pi*R_i^2).
	for (bool point : { false, true }) for (int width : { 1, 33 }) for (int iterations : { 1, 2, 4 }) {
		Scene scene{};
		scene.AddPrimitive(std::make_shared<Primitive>(Plane(0), Diffuse(0.5f)));
		if (point) scene.AddLight(std::make_shared<PointLight>(Vec3(0, 0, 2), Vec3(2)));
		else scene.AddLight(std::make_shared<FixedLight>());
		Initialize(scene);
		auto camera = CameraFor(width, 2, 0.1f);
		std::shared_ptr<Sampler> sampler = point ? std::static_pointer_cast<Sampler>(std::make_shared<DownwardSampler>()) :
			std::static_pointer_cast<Sampler>(std::make_shared<FixedSampler>());
		Renderer render(sampler, 1, iterations, 1, real(2.0 / 3), Mode::MergingOnly, 1);
		ASSERT_TRUE(render.RenderChecked(scene, *camera));
		double expected = 0;
		for (int i = 0; i < iterations; ++i) {
			const real radius = real(std::pow(i + 1.0, -(1 - double(real(2.0 / 3))) / 2));
			// Point intensity 2 gives betaL=8*pi and pair primary=4.
			expected += (point ? 4 : 1) / (Pi * double(radius) * radius * iterations);
		}
		const auto& s = render.GetStatistics();
		const real last = real(std::pow(double(iterations), -(1 - double(real(2.0 / 3))) / 2));
		EXPECT_EQ(s.mLastRadius, last);
		EXPECT_EQ(s.mLightAttempts, width * 2 * iterations);
		EXPECT_EQ(s.mCameraSamples, width * 2 * iterations);
		EXPECT_EQ(s.mPeakPhotons, width * 2);
		const std::string name = "gather-" + std::to_string(point) + "-" + std::to_string(width) + "-" + std::to_string(iterations);
		const double mean = Mean(Image(*camera, name));
		EXPECT_NEAR(mean, expected, expected * 0.008); // RGBE storage quantization only.
		Summary(render, name, mean);
	}
}

TEST_F(VCMRenderer, LogicalPathStreamsDoNotReplayOrDependOnWorkerAssignment) {
	std::vector<std::tuple<uint64, unsigned, real>> referenceEvents;
	for (int concurrency : { 1, 4 }) {
		Scene scene{};
		auto shape = Plane(0, 100);
		auto emitter = std::make_shared<AreaLight>(shape, Vec3(1));
		scene.AddLight(emitter);
		scene.AddPrimitive(std::make_shared<Primitive>(shape, Diffuse(0), emitter));
		Initialize(scene);
		auto recording = std::make_shared<Recording>();
		recording->pixels = 33 * 34;
		recording->stateful = true;
		auto camera = CameraFor(33, 34);
		Renderer render(std::make_shared<RecordingSampler>(recording), 0, 4, 0.5f, real(2.0 / 3),
			Mode::Combined, concurrency);
		ASSERT_TRUE(render.RenderChecked(scene, *camera));
		EXPECT_FALSE(recording->violation);
		const uint32 streamCount = 2 * 33 * 34 * 4;
		ASSERT_EQ(recording->seeds.size(), streamCount);
		std::map<uint32, int> cloneCounts;
		for (uint32 seed : recording->seeds) ++cloneCounts[seed];
		ASSERT_EQ(cloneCounts.size(), streamCount);
		for (uint32 seed = 0; seed < streamCount; ++seed) EXPECT_EQ(cloneCounts[seed], 1);
		std::map<uint32, std::unique_ptr<RandomSampler>> reference;
		std::map<uint32, uint64> ordinals;
		std::set<std::pair<uint64, unsigned>> dimensions;
		std::vector<std::tuple<uint64, unsigned, real>> events;
		for (const auto& event : recording->events) {
			if (!reference.count(event.seed)) reference[event.seed] = std::make_unique<RandomSampler>(event.seed);
			EXPECT_EQ(event.ordinal, ordinals[event.seed]++);
			EXPECT_EQ(event.value, reference[event.seed]->Sample(event.dimension, event.instance));
			EXPECT_EQ(event.seed % 2, event.instance % 2);
			EXPECT_EQ(event.seed, event.instance);
			EXPECT_TRUE(dimensions.insert({ event.instance, event.dimension }).second);
			events.emplace_back(event.instance, event.dimension, event.value);
		}
		std::sort(events.begin(), events.end());
		if (concurrency == 1) referenceEvents = events;
		else EXPECT_EQ(events, referenceEvents);
		EXPECT_EQ(dimensions.size(), 33 * 34 * 4 * 7);
		EXPECT_EQ(recording->starts.size(), 8);
		for (const auto& phase : recording->starts) EXPECT_EQ(phase.second.size(), recording->pixels);
		EXPECT_EQ(render.GetStatistics().mLightAttempts, 33 * 34 * 4);
		EXPECT_EQ(render.GetStatistics().mCameraSamples, 33 * 34 * 4);
		for (auto pixel : Image(*camera, "streams-" + std::to_string(concurrency))) EXPECT_EQ(pixel.x, 1);
	}
}

TEST_F(VCMRenderer, IsolatedCameraSplatsArePlacedOnceAndNormalizedAcrossBudgets) {
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(0, 0.01f, 0.04f), Diffuse()));
	auto source = Plane(2, 0.01f, 0.04f, -1);
	auto emitter = std::make_shared<AreaLight>(source, Vec3(1000));
	scene.AddLight(emitter);
	scene.AddPrimitive(std::make_shared<Primitive>(source, Diffuse(0), emitter));
	Initialize(scene);
	for (Mode mode : { Mode::Combined, Mode::ConnectionOnly }) {
		double reference = 0;
		for (int width : { 16, 32, 33 }) for (int iterations : { 1, 4, 16 }) {
			auto camera = CameraFor(width, width);
			for (int y = 0; y < width; ++y) for (int x = 0; x < width; ++x)
				ASSERT_FALSE(scene.Intersect(camera->GenerateRay(x, y, Vec2(0.5f), 0)));
			// Tiny R makes VM weight negligible in Combined, without disabling it.
			Renderer render(std::make_shared<FixedSampler>(), 1, iterations, 0.0001f, real(2.0 / 3), mode, 1);
			ASSERT_TRUE(render.RenderChecked(scene, *camera));
			const auto values = Image(*camera, "splats-" + std::to_string(int(mode)) + "-" +
				std::to_string(width) + "-" + std::to_string(iterations));
			const double mean = Mean(values);
			ASSERT_GT(mean, 0);
			if (width == 16 && iterations == 1) reference = mean;
			EXPECT_NEAR(mean, reference, reference * 0.015);
			int lit = 0;
			for (auto v : values) if (v.x > 0) ++lit;
			EXPECT_EQ(lit, 1);
			// Independent parallel-plane densities: f0=5000, f1=1/(4*pi),
			// r1=1/9, r0=1/(4*pi). P splats divided by P pixels leave
			// primary=.0002*1000/9 times the balance weight of cut (2,1).
			const double weight = (5000 / (4 * Pi)) / (5000 / (4 * Pi) + 5000.0 / 9 + 1 / (36 * Pi));
			const double expected = 0.0002 * 1000 / 9 * weight;
			EXPECT_NEAR(mean, expected, expected * 0.01); // RGBE quantization and tiny finite-radius term.
			Summary(render, "splats-" + std::to_string(int(mode)) + "-" +
				std::to_string(width) + "-" + std::to_string(iterations), mean);
		}
	}
}

TEST_F(VCMRenderer, EmptyMapsAndSpecularEnvironmentEmissionSurviveMergingOnly) {
	for (Mode mode : Modes) for (int material : { 0, 1, 2 }) {
		Scene scene{};
		std::shared_ptr<Material> surface = material == 1 ?
			std::static_pointer_cast<Material>(std::make_shared<MirrorMaterial>(Constant(1))) :
			std::static_pointer_cast<Material>(std::make_shared<GlassMaterial>(Constant(1), Constant(1), 1, 1));
		// A no-hit case still needs finite environment launch bounds.
		scene.AddPrimitive(std::make_shared<Primitive>(Plane(material == 0 ? 10.f : 0.f), surface));
		scene.AddLight(Environment());
		Initialize(scene);
		auto camera = CameraFor(3, 2);
		Renderer render(std::make_shared<FixedSampler>(), 1, 4, 0.5f, real(2.0 / 3), mode, 1);
		ASSERT_TRUE(render.RenderChecked(scene, *camera));
		EXPECT_EQ(render.GetStatistics().mPeakPhotons, 0);
		for (auto v : Image(*camera, "specular-env-" + std::to_string(int(mode)) + "-" + std::to_string(material)))
			EXPECT_NEAR(v.x, 2, 0.015);
	}
}

TEST_F(VCMRenderer, ConstantEnvironmentDiffuseRadianceHasBoundedFiniteRadiusBias) {
	for (Mode mode : Modes) {
		Scene scene{};
		scene.AddPrimitive(std::make_shared<Primitive>(Plane(0, 3), Diffuse(0.5f)));
		scene.AddLight(Environment());
		Initialize(scene);
		auto camera = CameraFor(16, 16, 12);
		Renderer render(std::make_shared<RandomSampler>(123), 1, 512, 1, real(2.0 / 3), mode, 1);
		ASSERT_TRUE(render.RenderChecked(scene, *camera));
		const std::string name = "diffuse-env-" + std::to_string(int(mode));
		const double mean = Mean(Image(*camera, name));
		EXPECT_NEAR(mean, 1, 0.18); // Finite disk boundary bias + bounded Monte Carlo variance, not PDF tolerance.
		Summary(render, name, mean);
	}
}

TEST_F(VCMRenderer, GeneratedEnvironmentSampleAndEndpointPdfAgreeNearPoles) {
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(0, 3), Diffuse(0.5f)));
	auto environment = Environment();
	scene.AddLight(environment);
	Initialize(scene);
	RandomSampler samples(0);
	for (int i = 0; i < 67 * 256; ++i) {
		std::array<real, 5> u{};
		for (int d = 0; d < 5; ++d) u[d] = samples.Sample(d, 2 * i);
		Intersection origin;
		Vec3 direction;
		real pdfPos, pdfDir, queriedPos, queriedDir;
		environment->SampleLight(&origin, &direction, &pdfPos, &pdfDir, Vec2(u[1], u[2]), Vec2(u[3], u[4]));
		ASSERT_TRUE(environment->PdfLight(origin, direction, &queriedPos, &queriedDir));
		EXPECT_EQ(pdfPos, queriedPos);
		ASSERT_NEAR(pdfDir, queriedDir, 1e-4 * queriedDir)
			<< std::setprecision(17) << "sample " << i << " u=" << u[1] << "," << u[2]
			<< " direction=" << direction;
	}
}

class ExpandedLaunchBounds : public BruteForce {
public:
	ExpandedLaunchBounds(const std::vector<std::shared_ptr<Primitive>>& primitives, real scale)
		: BruteForce(primitives), mScale(scale) {}
	AABB WorldBound() const override {
		const auto bounds = BruteForce::WorldBound();
		const Vec3 center = (bounds.minPoint + bounds.maxPoint) * 0.5f;
		const Vec3 extent = (bounds.maxPoint - bounds.minPoint) * (0.5f * mScale);
		return AABB(center - extent, center + extent);
	}
private:
	real mScale;
};

// Preserve the independent replicate identity when the legacy Clone adapter
// receives a logical path/phase key.
class IndependentSeedSampler : public Sampler {
public:
	explicit IndependentSeedSampler(uint32 seed) : mSeed(seed), mRandom(seed) {}
	real Sample(unsigned d, unsigned long long i) override { return mRandom.Sample(d, i); }
	std::shared_ptr<Sampler> Clone(uint32 stream) override {
		return std::make_shared<IndependentSeedSampler>(mSeed + 1664525u * (stream + 1));
	}
private:
	uint32 mSeed;
	RandomSampler mRandom;
};

TEST_F(VCMRenderer, ExpandedEnvironmentBoundsDoNotDimDiffuseIndependentSeeds) {
	// Declared before acceptance: 48 independent seed-pairs, 16x16, 256
	// iterations, depth 1, R0=.4, alpha=2/3, launch-radius scales 1 and 2.
	// All camera points and their disks are inside this triangle:
	// camera footprint radius < .038; .4+.038 < 1/sqrt(5).
	// Hence constant Le=2 and rho=.5 imply Lo=1 without a kernel-edge bias.
	// Accept |error| <= 4 standard errors + .008 RGBE quantization, and
	// require the 4-SE half-width <= .12 (also for the paired difference).
	// Pixels sharing photons are correlated; they are NOT independent trials.
	// A separate 12-pair pilot failed precision, not energy. Use fresh seeds
	// and four times as many independent pairs; do not widen the tolerances.
	constexpr size_t replicates = 48;
	std::ofstream csv(output + "\\vcm-environment-independent-seeds.csv");
	ASSERT_TRUE(csv.good());
	csv << "mode,scale,replicate,seed,mean,lightAttempts,peakPhotons\n" << std::setprecision(17);
	const auto moments = [=](const std::array<double, replicates>& samples) {
		double mean = 0, variance = 0;
		for (double value : samples) mean += value / replicates;
		for (double value : samples) variance += (value - mean) * (value - mean) / (replicates - 1);
		return std::make_pair(mean, std::sqrt(variance / replicates));
	};
	for (Mode mode : Modes) {
		std::array<std::array<double, replicates>, 2> means{};
		for (int scaleIndex = 0; scaleIndex < 2; ++scaleIndex) {
			const real scale = real(scaleIndex + 1);
			Scene scene{};
			scene.AddPrimitive(std::make_shared<Primitive>(Plane(0, 1), Diffuse(0.5f)));
			auto environment = Environment();
			scene.AddLight(environment);
			scene.SetAccelerator(std::make_shared<ExpandedLaunchBounds>(scene.GetPrimitives(), scale));
			scene.Initialize();
			real positionPdf, directionPdf;
			ASSERT_TRUE(environment->PdfLight(Intersection(), Vec3(0, 0, -1), &positionPdf, &directionPdf));
			EXPECT_NEAR(positionPdf, 1 / (2 * Pi * scale * scale), 1e-7);
			for (size_t replicate = 0; replicate < replicates; ++replicate) {
				const uint32 seed = 104729u * uint32(replicate + 101);
				auto camera = CameraFor(16, 16, 1);
				Renderer render(std::make_shared<IndependentSeedSampler>(seed), 1, 256, 0.4f,
					real(2.0 / 3), mode, 1);
				ASSERT_TRUE(render.RenderChecked(scene, *camera));
				const auto name = "expanded-env-" + std::to_string(int(mode)) + "-" +
					std::to_string(scaleIndex + 1) + "-" + std::to_string(replicate);
				const auto image = Image(*camera, name);
				ASSERT_EQ(image.size(), 256);
				means[scaleIndex][replicate] = Mean(image);
				const auto& stats = render.GetStatistics();
				EXPECT_EQ(stats.mLightAttempts, 256 * 256);
				if (mode != Mode::ConnectionOnly) EXPECT_LT(stats.mPeakPhotons, 256);
				csv << int(mode) << ',' << scale << ',' << replicate << ',' << seed << ','
					<< means[scaleIndex][replicate] << ',' << stats.mLightAttempts << ',' << stats.mPeakPhotons << '\n';
			}
			const auto result = moments(means[scaleIndex]);
			SCOPED_TRACE("mode=" + std::to_string(int(mode)) + " scale=" + std::to_string(scale));
			RecordProperty("mode_" + std::to_string(int(mode)) + "_scale_" + std::to_string(scaleIndex + 1) +
				"_mean", std::to_string(result.first));
			RecordProperty("mode_" + std::to_string(int(mode)) + "_scale_" + std::to_string(scaleIndex + 1) +
				"_standard_error", std::to_string(result.second));
			EXPECT_LE(4 * result.second, 0.12);
			EXPECT_NEAR(result.first, 1, 4 * result.second + 0.008);
		}
		std::array<double, replicates> differences{};
		for (size_t i = 0; i < replicates; ++i) differences[i] = means[1][i] - means[0][i];
		const auto difference = moments(differences);
		RecordProperty("mode_" + std::to_string(int(mode)) + "_paired_difference", std::to_string(difference.first));
		RecordProperty("mode_" + std::to_string(int(mode)) + "_paired_standard_error", std::to_string(difference.second));
		EXPECT_LE(4 * difference.second, 0.12);
		EXPECT_NEAR(difference.first, 0, 4 * difference.second + 0.008);
	}
}

TEST_F(VCMRenderer, NonunitGlassSlabAndMirrorVisibleEmitterKeepTheirRadiance) {
	for (Mode mode : Modes) for (bool slab : { false, true }) {
		Scene scene{};
		if (slab) {
			auto glass = std::make_shared<GlassMaterial>(Constant(1), Constant(1), 1, 1.5f);
			scene.AddPrimitive(std::make_shared<Primitive>(Plane(0), glass));
			scene.AddPrimitive(std::make_shared<Primitive>(Plane(-1, 10, 0, -1), glass));
			scene.AddLight(Environment());
		}
		else {
			scene.AddPrimitive(std::make_shared<Primitive>(Plane(0), std::make_shared<MirrorMaterial>(Constant(1))));
			auto shape = Plane(5, 10, 0, -1);
			auto emitter = std::make_shared<AreaLight>(shape, Vec3(2));
			scene.AddLight(emitter);
			scene.AddPrimitive(std::make_shared<Primitive>(shape, Diffuse(0), emitter));
		}
		Initialize(scene);
		auto camera = CameraFor(3, 2, 12);
		Renderer render(std::make_shared<FixedSampler>(), slab ? 2 : 1, 4, 0.5f, real(2.0 / 3), mode, 1);
		ASSERT_TRUE(render.RenderChecked(scene, *camera));
		for (auto v : Image(*camera, "specular-visible-" + std::to_string(int(mode)) + "-" + std::to_string(slab)))
			EXPECT_NEAR(v.x, 2, 0.015);
	}
}

TEST_F(VCMRenderer, EmptyLightAttemptsRemainInGatherDenominator) {
	class HalfEmptyLight : public FixedLight {
	public:
		Vec3 SampleLight(Intersection* h, Vec3* d, real* p, real* q, const Vec2& u, const Vec2& v) const override {
			const Vec3 value = FixedLight::SampleLight(h, d, p, q, u, v);
			if (attempts++ % 2) *q = 0; // Legitimate unsuccessful emission attempt.
			return value;
		}
		mutable int attempts = 0;
	};
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(0), Diffuse(0.5f)));
	scene.AddLight(std::make_shared<HalfEmptyLight>());
	Initialize(scene);
	auto camera = CameraFor(33, 2, 0.1f);
	Renderer render(std::make_shared<FixedSampler>(), 1, 1, 1, real(2.0 / 3), Mode::MergingOnly, 1);
	ASSERT_TRUE(render.RenderChecked(scene, *camera));
	EXPECT_EQ(render.GetStatistics().mLightAttempts, 66);
	EXPECT_EQ(render.GetStatistics().mPeakPhotons, 33);
	for (auto v : Image(*camera, "half-empty-attempts")) EXPECT_NEAR(v.x, 0.5 / Pi, 0.5 / Pi * 0.008);
}

TEST_F(VCMRenderer, EnvironmentSamplingRetainsEndpointPdfNearPoles) {
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(0, 3), Diffuse()));
	auto environment = Environment();
	scene.AddLight(environment);
	Initialize(scene);
	RandomSampler sampler(0);
	std::ofstream csv(output + "\\environment-endpoint-roundtrip.csv");
	ASSERT_TRUE(csv.good());
	csv << "sample,sampledPdf,evaluatedPdf,relativeError,x,y,z\n" << std::setprecision(17);
	double maximum = 0;
	for (int sample = 0; sample < 10000; ++sample) {
		RandomStateSequence random(std::shared_ptr<Sampler>(&sampler, [](Sampler*) {}), sample);
		random();
		const real u = random(), v = random(), a = random(), b = random();
		Intersection hit;
		Vec3 dir;
		real pos, pdf, evaluatedPos, evaluatedPdf;
		environment->SampleLight(&hit, &dir, &pos, &pdf, Vec2(u, v), Vec2(a, b));
		ASSERT_TRUE(environment->PdfLight(hit, dir, &evaluatedPos, &evaluatedPdf));
		const double error = std::abs(double(pdf) - evaluatedPdf) / pdf;
		maximum = std::max(maximum, error);
		csv << sample << ',' << pdf << ',' << evaluatedPdf << ',' << error << ','
			<< dir.x << ',' << dir.y << ',' << dir.z << '\n';
	}
	RecordProperty("maximum_endpoint_pdf_relative_error", std::to_string(maximum));
	EXPECT_LE(maximum, 1e-4);
}

struct Lifetime {
	std::mutex mutex;
	std::map<const void*, int> active;
	std::atomic<int> generation{ 0 };
	bool overwritten = false, stale = false;
	size_t queries = 0;
};
class LifetimeLobe : public DiffuseBSDF {
public:
	LifetimeLobe(Lifetime& state, int generation) : DiffuseBSDF(Vec3(0.5f)), mState(state), mGeneration(generation) {
		std::lock_guard<std::mutex> lock(mState.mutex);
		auto found = mState.active.find(this);
		if (found != mState.active.end() && found->second == generation) mState.overwritten = true;
		mState.active[this] = generation;
	}
	real Pdf(const Vec3& wo, const Vec3& wi) const override {
		std::lock_guard<std::mutex> lock(mState.mutex);
		mState.stale |= mState.generation.load() != mGeneration;
		++mState.queries;
		return DiffuseBSDF::Pdf(wo, wi);
	}
private:
	Lifetime& mState;
	int mGeneration;
};
class LifetimeMaterial : public Material {
public:
	explicit LifetimeMaterial(Lifetime& state) : mState(state) {}
	void ComputeScatteringFunction(Intersection* hit, MemoryPool& arena, TransportMode mode) const override {
		hit->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*hit);
		if (mode == TransportMode::Importance)
			hit->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, LifetimeLobe)(mState, mState.generation.load()));
		else hit->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(Vec3(0.5f)));
	}
private:
	Lifetime& mState;
};
class GenerationLight : public FixedLight {
public:
	GenerationLight(Lifetime& state, int64 count) : mState(state), mCount(count) {}
	Vec3 SampleLight(Intersection* h, Vec3* d, real* p, real* q, const Vec2& u, const Vec2& v) const override {
		std::lock_guard<std::mutex> lock(mState.mutex);
		mState.generation.store(int(mAttempts++ / mCount));
		return FixedLight::SampleLight(h, d, p, q, u, v);
	}
private:
	Lifetime& mState;
	int64 mCount;
	mutable int64 mAttempts = 0;
};

TEST_F(VCMRenderer, WorkerArenasSurviveReaders) {
	for (int concurrency : { 1, 4 }) {
		for (int iterations : { 1, 4, 12 }) {
			Lifetime state;
			Scene scene{};
			scene.AddPrimitive(std::make_shared<Primitive>(Plane(0), std::make_shared<LifetimeMaterial>(state)));
			scene.AddLight(std::make_shared<GenerationLight>(state, 65 * 2));
			Initialize(scene);
			auto camera = CameraFor(65, 2, 0.1f);
			Renderer render(std::make_shared<FixedSampler>(), 1, iterations, 1, real(2.0 / 3), Mode::Combined, concurrency);
			ASSERT_TRUE(render.RenderChecked(scene, *camera));
			EXPECT_FALSE(state.overwritten);
			EXPECT_FALSE(state.stale);
			EXPECT_GT(state.queries, 0);
			Summary(render, "lifetime-" + std::to_string(concurrency) + "-" + std::to_string(iterations),
				Mean(Image(*camera, "lifetime-" + std::to_string(concurrency) + "-" + std::to_string(iterations))));
		}
	}
}

TEST_F(VCMRenderer, CompletionOccursOnceAfterBothPhases) {
	Scene scene{};
	Initialize(scene);
	auto camera = CameraFor(1, 1);
	Renderer render(std::make_shared<FixedSampler>(), 0, 2, 1, real(2.0 / 3), Mode::Combined, 1);
	testing::internal::CaptureStderr();
	const bool success = render.RenderChecked(scene, *camera);
	const auto text = testing::internal::GetCapturedStderr();
	ASSERT_TRUE(success);
	const auto completion = text.find("100.0%");
	ASSERT_NE(completion, std::string::npos);
	EXPECT_GT(completion, text.find("VCM camera phase 2/2"));
	EXPECT_GT(completion, text.find("VCM iteration 2/2:"));
	EXPECT_EQ(text.find("100.0%", completion + 1), std::string::npos);
}

TEST_F(VCMRenderer, IterationSummariesReportCurrentRadiusPhotonsAndEta) {
	const std::regex summary(
		R"(^VCM iteration ([0-9]+)/3: radius ([0-9.e+-]+), ([0-9]+) photons; elapsed [0-9]+:[0-5][0-9]:[0-5][0-9], remaining ~([0-9]+:[0-5][0-9]:[0-5][0-9])$)");
	for (Mode mode : Modes) {
		Scene scene{};
		scene.AddPrimitive(std::make_shared<Primitive>(Plane(0), Diffuse(0.5f)));
		scene.AddLight(std::make_shared<FixedLight>());
		Initialize(scene);
		auto camera = CameraFor(2, 2, 0.1f);
		Renderer render(std::make_shared<FixedSampler>(), 1, 3, 0.5f, .75f, mode, 1);
		testing::internal::CaptureStderr();
		const bool success = render.RenderChecked(scene, *camera);
		const auto text = testing::internal::GetCapturedStderr();
		ASSERT_TRUE(success);
		int completed = 0;
		std::istringstream lines(text);
		std::string line;
		while (std::getline(lines, line)) {
			if (!line.empty() && line.back() == '\r') line.pop_back();
			if (line.find("VCM iteration ") != 0) continue;
			std::smatch match;
			ASSERT_TRUE(std::regex_match(line, match, summary)) << line;
			++completed;
			EXPECT_EQ(std::strtol(match[1].str().c_str(), nullptr, 10), completed);
			EXPECT_NEAR(std::strtod(match[2].str().c_str(), nullptr), .5 * std::pow(double(completed), -.125), 1e-6);
			EXPECT_EQ(std::strtol(match[3].str().c_str(), nullptr, 10), mode == Mode::ConnectionOnly ? 0 : 4);
			EXPECT_GT(text.find(line), text.find("VCM camera phase " + std::to_string(completed) + "/3"));
			if (completed < 3)
				EXPECT_LT(text.find(line), text.find("VCM light phase " + std::to_string(completed + 1) + "/3"));
			else EXPECT_EQ(match[4].str(), "0:00:00");
		}
		EXPECT_EQ(completed, 3);
	}
}

TEST_F(VCMRenderer, FailedIterationDoesNotReportCompletionOrEta) {
	class FailSecondIteration : public FixedSampler {
	public:
		// The second iteration's first path stream cannot be created.
		std::shared_ptr<Sampler> Clone(uint32 key) override {
			return key >= 8 ? nullptr : std::make_shared<FailSecondIteration>();
		}
	};
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(0), Diffuse(0.5f)));
	scene.AddLight(std::make_shared<FixedLight>());
	Initialize(scene);
	auto camera = CameraFor(2, 2, 0.1f);
	Renderer render(std::make_shared<FailSecondIteration>(), 1, 3, .5f, .75f, Mode::Combined, 1);
	testing::internal::CaptureStderr();
	const bool success = render.RenderChecked(scene, *camera);
	const auto text = testing::internal::GetCapturedStderr();
	EXPECT_FALSE(success);
	EXPECT_EQ(render.GetStatistics().mIterations, 1);
	EXPECT_NE(text.find("VCM iteration 1/3:"), std::string::npos);
	EXPECT_EQ(text.find("VCM iteration 2/3:"), std::string::npos);
	EXPECT_EQ(text.find("VCM iteration 3/3:"), std::string::npos);
	EXPECT_EQ(text.find("100.0%"), std::string::npos);
}

TEST_F(VCMRenderer, SingleWorkerFreshRendersAreDeterministicInEveryMode) {
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(0), Diffuse(0.5f)));
	scene.AddLight(std::make_shared<PointLight>(Vec3(0, 0, 2), Vec3(1)));
	Initialize(scene);
	for (Mode mode : Modes) {
		std::vector<Vec3> previous;
		for (int repeat = 0; repeat < 2; ++repeat) {
			auto camera = CameraFor(5, 3, 12);
			Renderer render(std::make_shared<RandomSampler>(17), 2, 4, 1, real(2.0 / 3), mode, 1);
			ASSERT_TRUE(render.RenderChecked(scene, *camera));
			const auto values = Image(*camera, "deterministic-" + std::to_string(int(mode)) + "-" + std::to_string(repeat));
			if (repeat) EXPECT_EQ(values, previous);
			else previous = values;
		}
	}
}

class VCMRendererNegative : public VCMRenderer {};

TEST_F(VCMRenderer, ValidNearOneJitterSurvivesOrdinaryPixelBoundaryRounding) {
	class NearOneSampler : public FixedSampler {
	public:
		real Sample(unsigned int, unsigned long long) override { return ONE_MINUS_EPSILON; }
		std::shared_ptr<Sampler> Clone(uint32) override { return std::make_shared<NearOneSampler>(); }
	};
	class JitterCamera : public PinHoleCamera {
	public:
		JitterCamera(int width) : PinHoleCamera(std::make_shared<Film>(width, 2),
			Vec3(0, 0, 3), Vec3(0), Vec3(0, 1, 0), 0.01f, 1) {}
		Ray GenerateRay(int x, int y, const Vec2& u, real offset) const override {
			if (u.x != 0.5f || u.y != 0.5f) {
				EXPECT_EQ(u.x, ONE_MINUS_EPSILON);
				EXPECT_EQ(u.y, ONE_MINUS_EPSILON);
				++samples;
			}
			return PinHoleCamera::GenerateRay(x, y, u, offset);
		}
		mutable int samples = 0;
	};
	for (int width : { 2, 1600 }) {
		Scene scene{};
		auto shape = Plane(0, 100);
		auto emitter = std::make_shared<AreaLight>(shape, Vec3(3));
		scene.AddLight(emitter);
		scene.AddPrimitive(std::make_shared<Primitive>(shape, Diffuse(0), emitter));
		Initialize(scene);
		JitterCamera camera(width);
		Renderer render(std::make_shared<NearOneSampler>(), 0, 1, 1, 0.5f, Mode::Combined, 1);
		EXPECT_EQ(real(1) + ONE_MINUS_EPSILON, 2);
		ASSERT_TRUE(render.RenderChecked(scene, camera));
		EXPECT_EQ(camera.samples, width * 2);
		EXPECT_EQ(render.GetStatistics().mCameraSamples, width * 2);
		for (const auto& value : Image(camera, "near-one-" + std::to_string(width))) EXPECT_EQ(value.x, 3);
	}
}

TEST_F(VCMRendererNegative, HaltonCapacityIsRejectedBeforeAnySampleCall) {
	class CountedHalton : public HaltonSampler {
	public:
		explicit CountedHalton(std::shared_ptr<int> calls) : calls(calls) {}
		real Sample(uint32 d, uint64 i) override { ++*calls; return HaltonSampler::Sample(d, i); }
		std::shared_ptr<Sampler> Clone(uint32) override { return std::make_shared<CountedHalton>(calls); }
		std::shared_ptr<int> calls;
	};
	Scene scene{};
	Initialize(scene);
	auto calls = std::make_shared<int>(0);
	Renderer render(std::make_shared<CountedHalton>(calls), 85, 1, 1, 0.5f, Mode::ConnectionOnly, 1);
	EXPECT_FALSE(render.RenderChecked(scene, *CameraFor(1, 1)));
	EXPECT_EQ(*calls, 0);
	EXPECT_EQ(render.GetStatistics().mLightAttempts, 0);
}

TEST_F(VCMRenderer, BundledRoundedUnitEndpointsUseTheirLegacySequenceConvention) {
	class RoundedSobol : public SobolSampler {
	public:
		real Sample(uint32, uint64) override { return SobolSampler::Sample(0, (uint64(1) << 32) - 1); }
		std::shared_ptr<Sampler> Clone(uint32) override { return std::make_shared<RoundedSobol>(); }
	};
	class RoundedRegularHalton : public RegularHaltonSampler {
	public:
		real Sample(uint32, uint64) override { return RegularHaltonSampler::Sample(0, (uint64(1) << 25) - 2); }
		std::shared_ptr<Sampler> Clone(uint32) override { return std::make_shared<RoundedRegularHalton>(); }
	};
	Scene scene{};
	Initialize(scene);
	for (const auto& sampler : { std::static_pointer_cast<Sampler>(std::make_shared<RoundedSobol>()),
		std::static_pointer_cast<Sampler>(std::make_shared<RoundedRegularHalton>()) }) {
		ASSERT_EQ(sampler->Sample(0, 0), 1);
		RandomStateSequence legacy(sampler, 0);
		EXPECT_EQ(legacy(), 0);
		Renderer render(sampler, 0, 1, 1, 0.5f, Mode::Combined, 1);
		EXPECT_TRUE(render.RenderChecked(scene, *CameraFor(2, 2)));
	}
}

struct SamplerCalls {
	uint32 dimensions;
	uint64 maximumIndex;
	int calls = 0, outside = 0;
	uint32 lastDimension = 0;
	int phaseCalls[2] = {};
};
template<class Base>
class DomainSampler : public Base {
public:
	explicit DomainSampler(std::shared_ptr<SamplerCalls> calls) : calls(calls) {}
	real Sample(uint32 d, uint64 i) override {
		++calls->calls;
		++calls->phaseCalls[i % 2];
		calls->lastDimension = d;
		if (d >= calls->dimensions || i > calls->maximumIndex) {
			++calls->outside;
			return 0; // The test double never performs an unsafe underlying access.
		}
		return Base::Sample(d, i);
	}
	std::shared_ptr<Sampler> Clone(uint32) override { return std::make_shared<DomainSampler<Base>>(calls); }
	std::shared_ptr<SamplerCalls> calls;
};

TEST_F(VCMRenderer, BundledDimensionLimitsCoverActualMaximumLightAndEyeWalks) {
	const uint64 signedIndex = uint64((std::numeric_limits<int64>::max)()) - 1;
	for (int kind : { 0, 1, 2 }) {
		const unsigned dimensions = kind == 0 ? 256 : kind == 1 ? 1024 : 1229;
		const int depth = kind == 0 ? 84 : kind == 1 ? 340 : 409;
		auto calls = std::make_shared<SamplerCalls>();
		calls->dimensions = dimensions;
		calls->maximumIndex = kind == 1 ? (uint64(1) << 52) - 1 : signedIndex;
		std::shared_ptr<Sampler> sampler;
		if (kind == 0) sampler = std::make_shared<DomainSampler<HaltonSampler>>(calls);
		else if (kind == 1) sampler = std::make_shared<DomainSampler<SobolSampler>>(calls);
		else sampler = std::make_shared<DomainSampler<RegularHaltonSampler>>(calls);

		Scene scene{};
		auto mirror = std::make_shared<MirrorMaterial>(Constant(1));
		scene.AddPrimitive(std::make_shared<Primitive>(Plane(0, 10000), mirror));
		scene.AddPrimitive(std::make_shared<Primitive>(Plane(1, 10000, 0, -1), mirror));
		scene.AddLight(std::make_shared<PointLight>(Vec3(0, 0, 0.5f), Vec3(1)));
		Initialize(scene);
		PinHoleCamera camera(std::make_shared<Film>(1, 1), Vec3(0, 0, 0.5f), Vec3(0),
			Vec3(0, 1, 0), 0.01f, 1);
		Renderer tooDeep(sampler, depth + 1, 1, 1, 0.5f, Mode::ConnectionOnly, 1);
		EXPECT_FALSE(tooDeep.RenderChecked(scene, camera));
		EXPECT_EQ(calls->calls, 0);
		Renderer render(sampler, depth, 1, 1, 0.5f, Mode::ConnectionOnly, 1);
		ASSERT_TRUE(render.RenderChecked(scene, camera));
		EXPECT_EQ(calls->calls, 2 * (2 + 3 * depth));
		EXPECT_EQ(calls->phaseCalls[0], 5 + 3 * (depth - 1));
		EXPECT_EQ(calls->phaseCalls[1], 2 + 3 * depth);
		EXPECT_EQ(calls->lastDimension, 1 + 3 * depth);
		EXPECT_EQ(calls->outside, 0);
		EXPECT_EQ(render.GetStatistics().mCameraSamples, 1);
	}
}

TEST_F(VCMRendererNegative, MapFailureJoinsWithoutStartingCameraWork) {
	Scene scene{};
	scene.AddPrimitive(std::make_shared<Primitive>(Plane(0, 100), Diffuse(0.5f)));
	scene.AddLight(std::make_shared<AreaLight>(Plane(2, 1, 0, -1), Vec3(1)));
	Initialize(scene);
	auto camera = CameraFor(33, 1, 0.01f);
	// Photons whose cell coordinates reach 2^62 cannot be indexed by the grid.
	Renderer render(std::make_shared<RandomSampler>(0), 1, 2, 1e-30f, 0.5f, Mode::Combined, 1);
	testing::internal::CaptureStderr();
	const bool success = render.RenderChecked(scene, *camera);
	const auto output = testing::internal::GetCapturedStderr();
	EXPECT_FALSE(success);
	EXPECT_EQ(output.find("100.0%"), std::string::npos);
	EXPECT_NE(output.find("failed after 33 of 132 work units"), std::string::npos);
	EXPECT_EQ(output.find("VCM iteration "), std::string::npos);
	EXPECT_EQ(output.find("VCM camera phase"), std::string::npos);
	EXPECT_EQ(render.GetStatistics().mLightAttempts, 33);
	EXPECT_EQ(render.GetStatistics().mCameraSamples, 0);
	Scene empty{};
	Initialize(empty);
	EXPECT_TRUE(render.RenderChecked(empty, *CameraFor(2, 1)));
}

TEST_F(VCMRendererNegative, InvalidParametersAndSamplerStreamsFail) {
	Scene scene{};
	Initialize(scene);
	auto camera = CameraFor(2, 2);
	auto sampler = std::make_shared<FixedSampler>();
	auto reject = [&](std::shared_ptr<Sampler> sample, int depth, int iterations, real radius, real alpha, Mode mode, int concurrency) {
		Renderer render(sample, depth, iterations, radius, alpha, mode, concurrency);
		EXPECT_FALSE(render.RenderChecked(scene, *camera));
		EXPECT_FALSE(render.GetStatistics().mSucceeded);
		EXPECT_EQ(render.GetStatistics().mCameraSamples, 0);
	};
	reject(nullptr, 1, 1, 1, 0.5f, Mode::Combined, 1);
	for (int depth : { -1, 410, INT_MAX }) reject(sampler, depth, 1, 1, 0.5f, Mode::Combined, 1);
	for (int n : { 0, -1 }) reject(sampler, 1, n, 1, 0.5f, Mode::Combined, 1);
	for (real radius : { 0.f, -1.f, INFINITY, NAN }) reject(sampler, 1, 1, radius, 0.5f, Mode::Combined, 1);
	for (real alpha : { 0.f, 1.f, -1.f, INFINITY, NAN }) reject(sampler, 1, 1, 1, alpha, Mode::Combined, 1);
	reject(sampler, 1, 1, 1, 0.5f, Mode(9), 1);
	reject(sampler, 1, 1, 1, 0.5f, Mode::Combined, 0);
	class NullClone : public FixedSampler {
		std::shared_ptr<Sampler> Clone(uint32) override { return nullptr; }
	};
	reject(std::make_shared<NullClone>(), 1, 1, 1, 0.5f, Mode::Combined, 1);
}

TEST_F(VCMRendererNegative, UnsupportedScenesFailChecked) {
	for (int kind : { 0, 1, 2 }) {
		Scene scene{};
		auto primitive = std::make_shared<Primitive>(Plane(0), kind == 0 ? nullptr : Diffuse());
		std::shared_ptr<HomogeneousMedium> medium;
		if (kind == 1) {
			medium = std::make_shared<HomogeneousMedium>(Vec3(0), Vec3(0.1f), 0);
			primitive->mMediumInterface = MediumInterface(medium.get(), nullptr);
		}
		scene.AddPrimitive(primitive);
		if (kind == 2) scene.AddLight(Environment(2, 10));
		Initialize(scene);
		auto camera = CameraFor(1, 1);
		Renderer render(std::make_shared<FixedSampler>(), 1, 1, 1);
		EXPECT_FALSE(render.RenderChecked(scene, *camera));
		EXPECT_EQ(render.GetStatistics().mLightAttempts, 0);
	}
}

TEST_F(VCMRendererNegative, CancellationAndMissingSamplerStreamsDrainBothPhaseLifetimes) {
	for (int phase : { 0, 1 }) for (bool failure : { false, true }) {
		Scene scene{};
		scene.AddPrimitive(std::make_shared<Primitive>(Plane(0), Diffuse()));
		scene.AddLight(std::make_shared<FixedLight>());
		Initialize(scene);
		auto camera = CameraFor(65, 2, 0.1f);
		auto recording = std::make_shared<Recording>();
		recording->pixels = 130;
		Renderer render(std::make_shared<RecordingSampler>(recording), 1, 4, 1, real(2.0 / 3), Mode::Combined, 4);
		recording->cancel = &render;
		if (failure) recording->failPhase = phase;
		else recording->cancelPhase = phase;
		testing::internal::CaptureStderr();
		const bool success = render.RenderChecked(scene, *camera);
		const auto text = testing::internal::GetCapturedStderr();
		EXPECT_FALSE(success);
		EXPECT_EQ(text.find("100.0%"), std::string::npos);
		EXPECT_FALSE(render.GetStatistics().mSucceeded);
		EXPECT_EQ(render.GetStatistics().mIterations, 0);
		EXPECT_EQ(text.find("VCM iteration "), std::string::npos);
		EXPECT_EQ(render.GetStatistics().mLightAttempts, phase == 0 ? 0 : 130);
		EXPECT_EQ(render.GetStatistics().mCameraSamples, 0);
		recording->cancel = nullptr;
		recording->failPhase = -1;
		recording->starts.clear();
		// A joined failure/cancellation leaves the renderer reusable, not a live
		// task retaining paths or samplers from the discarded generation.
		ASSERT_TRUE(render.RenderChecked(scene, *CameraFor(65, 2, 0.1f)));
	}
}

class MixedMaterial : public Material {
public:
	void ComputeScatteringFunction(Intersection* hit, MemoryPool& arena, TransportMode) const override {
		hit->mBsdf = GYT_MEMORY_POOL_ALLOC(arena, BSDF)(*hit);
		hit->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, SpecularBSDF)(Vec3(1)));
		hit->mBsdf->Add(GYT_MEMORY_POOL_ALLOC(arena, DiffuseBSDF)(Vec3(1)));
	}
};

TEST_F(VCMRendererNegative, MixedDeltaAndContinuousLobesAreUnsupported) {
	for (Mode mode : Modes) {
		Scene scene{};
		scene.AddPrimitive(std::make_shared<Primitive>(Plane(0), std::make_shared<MixedMaterial>()));
		Initialize(scene);
		auto camera = CameraFor(1, 1);
		Renderer render(std::make_shared<FixedSampler>(), 1, 1, 1, real(2.0 / 3), mode, 1);
		EXPECT_FALSE(render.RenderChecked(scene, *camera));
		EXPECT_EQ(render.GetStatistics().mIterations, 0);
	}
}

}
