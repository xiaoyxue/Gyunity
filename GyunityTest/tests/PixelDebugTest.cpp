#include "gtest/gtest.h"
#include "common/DebugUtils.h"
#include <atomic>
#include <thread>
#include <vector>

using namespace Gyunity;

namespace {

class PixelDebug : public testing::Test {
	void SetUp() override { SetPixelDebugEnabled(false); }
	void TearDown() override { SetPixelDebugEnabled(false); }
};

}

TEST_F(PixelDebug, CurrentThreadWorksOutsideAnArena) {
	EXPECT_FALSE(IsPixelDebugEnabled());
	SetPixelDebugEnabled(true);
	EXPECT_TRUE(IsPixelDebugEnabled());
	SetPixelDebugEnabled(false);
	EXPECT_FALSE(IsPixelDebugEnabled());
}

TEST_F(PixelDebug, PixelSelectionResetsOnMismatch) {
	SelectDebugPixel(10, 20, 10, 20);
	EXPECT_TRUE(IsPixelDebugEnabled());
	SelectDebugPixel(10, 21, 10, 20);
	EXPECT_FALSE(IsPixelDebugEnabled());
}

TEST_F(PixelDebug, RegionChecksBothAxesAndInclusiveBounds) {
	for (real x : { 10.f, 15.f, 20.f }) {
		for (real y : { 30.f, 35.f, 40.f }) {
			SelectDebugRegion(x, y, 10, 30, 20, 40);
			EXPECT_TRUE(IsPixelDebugEnabled());
		}
	}
	SelectDebugRegion(15, 41, 10, 30, 20, 40);
	EXPECT_FALSE(IsPixelDebugEnabled());
	SelectDebugRegion(21, 35, 10, 30, 20, 40);
	EXPECT_FALSE(IsPixelDebugEnabled());
	SelectDebugRegion(9, 35, 10, 30, 20, 40);
	EXPECT_FALSE(IsPixelDebugEnabled());
	SelectDebugRegion(15, 29, 10, 30, 20, 40);
	EXPECT_FALSE(IsPixelDebugEnabled());
}

TEST_F(PixelDebug, LegacyMacrosDoNotIndexByThreadNumber) {
	GYT_DEBUG_PIXEL_V2(10, 20, 10, 20, -1);
	EXPECT_TRUE(IsPixelDebugEnabled());
	GYT_DEBUG_PIXEL_V3(10, 21, 0, 0, 10, 20, 1000000);
	EXPECT_FALSE(IsPixelDebugEnabled());
	int x = 1, y = 2, calls = 0;
	GYT_DEBUG_PIXEL(1, 2, ++calls);
	EXPECT_EQ(calls, 1);
	bool entered = false;
	GYT_DEBUG_PIXEL_IF(-1) { entered = true; }
	EXPECT_TRUE(entered);
}

TEST_F(PixelDebug, StateIsIndependentAcrossMoreThanOneHundredThreads) {
	SetPixelDebugEnabled(true);
	std::atomic<int> ready(0), failures(0);
	std::atomic<bool> start(false);
	std::vector<std::thread> workers;
	for (int i = 0; i < 128; ++i) {
		workers.emplace_back([&, i] {
			if (IsPixelDebugEnabled()) ++failures;
			const bool selected = (i % 2) != 0;
			SetPixelDebugEnabled(selected);
			++ready;
			while (!start.load()) std::this_thread::yield();
			for (int check = 0; check < 100; ++check) {
				if (IsPixelDebugEnabled() != selected) ++failures;
				std::this_thread::yield();
			}
		});
	}
	while (ready.load() != 128) std::this_thread::yield();
	start = true;
	for (auto& worker : workers) worker.join();
	EXPECT_EQ(failures.load(), 0);
	EXPECT_TRUE(IsPixelDebugEnabled());
}
