#include "gtest/gtest.h"
#include "system/ProgressReporter.h"
#include <thread>
#include <vector>

using namespace Gyunity;

TEST(ProgressReporter, ReportsCompletedWorkFractionsAndDefersCompletion) {
	testing::internal::CaptureStderr();
	ProgressReporter progress("Test tiles", 4);
	progress.Advance();
	progress.Advance();
	progress.Advance();
	progress.Advance();
	const std::string running = testing::internal::GetCapturedStderr();
	EXPECT_NE(running.find("  0.0%"), std::string::npos);
	EXPECT_NE(running.find(" 25.0%"), std::string::npos);
	EXPECT_NE(running.find(" 50.0%"), std::string::npos);
	EXPECT_NE(running.find(" 75.0%"), std::string::npos);
	EXPECT_EQ(running.find("100.0%"), std::string::npos);
	testing::internal::CaptureStderr();
	EXPECT_TRUE(progress.Finish(true));
	EXPECT_TRUE(progress.Finish(true));
	const std::string finished = testing::internal::GetCapturedStderr();
	const size_t completion = finished.find("100.0%");
	ASSERT_NE(completion, std::string::npos);
	EXPECT_EQ(finished.find("100.0%", completion + 1), std::string::npos);
}

TEST(ProgressReporter, FailureAndIncompleteWorkCannotReportOneHundredPercent) {
	testing::internal::CaptureStderr();
	ProgressReporter failed("Failed tiles", 2);
	failed.Advance();
	EXPECT_FALSE(failed.Finish(false));
	ProgressReporter incomplete("Incomplete tiles", 2);
	incomplete.Advance();
	EXPECT_FALSE(incomplete.Finish(true));
	const std::string output = testing::internal::GetCapturedStderr();
	EXPECT_EQ(output.find("100.0%"), std::string::npos);
	EXPECT_NE(output.find("Failed tiles failed after 1 of 2 work units."), std::string::npos);
	EXPECT_NE(output.find("Incomplete tiles failed after 1 of 2 work units."), std::string::npos);
}

TEST(ProgressReporter, ConcurrentUpdatesStayMonotonicAndThrottled) {
	testing::internal::CaptureStderr();
	ProgressReporter progress("Concurrent tiles", 8192);
	std::vector<std::thread> workers;
	for (int i = 0; i < 8; ++i) {
		workers.emplace_back([&] {
			for (int j = 0; j < 1024; ++j) progress.Advance();
		});
	}
	for (auto& worker : workers) worker.join();
	EXPECT_TRUE(progress.Finish(true));
	const std::string output = testing::internal::GetCapturedStderr();
	const std::string label = "Percentage:";
	size_t cursor = 0;
	double previous = -1;
	int reports = 0;
	while ((cursor = output.find(label, cursor)) != std::string::npos) {
		cursor += label.size();
		const double percent = std::strtod(output.c_str() + cursor, nullptr);
		EXPECT_GT(percent, previous);
		EXPECT_LE(percent, 100);
		previous = percent;
		++reports;
	}
	EXPECT_EQ(previous, 100);
	EXPECT_LE(reports, 1001);
}

TEST(ProgressReporter, EmptyWorkCanFinishButOvercountIsAnError) {
	testing::internal::CaptureStderr();
	ProgressReporter empty("Empty work", 0);
	EXPECT_TRUE(empty.Finish(true));
	ProgressReporter overcount("Overcount", 1);
	overcount.Advance();
	overcount.Advance();
	EXPECT_FALSE(overcount.Finish(true));
	const std::string output = testing::internal::GetCapturedStderr();
	EXPECT_NE(output.find("Overcount failed after 1 of 1 work units."), std::string::npos);
}
