#include "gtest/gtest.h"
#include "system/ThreadedTaskManager.h"
#include "system/ProgressReporter.h"
#include "system/Threading.h"
#include <condition_variable>
#include <limits>
#include <mutex>

using namespace Gyunity;

namespace {

using Tasks = ThreadedTaskManager;
using Status = Tasks::Status;
using Context = Tasks::Context;
using Clock = std::chrono::steady_clock;

Status Complete(size_t, const Context&) { return Status::Success; }

void ExpectAccounting(const Tasks::Result& result) {
	EXPECT_LE(result.mStarted, result.mPlanned);
	EXPECT_LE(result.mCompleted + result.mFailed, result.mStarted);
	EXPECT_EQ(result.mCompleted + result.mFailed + result.mCancelled, result.mPlanned);
	EXPECT_EQ(result.mActive, 0u);
	EXPECT_LE(result.mPeakConcurrency, result.mStarted);
	EXPECT_GE(result.mElapsed.count(), 0);
	if (result.mConcurrencyLimit > 0)
		EXPECT_LE(result.mPeakConcurrency, static_cast<size_t>(result.mConcurrencyLimit));
	if (result.mFailed != 0) EXPECT_EQ(result.mStatus, Status::Failed);
	if (result.mStatus == Status::Success) {
		EXPECT_EQ(result.mCompleted, result.mPlanned);
		EXPECT_EQ(result.mCancelled, 0u);
		EXPECT_EQ(result.mFailed, 0u);
	}
}

void UpdatePeak(std::atomic<size_t>& peak, size_t current) {
	size_t previous = peak.load();
	while (previous < current && !peak.compare_exchange_weak(previous, current)) {}
}

}

TEST(ThreadedTaskManager, EmptyRangesAreSuccessfulWithoutCallbacks) {
	for (size_t offset : { size_t(0), size_t(23) }) {
		size_t calls = 0;
		const auto result = Tasks::Run("Empty phase", offset, offset,
			[&](size_t, const Context&) { ++calls; return Status::Success; }, 1);
		EXPECT_EQ(result.mName, "Empty phase");
		EXPECT_EQ(result.mStatus, Status::Success);
		EXPECT_EQ(result.mPlanned, 0u);
		EXPECT_EQ(result.mStarted, 0u);
		EXPECT_EQ(result.mPeakConcurrency, 0u);
		EXPECT_EQ(calls, 0u);
		ExpectAccounting(result);
	}
}

TEST(ThreadedTaskManager, NonzeroOffsetVisitsStableIndicesExactlyOnce) {
	const size_t begin = 37, count = 4096;
	std::vector<std::atomic<int>> visits(count);
	for (auto& visit : visits) visit = 0;
	std::atomic<int> invalid(0);
	const auto result = Tasks::Run("Indexed phase", begin, begin + count,
		[&](size_t index, const Context& context) {
			if (index < begin || index >= begin + count || context.IsCancellationRequested())
				++invalid;
			else
				++visits[index - begin];
			return Status::Success;
		}, 4);
	EXPECT_EQ(result.mStatus, Status::Success);
	EXPECT_EQ(result.mPlanned, count);
	EXPECT_EQ(result.mStarted, count);
	EXPECT_EQ(result.mCompleted, count);
	EXPECT_EQ(invalid.load(), 0);
	for (const auto& visit : visits) EXPECT_EQ(visit.load(), 1);
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, SizeTIndicesNearTheUpperBoundDoNotNarrowOrOverflow) {
	const size_t end = (std::numeric_limits<size_t>::max)(), begin = end - 15;
	std::vector<std::atomic<int>> visits(15);
	for (auto& visit : visits) visit = 0;
	const auto result = Tasks::Run("Large indices", begin, end,
		[&](size_t index, const Context&) {
			if (index < begin || index >= end) return Status::Failed;
			++visits[index - begin];
			return Status::Success;
		}, 2);
	EXPECT_EQ(result.mStatus, Status::Success);
	EXPECT_EQ(result.mCompleted, 15u);
	for (const auto& visit : visits) EXPECT_EQ(visit.load(), 1);
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, InvalidConfigurationFailsBeforeStartingEvenForEmptyWork) {
	std::atomic<int> calls(0);
	const Tasks::Task task = [&](size_t, const Context&) { ++calls; return Status::Success; };
	const auto check = [&](const std::string& name, size_t begin, size_t end,
		const Tasks::Task& callback, int limit) {
		const auto result = Tasks::Run(name, begin, end, callback, limit);
		EXPECT_EQ(result.mStatus, Status::Failed);
		EXPECT_EQ(result.mPlanned, end >= begin ? end - begin : 0);
		EXPECT_EQ(result.mStarted, 0u);
		EXPECT_EQ(result.mFailed, 0u);
		EXPECT_EQ(result.mCancelled, result.mPlanned);
		EXPECT_EQ(result.mConcurrencyLimit, 0);
		ExpectAccounting(result);
	};
	for (size_t end : { size_t(0), size_t(8) }) {
		for (int limit : { 0, -2, (std::numeric_limits<int>::min)() })
			check("Invalid limit", 0, end, task, limit);
		check("", 0, end, task, 1);
		check(" \t\r\n", 0, end, task, 1);
		check("Missing callback", 0, end, Tasks::Task(), 1);
	}
	check("Invalid range", 9, 8, task, 1);
	EXPECT_EQ(calls.load(), 0);
}

TEST(ThreadedTaskManager, DefaultConcurrencyPreservesTheBuildConfiguration) {
#ifdef _DEBUG
	EXPECT_EQ(Tasks::DefaultConcurrency(), 1);
	const int expected = 1;
#else
	EXPECT_EQ(Tasks::DefaultConcurrency(), -1);
	tbb::task_arena automatic;
	automatic.initialize();
	const int expected = automatic.max_concurrency();
#endif
	std::atomic<int> mismatches(0);
	const auto result = Tasks::Run("Default phase", 0, 128, [&](size_t, const Context&) {
		if (tbb::this_task_arena::max_concurrency() != expected) ++mismatches;
		return Status::Success;
	});
	EXPECT_EQ(result.mStatus, Status::Success);
	EXPECT_EQ(result.mConcurrencyLimit, expected);
	EXPECT_EQ(mismatches.load(), 0);
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, ExplicitAndAutomaticLimitsBoundObservedConcurrency) {
	for (int limit : { 1, 2, 4, -1 }) {
		SCOPED_TRACE(limit);
		std::atomic<size_t> active(0), peak(0);
		std::atomic<int> arenaLimit(0);
		const auto result = Tasks::Run("Limited phase", 0, 256,
			[&](size_t, const Context&) {
				UpdatePeak(peak, ++active);
				arenaLimit = tbb::this_task_arena::max_concurrency();
				for (int i = 0; i < 32; ++i) std::this_thread::yield();
				--active;
				return Status::Success;
			}, limit);
		EXPECT_EQ(result.mStatus, Status::Success);
		EXPECT_EQ(active.load(), 0u);
		EXPECT_GT(peak.load(), 0u);
		EXPECT_LE(peak.load(), result.mPeakConcurrency);
		EXPECT_EQ(arenaLimit.load(), result.mConcurrencyLimit);
		if (limit > 0) EXPECT_EQ(result.mConcurrencyLimit, limit);
		ExpectAccounting(result);
	}
}

TEST(ThreadedTaskManager, ElapsedTimeIncludesCallbackWorkAndIsBoundedByTheCall) {
	std::chrono::duration<double> work(0);
	const auto start = Clock::now();
	const auto result = Tasks::Run("Timed phase", 0, 1, [&](size_t, const Context&) {
		const auto taskStart = Clock::now();
		std::this_thread::sleep_for(std::chrono::milliseconds(1));
		work = Clock::now() - taskStart;
		return Status::Success;
	}, 1);
	const std::chrono::duration<double> elapsed = Clock::now() - start;
	EXPECT_EQ(result.mStatus, Status::Success);
	EXPECT_GE(result.mElapsed.count(), work.count());
	EXPECT_LE(result.mElapsed.count(), elapsed.count());
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, ScopedLimitsLeaveTheEnclosingAndLegacyArenasUnchanged) {
	tbb::task_arena enclosing(3);
	enclosing.execute([&]() {
		EXPECT_EQ(tbb::this_task_arena::max_concurrency(), 3);
		const auto result = Tasks::Run("Inner phase", 0, 32, Complete, 1);
		EXPECT_EQ(result.mConcurrencyLimit, 1);
		EXPECT_EQ(tbb::this_task_arena::max_concurrency(), 3);
		std::atomic<int> calls(0), mismatches(0);
		ParallelFor(0, 64, [&](int) {
#ifdef _DEBUG
			const int expected = 1;
#else
			const int expected = 3;
#endif
			if (tbb::this_task_arena::max_concurrency() != expected || ThreadIndex() < 0)
				++mismatches;
			++calls;
		});
		EXPECT_EQ(calls.load(), 64);
		EXPECT_EQ(mismatches.load(), 0);
		EXPECT_EQ(tbb::this_task_arena::max_concurrency(), 3);
		ExpectAccounting(result);
	});
	EXPECT_EQ(ThreadsNumber(), NumSystemCores());
}

TEST(ThreadedTaskManager, NestedTbbWorkDoesNotReenterOtherGroupCallbacks) {
	std::atomic<size_t> active(0), peak(0), innerCalls(0);
	const auto result = Tasks::Run("Nested parallel work", 0, 64,
		[&](size_t, const Context&) {
			UpdatePeak(peak, ++active);
			tbb::parallel_for(0, 32, [&](int) {
				++innerCalls;
				std::this_thread::yield();
			});
			--active;
			return Status::Success;
		}, 2);
	EXPECT_EQ(result.mStatus, Status::Success);
	EXPECT_EQ(active.load(), 0u);
	EXPECT_LE(peak.load(), 2u);
	EXPECT_EQ(innerCalls.load(), 64u * 32);
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, ExplicitFailureCancelsNotStartedWork) {
	size_t calls = 0;
	const auto result = Tasks::Run("Failed phase", 50, 67,
		[&](size_t, const Context&) { return ++calls == 3 ? Status::Failed : Status::Success; }, 1);
	EXPECT_EQ(result.mStatus, Status::Failed);
	EXPECT_EQ(result.mStarted, 3u);
	EXPECT_EQ(result.mCompleted, 2u);
	EXPECT_EQ(result.mFailed, 1u);
	EXPECT_EQ(result.mCancelled, 14u);
	EXPECT_EQ(calls, 3u);
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, CancellationSkipsTheRestOfAVeryLargeRange) {
	const size_t count = (std::numeric_limits<size_t>::max)();
	const auto result = Tasks::Run("Large cancelled phase", 0, count,
		[](size_t, const Context&) { return Status::Cancelled; }, 1);
	EXPECT_EQ(result.mStatus, Status::Cancelled);
	EXPECT_EQ(result.mPlanned, count);
	EXPECT_EQ(result.mStarted, 1u);
	EXPECT_EQ(result.mCancelled, count);
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, CallbackCancellationDrainsWorkBeforeReturning) {
	std::mutex mutex;
	std::condition_variable changed;
	bool paused = false, release = false;
	std::atomic<bool> returned(false);
	std::atomic<size_t> calls(0), active(0), exited(0), afterReturn(0);
	size_t activeAtReturn = 0, exitedAtReturn = 0;
	Tasks::Result result;
	std::thread runner([&]() {
		result = Tasks::Run("Cancelled phase", 0, 256, [&](size_t, const Context& context) {
			if (returned.load()) ++afterReturn;
			++active;
			if (calls.fetch_add(1) == 0) {
				context.RequestCancellation();
				std::unique_lock<std::mutex> lock(mutex);
				paused = true;
				changed.notify_one();
				changed.wait(lock, [&]() { return release; });
			}
			--active;
			++exited;
			if (returned.load()) ++afterReturn;
			return Status::Cancelled;
		}, 4);
		activeAtReturn = active.load();
		exitedAtReturn = exited.load();
		returned = true;
	});
	{
		std::unique_lock<std::mutex> lock(mutex);
		const bool signalled = changed.wait_for(lock, std::chrono::seconds(5), [&]() { return paused; });
		EXPECT_TRUE(signalled);
		EXPECT_FALSE(returned.load());
		EXPECT_GT(active.load(), 0u);
		release = true;
		changed.notify_one();
	}
	runner.join();
	EXPECT_TRUE(returned.load());
	EXPECT_EQ(result.mStatus, Status::Cancelled);
	EXPECT_EQ(result.mStarted, calls.load());
	EXPECT_EQ(exited.load(), calls.load());
	EXPECT_EQ(exitedAtReturn, result.mStarted);
	EXPECT_EQ(activeAtReturn, 0u);
	EXPECT_EQ(active.load(), 0u);
	EXPECT_EQ(afterReturn.load(), 0u);
	EXPECT_EQ(result.mCompleted, 0u);
	EXPECT_EQ(result.mFailed, 0u);
	EXPECT_EQ(result.mCancelled, 256u);
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, FailureNotifiesAnyInflightCallbacksAndDrainsTheirCleanup) {
	std::atomic<size_t> calls(0), observed(0), exited(0);
	std::atomic<bool> returned(false);
	std::atomic<size_t> afterReturn(0);
	const auto result = Tasks::Run("Failure with in-flight work", 0, 256,
		[&](size_t, const Context& context) {
			if (returned.load()) ++afterReturn;
			const bool first = calls.fetch_add(1) == 0;
			if (first) {
				// Give peers a chance to enter, without requiring any extra worker to exist.
				std::this_thread::sleep_for(std::chrono::milliseconds(2));
			}
			else {
				const auto deadline = Clock::now() + std::chrono::seconds(5);
				while (!context.IsCancellationRequested() && Clock::now() < deadline)
					std::this_thread::yield();
				if (context.IsCancellationRequested()) ++observed;
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
			++exited;
			if (returned.load()) ++afterReturn;
			return first ? Status::Failed : Status::Cancelled;
		}, 4);
	returned = true;
	EXPECT_EQ(result.mStatus, Status::Failed);
	EXPECT_EQ(result.mFailed, 1u);
	EXPECT_EQ(result.mCompleted, 0u);
	EXPECT_EQ(result.mStarted, calls.load());
	EXPECT_EQ(exited.load(), calls.load());
	EXPECT_EQ(observed.load(), calls.load() - 1);
	EXPECT_EQ(afterReturn.load(), 0u);
	RecordProperty("in_flight_peers", static_cast<int>(observed.load()));
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, CancelledTaskResultAlsoRequestsGroupCancellation) {
	size_t calls = 0;
	const auto result = Tasks::Run("Task requests cancellation", 0, 13,
		[&](size_t, const Context&) { return ++calls == 2 ? Status::Cancelled : Status::Success; }, 1);
	EXPECT_EQ(result.mStatus, Status::Cancelled);
	EXPECT_EQ(result.mStarted, 2u);
	EXPECT_EQ(result.mCompleted, 1u);
	EXPECT_EQ(result.mFailed, 0u);
	EXPECT_EQ(result.mCancelled, 12u);
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, FailureTakesPrecedenceOverCancellationAndInvalidStatusIsFailure) {
	for (Status status : { Status::Failed, static_cast<Status>(-1) }) {
		const auto result = Tasks::Run("Failure precedence", 0, 8,
			[&](size_t, const Context& context) {
				context.RequestCancellation();
				EXPECT_TRUE(context.IsCancellationRequested());
				return status;
			}, 1);
		EXPECT_EQ(result.mStatus, Status::Failed);
		EXPECT_EQ(result.mStarted, 1u);
		EXPECT_EQ(result.mFailed, 1u);
		EXPECT_EQ(result.mCancelled, 7u);
		ExpectAccounting(result);
	}
}

TEST(ThreadedTaskManager, CompletedTaskRemainsCompletedButCancellationCannotReportSuccess) {
	const auto result = Tasks::Run("Late cancellation", 0, 1,
		[](size_t, const Context& context) {
			context.RequestCancellation();
			return Status::Success;
		}, 1);
	EXPECT_EQ(result.mStatus, Status::Cancelled);
	EXPECT_EQ(result.mCompleted, 1u);
	EXPECT_EQ(result.mCancelled, 0u);
	ExpectAccounting(result);
}

TEST(ThreadedTaskManager, IndependentRunsAlwaysHaveFreshCancellationAndCounters) {
	for (int repeat = 0; repeat < 3; ++repeat) {
		for (Status status : { Status::Failed, Status::Cancelled, Status::Success }) {
			const auto previous = Tasks::Run("Previous phase", 0, 16,
				[&](size_t, const Context&) { return status; }, 2);
			EXPECT_EQ(previous.mStatus, status);
			ExpectAccounting(previous);
			std::atomic<int> stale(0);
			const auto next = Tasks::Run("Independent phase", 17, 40,
				[&](size_t, const Context& context) {
					if (context.IsCancellationRequested()) ++stale;
					return Status::Success;
				}, 2);
			EXPECT_EQ(next.mName, "Independent phase");
			EXPECT_EQ(next.mStatus, Status::Success);
			EXPECT_EQ(next.mCompleted, 23u);
			EXPECT_EQ(stale.load(), 0);
			ExpectAccounting(next);
		}
	}
}

TEST(ThreadedTaskManager, BorrowedProgressAdvancesCompletedUnitsAcrossPhasesWithoutFinishing) {
	testing::internal::CaptureStderr();
	ProgressReporter progress("Two phases", 6);
	const auto first = Tasks::Run("First phase", 0, 2, Complete, 2, &progress);
	const auto second = Tasks::Run("Second phase", 20, 24, Complete, 2, &progress);
	const auto running = testing::internal::GetCapturedStderr();
	EXPECT_EQ(first.mStatus, Status::Success);
	EXPECT_EQ(second.mStatus, Status::Success);
	EXPECT_NE(running.find(" 33.3%"), std::string::npos);
	EXPECT_NE(running.find(" 99.9%"), std::string::npos);
	EXPECT_EQ(running.find("100.0%"), std::string::npos);
	EXPECT_EQ(running.find("failed after"), std::string::npos);
	testing::internal::CaptureStderr();
	EXPECT_TRUE(progress.Finish(true));
	const auto finished = testing::internal::GetCapturedStderr();
	EXPECT_NE(finished.find("100.0%"), std::string::npos);
}

TEST(ThreadedTaskManager, FailedAndCancelledUnitsDoNotAdvanceOrFinishBorrowedProgress) {
	for (Status status : { Status::Failed, Status::Cancelled }) {
		testing::internal::CaptureStderr();
		ProgressReporter progress("Interrupted phase", 3);
		size_t calls = 0;
		const auto result = Tasks::Run("Interrupted phase", 0, 3,
			[&](size_t, const Context&) { return ++calls == 1 ? Status::Success : status; }, 1, &progress);
		const auto running = testing::internal::GetCapturedStderr();
		EXPECT_EQ(result.mStatus, status);
		EXPECT_EQ(result.mCompleted, 1u);
		EXPECT_NE(running.find(" 33.3%"), std::string::npos);
		EXPECT_EQ(running.find(" 66.6%"), std::string::npos);
		EXPECT_EQ(running.find("100.0%"), std::string::npos);
		EXPECT_EQ(running.find("failed after"), std::string::npos);
		testing::internal::CaptureStderr();
		EXPECT_FALSE(progress.Finish(false));
		const auto finished = testing::internal::GetCapturedStderr();
		EXPECT_NE(finished.find("failed after 1 of 3 work units."), std::string::npos);
		EXPECT_EQ(finished.find("100.0%"), std::string::npos);
		ExpectAccounting(result);
	}
}

TEST(ThreadedTaskManager, EmptyAndInvalidRunsLeaveProgressOwnershipWithTheCaller) {
	testing::internal::CaptureStderr();
	ProgressReporter progress("Unused progress", 1);
	const auto empty = Tasks::Run("Empty phase", 7, 7, Complete, 1, &progress);
	const auto invalid = Tasks::Run("Invalid phase", 0, 1, Complete, 0, &progress);
	const auto running = testing::internal::GetCapturedStderr();
	EXPECT_EQ(empty.mStatus, Status::Success);
	EXPECT_EQ(invalid.mStatus, Status::Failed);
	EXPECT_EQ(running.find(" 99.9%"), std::string::npos);
	EXPECT_EQ(running.find("100.0%"), std::string::npos);
	EXPECT_EQ(running.find("failed after"), std::string::npos);
	testing::internal::CaptureStderr();
	progress.Advance();
	EXPECT_TRUE(progress.Finish(true));
	const auto finished = testing::internal::GetCapturedStderr();
	EXPECT_NE(finished.find("100.0%"), std::string::npos);
}
