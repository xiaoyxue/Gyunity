#include "ThreadedTaskManager.h"
#include "ProgressReporter.h"
#include "tbb/parallel_for.h"
#include "tbb/task_arena.h"
#include <atomic>

GYT_NAMESPACE_BEGIN

bool ThreadedTaskManager::Context::IsCancellationRequested() const {
	return mGroup.is_group_execution_cancelled();
}

void ThreadedTaskManager::Context::RequestCancellation() const {
	mGroup.cancel_group_execution();
}

int ThreadedTaskManager::DefaultConcurrency() {
#ifdef _DEBUG
	return 1;
#else
	return -1;
#endif
}

ThreadedTaskManager::Result ThreadedTaskManager::Run(const std::string& name,
	size_t begin, size_t end, const Task& task, int maxConcurrency, ProgressReporter* progress) {
	const auto start = std::chrono::steady_clock::now();
	Result result;
	result.mName = name;
	result.mPlanned = end >= begin ? end - begin : 0;
	result.mCancelled = result.mPlanned;
	const auto finish = [&]() {
		result.mElapsed = std::chrono::steady_clock::now() - start;
		return result;
	};

	const char* error = nullptr;
	if (name.find_first_not_of(" \t\r\n") == std::string::npos)
		error = "a nonempty group name is required";
	else if (end < begin)
		error = "the range end must not precede its begin";
	else if (maxConcurrency != -1 && maxConcurrency <= 0)
		error = "concurrency must be positive or -1 (automatic)";
	else if (!task)
		error = "a task callback is required";
	if (error) {
		GetDefaultLogger()->Error(fmt::format("Threaded task group '{}': {}.", name, error), false);
		return finish();
	}

	std::atomic<size_t> started(0), completed(0), failed(0), active(0), peak(0);
	{
		tbb::task_arena arena(maxConcurrency);
		arena.initialize();
		result.mConcurrencyLimit = arena.max_concurrency();
		tbb::task_group_context group(tbb::task_group_context::isolated);
		const Context context(group);
		arena.execute([&]() {
			tbb::parallel_for(tbb::blocked_range<size_t>(begin, end),
				[&](const tbb::blocked_range<size_t>& range) {
					for (size_t index = range.begin(); index != range.end(); ++index) {
						if (context.IsCancellationRequested()) break;
						started.fetch_add(1, std::memory_order_relaxed);
						const size_t running = active.fetch_add(1, std::memory_order_relaxed) + 1;
						size_t previous = peak.load(std::memory_order_relaxed);
						while (previous < running && !peak.compare_exchange_weak(previous, running,
							std::memory_order_relaxed)) {}

						// Nested TBB work must not reenter another callback while this one is live.
						const Status status = tbb::this_task_arena::isolate([&]() {
							return task(index, context);
						});
						switch (status) {
						case Status::Success:
							completed.fetch_add(1, std::memory_order_relaxed);
							if (progress) progress->Advance();
							break;
						case Status::Cancelled:
							context.RequestCancellation();
							break;
						case Status::Failed:
						default:
							failed.fetch_add(1, std::memory_order_relaxed);
							context.RequestCancellation();
							break;
						}
						active.fetch_sub(1, std::memory_order_relaxed);
					}
				}, group);
		});
		result.mStatus = failed.load() != 0 ? Status::Failed :
			(context.IsCancellationRequested() ? Status::Cancelled : Status::Success);
	}
	result.mStarted = started.load();
	result.mCompleted = completed.load();
	result.mFailed = failed.load();
	result.mCancelled = result.mPlanned - result.mCompleted - result.mFailed;
	result.mActive = active.load();
	result.mPeakConcurrency = peak.load();
	if (result.mFailed != 0) {
		GetDefaultLogger()->Error(fmt::format(
			"Threaded task group '{}': {} callback(s) returned Failed or an invalid status.",
			name, result.mFailed), false);
	}
	return finish();
}

GYT_NAMESPACE_END
