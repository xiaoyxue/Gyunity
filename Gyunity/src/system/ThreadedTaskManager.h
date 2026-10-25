#pragma once

#include "common/Core.h"
#include <chrono>

namespace tbb { class task_group_context; }

GYT_NAMESPACE_BEGIN

class ProgressReporter;

class ThreadedTaskManager {
public:
	enum class Status { Success, Failed, Cancelled };

	class Context {
	public:
		Context(const Context&) = delete;
		Context& operator=(const Context&) = delete;
		bool IsCancellationRequested() const;
		void RequestCancellation() const;

	private:
		friend class ThreadedTaskManager;
		explicit Context(tbb::task_group_context& group) : mGroup(group) {}
		tbb::task_group_context& mGroup;
	};

	struct Result {
		std::string mName;
		Status mStatus = Status::Failed;
		size_t mPlanned = 0, mStarted = 0, mCompleted = 0, mFailed = 0, mCancelled = 0;
		size_t mActive = 0, mPeakConcurrency = 0;
		int mConcurrencyLimit = 0;
		std::chrono::duration<double> mElapsed{ 0 };
	};

	// Callbacks must not throw or launch unjoined work. Context is valid only during Run.
	using Task = std::function<Status(size_t, const Context&)>;

	static int DefaultConcurrency();

	// Synchronous [begin, end) phase barrier. Progress is borrowed; only the caller may Finish it.
	static Result Run(const std::string& name, size_t begin, size_t end, const Task& task,
		int maxConcurrency = DefaultConcurrency(), ProgressReporter* progress = nullptr);
};

GYT_NAMESPACE_END
