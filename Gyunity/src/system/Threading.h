#pragma once

#include <thread>
#include <atomic>
#include "common/Core.h"
#include "tbb/parallel_for.h"

GYT_NAMESPACE_BEGIN

class Spinlock {
protected:
	std::atomic<bool> mLatch;

public:
	Spinlock() : Spinlock(false) {
	}

	Spinlock(bool flag) {
		mLatch.store(flag);
	}

	Spinlock(int flag) : Spinlock(flag != 0) {
	}

	void lock() {
		bool unlatched = false;
		while (!mLatch.compare_exchange_weak(unlatched, true,
			std::memory_order_acquire)) {
			unlatched = false;
		}
	}

	void unlock() {
		mLatch.store(false, std::memory_order_release);
	}

	Spinlock(const Spinlock &o) {
		// We just ignore racing condition here...
		mLatch.store(o.mLatch.load());
	}

	Spinlock &operator=(const Spinlock &o) {
		// We just ignore racing condition here...
		mLatch.store(o.mLatch.load());
		return *this;
	}
};


int ThreadIndex();
int NumSystemCores();
int ThreadsNumber();

template<typename Range, typename T>
void ParallelFor(Range begin, Range end, const T &target) {
	//tbb::task_arena limited_arena(1);
	//limited_arena.execute([&]() {tbb::parallel_for(begin, end, target); });
#ifdef _DEBUG
	tbb::task_arena limited_arena(1);
	limited_arena.execute([&]() {tbb::parallel_for(begin, end, target); });
#else
	tbb::parallel_for(begin, end, target);

	//tbb::task_arena limited_arena(q);
	//limited_arena.execute([&]() {tbb::parallel_for(begin, end, target); });
#endif
}

GYT_NAMESPACE_END