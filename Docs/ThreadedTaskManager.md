## Scoped threaded task groups
`system\ThreadedTaskManager.h` adds a small, synchronous TBB-backed preparation
API for future staged renderers. It does not replace `ParallelFor` or migrate
LT, SPPM, BDPT, their RNGs, `ThreadIndex`, `ThreadsNumber`, or `Spinlock`.
Like Yuanming's `ThreadedTaskManager`, it wraps scoped `task_arena` and
`parallel_for`, not a persistent custom pool, async queue, or dependency graph.

```cpp
bool PrepareValues(std::vector<size_t>& values, ProgressReporter& progress) {
    using Tasks = ThreadedTaskManager;
    const auto phase = Tasks::Run("Prepare values", 0, values.size(),
        [&](size_t index, const Tasks::Context& context) {
            if (context.IsCancellationRequested()) return Tasks::Status::Cancelled;
            values[index] = index;
            return Tasks::Status::Success;
        }, Tasks::DefaultConcurrency(), &progress);
    return phase.mStatus == Tasks::Status::Success;
}
```

- `Run(name, begin, end, task, maxConcurrency, progress)` processes `[begin, end)`
  using stable `size_t` task indices, including nonzero offsets. Task index, not
  the arena-local `ThreadIndex()`, is the identity for data and RNG ownership.
  There is no promised order, worker affinity, or minimum number of workers.
- The optional concurrency argument defaults to **1 in Debug**, **-1 (TBB
  automatic) in Release**. A positive maximum, or explicit `-1`, overrides either
  build's default only for this run. No process-global control is installed.
  Nested TBB work is isolated from sibling callbacks to avoid callback reentry.
- Returning is a phase barrier: all started callbacks and their progress updates
  have drained, including on failure/cancellation. Callbacks must be thread-safe,
  must not throw, and must not launch unjoined work or retain the run's context.
  Exceptions from unexpected infrastructure failures are not translated into
  task statuses. This API is not an exception-handling framework.
- A callback returns `Status::Success` for a completed unit, `Failed` for an
  explicit failure, or `Cancelled` for an abandoned unit. Either of the latter
  requests cancellation of remaining work. `Context::RequestCancellation()` is
  also thread-safe; long-running callbacks should poll `IsCancellationRequested()`.
  Cancellation is cooperative, not forced termination: a callback racing with a
  request may still start, and started callbacks must finish or cooperate.
- Group status is `Failed` if any callback fails (an invalid enum value also
  fails), otherwise `Cancelled` if cancellation was requested, otherwise
  `Success`. Failure takes precedence. Completed callbacks remain completed even
  after a cancellation request; a late request can therefore produce `Cancelled`
  with zero cancelled units. Independent runs always have fresh state.
- Empty ranges succeed, but validation still applies. A blank name, reversed
  range, null/empty callback, or limit other than a positive integer or `-1`
  logs an error without throwing and returns `Failed` before starting work.
  Callback failures are logged once per group, not per task.

`Result` contains the group name and final coarse-task statistics:
`mPlanned = end - begin`, `mStarted` counts admitted callback invocations,
`mCompleted` counts successful callbacks, and `mFailed` counts failed callbacks.
`mCancelled` includes **both abandoned started tasks and all not-started tasks**.
Thus `planned = completed + failed + cancelled`, and
`started - completed - failed` counts abandoned started tasks.
`mActive` is zero after the barrier; `mPeakConcurrency` is the peak number of live
callbacks (including completion bookkeeping). `mConcurrencyLimit` is the resolved
arena maximum. `mElapsed` is a `steady_clock` duration in seconds, including setup
and draining. Validation failures have no arena/started/failed tasks and cancel
the whole valid range; a reversed range has zero planned tasks.

The optional `ProgressReporter*` is borrowed and must outlive the run. The caller
must provide enough remaining units across all phases. Only successful callbacks
call `Advance()` once; failed, abandoned, skipped, and empty work never advances.
The manager **never calls `Finish`**, even for a successful final phase. The render
controller must stop dependent phases on non-success and call `Finish(false)`,
or `Finish(true)` only after the entire render succeeds. No per-vertex telemetry
or extra success logging is added.
