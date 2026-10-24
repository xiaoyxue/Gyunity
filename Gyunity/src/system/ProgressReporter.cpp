#include "ProgressReporter.h"

GYT_NAMESPACE_BEGIN

ProgressReporter::ProgressReporter(const std::string& label, size_t total)
	: mLabel(label), mTotal(total) {
	std::fprintf(stderr, "%s ...\n", mLabel.c_str());
	PrintPercentage(0);
}

void ProgressReporter::PrintPercentage(int tenths) const {
	std::fprintf(stderr, "\rPercentage: %5.1f%%", tenths / 10.0);
	std::fflush(stderr);
}

void ProgressReporter::Advance() {
	std::lock_guard<std::mutex> lock(mMutex);
	if (mFinished || mCompleted == mTotal) {
		mInvalid = true;
		GetDefaultLogger()->Error("Progress cannot advance beyond its total or after finishing.", false);
		return;
	}
	++mCompleted;
	const int tenths = std::min(999, static_cast<int>(1000.0 * mCompleted / mTotal));
	if (tenths > mReportedTenths) {
		PrintPercentage(tenths);
		mReportedTenths = tenths;
	}
}

bool ProgressReporter::Finish(bool success) {
	std::lock_guard<std::mutex> lock(mMutex);
	if (mFinished) return mSucceeded;
	mSucceeded = success && mCompleted == mTotal && !mInvalid;
	if (mSucceeded) {
		PrintPercentage(1000);
		std::fprintf(stderr, "\n");
	}
	else {
		std::fprintf(stderr, "\n%s failed after %zu of %zu work units.\n",
			mLabel.c_str(), mCompleted, mTotal);
	}
	std::fflush(stderr);
	mFinished = true;
	return mSucceeded;
}

GYT_NAMESPACE_END
