#pragma once
#include "common/Core.h"
#include <mutex>

GYT_NAMESPACE_BEGIN

class ProgressReporter {
public:
	ProgressReporter(const std::string& label, size_t total);
	void Advance();
	// Optional phase label; does not change completed work or finish the report.
	void SetPhase(const std::string& label);
	// Call after all workers have joined. Only complete, successful work reports 100%.
	bool Finish(bool success);

private:
	void PrintPercentage(int tenths) const;

	std::string mLabel;
	size_t mTotal, mCompleted = 0;
	int mReportedTenths = 0;
	bool mFinished = false, mSucceeded = false, mInvalid = false;
	std::mutex mMutex;
};

GYT_NAMESPACE_END
