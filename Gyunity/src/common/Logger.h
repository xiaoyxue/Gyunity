#pragma once

#include "Core.h"
#include "spdlog/common.h"
#include "fmt/fmt.h"

namespace spdlog {
	class logger;
}

GYT_NAMESPACE_BEGIN

class Logger {
protected:
	std::string mLevelName;
	std::shared_ptr<spdlog::logger> mConsole;
public:
	Logger();
	virtual ~Logger();
	virtual void Info(const std::string& msg) const;
	virtual void Warn(const std::string& msg) const;
	virtual void Debug(const std::string& msg) const;
	virtual void Error(const std::string& msg, bool raiseException = true) const;
	virtual void Critical(const std::string& msg, bool raiseException = true) const;
	virtual void Flush() const;
	virtual void SetLevel(const std::string& levelName = "Info");
	static int LevelFromString(const std::string& levelName);
};

class DefaultLogger : public Logger
{
public:
	DefaultLogger();
};

class InfoLogger : public Logger
{
public:
	InfoLogger();
};

Logger* GetDefaultLogger();
Logger* GetInfoLogger();

GYT_NAMESPACE_END