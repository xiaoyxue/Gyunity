#include "Logger.h"
#include "spdlog/spdlog.h"
#include "spdlog/sinks/stdout_color_sinks.h"

GYT_NAMESPACE_BEGIN


void Logger::Info(const std::string& msg) const
{
	mConsole->info(msg);
}

void Logger::Warn(const std::string& msg) const
{
	mConsole->warn(msg);
}

void Logger::Debug(const std::string& msg) const
{
	mConsole->debug(msg);
}

void Logger::Error(const std::string& msg, bool raiseException /*= true*/) const
{
	mConsole->error(msg);
	if (raiseException)
		throw std::exception(msg.c_str());
}

void Logger::Critical(const std::string& msg, bool raiseException /*= true*/) const
{
	mConsole->critical(msg);
	if (raiseException)
		throw std::exception(msg.c_str());
}

void Logger::Flush() const
{
	mConsole->flush();
}

void Logger::SetLevel(const std::string& levelName /*= "Info"*/)
{
	auto newLevel = LevelFromString(levelName);
	mLevelName = levelName;
	spdlog::set_level(spdlog::level::level_enum(newLevel));
}


int Logger::LevelFromString(const std::string& levelName)
{
	if (levelName == "Trace") {
		return spdlog::level::trace;
	}
	else if (levelName == "Debug") {
		return spdlog::level::debug;
	}
	else if (levelName == "Info") {
		return spdlog::level::info;
	}
	else if (levelName == "Warn") {
		return spdlog::level::warn;
	}
	else if (levelName == "Error") {
		return spdlog::level::err;
	}
	else if (levelName == "Critical") {
		return spdlog::level::critical;
	}
	else if (levelName == "Off") {
		return spdlog::level::off;
	}
	else {
		GYT_ERROR(fmt::format("Level name \"{}\" is not in [Trace, Debug, Info, Warn, Error, Critical, Off]", levelName));
	}
}

Logger::Logger()
{

}

Logger::~Logger()
{

}

DefaultLogger::DefaultLogger()
{
	mConsole = spdlog::stdout_color_mt("ConsoleLogger");
	mConsole->set_pattern("%^[%l] [%D %X.%e] %v%$");
	SetLevel();
}

InfoLogger::InfoLogger()
{
	mConsole = spdlog::stdout_color_mt("InfoLogger");
	mConsole->set_pattern("%v");
	SetLevel();

}

Logger* GetDefaultLogger()
{
	static DefaultLogger defaultLogger;
	return &defaultLogger;
}

Logger* GetInfoLogger()
{
	static InfoLogger infoLogger;
	return &infoLogger;
}


GYT_NAMESPACE_END