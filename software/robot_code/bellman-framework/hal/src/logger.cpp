#include "logger.hpp"

namespace bellman
{
	namespace hal {
		Logger::Logger()
		{
			m_ds = DriverStation::GetInstance();
		}

		Logger* Logger::GetInstance()
		{
			if (instance == nullptr)
			{
				instance = new Logger();
			}
			return instance;
		}

		void Logger::LogMessage(std::string caller, std::string message) { Logger::LogMessage(caller, message, Logger::Level::kUnknown); }

		void Logger::LogMessage(std::string caller, std::string message, Logger::Level level)
		{
			std::string formatted_message = "[" + caller + "][" + LevelToString(level) + "] " + message; // Format the message

			std::cout << formatted_message << std::endl; // Put it on cout

			m_ds->SendLogMessage(LevelToString(level), caller, message); // Send it to the driver station
		}

		std::string Logger::LevelToString(Logger::Level level)
		{
			switch (level)
			{
			case Level::kDebug: return "DEBUG";
			case Level::kInfo: return "INFO";
			case Level::kWarning: return "WARNING";
			case Level::kError: return "ERROR";
			case Level::kFatal: return "FATAL";
			case Level::kUnknown: return "UNKNOWN";
			default: return "UNKNOWN";
			}
		}
	}

	hal::Logger* hal::Logger::instance = nullptr;
}
