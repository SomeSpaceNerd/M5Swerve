#pragma once

#include "driver_station.hpp"
#include <string>
#include <iostream>

namespace bellman
{
	namespace hal
	{
		class Logger
		{
		public:
			enum class Level
			{
				kDebug,
				kInfo,
				kWarning,
				kError,
				kFatal,
				kUnknown,
			};

			static Logger* GetInstance();

			void LogMessage(std::string caller, std::string message);
			void LogMessage(std::string caller, std::string message, Logger::Level level);

		private:
			Logger();
			static Logger* instance;

			DriverStation* m_ds;

			std::string LevelToString(Logger::Level level);
		};
	}
}

