#pragma once

#include <string>
#include <memory>
#include <unordered_map>

namespace bellman
{
	class RobotMode
	{
		public:
			virtual void Init() = 0; // Called when transitioning into the enabled state in this opmode
			virtual void Periodic() = 0; // Called every 10ms while in this opmode
	};

	namespace hal
	{
		class State
		{
			public:
				static State* GetInstance();

				// OpModes
				void RegisterMode(std::string name, std::shared_ptr<RobotMode> mode);
				std::unordered_map<std::string, std::shared_ptr<RobotMode>>& GetModes();

				// Enable/Disable state
				bool GetEnabled();
				void Disable();
				void Enable();

				// Emergency stop state
				void EStop();
				bool GetEStopped();

			private:
				State();
				static State* instance;

				std::unordered_map<std::string, std::shared_ptr<RobotMode>> m_opmodes;

				bool m_enabled;
				bool m_e_stopped;
		};
	}
}