#include "state_control.hpp"

namespace bellman
{
	namespace hal
	{
		State::State()
		{
			m_enabled = false;
			m_e_stopped = false;
		}

		State* State::GetInstance()
		{
			if (instance == nullptr)
			{
				instance = new State();
			}
			return instance;
		}

		/**
		* @brief Register a robot opmode
		* 
		* @param name The friendly name of the opmode
		* @param mode The robot mode object derived from RobotMode
		*/
		void State::RegisterMode(std::string name, std::shared_ptr<RobotMode> mode) { m_opmodes[name] = mode; }

		/**
		* @brief Returns an unordered map of every registered robot opmode's name and shared pointer
		*/
		std::unordered_map<std::string, std::shared_ptr<RobotMode>>& State::GetModes() { return m_opmodes; }

		/**
		* @brief Gets the robot's current enable state
		*
		* @returns True if the robot is enabled
		*/
		bool State::GetEnabled() { return m_enabled; }

		/**
		* @brief Immediately disables the robot
		*/
		void State::Disable() { m_enabled = false; }

		/**
		* @brief Immediately enables the robot
		* @brief THIS FUNCTION CANNOT BE CALLED FROM ROBOT CODE
		*/
		void State::Enable() { m_enabled = true; }

		/**
		* @brief Immediately emergency stops the robot and prevents it from being enabled until it is power cycled
		*/
		void State::EStop() { m_e_stopped = true; }

		/**
		* @brief Gets the robot's e-stop state
		* 
		* @returns True if the robot is emergency stopped
		*/
		bool State::GetEStopped() { return m_e_stopped; }
	}

	hal::State* hal::State::instance = nullptr;
}