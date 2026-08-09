// This is the header file for your robot program
#pragma once

// All of your includes should be here
#include "hal.hpp"
#include "state_control.hpp"
#include "device.hpp"
#include <memory>
// CoreMP135/M5Swerve specific imports
#include "buses/i2c.hpp"
#include "components/stamp_io.hpp"
#include "gpio.hpp"
#include <chrono>
#include <thread>

#include <cmath>
#include "constants.hpp"
#include "buses/uart.hpp"
#include "components/sc_servo.hpp"
#include "components/roller485.hpp"
#include "logger.hpp"
#include "driver_station.hpp"
#include "components/ina228.hpp"
#include "components/bno055.hpp"

class Robot
{
	public:
		Robot(int dt);

		void RobotPeriodic(unsigned int loopcounts);
		void DisabledInit();
		void DisabledPeriodic();

		// Represents the state of a swerve module
		struct ModuleState
		{
			double angle_deg; // Mount-frame heading in degrees (0-360). After FoldModule, confined to the 180 deg window centered on the module's SERVO_ZERO
			double motor_rpm; // Signed RPM
		};

	private:
		int m_dt; // This variable is how often the periodic code is being run in milliseconds
		bellman::hal::IHal* m_hal;
		bellman::hal::State* m_state;

		// CoreMP135/M5Swerve specific code
		bellman::hal::GPIO* m_gpio;
		bellman::hal::bus::I2C* m_i2c;
		bellman::hal::component::StampIO* m_stamp_io;

		// Sensors
		bellman::hal::component::INA228* m_ina228;
		bellman::hal::component::BNO055* m_bno055;

		// Private functions
		void SetChassisSpeed(double vx, double vy, double psi, bool field);
		ModuleState CalculateModule(double vx_robot, double vy_robot, double psi_robot, double module_x, double  module_y);
		ModuleState FoldModule(const ModuleState& state, double servo_zero);
		int MountAngleToServoSteps(double mount_deg, double servo_zero);
		double NormalizeAngle360(double deg);
		double ShortestSignedAngle(double from_deg, double to_deg);
		void FieldToRobot(double* vx, double* vy);

		bellman::hal::bus::UART* m_steer_uart;
		bellman::hal::bus::UART* m_drive_rs485;
		bellman::hal::Logger* m_logger;
		bellman::hal::DriverStation* m_ds;

		// Drive motors
		bellman::hal::component::Roller485* m_fr_drive;
		bellman::hal::component::Roller485* m_br_drive;
		bellman::hal::component::Roller485* m_bl_drive;
		bellman::hal::component::Roller485* m_fl_drive;

		// Steer motors
		bellman::hal::component::SCServo* m_fr_steer;
		bellman::hal::component::SCServo* m_br_steer;
		bellman::hal::component::SCServo* m_bl_steer;
		bellman::hal::component::SCServo* m_fl_steer;

		// Module states
		ModuleState m_modules[4] = {};
		double m_drive_cmd[4] = { 0.0, 0.0, 0.0, 0.0 };

		// Active scale factors
		double m_xy_sfactor = 0.0;
		double m_r_sfactor = 0.0;

		// Field-mode variables
		float m_gyro_offset = 0.0f;

	// Opmodes
	class TeleopMode : public bellman::RobotMode
	{
		public:
			TeleopMode(Robot* r);
			void Init() override;
			void Periodic() override;

		private:
			Robot* m_robot;

			float ShapeStick(float stick, float deadband, float slope);

			bool m_field_mode = false;
			bool m_mode_toggled = false;
			bool m_sfactor_toggled = false; // Tracks the toggle button to prevent constantly switching since it isn't a toggle switch
			bool m_high_sfactor_active = true;
	};
};