#pragma once

namespace Constants
{
	const double PI = 3.14159265358979323846;

	// Motor ID constants
	namespace Motors
	{
		const int FR_DRIVE = 1;
		const int BR_DRIVE = 2;
		const int BL_DRIVE = 3;
		const int FL_DRIVE = 4;

		const int FR_STEER = 1;
		const int BR_STEER = 2;
		const int BL_STEER = 3;
		const int FL_STEER = 4;
	}

	// Swerve measurements
	namespace Swerve
	{
		const double OFFSET_WIDTH = 0.095; // Horizontal offset of the swerve wheels in meters
		const double OFFSET_HEIGHT = 0.095; // Vertical offset of the swerve wheels in meters

		const double WHEEL_RADIUS = 0.020; // Wheel radius in meters

		const double DRIVE_RATIO = 62.0 / 23.0; // 62 motor revs results in 23 wheel revs, roughly 2.6956:1
		const double STEER_RATIO = 37.0 / 58.0; // 37 servo degrees results in 58 steering mount degrees, roughly 1:0.6379

		const double MAX_DRIVE_MOTOR_RPM = 1200; // Motor can do 1500 unloaded, 1200 through the geartrain, but this should probably be lower

		const double SERVO_MIN_POS = 30; // The minimum "zero" position of the servos (defaults to 30 in fimrware)
		const double SERVO_MAX_POS = 993; // The maximum position of the servos (defaults to 993 in firmware)
		const double SERVO_DEG_PER_STEP = 0.293; // The number of degrees per "step" unit of the servo (DO NOT CHANGE THIS)
		const double SERVO_ZEROS[4] = // The directions the wheels face when servos are at MIN POS
		{
			0.0, // FR
			180.0, // BR
			180.0, // BL
			0.0 // FL
		};

		// Deadband for when the drivetrain is considered "idle"
		const double IDLE_THRESHOLD = 0.01;

		// How close to the target angle the steer motors need to be before commanding the drive motors
		const double STEER_TOLERANCE = 15.0; // Was 5 on previous setup, could possibly be upped to 30

		// Drive motor speed ramp values (roughly 0-1200rpm in 1 second)
		// Note: Constant acceleration would just be 1200 for both
		const double MIN_RAMP_RPM_PER_SEC = 900.0;
		const double MAX_RAMP_RPM_PER_SEC = 1500.0;

		// High speed mode scale factors
		const double XY_SFACTOR_HIGH = 1; // Max robot X/Y speed in m/s
		const double R_SFACTOR_HIGH = 5; // Max robot rotation speed in rad/s 

		// Low speed mode scale factors (half of high speed values)
		const double XY_SFACTOR_LOW = 0.5;
		const double R_SFACTOR_LOW = 2.5;
	}

	// Joystick constants (currently setup for an Xbox controller)
	namespace Controls
	{
		// Drive joystick axes
		const int X_AXIS_NUM = 0;
		const int Y_AXIS_NUM = 1;
		const int R_AXIS_NUM = 2;

		// Buttons
		const int MODE_TOGGLE = 4; // Left bumper
		const int RESET_GYRO = 5; // Right bumper
		const int SFACTOR_TOGGLE = 8; // Left Stick

		// (All values here copied from 230, might need to be tuned)
		// Joystick deadband
		const double X_DEADBAND = 0.1;
		const double Y_DEADBAND = 0.1;
		const double R_DEADBAND = 0.1;

		// Joystick shaping slope
		const double X_SLOPE = 0.6;
		const double Y_SLOPE = 0.6;
		const double R_SLOPE = 1.0;
	}
}