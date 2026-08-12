#include "robot.hpp"

Robot::Robot(int dt) : m_dt(dt)
{
	m_hal = bellman::hal::IHal::GetInstance();
	m_state = bellman::hal::State::GetInstance();
	m_state->RegisterMode("TeleOp", std::make_shared<Robot::TeleopMode>(this)); // Register the TeleOp opmode
	m_hal->ConfigureHAL({ "M5Swerve", 1, "LiPo", 3, "eth0" }); // Configure the HAL

	// CoreMP135 specific Code
	m_gpio = bellman::hal::GPIO::GetInstance();
	m_gpio->SetPin("PI3", true); // Enables 5V output on the M-BUS
	std::this_thread::sleep_for(std::chrono::milliseconds(250)); // Wait some time for the StampIO to start up
	m_i2c = m_hal->GetI2C("/dev/i2c-2");
	m_stamp_io = m_hal->GetComponent<bellman::hal::component::StampIO>(m_i2c, 0x45);
	m_stamp_io->SetPinMode(4, bellman::hal::component::StampIO::PinMode::kNeopixel);
	std::optional<std::unique_ptr<bellman::hal::component::Light>> ril = m_stamp_io->GetLight(4);
	if (ril.has_value()) { m_hal->ConfigureRIL(std::move(ril.value()), bellman::hal::component::Light::ColorCorrection::kSMD5050); }

	// Setup sensors
	m_ina228 = m_hal->GetComponent<bellman::hal::component::INA228>(m_i2c, 0x40);
	m_bno055 = m_hal->GetComponent<bellman::hal::component::BNO055>(m_i2c, 0x28, bellman::hal::component::BNO055::OperationMode::kImu); // Setup the IMU for 6DoF relative fusion, kNdof kinda works but motors cause drift, kNdofFmcOff might be a good compromise
	m_bno055->SetAxisRemap(bellman::hal::component::BNO055::AxisRemapPlacement::kP1); // Set the IMU axis remap to match the robot frame (right handed, X+ right, Y+ forward, Z+ up) 

	// Setup classes
	m_logger = bellman::hal::Logger::GetInstance();
	m_ds = bellman::hal::DriverStation::GetInstance();

	// Setup buses
	m_steer_uart = m_hal->GetUART({ "/dev/ttySTM2", 1000000, 8, 1, bellman::hal::bus::UART::Parity::kNone, true }); // M-BUS USART2
	m_drive_rs485 = m_hal->GetUART({ "/dev/ttySTM3", 1000000, 8, 1, bellman::hal::bus::UART::Parity::kNone, true }); // RS485 USART3

	// Setup drive motors
	m_fr_drive = m_hal->GetComponent<bellman::hal::component::Roller485>(m_drive_rs485, Constants::Motors::FR_DRIVE);
	m_br_drive = m_hal->GetComponent<bellman::hal::component::Roller485>(m_drive_rs485, Constants::Motors::BR_DRIVE);
	m_bl_drive = m_hal->GetComponent<bellman::hal::component::Roller485>(m_drive_rs485, Constants::Motors::BL_DRIVE);
	m_fl_drive = m_hal->GetComponent<bellman::hal::component::Roller485>(m_drive_rs485, Constants::Motors::FL_DRIVE);

	// Setup steer motors
	m_fr_steer = m_hal->GetComponent<bellman::hal::component::SCServo>(m_steer_uart, Constants::Motors::FR_STEER);
	m_br_steer = m_hal->GetComponent<bellman::hal::component::SCServo>(m_steer_uart, Constants::Motors::BR_STEER);
	m_bl_steer = m_hal->GetComponent<bellman::hal::component::SCServo>(m_steer_uart, Constants::Motors::BL_STEER);
	m_fl_steer = m_hal->GetComponent<bellman::hal::component::SCServo>(m_steer_uart, Constants::Motors::FL_STEER);

	// Default to high speed scale factors
	m_xy_sfactor = Constants::Swerve::XY_SFACTOR_HIGH;
	m_r_sfactor = Constants::Swerve::R_SFACTOR_HIGH;
}

void Robot::RobotPeriodic(unsigned int loopcounts)
{
	// Code here will run every dt milliseconds regardless of the robot's current state
	// Loopcounts is the number of loop iterations that have run since the start of the program

	// Update the battery voltage (DO NOT CHANGE THIS)
	float bus_volts = m_ina228->ReadBusVoltage();
	if (!std::isnan(bus_volts)) { m_hal->SetBattery(bus_volts); }
	else { m_logger->LogMessage("Battery Updater", "Unable to get battery voltage!", bellman::hal::Logger::Level::kError); }
}

void Robot::DisabledInit()
{
	// Code here will run once when transitioning to the disabled state
	// This should be used to set variables and deinitialize the robot before being disabled
}

void Robot::DisabledPeriodic()
{
	// Code here will run every dt milliseconds only when the robot is disabled
	// Code here CANNOT send commands to actuators, use this to handle sensors or similar
}

// Calculates and sets the chassis velocities
// vx and vy are in m/s, psi is in rad/s
// X+ = right, Y+ = forward, Z+ = up, psi+ = CW rotation when viewed from above
void Robot::SetChassisSpeed(double vx, double vy, double psi, bool field)
{
	// Stop the drivetrain if idle, leaving the steering servos at their last commanded position
	if (std::abs(vx) < Constants::Swerve::IDLE_THRESHOLD && std::abs(vy) < Constants::Swerve::IDLE_THRESHOLD && std::abs(psi) < Constants::Swerve::IDLE_THRESHOLD)
	{
		// Stop all drive motors and clear drive commands
		for (int i = 0; i < 4; i++)
		{
			m_drive_cmd[i] = 0.0;
			m_modules[i].motor_rpm = 0.0;
		}
		m_fr_drive->SetSpeed(0);
		m_br_drive->SetSpeed(0);
		m_bl_drive->SetSpeed(0);
		m_fl_drive->SetSpeed(0);
		return;
	}
	
	// Rotate into field frame if requested
	if (field) { FieldToRobot(&vx, &vy); }

	// Compute raw module states (mount degrees + motor rpm)
	m_modules[0] = CalculateModule(vx, vy, psi, +Constants::Swerve::OFFSET_WIDTH, +Constants::Swerve::OFFSET_HEIGHT); // FR
	m_modules[1] = CalculateModule(vx, vy, psi, +Constants::Swerve::OFFSET_WIDTH, -Constants::Swerve::OFFSET_HEIGHT); // BR
	m_modules[2] = CalculateModule(vx, vy, psi, -Constants::Swerve::OFFSET_WIDTH, -Constants::Swerve::OFFSET_HEIGHT); // BL
	m_modules[3] = CalculateModule(vx, vy, psi, -Constants::Swerve::OFFSET_WIDTH, +Constants::Swerve::OFFSET_HEIGHT); // FL

	// Fold each module to be within reach of its servo (and flip motor direction if needed)
	m_modules[0] = FoldModule(m_modules[0], Constants::Swerve::SERVO_ZEROS[0]);
	m_modules[1] = FoldModule(m_modules[1], Constants::Swerve::SERVO_ZEROS[1]);
	m_modules[2] = FoldModule(m_modules[2], Constants::Swerve::SERVO_ZEROS[2]);
	m_modules[3] = FoldModule(m_modules[3], Constants::Swerve::SERVO_ZEROS[3]);

	// Enforce max motor RPM scaling
	double max_rpm = 0.0;
	for (int i = 0; i < 4; ++i) max_rpm = std::max(max_rpm, std::fabs(m_modules[i].motor_rpm));
	if (max_rpm > Constants::Swerve::MAX_DRIVE_MOTOR_RPM)
	{
		double scale = Constants::Swerve::MAX_DRIVE_MOTOR_RPM / max_rpm;
		for (int i = 0; i < 4; ++i) m_modules[i].motor_rpm *= scale;
	}

	// Convert each folded mount angle to a servo step target
	int target_pos[4];
	for (int i = 0; i < 4; ++i)
	{
		target_pos[i] = MountAngleToServoSteps(m_modules[i].angle_deg, Constants::Swerve::SERVO_ZEROS[i]);
	}

	// Set the steer motor angles
	m_fr_steer->SetPos(target_pos[0], 0, 0);
	m_br_steer->SetPos(target_pos[1], 0, 0);
	m_bl_steer->SetPos(target_pos[2], 0, 0);
	m_fl_steer->SetPos(target_pos[3], 0, 0);

	// Wait until steering motors are at the correct angle to start driving
	float STEER_TOLERANCE_STEPS = Constants::Swerve::STEER_TOLERANCE / Constants::Swerve::SERVO_DEG_PER_STEP;
	bool steering_ready =
		std::abs(static_cast<float>(m_fr_steer->GetPos()) - target_pos[0]) <= STEER_TOLERANCE_STEPS &&
		std::abs(static_cast<float>(m_br_steer->GetPos()) - target_pos[1]) <= STEER_TOLERANCE_STEPS &&
		std::abs(static_cast<float>(m_bl_steer->GetPos()) - target_pos[2]) <= STEER_TOLERANCE_STEPS &&
		std::abs(static_cast<float>(m_fl_steer->GetPos()) - target_pos[3]) <= STEER_TOLERANCE_STEPS;

	// Ramp and/or set drive speeds
	if (steering_ready)
	{
		for (int i = 0; i < 4; ++i)
		{
			double target = m_modules[i].motor_rpm;

			// If reversing direction, don't ramp down to negative across zero (instant flip)
			if (target * m_drive_cmd[i] < 0.0)
			{
				m_drive_cmd[i] = 0.0;
				continue;
			}

			// Stop near zero
			if (std::abs(target) < 1.0)
			{
				m_drive_cmd[i] = 0.0;
				continue;
			}

			// ramping
			double frac = std::min(std::abs(m_drive_cmd[i]) / Constants::Swerve::MAX_DRIVE_MOTOR_RPM, 1.0);
			double ramp = Constants::Swerve::MAX_RAMP_RPM_PER_SEC - (Constants::Swerve::MAX_RAMP_RPM_PER_SEC - Constants::Swerve::MIN_RAMP_RPM_PER_SEC) * frac;
			double max_delta = ramp * (m_dt / 1000.0);
			double delta = target - m_drive_cmd[i];
			if (delta > max_delta) delta = max_delta;
			if (delta < -max_delta) delta = -max_delta;
			m_drive_cmd[i] += delta;
		}
	}
	else
	{
		for (int i = 0; i < 4; ++i) m_drive_cmd[i] = 0.0;
	}

	// Apply drive motor speeds (rounded to integer)
	m_fr_drive->SetSpeed(static_cast<int32_t>(std::round(m_drive_cmd[0])));
	m_br_drive->SetSpeed(-static_cast<int32_t>(std::round(m_drive_cmd[1])));
	m_bl_drive->SetSpeed(static_cast<int32_t>(std::round(m_drive_cmd[2])));
	m_fl_drive->SetSpeed(-static_cast<int32_t>(std::round(m_drive_cmd[3])));
}

// Normalize angle to 0-360
double Robot::NormalizeAngle360(double deg)
{
	while (deg < 0.0) deg += 360.0;
	while (deg >= 360.0) deg -= 360.0;
	return deg;
}

// Shortest signed angle difference from a to b in degrees between -180 and 180
double Robot::ShortestSignedAngle(double from_deg, double to_deg)
{
	double diff = NormalizeAngle360(to_deg) - NormalizeAngle360(from_deg);
	if (diff >= 180.0) diff -= 360.0;
	if (diff < -180.0) diff += 360.0;
	return diff;
}

// Calculates kinematics for a swerve module
// vx and vy are in m/s, vpsi_robot is in rad/s
// X+ = right, Y+ = forward, Z+ = up, psi+ = CW rotation when viewed from above
Robot::ModuleState Robot::CalculateModule(double vx_robot, double vy_robot, double vpsi_robot, double module_x, double module_y)
{
	double vx = vx_robot + vpsi_robot * module_y;
	double vy = vy_robot - vpsi_robot * module_x;

	// Wheel linear speed
	double wheel_speed = std::hypot(vx, vy); // m/s

	double heading_rad = std::atan2(vy, vx);
	double heading_deg = heading_rad * 180.0 / Constants::PI;
	double mount_deg = heading_deg - 90.0; // now 0 = forward (+Y)
	mount_deg = NormalizeAngle360(mount_deg);

	// Convert speed to motor RPM
	double wheel_rpm = wheel_speed * 60.0 / (2.0 * Constants::PI * Constants::Swerve::WHEEL_RADIUS);
	double motor_rpm = wheel_rpm * Constants::Swerve::DRIVE_RATIO;

	return { mount_deg, motor_rpm };
}

// Convert field-frame inputs to robot-frame commands
void Robot::FieldToRobot(double* vx, double* vy)
{
	// Current robot heading in radians
	double theta = m_bno055->GetEulerAngles().z * Constants::PI / 180.0;

	// Velocity-based prediction to make field movements smoother
	// TODO: Would be a good idea to tune the 0.5s value at the end
	theta = theta + (m_bno055->GetGyroscope().z - m_gyro_offset) * (Constants::PI / 180.0) * 0.05;

	// Invert theta to align with CW+ rotation convention
	theta = -theta;

	// Save the original field-relative commands
	double fieldX = *vx;
	double fieldY = *vy;

	// Rotate by -theta
	*vx = cos(theta) * fieldX + sin(theta) * fieldY;
	*vy = -sin(theta) * fieldX + cos(theta) * fieldY;
}

// Folds a desired wheel heading to the valid servo range
Robot::ModuleState Robot::FoldModule(const ModuleState& state, double servo_zero)
{
	ModuleState out = state;

	double zero = NormalizeAngle360(servo_zero);
	double delta = ShortestSignedAngle(zero, NormalizeAngle360(state.angle_deg)); // (-180, 180]

	bool flip = delta < 0.0;
	if (flip)
	{
		delta += (delta > 0.0) ? -180.0 : 180.0;
		out.motor_rpm = -out.motor_rpm;
	}

	out.angle_deg = NormalizeAngle360(zero + delta);
	return out;
}

// Converts an folded mount heading to a servo target step position
int Robot::MountAngleToServoSteps(double mount_deg, double servo_zero)
{
	double zero = NormalizeAngle360(servo_zero);
	double delta_mount_deg = ShortestSignedAngle(zero, NormalizeAngle360(mount_deg));
	double delta_servo_deg = delta_mount_deg / Constants::Swerve::STEER_RATIO;

	double steps = Constants::Swerve::SERVO_MIN_POS + (std::fabs(delta_servo_deg) / Constants::Swerve::SERVO_DEG_PER_STEP);
	steps = std::clamp(steps, Constants::Swerve::SERVO_MIN_POS, Constants::Swerve::SERVO_MAX_POS);

	return static_cast<int>(std::round(steps));
}

// TeleOp Mode
Robot::TeleopMode::TeleopMode(Robot* r) : m_robot(r) 
{
	// This constructor runs once when this mode is registered with the state control module
	// Use this to setup variables and states specific to this mode, right now nothing needs to be done here
}

void Robot::TeleopMode::Init()
{
	// This runs once when transitioning into the enable state when the TeleOp mode is selected, right now nothing needs to be done here
}

void Robot::TeleopMode::Periodic()
{
	// Get the current controller inputs
	std::optional<float> x_joy = m_robot->m_ds->GetAxis(0, Constants::Controls::X_AXIS_NUM);
	std::optional<float> y_joy = m_robot->m_ds->GetAxis(0, Constants::Controls::Y_AXIS_NUM);
	std::optional<float> r_joy = m_robot->m_ds->GetAxis(0, Constants::Controls::R_AXIS_NUM);
	std::optional<bool> reset_gyro = m_robot->m_ds->GetButton(0, Constants::Controls::RESET_GYRO);
	std::optional<bool> toggle_mode = m_robot->m_ds->GetButton(0, Constants::Controls::MODE_TOGGLE);
	std::optional<bool> toggle_sfactor = m_robot->m_ds->GetButton(0, Constants::Controls::SFACTOR_TOGGLE);

	// Reset the gyro offset if pressed
	if (reset_gyro.has_value())
	{
		if (reset_gyro.value()) { m_robot->m_gyro_offset = m_robot->m_bno055->GetEulerAngles().z; }
	}

	// Switch control modes if pressed
	if (toggle_mode.has_value())
	{
		if (toggle_mode.value())
		{
			if (!m_mode_toggled)
			{
				m_field_mode = !m_field_mode;
				m_mode_toggled = true;

				if (m_field_mode) { m_robot->m_logger->LogMessage("Swerve", "Driving in field mode", bellman::hal::Logger::Level::kInfo); }
				else { m_robot->m_logger->LogMessage("Swerve", "Driving in robot mode", bellman::hal::Logger::Level::kInfo); }
			}
		}
		else { m_mode_toggled = false; }
	}

	// Switch between the 2 sfactor modes
	if (toggle_sfactor.has_value())
	{
		if (toggle_sfactor.value())
		{
			if (!m_sfactor_toggled)
			{
				m_high_sfactor_active = !m_high_sfactor_active;
				m_sfactor_toggled = true;

				if (m_high_sfactor_active)
				{
					m_robot->m_xy_sfactor = Constants::Swerve::XY_SFACTOR_HIGH;
					m_robot->m_r_sfactor = Constants::Swerve::R_SFACTOR_HIGH;
					m_robot->m_logger->LogMessage("Swerve", "S-Factor set to high", bellman::hal::Logger::Level::kInfo);
				}
				else
				{
					m_robot->m_xy_sfactor = Constants::Swerve::XY_SFACTOR_LOW;
					m_robot->m_r_sfactor = Constants::Swerve::R_SFACTOR_LOW;
					m_robot->m_logger->LogMessage("Swerve", "S-Factor set to low", bellman::hal::Logger::Level::kInfo);
				}
			}
		}
		else { m_sfactor_toggled = false; }
	}

	if (x_joy.has_value() && y_joy.has_value() && r_joy.has_value())
	{
		// Shape joysticks
		double x_shaped = ShapeStick(x_joy.value(), Constants::Controls::X_DEADBAND, Constants::Controls::X_SLOPE);
		double y_shaped = ShapeStick(-y_joy.value(), Constants::Controls::Y_DEADBAND, Constants::Controls::Y_SLOPE); // SDL Reports Y up as negative
		double r_shaped = ShapeStick(r_joy.value(), Constants::Controls::R_DEADBAND, Constants::Controls::R_SLOPE);

		// Scale joysticks to expected units
		double x_scaled = x_shaped * m_robot->m_xy_sfactor;
		double y_scaled = y_shaped * m_robot->m_xy_sfactor;
		double r_scaled = r_shaped * m_robot->m_r_sfactor;

		// Command chassis velocities
		m_robot->SetChassisSpeed(x_scaled, y_scaled, r_scaled, m_field_mode);
	}
}

// Applies deadband and input shaping to a joystick
float Robot::TeleopMode::ShapeStick(float stick, float deadband, float slope)
{
	float output = std::max((std::fabs(stick) - deadband) / (1.0f - deadband), 0.0f); // Apply deadband and rescale
	output = std::copysign(output, stick); // Carry over sign
	output *= (0.5f * slope * std::fabs(output) - 0.5f * slope + 1.0f); // Apply shaping curve
	return output;
}