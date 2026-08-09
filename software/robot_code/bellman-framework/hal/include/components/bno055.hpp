#pragma once

#include "component.hpp"
#include "buses/i2c.hpp"


namespace bellman
{
	namespace hal
	{
		namespace component
		{
			class BNO055 : public Component
			{
				public:
					enum OperationMode
					{
						kConfig = 0x00,
						kAccelOnly = 0x01,
						kMagOnly = 0x02,
						kGyroOnly = 0x03,
						kAccelMag = 0x04,
						kAccelGyro = 0x05,
						kMagGyro = 0x06,
						kAccelMagGyro = 0x07, // All three sensors but no fusion
						kImu = 0x08, // Accel + gyro fusion, relative orientation
						kCompass = 0x09, // Accel + mag fusion, absolute heading
						kM4G = 0x0A,
						kNdofFmcOff = 0x0B,
						kNdof = 0x0C // Accel + gyro + mag fusion, absolute orientation
					};

					enum AxisRemapPlacement
					{
						kP0 = 0,
						kP1 = 1, // Factory default
						kP2 = 2,
						kP3 = 3,
						kP4 = 4,
						kP5 = 5,
						kP6 = 6,
						kP7 = 7
					};

					struct Vector3
					{
						float x;
						float y;
						float z;
					};

					struct Quaternion
					{
						float w;
						float x;
						float y;
						float z;
					};

					struct CalibrationStatus
					{
						uint8_t system;
						uint8_t gyro;
						uint8_t accel;
						uint8_t mag;
					};

					BNO055(bus::I2C* i2c_bus, uint8_t address, OperationMode mode = kNdof);
					~BNO055() override = default;

					void Periodic() override;
					void Enable() override;
					void Disable() override;

					bool SetMode(OperationMode mode);
					OperationMode GetMode();

					bool SetAxisRemap(AxisRemapPlacement placement);

					Vector3 GetAccelerometer();
					Vector3 GetMagnetometer();
					Vector3 GetGyroscope();
					Vector3 GetEulerAngles();
					Vector3 GetLinearAcceleration();
					Vector3 GetGravity();
					Quaternion GetQuaternion();
					int8_t GetTemperature();

					CalibrationStatus GetCalibrationStatus();
					bool IsFullyCalibrated();

				private:
					Vector3 ReadVector(uint8_t reg, float scale);

					bus::I2C* m_i2c;
					uint8_t m_address;
					Logger* m_logger;
					bool m_initialized = false;
					OperationMode m_mode;
			};
		}
	}
}