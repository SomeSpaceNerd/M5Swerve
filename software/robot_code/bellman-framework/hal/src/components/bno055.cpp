#include "components/bno055.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <thread>

namespace bellman
{
	namespace hal
	{
		namespace component
		{
			namespace
			{
				// Register addresses (page 0)
				constexpr uint8_t kRegChipId = 0x00;
				constexpr uint8_t kRegAccelData = 0x08;
				constexpr uint8_t kRegMagData = 0x0E;
				constexpr uint8_t kRegGyroData = 0x14;
				constexpr uint8_t kRegEulerData = 0x1A;
				constexpr uint8_t kRegQuaternionData = 0x20;
				constexpr uint8_t kRegLinearAccelData = 0x28;
				constexpr uint8_t kRegGravityData = 0x2E;
				constexpr uint8_t kRegTemp = 0x34;
				constexpr uint8_t kRegCalibStat = 0x35;
				constexpr uint8_t kRegPageId = 0x07;
				constexpr uint8_t kRegOprMode = 0x3D;
				constexpr uint8_t kRegPwrMode = 0x3E;
				constexpr uint8_t kRegSysTrigger = 0x3F;
				constexpr uint8_t kRegAxisMapConfig = 0x41;
				constexpr uint8_t kRegAxisMapSign = 0x42;

				constexpr uint8_t kChipId = 0xA0;
				constexpr uint8_t kPowerModeNormal = 0x00;
				constexpr uint8_t kResetCommand = 0x20;

				constexpr int kResetRetries = 50;
				constexpr int kResetRetryDelayMs = 50; // Was 10, that spit out tons of errors

				// AXIS_MAP_CONFIG / AXIS_MAP_SIGN values for each of the 8 documented placements
				// (BNO055 datasheet section 3.4); indexed by AxisRemapPlacement
				constexpr uint8_t kAxisRemapConfigTable[8] = { 0x21, 0x24, 0x24, 0x21, 0x24, 0x21, 0x21, 0x24 };
				constexpr uint8_t kAxisRemapSignTable[8] = { 0x04, 0x00, 0x06, 0x02, 0x03, 0x01, 0x07, 0x05 };

				// LSB sizes, per the BNO055 datasheet (default unit selection)
				constexpr float kAccelLsbPerMps2 = 100.0f; // Accelerometer, linear accel, gravity
				constexpr float kMagLsbPerUt = 16.0f;
				constexpr float kGyroLsbPerDps = 16.0f;
				constexpr float kEulerLsbPerDeg = 16.0f;
				constexpr float kQuaternionScale = 1.0f / 16384.0f; // 1 / 2^14
			}

			BNO055::BNO055(bus::I2C* i2c_bus, uint8_t address, OperationMode mode) : m_i2c(i2c_bus), m_address(address)
			{
				m_logger = Logger::GetInstance();
				m_i2c->SetSlave(m_address);

				std::optional<uint8_t> chip_id = m_i2c->ReadReg(kRegChipId);
				if (!chip_id.has_value() || chip_id.value() != kChipId)
				{
					m_initialized = false;
					m_logger->LogMessage("BNO055", "Failed to setup BNO055", Logger::Level::kError);
					return;
				}

				// Switch to config mode before resetting, just in case since this is the power-on default
				SetMode(kConfig);

				if (!m_i2c->WriteReg(kRegSysTrigger, kResetCommand))
				{
					m_initialized = false;
					m_logger->LogMessage("BNO055", "Failed to reset BNO055", Logger::Level::kError);
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(30));

				m_logger->LogMessage("BNO055", "Configuring BNO055, you may see I2C errors", Logger::Level::kInfo);
				bool chip_ready = false;
				for (int i = 0; i < kResetRetries; i++)
				{
					std::optional<uint8_t> id = m_i2c->ReadReg(kRegChipId);
					if (id.has_value() && id.value() == kChipId)
					{
						chip_ready = true;
						break;
					}
					std::this_thread::sleep_for(std::chrono::milliseconds(kResetRetryDelayMs));
				}

				if (!chip_ready)
				{
					m_initialized = false;
					m_logger->LogMessage("BNO055", "BNO055 did not come back up after reset", Logger::Level::kError);
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(50));

				if (!m_i2c->WriteReg(kRegPwrMode, kPowerModeNormal))
				{
					m_initialized = false;
					m_logger->LogMessage("BNO055", "Failed to set power mode", Logger::Level::kError);
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(10));

				if (!m_i2c->WriteReg(kRegPageId, 0x00) || !m_i2c->WriteReg(kRegSysTrigger, 0x00))
				{
					m_initialized = false;
					m_logger->LogMessage("BNO055", "Failed to configure BNO055", Logger::Level::kError);
					return;
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(10));

				if (!SetMode(mode))
				{
					m_initialized = false;
					m_logger->LogMessage("BNO055", "Failed to set initial operating mode", Logger::Level::kError);
					return;
				}

				m_initialized = true;
				m_logger->LogMessage("BNO055", "Started BNO055", Logger::Level::kDebug);
			}

			/**
			* @brief Sets the sensor's operating mode
			*
			* @param mode The operating mode to switch to
			*
			* @returns True if setting the mode was successful
			*/
			bool BNO055::SetMode(OperationMode mode)
			{
				m_i2c->SetSlave(m_address);
				if (!m_i2c->WriteReg(kRegOprMode, static_cast<uint8_t>(mode)))
				{
					m_logger->LogMessage("BNO055", "Failed to set operating mode", Logger::Level::kError);
					return false;
				}

				m_mode = mode;
				std::this_thread::sleep_for(std::chrono::milliseconds(30)); // Mode switches take effect on the sensor after a short delay

				return true;
			}

			/**
			* @brief Gets the sensor's current operating mode
			*
			* @returns The current operating mode
			*/
			BNO055::OperationMode BNO055::GetMode()
			{
				return m_mode;
			}

			/**
			* @brief Remaps which physical axis of the sensor package feeds each reported axis
			*
			* @param placement One of the 8 documented axis placements (kP0-kP7)
			*
			* @returns True if applying the remap was successful
			*/
			bool BNO055::SetAxisRemap(AxisRemapPlacement placement)
			{
				OperationMode mode_before = m_mode;

				if (!SetMode(kConfig)) { return false; }

				m_i2c->SetSlave(m_address);
				bool ok = m_i2c->WriteReg(kRegAxisMapConfig, kAxisRemapConfigTable[placement]) && m_i2c->WriteReg(kRegAxisMapSign, kAxisRemapSignTable[placement]);

				if (!ok)
				{
					m_logger->LogMessage("BNO055", "Failed to set axis remap", Logger::Level::kError);
				}
				std::this_thread::sleep_for(std::chrono::milliseconds(10));

				return SetMode(mode_before) && ok;
			}

			/**
			* @brief Reads a 3-axis vector starting at the given register and scales it to physical units
			*
			* @param reg The register address of the first (X LSB) byte
			* @param scale The number of LSBs per physical unit
			*
			* @returns The scaled vector, or a vector of NAN on failure
			*/
			BNO055::Vector3 BNO055::ReadVector(uint8_t reg, float scale)
			{
				if (!m_initialized) { return { NAN, NAN, NAN }; }

				m_i2c->SetSlave(m_address);
				std::optional<std::vector<uint8_t>> regs = m_i2c->ReadRegs(reg, 6);
				if (!regs.has_value())
				{
					m_logger->LogMessage("BNO055", "Failed to read vector data", Logger::Level::kError);
					return { NAN, NAN, NAN };
				}

				int16_t x = static_cast<int16_t>((*regs)[0] | ((*regs)[1] << 8));
				int16_t y = static_cast<int16_t>((*regs)[2] | ((*regs)[3] << 8));
				int16_t z = static_cast<int16_t>((*regs)[4] | ((*regs)[5] << 8));

				return { static_cast<float>(x) / scale, static_cast<float>(y) / scale, static_cast<float>(z) / scale };
			}

			/**
			* @brief Reads the accelerometer
			*
			* @returns The acceleration in m/s^2, or NAN components on failure
			*/
			BNO055::Vector3 BNO055::GetAccelerometer()
			{
				return ReadVector(kRegAccelData, kAccelLsbPerMps2);
			}

			/**
			* @brief Reads the magnetometer
			*
			* @returns The magnetic field in microtesla, or NAN components on failure
			*/
			BNO055::Vector3 BNO055::GetMagnetometer()
			{
				return ReadVector(kRegMagData, kMagLsbPerUt);
			}

			/**
			* @brief Reads the gyroscope
			*
			* @returns The angular velocity in degrees/s, or NAN components on failure
			*/
			BNO055::Vector3 BNO055::GetGyroscope()
			{
				return ReadVector(kRegGyroData, kGyroLsbPerDps);
			}

			/**
			* @brief Reads the fused Euler orientation. Only valid in a fusion mode (e.g. kNdof)
			*
			* @returns x = pitch, y = roll, z = heading, in degrees, or NAN components on failure
			*/
			BNO055::Vector3 BNO055::GetEulerAngles()
			{
				Vector3 data = ReadVector(kRegEulerData, kEulerLsbPerDeg);
				return { static_cast<float>(data.z), static_cast<float>(data.y), static_cast<float>(data.x) }; // Sensor reports data as heading (yaw), roll, pitch, match that correctly to robot coordinate frame
			}

			/**
			* @brief Reads the fused linear acceleration (gravity removed). Only valid in a fusion mode
			*
			* @returns The linear acceleration in m/s^2, or NAN components on failure
			*/
			BNO055::Vector3 BNO055::GetLinearAcceleration()
			{
				return ReadVector(kRegLinearAccelData, kAccelLsbPerMps2);
			}

			/**
			* @brief Reads the fused gravity vector. Only valid in a fusion mode
			*
			* @returns The gravity vector in m/s^2, or NAN components on failure
			*/
			BNO055::Vector3 BNO055::GetGravity()
			{
				return ReadVector(kRegGravityData, kAccelLsbPerMps2);
			}

			/**
			* @brief Reads the fused absolute orientation quaternion. Only valid in a fusion mode
			*
			* @returns The orientation quaternion, or NAN components on failure
			*/
			BNO055::Quaternion BNO055::GetQuaternion()
			{
				if (!m_initialized) { return { NAN, NAN, NAN, NAN }; }

				m_i2c->SetSlave(m_address);
				std::optional<std::vector<uint8_t>> regs = m_i2c->ReadRegs(kRegQuaternionData, 8);
				if (!regs.has_value())
				{
					m_logger->LogMessage("BNO055", "Failed to read quaternion data", Logger::Level::kError);
					return { NAN, NAN, NAN, NAN };
				}

				int16_t w = static_cast<int16_t>((*regs)[0] | ((*regs)[1] << 8));
				int16_t x = static_cast<int16_t>((*regs)[2] | ((*regs)[3] << 8));
				int16_t y = static_cast<int16_t>((*regs)[4] | ((*regs)[5] << 8));
				int16_t z = static_cast<int16_t>((*regs)[6] | ((*regs)[7] << 8));

				return { static_cast<float>(w) * kQuaternionScale, static_cast<float>(x) * kQuaternionScale, static_cast<float>(y) * kQuaternionScale, static_cast<float>(z) * kQuaternionScale };
			}

			/**
			* @brief Reads the onboard die temperature
			*
			* @returns The temperature in degrees Celsius, or INT8_MIN on failure
			*/
			int8_t BNO055::GetTemperature()
			{
				if (!m_initialized) { return INT8_MIN; }

				m_i2c->SetSlave(m_address);
				std::optional<uint8_t> value = m_i2c->ReadReg(kRegTemp);
				if (!value.has_value())
				{
					m_logger->LogMessage("BNO055", "Failed to read temperature", Logger::Level::kError);
					return INT8_MIN;
				}

				return static_cast<int8_t>(value.value());
			}

			/**
			* @brief Reads the sensor's calibration status. Each field ranges from 0 (uncalibrated) to 3 (fully calibrated)
			*
			* @returns The calibration status, or all fields set to 0xFF on failure
			*/
			BNO055::CalibrationStatus BNO055::GetCalibrationStatus()
			{
				if (!m_initialized) { return { 0xFF, 0xFF, 0xFF, 0xFF }; }

				m_i2c->SetSlave(m_address);
				std::optional<uint8_t> value = m_i2c->ReadReg(kRegCalibStat);
				if (!value.has_value())
				{
					m_logger->LogMessage("BNO055", "Failed to read calibration status", Logger::Level::kError);
					return { 0xFF, 0xFF, 0xFF, 0xFF };
				}

				uint8_t data = value.value();
				return { static_cast<uint8_t>((data >> 6) & 0x03), static_cast<uint8_t>((data >> 4) & 0x03), static_cast<uint8_t>((data >> 2) & 0x03), static_cast<uint8_t>(data & 0x03) };
			}

			/**
			* @brief Checks whether every sensor used by the current operating mode is fully calibrated.
			*
			* @returns True if fully calibrated
			*/
			bool BNO055::IsFullyCalibrated()
			{
				if (!m_initialized) { return false; }

				CalibrationStatus status = GetCalibrationStatus();

				switch (m_mode)
				{
				case kAccelOnly: return status.accel == 3;
				case kMagOnly: return status.mag == 3;
				case kGyroOnly:
				case kM4G: return status.gyro == 3;
				case kAccelMag:
				case kCompass: return status.accel == 3 && status.mag == 3;
				case kAccelGyro:
				case kImu: return status.accel == 3 && status.gyro == 3;
				case kMagGyro: return status.mag == 3 && status.gyro == 3;
				default: return status.system == 3 && status.gyro == 3 && status.accel == 3 && status.mag == 3;
				}
			}

			void BNO055::Enable()
			{
			}

			void BNO055::Disable()
			{
			}

			void BNO055::Periodic()
			{
			}
		}
	}
}