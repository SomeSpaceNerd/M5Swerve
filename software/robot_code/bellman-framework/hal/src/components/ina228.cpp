#include "components/ina228.hpp"

namespace bellman
{
	namespace hal
	{
		namespace component
		{
			namespace
			{
				// Register addresses
				constexpr uint8_t kRegAdcConfig = 0x01;
				constexpr uint8_t kRegShuntCal = 0x02;
				constexpr uint8_t kRegVShunt = 0x04;
				constexpr uint8_t kRegVBus = 0x05;
				constexpr uint8_t kRegDieTemp = 0x06;
				constexpr uint8_t kRegCurrent = 0x07;
				constexpr uint8_t kRegPower = 0x08;
				constexpr uint8_t kRegEnergy = 0x09;
				constexpr uint8_t kRegCharge = 0x0A;
				constexpr uint8_t kRegMfgUid = 0x3E;
				constexpr uint8_t kRegDvcUid = 0x3F;

				// ADC_CONFIG mode field, bits 15:12
				constexpr uint8_t kModeContinuous = 0x0F;

				constexpr uint16_t kManufacturerId = 0x5449;
				constexpr uint16_t kDeviceId = 0x228;

				// LSB sizes
				constexpr float kBusVoltageLsb_uV = 195.3125f;
				constexpr float kShuntVoltageLsb_nV = 312.5f; // Assumes ADCRANGE = 0
				constexpr float kDieTempLsb_mC = 7.8125f;
				constexpr float kPowerLsbScale = 3.2f; // Power LSB = 3.2 * current_lsb
				constexpr float kEnergyLsbScale = 16.0f; // Energy LSB = 16 * power LSB
			}

			INA228::INA228(bus::I2C* i2c_bus, uint8_t address, float shunt_ohms, float max_current_amps) : m_i2c(i2c_bus), m_address(address)
			{
				m_logger = Logger::GetInstance();
				m_i2c->SetSlave(m_address);

				// Make sure we're actuall talking to an INA228
				std::optional<std::vector<uint8_t>> mfg_id = m_i2c->ReadRegs(kRegMfgUid, 2);
				std::optional<std::vector<uint8_t>> dvc_id = m_i2c->ReadRegs(kRegDvcUid, 2);

				if (!mfg_id.has_value() || !dvc_id.has_value())
				{
					m_initialized = false;
					m_logger->LogMessage("INA228", "Failed to setup INA228", Logger::Level::kError);
					return;
				}

				uint16_t manufacturer_id = (static_cast<uint16_t>((*mfg_id)[0]) << 8) | (*mfg_id)[1];
				uint16_t device_id = ((static_cast<uint16_t>((*dvc_id)[0]) << 8) | (*dvc_id)[1]) >> 4;

				if (manufacturer_id != kManufacturerId || device_id != kDeviceId)
				{
					m_initialized = false;
					m_logger->LogMessage("INA228", "Unexpected manufacturer or device ID", Logger::Level::kError);
					return;
				}

				// Shunt calibration, from the INA228 datasheet (assumes ADCRANGE = 0, which is +/-163.84mV)
				m_current_lsb = max_current_amps / static_cast<float>(1 << 19);
				uint16_t shunt_cal = static_cast<uint16_t>((13107.2e6f * shunt_ohms * m_current_lsb) + 0.5f);
				uint8_t shunt_cal_bytes[2] = { static_cast<uint8_t>(shunt_cal >> 8), static_cast<uint8_t>(shunt_cal & 0xFF) };

				if (!m_i2c->WriteRegs(kRegShuntCal, shunt_cal_bytes, 2))
				{
					m_initialized = false;
					m_logger->LogMessage("INA228", "Failed to set shunt calibration", Logger::Level::kError);
					return;
				}

				// Force continuous conversion mode
				std::optional<std::vector<uint8_t>> adc_cfg = m_i2c->ReadRegs(kRegAdcConfig, 2);
				if (!adc_cfg.has_value())
				{
					m_initialized = false;
					m_logger->LogMessage("INA228", "Failed to read ADC config", Logger::Level::kError);
					return;
				}

				uint8_t adc_cfg_bytes[2] = { static_cast<uint8_t>((kModeContinuous << 4) | ((*adc_cfg)[0] & 0x0F)), (*adc_cfg)[1] };
				if (!m_i2c->WriteRegs(kRegAdcConfig, adc_cfg_bytes, 2))
				{
					m_initialized = false;
					m_logger->LogMessage("INA228", "Failed to set continuous mode", Logger::Level::kError);
					return;
				}

				m_initialized = true;
				m_logger->LogMessage("INA228", "Started INA228", Logger::Level::kDebug);
			}

			/**
			* @brief Reads the bus voltage measured by the INA228
			*
			* @returns The bus voltage in volts, or NAN on failure
			*/
			float INA228::ReadBusVoltage()
			{
				if (!m_initialized) { return NAN; }

				m_i2c->SetSlave(m_address);
				std::optional<std::vector<uint8_t>> regs = m_i2c->ReadRegs(kRegVBus, 3);
				if (!regs.has_value())
				{
					m_logger->LogMessage("INA228", "Failed to read bus voltage", Logger::Level::kError);
					return NAN;
				}

				uint32_t raw = (static_cast<uint32_t>((*regs)[0]) << 16) | (static_cast<uint32_t>((*regs)[1]) << 8) | (*regs)[2];
				raw >>= 4; // Bottom 4 bits are reserved

				return static_cast<float>(raw) * kBusVoltageLsb_uV / 1e6f;
			}

			/**
			* @brief Reads the shunt voltage measured by the INA228
			*
			* @returns The shunt voltage in millivolts, or NAN on failure
			*/
			float INA228::ReadShuntVoltage()
			{
				if (!m_initialized) { return NAN; }

				m_i2c->SetSlave(m_address);
				std::optional<std::vector<uint8_t>> regs = m_i2c->ReadRegs(kRegVShunt, 3);
				if (!regs.has_value())
				{
					m_logger->LogMessage("INA228", "Failed to read shunt voltage", Logger::Level::kError);
					return NAN;
				}

				int32_t raw = (static_cast<int32_t>((*regs)[0]) << 16) | (static_cast<int32_t>((*regs)[1]) << 8) | (*regs)[2];
				if (raw & 0x800000) { raw |= 0xFF000000; } // Sign extend 24-bit value
				raw >>= 4; // Bottom 4 bits are reserved

				return static_cast<float>(raw) * kShuntVoltageLsb_nV / 1e6f;
			}

			/**
			* @brief Reads the current measured by the INA228
			*
			* @returns The current in milliamps, or NAN on failure
			*/
			float INA228::ReadCurrent()
			{
				if (!m_initialized) { return NAN; }

				m_i2c->SetSlave(m_address);
				std::optional<std::vector<uint8_t>> regs = m_i2c->ReadRegs(kRegCurrent, 3);
				if (!regs.has_value())
				{
					m_logger->LogMessage("INA228", "Failed to read current", Logger::Level::kError);
					return NAN;
				}

				int32_t raw = (static_cast<int32_t>((*regs)[0]) << 16) | (static_cast<int32_t>((*regs)[1]) << 8) | (*regs)[2];
				if (raw & 0x800000) { raw |= 0xFF000000; } // Sign extend 24-bit value
				raw >>= 4; // Bottom 4 bits are reserved

				return static_cast<float>(raw) * m_current_lsb * 1000.0f;
			}

			/**
			* @brief Reads the power measured by the INA228
			*
			* @returns The power in milliwatts, or NAN on failure
			*/
			float INA228::ReadPower()
			{
				if (!m_initialized) { return NAN; }

				m_i2c->SetSlave(m_address);
				std::optional<std::vector<uint8_t>> regs = m_i2c->ReadRegs(kRegPower, 3);
				if (!regs.has_value())
				{
					m_logger->LogMessage("INA228", "Failed to read power", Logger::Level::kError);
					return NAN;
				}

				uint32_t raw = (static_cast<uint32_t>((*regs)[0]) << 16) | (static_cast<uint32_t>((*regs)[1]) << 8) | (*regs)[2];

				return static_cast<float>(raw) * kPowerLsbScale * m_current_lsb * 1000.0f;
			}

			/**
			* @brief Reads the accumulated energy measured by the INA228
			*
			* @returns The energy in joules, or NAN on failure
			*/
			float INA228::ReadEnergy()
			{
				if (!m_initialized) { return NAN; }

				m_i2c->SetSlave(m_address);
				std::optional<std::vector<uint8_t>> regs = m_i2c->ReadRegs(kRegEnergy, 5);
				if (!regs.has_value())
				{
					m_logger->LogMessage("INA228", "Failed to read energy", Logger::Level::kError);
					return NAN;
				}

				float raw = 0.0f;
				for (uint8_t byte : *regs) { raw = (raw * 256.0f) + byte; }

				return raw * kEnergyLsbScale * kPowerLsbScale * m_current_lsb;
			}

			/**
			* @brief Reads the accumulated charge measured by the INA228
			*
			* @returns The charge in coulombs, or NAN on failure
			*/
			float INA228::ReadCharge()
			{
				if (!m_initialized) { return NAN; }

				m_i2c->SetSlave(m_address);
				std::optional<std::vector<uint8_t>> regs = m_i2c->ReadRegs(kRegCharge, 5);
				if (!regs.has_value())
				{
					m_logger->LogMessage("INA228", "Failed to read charge", Logger::Level::kError);
					return NAN;
				}

				int64_t raw = 0;
				for (uint8_t byte : *regs) { raw = (raw << 8) | byte; }

				if (raw & (static_cast<int64_t>(1) << 39)) { raw |= 0xFFFFFF0000000000LL; } // Sign extend 40-bit value

				return static_cast<float>(raw) * m_current_lsb;
			}

			/**
			* @brief Reads the die temperature of the INA228
			*
			* @returns The temperature in degrees Celsius, or NAN on failure
			*/
			float INA228::ReadDieTemp()
			{
				if (!m_initialized) { return NAN; }

				m_i2c->SetSlave(m_address);
				std::optional<std::vector<uint8_t>> regs = m_i2c->ReadRegs(kRegDieTemp, 2);
				if (!regs.has_value())
				{
					m_logger->LogMessage("INA228", "Failed to read die temperature", Logger::Level::kError);
					return NAN;
				}

				int16_t raw = (static_cast<int16_t>((*regs)[0]) << 8) | (*regs)[1];

				return static_cast<float>(raw) * kDieTempLsb_mC / 1000.0f;
			}

			void INA228::Enable()
			{
			}

			void INA228::Disable()
			{
			}

			void INA228::Periodic()
			{
			}
		}
	}
}