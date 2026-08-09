#pragma once

#include "component.hpp"
#include "buses/i2c.hpp"
#include <cmath>
#include <cstdint>

namespace bellman
{
	namespace hal
	{
		namespace component
		{
			class INA228 : public Component
			{
				public:
					INA228(bus::I2C* i2c_bus, uint8_t address, float shunt_ohms = 0.015f, float max_current_amps = 10.0f);
					~INA228() override = default;

					void Periodic() override;
					void Enable() override;
					void Disable() override;

					float ReadBusVoltage();
					float ReadShuntVoltage();
					float ReadCurrent();
					float ReadPower();
					float ReadEnergy();
					float ReadCharge();
					float ReadDieTemp();

				private:
					bus::I2C* m_i2c;
					uint8_t m_address;
					Logger* m_logger;
					bool m_initialized = false;
					float m_current_lsb;
			};
		}
	}
}