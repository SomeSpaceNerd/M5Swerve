#pragma once

#include <optional>
#include "component.hpp"
#include "light.hpp"
#include "buses/i2c.hpp"
#include "logger.hpp"

namespace bellman
{
	namespace hal
	{
		namespace component
		{
			class StampIO;

			class StampIOLight : public Light
			{
				public:
					StampIOLight(StampIO& owner, uint8_t pin);
					~StampIOLight() override = default;

					void SetOn(bool on) override;
					void SetColor(uint8_t red, uint8_t green, uint8_t blue) override;
					void SetCorrection(Light::ColorCorrection correction) override;

				private:
					StampIO& m_owner;
					uint8_t m_pin;
					bool m_on;
					uint8_t m_color[3];
					uint32_t m_correction;
			};

			class StampIO : public Component
			{
				public:
					enum PinMode
					{
						kInput = 0,
						kOutput = 1,
						kADC = 2,
						kServo = 3,
						kNeopixel = 4,
						kPWM = 5
					};

					StampIO(bus::I2C* i2c_bus, uint8_t address);
					~StampIO() override = default;

					void Periodic() override;
					void Enable() override;
					void Disable() override;

					bool SetPinMode(uint8_t pin, PinMode pin_mode);

					bool SetDigitalOut(uint8_t pin, bool value);
					bool ReadDigitalIn(uint8_t pin);
					int ReadADC(uint8_t pin);
					bool SetServoAngle(uint8_t pin, uint8_t angle);
					bool SetServoPeriod(uint8_t pin, uint16_t time_us);
					bool SetNeopixel(uint8_t pin, uint8_t red, uint8_t green, uint8_t blue);
					std::optional<std::unique_ptr<Light>> GetLight(uint8_t pin);
					bool SetPWM(uint8_t pin, uint8_t duty_cycle);

				private:
					bus::I2C* m_i2c;
					uint8_t m_address;
					Logger* m_logger;
					bool m_initialized = false;
					PinMode m_pin_modes[8]; // Tracks which modes the pins are currently defined as
			};
		}
	}
}