#include "components/stamp_io.hpp"

namespace bellman
{
	namespace hal
	{
		namespace component
		{
			StampIOLight::StampIOLight(StampIO& owner, uint8_t pin) : m_owner(owner), m_pin(pin)
			{
				m_color[0] = 0x00;
				m_color[1] = 0x00;
				m_color[2] = 0x00;
				m_correction = 0xFFFFFF;
			}

			void StampIOLight::SetOn(bool on)
			{
				m_on = on;

				if (m_on) { m_owner.SetNeopixel(m_pin, m_color[0], m_color[1], m_color[2]); }
				else { m_owner.SetNeopixel(m_pin, 0x00, 0x00, 0x00); }
			}

			void StampIOLight::SetColor(uint8_t red, uint8_t green, uint8_t blue)
			{
				// Set colors based on correction factor
				m_color[0] = (uint16_t)red * ((m_correction >> 16) & 0xFF) / 255;
				m_color[1] = (uint16_t)green * ((m_correction >> 8) & 0xFF) / 255;
				m_color[2] = (uint16_t)blue * (m_correction & 0xFF) / 255;

				if (m_on) { m_owner.SetNeopixel(m_pin, m_color[0], m_color[1], m_color[2]); }
			}

			void StampIOLight::SetCorrection(Light::ColorCorrection correction) { m_correction = correction; }

			StampIO::StampIO(bus::I2C* i2c_bus, uint8_t address) : m_i2c(i2c_bus), m_address(address)
			{
				m_logger = Logger::GetInstance();
				m_i2c->SetSlave(m_address);

				// Attempt to read the firmware version to sanity check
				std::optional<uint8_t> fw_ver = m_i2c->ReadReg(0xFE);
				if (!fw_ver.has_value())
				{
					m_initialized = false;
					m_logger->LogMessage("StampIO", "Failed to setup StampIO", Logger::Level::kError);
					return;
				}

				m_logger->LogMessage("StampIO", "Started StampIO with FW ver " + std::to_string(fw_ver.value()), Logger::Level::kDebug);

				// Set all pins to digital out to start
				for (uint8_t i = 0; i <= 7; i++) 
				{ 
					if (!SetPinMode(i, PinMode::kOutput))
					{
						m_initialized = false;
						m_logger->LogMessage("StampIO", "Failed to initalize pin modes", Logger::Level::kError);
						return;
					}
				}
				m_initialized = true;
			}

			/**
			* @brief Sets the pin mode for a pin on the StampIO
			* 
			* @param pin The pin number to set (0-7)
			* @param pin_mode The mode to set the pin to
			* 
			* @returns True if setting the pin mode was successful
			*/
			bool StampIO::SetPinMode(uint8_t pin, PinMode pin_mode)
			{
				if (pin < 0 || pin > 7) 
				{ 
					m_logger->LogMessage("StampIO", "Invalid pin", Logger::Level::kError);
					return false; 
				}

				m_i2c->SetSlave(m_address);
				if (!m_i2c->WriteReg(pin, pin_mode))
				{
					m_logger->LogMessage("StampIO", "Failed to set pin mode", Logger::Level::kError);
					return false;
				}

				m_pin_modes[pin] = pin_mode;
				return true;
			}

			/**
			* @brief Sets the state of a digital output pin on the StampIO
			* 
			* @param pin The pin number to set (0-7)
			* @param value The state to set the pin to
			* 
			* @returns True if setting the pin state was successful
			*/
			bool StampIO::SetDigitalOut(uint8_t pin, bool value)
			{
				if (!m_initialized) { return false; }

				if (pin < 0 || pin > 7)
				{
					m_logger->LogMessage("StampIO", "Invalid pin", Logger::Level::kError);
					return false;
				}

				if (m_pin_modes[pin] != PinMode::kOutput)
				{
					m_logger->LogMessage("StampIO", "Pin must be in output mode to set digital output", Logger::Level::kError);
					return false;
				}

				m_i2c->SetSlave(m_address);
				return m_i2c->WriteReg(0x10 + pin, static_cast<uint8_t>(value));
			}

			/**
			* @brief Reads the digital state of a pin on the StampIO
			* 
			* @param pin The pin number to read (0-7)
			* 
			* @returns The state of the pin
			*/
			bool StampIO::ReadDigitalIn(uint8_t pin)
			{
				if (!m_initialized) { return false; }

				if (pin < 0 || pin > 7)
				{
					m_logger->LogMessage("StampIO", "Invalid pin", Logger::Level::kError);
					return false;
				}

				if (m_pin_modes[pin] != PinMode::kInput)
				{
					m_logger->LogMessage("StampIO", "Pin must be in input mode to read digital state", Logger::Level::kError);
					return false;
				}

				m_i2c->SetSlave(m_address);
				std::optional<uint8_t> data = m_i2c->ReadReg(0x20 + pin);
				if (data.has_value()) { return static_cast<bool>(data.value()); }
				else { return false; }
			}

			/**
			* @brief Reads the analog state of a pin on the StampIO
			* 
			* @param The pin number to read (0-7)
			* 
			* @returns The value of the pin (0-4095), or -1 if on communication failure
			*/
			int StampIO::ReadADC(uint8_t pin)
			{
				if (!m_initialized) { return -1; }

				if (pin < 0 || pin > 7)
				{
					m_logger->LogMessage("StampIO", "Invalid pin", Logger::Level::kError);
					return -1;
				}

				if (m_pin_modes[pin] != PinMode::kADC)
				{
					m_logger->LogMessage("StampIO", "Pin must be in ADC mode to read analog state", Logger::Level::kError);
					return -1;
				}

				m_i2c->SetSlave(m_address);
				std::optional<uint8_t> value_low = m_i2c->ReadReg(0x40 + (pin * 2));
				std::optional<uint8_t> value_high = m_i2c->ReadReg(0x41 + (pin * 2));

				if (!value_low.has_value() || !value_high.has_value())
				{
					m_logger->LogMessage("StampIO", "Failed to read analog pin value", Logger::Level::kError);
					return -1;
				}
				
				return static_cast<int>((value_high.value() << 8) | value_low.value());
			}

			/**
			* @brief Sets the angle of a servo attached to a pin on the StampIO
			* 
			* @param pin The pin number the servo is on (0-7)
			* @param angle The angle to set (0-180)
			* 
			* @returns True if setting the output was successful
			*/
			bool StampIO::SetServoAngle(uint8_t pin, uint8_t angle)
			{
				if (!m_initialized) { return false; }

				if (pin < 0 || pin > 7)
				{
					m_logger->LogMessage("StampIO", "Invalid pin", Logger::Level::kError);
					return false;
				}

				if (m_pin_modes[pin] != PinMode::kServo)
				{
					m_logger->LogMessage("StampIO", "Pin must be in servo mode to set servo angle", Logger::Level::kError);
					return false;
				}

				if (angle > 180)
				{
					m_logger->LogMessage("StampIO", "Invalid servo angle", Logger::Level::kError);
					return false;
				}

				m_i2c->SetSlave(m_address);
				return m_i2c->WriteReg(0x50 + pin, angle);
			}

			/**
			* @brief Sets the PWM pulse width for a servo attached to a pin on the StampIO
			* 
			* @param The pin number the servo is on (0-7)
			* @param time_us The time period to set in microseconds (0-2500)
			* 
			* @returns True if setting the output was successful
			*/
			bool StampIO::SetServoPeriod(uint8_t pin, uint16_t time_us)
			{
				if (!m_initialized) { return false; }

				if (pin < 0 || pin > 7)
				{
					m_logger->LogMessage("StampIO", "Invalid pin", Logger::Level::kError);
					return false;
				}

				if (m_pin_modes[pin] != PinMode::kServo)
				{
					m_logger->LogMessage("StampIO", "Pin must be in servo mode to set servo pulse period", Logger::Level::kError);
					return false;
				}

				if (time_us > 2500)
				{
					m_logger->LogMessage("StampIO", "Invalid time period", Logger::Level::kError);
					return false;
				}

				uint8_t pulse[2] =
				{
					static_cast<uint8_t>(time_us & 0xFF),
					static_cast<uint8_t>(time_us >> 8)
				};
				m_i2c->SetSlave(m_address);
				return m_i2c->WriteRegs(0x60 + pin * 2, pulse, 2);
			}

			/**
			* @brief Sets the color of a neopixel attached to a pin on the StampIO
			* 
			* @param red The red value (0-255)
			* @param green The green value (0-255)
			* @param blue The blue value (0-255)
			* 
			* @returns True if setting the color was successful
			*/
			bool StampIO::SetNeopixel(uint8_t pin, uint8_t red, uint8_t green, uint8_t blue)
			{
				if (!m_initialized) { return false; }

				if (pin < 0 || pin > 7)
				{
					m_logger->LogMessage("StampIO", "Invalid pin", Logger::Level::kError);
					return false;
				}

				if (m_pin_modes[pin] != PinMode::kNeopixel)
				{
					m_logger->LogMessage("StampIO", "Pin must be in neopixel mode to set color", Logger::Level::kError);
					return false;
				}

				uint8_t rgb[3] = { red, green, blue };
				m_i2c->SetSlave(m_address);
				return m_i2c->WriteRegs(0x70 + pin * 3, rgb, 3);
			}

			/**
			* @brief Gets a light object for a neopixel attached to a pin on the StampIO
			* 
			* @param The pin number the light is connected to (0-7)
			* 
			* @returns A pointer to the light object or nullopt if a failure occured
			*/
			std::optional<std::unique_ptr<Light>> StampIO::GetLight(uint8_t pin)
			{
				if (!m_initialized) { return std::nullopt; }

				if (pin < 0 || pin > 7)
				{
					m_logger->LogMessage("StampIO", "Invalid pin", Logger::Level::kError);
					return std::nullopt;
				}

				if (m_pin_modes[pin] != PinMode::kNeopixel)
				{
					m_logger->LogMessage("StampIO", "Pin must be in neopixel mode to create a light object", Logger::Level::kError);
					return std::nullopt;
				}

				return std::make_unique<StampIOLight>(*this, pin);
			}

			/**
			* @brief Sets the PWM duty cycle of a StampIO pin
			* 
			* @param The pin to set (0-7)
			* @param The duty cycle to set (0-100)
			* 
			* @returns True if setting the duty cycle was successful
			*/
			bool StampIO::SetPWM(uint8_t pin, uint8_t duty_cycle)
			{
				if (!m_initialized) { return false; }

				if (pin < 0 || pin > 7)
				{
					m_logger->LogMessage("StampIO", "Invalid pin", Logger::Level::kError);
					return false;
				}

				if (m_pin_modes[pin] != PinMode::kPWM)
				{
					m_logger->LogMessage("StampIO", "Pin must be in PWM mode to set PWM duty cycle", Logger::Level::kError);
					return false;
				}

				if (duty_cycle > 100)
				{
					m_logger->LogMessage("StampIO", "Invalid duty cycle", Logger::Level::kError);
					return false;
				}

				m_i2c->SetSlave(m_address);
				return m_i2c->WriteReg(0x90 + pin, duty_cycle);
			}

			void StampIO::Periodic()
			{
			}

			void StampIO::Enable()
			{
			}

			void StampIO::Disable()
			{
			}
		}
	}
}