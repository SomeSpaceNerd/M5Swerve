#include "hal.hpp"

namespace bellman
{
	namespace hal
	{
		IHal::IHal()
		{
			m_device = Device::GetInstance();
			m_state = State::GetInstance();
			m_ds = DriverStation::GetInstance();
		}

		IHal* IHal::GetInstance()
		{
			if (instance == nullptr)
			{
				instance = new IHal();
			}
			return instance;
		}

		/**
		* @brief Configures the HAL for the device
		* 
		* @param device_info The Device struct to configure the HAL with
		*/
		void IHal::ConfigureHAL(Device::DeviceInfo device_info) 
		{ 
			bool result = m_device->ConfigureHAL(device_info);
			if (!result) 
			{ 
				m_configured = false;
				std::cerr << "Invalid HAL configuration" << std::endl;
				m_device->SetStickyFault("Invalid HAL configuration"); 
				return;
			}

			// Setup battery voltages
			m_bat_voltages = m_bat_types.at(m_device->GetDeviceInfo().bat_type);

			// Setup driver station after configuration so it can use the configuration (ID and robot network)
			m_ds->ConfigureDS();
			if (!m_ds->m_initialized)
			{
				m_configured = false;
				return;
			}
			else { m_configured = true; }
		}

		/**
		* @brief Configures an RIL (Robot Indicator Light) object for the HAL
		*/
		void IHal::ConfigureRIL(std::unique_ptr<component::Light> ril, component::Light::ColorCorrection correction)
		{
			m_ril = std::move(ril);
			m_ril->SetCorrection(correction);
			m_ril_configured = true;
			UpdateRIL();
			m_ril_on = true;
			m_ril->SetOn(m_ril_on);
		}

		// NoDS takes precedence, then fault, warning, and healthy
		void IHal::UpdateRIL()
		{
			if (m_ril_configured)
			{
				if (!m_ds->GetHasDS())
				{
					m_ril->SetColor(m_ril_colors.at("NoDS")[0], m_ril_colors.at("NoDS")[1], m_ril_colors.at("NoDS")[2]);
					return;
				}

				if (m_e_stopped)
				{
					m_ril->SetColor(m_ril_colors.at("Fault")[0], m_ril_colors.at("Fault")[1], m_ril_colors.at("Fault")[2]);
					return;
				}

				if (m_warned > 0)
				{
					m_ril->SetColor(m_ril_colors.at("Warning")[0], m_ril_colors.at("Warning")[1], m_ril_colors.at("Warning")[2]);
					return;
				}

				m_ril->SetColor(m_ril_colors.at("Healthy")[0], m_ril_colors.at("Healthy")[1], m_ril_colors.at("Healthy")[2]);
			}
		}

		/**
		* @brief Get the device information that the HAL was configured with
		* 
		* @returns The Device struct the HAL was configured with
		*/
		std::optional<Device::DeviceInfo> IHal::GetDeviceInfo() 
		{ 
			if (m_configured) { return m_device->GetDeviceInfo(); } 
			else { return std::nullopt; }
		}

		/**
		* @brief Sets the battery voltage used by the HAL
		* 
		* @param battery_voltage The current voltage of the battery
		*/
		void IHal::SetBattery(float battery_voltage) 
		{
			if (m_configured)
			{
				if (m_device->GetDeviceInfo().bat_type == "None") { m_ds->SetBattery(100, battery_voltage); } // Constant power source
				else
				{
					// Calculate the percentage based on the dead and fully charged voltages, also clamp to 0-100% just in case
					float bat_scalar = static_cast<float>(m_device->GetDeviceInfo().bat_cells);
					m_ds->SetBattery(std::clamp((battery_voltage - (m_bat_voltages[3] * bat_scalar)) / ((m_bat_voltages[0] * bat_scalar) - (m_bat_voltages[3] * bat_scalar)), 0.0f, 1.0f) * 100.0f, battery_voltage);

					// Calculate the rolling average battery voltage
					if (m_bat_count < BAT_WINDOW) {
						m_bat_buffer[m_bat_index] = battery_voltage;
						m_bat_sum += battery_voltage;
						++m_bat_count;
					}
					else {
						m_bat_sum -= m_bat_buffer[m_bat_index];
						m_bat_buffer[m_bat_index] = battery_voltage;
						m_bat_sum += battery_voltage;
					}
					m_bat_index++;
					if (m_bat_index == BAT_WINDOW) m_bat_index = 0;
					m_bat_avg = m_bat_sum / static_cast<float>(m_bat_count);

					// Normal voltage
					if (m_bat_avg > (m_bat_voltages[1] * m_device->GetDeviceInfo().bat_cells)) 
					{ 
						m_warned = 0; 
						UpdateRIL();
					}

					// Low
					if (m_bat_avg <= (m_bat_voltages[1] * m_device->GetDeviceInfo().bat_cells))
					{
						if (m_warned < 1)
						{
							m_ds->SendLogMessage("WARNING", "HAL", "Battery is low");
							m_warned = 1;
							UpdateRIL();
						}
					}

					// Critically low
					if (m_bat_avg <= (m_bat_voltages[2] * m_device->GetDeviceInfo().bat_cells))
					{
						if (m_warned < 2)
						{
							m_ds->SendLogMessage("WARNING", "HAL", "Battery is critically low, consider disabling soon");
							m_warned = 2;
							UpdateRIL();
						}
					}

					// Dangerously low
					if (m_bat_avg <= (m_bat_voltages[3] * m_device->GetDeviceInfo().bat_cells))
					{
						if (m_warned < 3)
						{
							m_state->Disable();
							//m_ds->SendLogMessage("FATAL", "HAL", "Battery is dangerously low, disabling");
							m_device->SetStickyFault("Battery is dangerously low"); // Set a sticky fault when the battery is at risk of over-discharge
							m_warned = 3;
							UpdateRIL();
						}
					}
				}
			}
		}

		void IHal::Periodic()
		{
			if (m_configured)
			{
				if (m_ril_configured)
				{
					// Increment the enabled loopcounter
					if (m_enabled) { m_enabled_loopcounts++; }

					// Update the RIL
					UpdateRIL();

					// Blink the RIL at 1Hz
					if ((m_enabled_loopcounts % 50 == 0) && m_enabled)
					{
						m_ril_on = !m_ril_on;
						m_ril->SetOn(m_ril_on);
					}

					// Call periodic on every component that is currently defined
					for (const std::unique_ptr<component::Component>& component : m_components)
					{
						component->Periodic();
					}

					// Call periodic on every bus that is currently defined
					for (const std::unique_ptr<bus::Bus>& bus : m_buses)
					{
						bus->Periodic();
					}
				}
				else { m_device->SetStickyFault("HAL Periodic called without valid RIL configuration"); }
			}
			else { m_device->SetStickyFault("HAL Periodic called without valid configuration"); }
		}

		void IHal::Disable(bool e_stopped)
		{
			// Call disable on every component that is currently defined
			for (const std::unique_ptr<component::Component>& component : m_components)
			{
				component->Disable();
			}

			m_enabled = false;
			m_enabled_loopcounts = 0;
			m_e_stopped = e_stopped;
			UpdateRIL();

			// Set the RIL solid on
			m_ril_on = true;
			m_ril->SetOn(m_ril_on);
		}

		void IHal::Enable()
		{
			if (m_configured)
			{
				if (m_ril_configured)
				{
					m_enabled = true;

					for (const std::unique_ptr<component::Component>& component : m_components)
					{
						component->Enable();
					}
				}
				else { m_device->SetStickyFault("HAL Enable called without valid RIL configuration"); }
			}
			else { m_device->SetStickyFault("HAL Enable called without valid configuration"); }
		}

		/**
		* @brief Gets a new UART object
		*
		* @param config The UARTConfig object containing the channel configuration
		*
		* @returns A pointer to the UART object
		*/
		bus::UART* IHal::GetUART(bus::UART::UARTConfig config)
		{
			if (m_configured)
			{
				// Check if the channel is already opened and return nullptr if it is
				std::vector<std::string>::iterator it = std::find(m_uarts.begin(), m_uarts.end(), config.channel);
				if (it != m_uarts.end())
				{
					return nullptr;
				}
				m_uarts.push_back(config.channel);

				m_buses.push_back(std::make_unique<bus::UART>(config));
				if (!static_cast<bus::UART*>(m_buses.back().get())->m_initialized)
				{
					m_uarts.pop_back();
					m_buses.pop_back();
					return nullptr;
				}
				return static_cast<bus::UART*>(m_buses.back().get());
			}
			else 
			{ 
				m_device->SetStickyFault("GetUART called without valid HAL configuration"); 
				return nullptr;
			}
		}

		/**
		* @brief Gets a new I2C object
		*
		* @param channel The channel identifier string
		*
		* @returns A pointer to the I2C object
		*/
		bus::I2C* IHal::GetI2C(std::string channel)
		{
			if (m_configured)
			{
				// Check if the channel is already opened and return nullptr if it is
				std::vector<std::string>::iterator it = std::find(m_i2cs.begin(), m_i2cs.end(), channel);
				if (it != m_i2cs.end())
				{
					return nullptr;
				}
				m_i2cs.push_back(channel);

				m_buses.push_back(std::make_unique<bus::I2C>(channel));
				if (!static_cast<bus::I2C*>(m_buses.back().get())->m_initialized)
				{
					m_i2cs.pop_back();
					m_buses.pop_back();
					return nullptr;
				}
				return static_cast<bus::I2C*>(m_buses.back().get());
			}
			else 
			{ 
				m_device->SetStickyFault("GetI2C called without valid HAL configuration");
				return nullptr; 
			}
		}
	}

	hal::IHal* hal::IHal::instance = nullptr;
}