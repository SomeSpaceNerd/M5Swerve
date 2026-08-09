#pragma once

#include "driver_station.hpp"
#include "state_control.hpp"
#include "device.hpp"
#include "buses/bus.hpp"
#include "buses/uart.hpp"
#include "buses/i2c.hpp"
#include "components/component.hpp"
#include "components/light.hpp"
#include <vector>
#include <string>
#include <algorithm>
#include <memory>
#include <array>
#include <optional>
#include <string_view>
#include <unordered_map>

namespace bellman
{
    namespace hal
    {
        class IHal
        {
        public:
            static IHal* GetInstance();

            void ConfigureHAL(Device::DeviceInfo device_info);
            std::optional<Device::DeviceInfo> GetDeviceInfo();
            void ConfigureRIL(std::unique_ptr<component::Light> ril, component::Light::ColorCorrection correction);

            void Periodic();
            void Enable();
            void Disable(bool e_stopped);

            void SetBattery(float battery_voltage);

            bus::UART* GetUART(bus::UART::UARTConfig config);
            bus::I2C* GetI2C(std::string channel);

            // Template factory function for creating any component type
            template<typename T, typename... Args>
            T* GetComponent(Args&&... args)
            {
                static_assert(std::is_base_of_v<component::Component, T>, "T must derive from Component");

                std::unique_ptr<T> ptr = std::make_unique<T>(std::forward<Args>(args)...);
                T* raw = ptr.get();
                m_components.push_back(std::move(ptr));
                return raw;
            }

        private:
            IHal();
            static IHal* instance;

            // Configuration
            bool m_configured = false;
            bool m_ril_configured = false;

            // State tracking
            bool m_enabled = false;
            unsigned int m_enabled_loopcounts = 0;
            bool m_ril_on = false;
            bool m_e_stopped = false;

            // RIL
            std::unique_ptr<component::Light> m_ril;
            void UpdateRIL();
            // RGB RIL colors
            const std::unordered_map<std::string_view, std::array<uint8_t, 3>> m_ril_colors =
            {
                {"Healthy", {255, 40, 0}}, // Orange (normal operation)
                {"Warning", {255, 140, 0}}, // Yellow (low battery)
                {"Fault", {255, 0, 0}}, // Red (sticky faults)
                {"NoDS", {0, 0, 255}} // Blue (no Driver Station)
            };

            // Classes
            DriverStation* m_ds;
            State* m_state;
            Device* m_device;

            //Battery
            static constexpr std::size_t BAT_WINDOW = 250; // How many samples to keep to calculate average battery voltage
            std::array<float, BAT_WINDOW> m_bat_buffer;
            std::size_t m_bat_index = 0;
            std::size_t m_bat_count = 0;
            float m_bat_sum = 0.0f;
            float m_bat_avg = 0.0f;
            int m_warned = 0; // Tracks which warning messages have been given already
            std::array<float, 4> m_bat_voltages;

            // Battery chemistry and voltages
            const std::unordered_map<std::string_view, std::array<float, 4>> m_bat_types =
            {
                //Chemistry, Full, Low, Critical, Dangerous
                {"None", {0.00f, 0.00f, 0.00f, 0.00f}},
                {"LiPo", {4.20f, 3.50f, 3.40f, 3.20f}},
                {"Li-Ion", {4.20f, 3.50f, 3.30f, 3.00f}},
                {"LiHv", {4.35f, 3.60f, 3.50f, 3.30f}},
                {"LiFePO4", {3.60f, 3.20f, 3.00f, 2.50f}},
                {"NiMH", {1.45f, 1.15f, 1.05f, 1.00f}},
                {"NiCd", {1.40f, 1.10f, 1.00f, 0.90}},
                {"Alkaline", {1.60f, 1.20f, 1.00f, 0.80f}},
                {"Lead-Acid", {2.12f, 2.00f, 1.92f, 1.75f}}
            };

            // Buses
            std::vector<std::unique_ptr<bus::Bus>> m_buses; // Holds all of the currently defined buses

            std::vector<std::string> m_uarts; // Holds all of the currently used UART channels
            std::vector<std::string> m_i2cs; // Holds all of the currently used I2C channels

            // Components
            std::vector < std::unique_ptr<component::Component>> m_components;
        };
    }
}
