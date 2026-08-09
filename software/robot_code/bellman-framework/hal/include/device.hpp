#pragma once

#include <vector>
#include <string>
#include <string_view>

namespace bellman
{
    namespace hal
    {
        class Device
        {
            public:
                inline static constexpr std::string_view m_version = FEMTOBOT_VERSION;

                struct DeviceInfo
                {
                    std::string device_name = "UNKNOWN";
                    int ID = 0;
                    std::string bat_type = "None";
                    int bat_cells = 0;
                    std::string robot_network = "eth0";
                };

                static Device* GetInstance();

                bool ConfigureHAL(const DeviceInfo& device_info);
                const DeviceInfo& GetDeviceInfo() const { return m_device_info; }

                void SetStickyFault(std::string identifier) { m_sticky_faults.push_back(identifier); }
                const std::vector<std::string>& GetStickyFaults() const {return m_sticky_faults; }

            private:
                Device();
                static Device* instance;

                DeviceInfo m_device_info;

                std::vector<std::string> m_sticky_faults;
        };
    }
}