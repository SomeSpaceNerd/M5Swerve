#include "device.hpp"

namespace bellman
{
	namespace hal
	{
		Device::Device()
		{
			// Constructor stuff
		}

		Device* Device::GetInstance()
		{
			if (instance == nullptr)
			{
				instance = new Device();
			}
			return instance;
		}
		
		bool Device::ConfigureHAL(const DeviceInfo& device_info)
		{
			if (device_info.ID > 25599) { return false; } // IPv4 octet range only allows for an ID under 25599 (which splits to the multicast address 239.255.99.1)
			// Add checks here for required but empty data (like RIL info)

			m_device_info = device_info;
			return true;
		}

		Device* Device::instance = nullptr;
	}
}