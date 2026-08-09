#pragma once

#include "buses/bus.hpp"
#include "logger.hpp"
#include <string>
#include <fcntl.h>
#include <cstring>
#include <cstdio>
#include <optional>

#ifdef __linux__
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#endif

namespace bellman
{
	namespace hal
	{
		namespace bus
		{
			class I2C : public Bus
			{
			public:
				I2C(std::string channel);
				~I2C() override;

				void Periodic() override;

				bool SetSlave(uint8_t addr);

				bool WriteBytes(const uint8_t* data, size_t len);

				bool Transfer(const uint8_t* tx, size_t txLen, std::vector<uint8_t>& rx, size_t rxLen);

				bool WriteReg(uint8_t reg, uint8_t value);

				bool WriteRegs(uint8_t reg, const uint8_t* data, size_t len);

				std::optional<uint8_t> ReadReg(uint8_t reg);

				std::optional<std::vector<uint8_t>> ReadRegs(uint8_t reg, size_t len);

				bool m_initialized;

			private:
				Logger* m_logger;

				uint8_t m_addr;
				int m_fd;
			};
		}
	}
}