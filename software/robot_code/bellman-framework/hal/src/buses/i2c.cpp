#include "buses/i2c.hpp"

namespace bellman
{
    namespace hal
    {
        namespace bus
        {
            I2C::I2C(std::string channel)
            {
                m_logger = Logger::GetInstance();
                m_initialized = false;

#ifdef __linux__
                m_fd = open(channel.c_str(), O_RDWR);
                if (m_fd < 0)
                {
                    m_logger->LogMessage("I2C", "Failed to open I2C Bus", Logger::Level::kError);
                    return;
                }
                m_initialized = true;
                return;
#endif
            }

            I2C::~I2C()
            {
#ifdef __linux__
                if (m_initialized && m_fd >= 0)
                {
                    close(m_fd);
                    m_fd = -1;
                }
#endif
            }

            bool I2C::SetSlave(uint8_t addr)
            {
#ifdef __linux__
                if (ioctl(m_fd, I2C_SLAVE, addr) < 0)
                {
                    m_logger->LogMessage("I2C", "Set slave address failed", Logger::Level::kError);
                    return false;
                }

                m_addr = addr;
                return true;
#else
                return false;
#endif
            }

            bool I2C::WriteBytes(const uint8_t * data, size_t len)
            {
#ifdef __linux__
                if (write(m_fd, data, len) != (ssize_t)len)
                {
                    m_logger->LogMessage("I2C", "writeBytes failed", Logger::Level::kError);
                    return false;
                }

                return true;
#else
                return false;
#endif
            }

            bool I2C::Transfer(const uint8_t * tx, size_t txLen, std::vector<uint8_t>&rx, size_t rxLen)
            {
#ifdef __linux__
                rx.clear();
                rx.resize(rxLen);

                struct i2c_msg msgs[2];

                msgs[0].addr = m_addr;
                msgs[0].flags = 0;
                msgs[0].len = txLen;
                msgs[0].buf = (uint8_t*)tx;

                msgs[1].addr = m_addr;
                msgs[1].flags = I2C_M_RD;
                msgs[1].len = rxLen;
                msgs[1].buf = &rx[0];

                struct i2c_rdwr_ioctl_data ioctl_data;

                ioctl_data.msgs = msgs;
                ioctl_data.nmsgs = 2;

                if (ioctl(m_fd, I2C_RDWR, &ioctl_data) < 0)
                {
                    m_logger->LogMessage("I2C", "transfer failed", Logger::Level::kError);
                    return false;
                }

                return true;
#else
                return false;
#endif
            }

            bool I2C::WriteReg(uint8_t reg, uint8_t value)
            {
                uint8_t buf[2];

                buf[0] = reg;
                buf[1] = value;

                return WriteBytes(buf, 2);
            }

            bool I2C::WriteRegs(uint8_t reg, const uint8_t* data, size_t len)
            {
                std::vector<uint8_t> buf;
                buf.reserve(len + 1);

                buf.push_back(reg);
                buf.insert(buf.end(), data, data + len);

                return WriteBytes(buf.data(), buf.size());
            }

            std::optional<uint8_t> I2C::ReadReg(uint8_t reg)
            {
                if (!WriteBytes(&reg, 1))
                {
                    return std::nullopt;
                }

                uint8_t value;

#ifdef __linux__
                if (read(m_fd, &value, 1) != 1)
                {
                    m_logger->LogMessage("I2C", "readReg failed", Logger::Level::kError);
                    return std::nullopt;
                }

                return value;
#else
                return std::nullopt;
#endif
            }

            std::optional<std::vector<uint8_t>> I2C::ReadRegs(uint8_t reg, size_t len)
            {
#ifdef __linux__
                if (len == 0) { return std::vector<uint8_t>{}; }
                std::vector<uint8_t> rx;
                if (!Transfer(&reg, 1, rx, len)) { return std::nullopt; }
                return rx;
#else
                return std::nullopt;
#endif
            }

            void I2C::Periodic()
            {

            }
        }
    }
}