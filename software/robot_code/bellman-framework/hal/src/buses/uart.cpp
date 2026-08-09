#include "buses/uart.hpp"
#include <algorithm>

// termios2
#ifdef __linux__
struct termios2 {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t c_line;
    cc_t c_cc[19];
    speed_t c_ispeed;
    speed_t c_ospeed;
};

#ifndef TCGETS2
#define TCGETS2 _IOR('T', 0x2A, struct termios2)
#define TCSETS2 _IOW('T', 0x2B, struct termios2)
#endif

#ifndef BOTHER
#define BOTHER 0010000
#endif

#endif

namespace bellman
{
    namespace hal
    {
        namespace bus
        {
            UART::UART(UARTConfig config) : m_config(config)
            {
                m_logger = Logger::GetInstance();
                m_initialized = false;
                m_stop_thread = false;

#ifdef __linux__
                m_port = open(config.channel.c_str(), O_RDWR | O_NOCTTY); // O_NONBLOCK no longer needed in dedicated thread, O_SYNC isn't really a good idea to use with serial ports

                if (m_port < 0)
                {
                    m_logger->LogMessage("UART", "Failed to open channel: " + config.channel + " , " + strerror(errno), Logger::Level::kError);
                    return;
                }

                if (!ConfigureSerial(m_port, config.baud, config.stop_bits, config.data_bits, config.parity)) {
                    close(m_port);
                    return;
                }

                if (!ConfigureRS485(config.rs485_mode))
                {
                    close(m_port);
                    return;
                }

                m_initialized = true;
                m_write_thread = std::thread(&UART::IOThread, this);
#endif
            }

            UART::~UART()
            {
#ifdef __linux__
                {
                    std::lock_guard<std::mutex> lock(m_write_mutex);
                    m_stop_thread = true;
                }

                m_write_cv.notify_all();

                if (m_write_thread.joinable()) { m_write_thread.join(); }

                if (m_initialized && m_port >= 0)
                {
                    close(m_port);
                    m_port = -1;
                }
#endif
            }

#ifdef __linux__
            void UART::IOThread()
            {
                while (true)
                {
                    WorkItem item;
                    {
                        std::unique_lock<std::mutex> lock(m_write_mutex);
                        m_write_cv.wait(lock, [this] { return !m_work_queue.empty() || m_stop_thread; });

                        if (m_stop_thread && m_work_queue.empty()) { break; }

                        item = std::move(m_work_queue.front());
                        m_work_queue.pop();
                    }

                    // Discard stale unread bytes before starting a new transaction
                    tcflush(m_port, TCIFLUSH);

                    bool write_ok = true;
                    size_t total = 0;
                    while (total < item.tx.size())
                    {
                        ssize_t n = write(m_port, item.tx.data() + total, item.tx.size() - total);

                        if (n < 0)
                        {
                            m_logger->LogMessage("UART", "Write failed: " + std::string(strerror(errno)), Logger::Level::kError);
                            write_ok = false;
                            break;
                        }

                        if (n == 0) { break; }

                        total += n;
                    }

                    // Sleep the exact amount of time required for the transaction +10% for jitter
                    // replacement for tcdrain because it's slow
                    if (write_ok)
                    {
                        const int bits_per_frame = 1 + m_config.data_bits + (m_config.parity != UART::Parity::kNone ? 1 : 0) + m_config.stop_bits;
                        const long long tx_us = ((long long)item.tx.size() * bits_per_frame * 1000000LL) / m_config.baud;
                        std::this_thread::sleep_for(std::chrono::microseconds(tx_us * 11 / 10));

                        if (m_config.turnaround_delay.count() > 0)
                        {
                            std::this_thread::sleep_for(m_config.turnaround_delay);
                        }
                    }

                    std::vector<uint8_t> response;
                    bool rx_ok = true;

                    if (write_ok && item.expected_rx_len > 0)
                    {
                        response.reserve(item.expected_rx_len);
                        rx_ok = false;
                        const auto deadline = std::chrono::steady_clock::now() + m_config.response_timeout;

                        // Busy-poll
                        while (response.size() < item.expected_rx_len)
                        {
                            uint8_t buf[64];
                            size_t want = std::min(sizeof(buf), item.expected_rx_len - response.size());
                            ssize_t n = read(m_port, buf, want);

                            if (n > 0)
                            {
                                response.insert(response.end(), buf, buf + n);
                                if (response.size() >= item.expected_rx_len) { rx_ok = true; break; }
                            }

                            if (std::chrono::steady_clock::now() >= deadline) { break; }
                        }

                        if (!rx_ok)
                        {
                            m_logger->LogMessage("UART", "Transaction timed out waiting for response ("
                                + std::to_string(response.size()) + "/" + std::to_string(item.expected_rx_len) + " bytes)",
                                Logger::Level::kError);
                        }
                    }
                    else if (!write_ok)
                    {
                        rx_ok = false;
                    }

                    if (item.callback) { item.callback(write_ok && rx_ok, std::move(response)); }
                }
            }

            bool UART::ConfigureSerial(int fd, int baud, int stop_bits, int data_bits, UART::Parity parity) {
                struct termios2 tty {};

                if (ioctl(fd, TCGETS2, &tty) < 0)
                {
                    m_logger->LogMessage("UART", "TCGETS2 failed: " + std::string(strerror(errno)), Logger::Level::kError);
                    return false;
                }

                // Baud rate
                tty.c_cflag &= ~CBAUD;
                tty.c_cflag |= BOTHER;
                tty.c_ispeed = baud;
                tty.c_ospeed = baud;

                // Data bits
                tty.c_cflag &= ~CSIZE;
                switch (data_bits) {
                case 5: tty.c_cflag |= CS5; break;
                case 6: tty.c_cflag |= CS6; break;
                case 7: tty.c_cflag |= CS7; break;
                case 8: tty.c_cflag |= CS8; break;
                default:
                    m_logger->LogMessage("UART", "Invalid data bits, defaulting to 8", Logger::Level::kError);
                    tty.c_cflag |= CS8;
                    break;
                }

                // Stop bits
                if (stop_bits == 2) { tty.c_cflag |= CSTOPB; }
                else { tty.c_cflag &= ~CSTOPB; }

                // Parity
                switch (parity)
                {
                case UART::Parity::kNone:
                    tty.c_cflag &= ~PARENB;
                    break;

                case UART::Parity::kOdd:
                    tty.c_cflag |= PARENB;
                    tty.c_cflag |= PARODD;
                    break;

                case UART::Parity::kEven:
                    tty.c_cflag |= PARENB;
                    tty.c_cflag &= ~PARODD;
                    break;
                }

                // Control flags
                tty.c_cflag |= CREAD | CLOCAL;
                tty.c_cflag &= ~CRTSCTS;

                tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Input modes

                tty.c_oflag &= ~OPOST; // Output modes

                tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Local modes

                // Read behavior
                tty.c_cc[VMIN] = 0;
                tty.c_cc[VTIME] = 0; // 0 second timeout to avoid massive loop overruns

                // apply settings
                tcflush(fd, TCIOFLUSH);
                if (ioctl(fd, TCSETS2, &tty) == 0)
                {
                    ioctl(fd, TCGETS2, &tty);
                    m_logger->LogMessage("UART", "Configured channel at " + std::to_string(tty.c_ospeed) + " baud", Logger::Level::kInfo);
                }
                else
                {
                    m_logger->LogMessage("UART", "TCSETS2 failed: " + std::string(strerror(errno)), Logger::Level::kError);
                    return false;
                }
                return true;
            }

            bool UART::ConfigureRS485(bool enabled)
            {
                struct serial_rs485 rs485conf;
                std::memset(&rs485conf, 0, sizeof(rs485conf));

                if (ioctl(m_port, TIOCGRS485, &rs485conf) < 0)
                {
                    m_logger->LogMessage("UART", "TIOCGRS485 failed: " + std::string(strerror(errno)), Logger::Level::kError);
                    return false;
                }

                if (enabled)
                {
                    rs485conf.flags |= SER_RS485_ENABLED;
                    rs485conf.flags |= SER_RS485_RTS_ON_SEND;
                    rs485conf.flags &= ~SER_RS485_RTS_AFTER_SEND;

                    rs485conf.delay_rts_before_send = 0;
                    rs485conf.delay_rts_after_send = 0;
                }
                else
                {
                    rs485conf.flags &= ~SER_RS485_ENABLED;
                }

                if (ioctl(m_port, TIOCSRS485, &rs485conf) < 0)
                {
                    m_logger->LogMessage("UART", "TIOCSRS485 failed: " + std::string(strerror(errno)), Logger::Level::kError);
                    return false;
                }

                return true;
            }
#endif

            bool UART::QueueTransaction(const uint8_t* tx_data, size_t tx_len, size_t expected_rx_len, TransactionCallback callback)
            {
#ifdef __linux__
                if (!m_initialized) { return false; }

                {
                    std::unique_lock<std::mutex> lock(m_write_mutex);
                    if (m_stop_thread) { return false; }
                    if (m_work_queue.size() >= m_config.max_queue_depth) { return false; }

                    m_work_queue.push(WorkItem{ std::vector<uint8_t>(tx_data, tx_data + tx_len), expected_rx_len, std::move(callback) });
                }
                m_write_cv.notify_one();
                return true;
#else
                return false;
#endif
            }

            bool UART::WriteBytes(const uint8_t* data, size_t len) {
                return QueueTransaction(data, len, 0, nullptr);
            }

            bool UART::WriteString(const std::string& str) { return WriteBytes(reinterpret_cast<const uint8_t*>(str.data()), str.size()); }

            std::optional<std::vector<uint8_t>> UART::ReadBytes(size_t len) {
#ifdef __linux__
                std::vector<uint8_t> buffer(len);

                ssize_t n = read(m_port, buffer.data(), len);

                if (n <= 0)
                {
                    //if (errno == EAGAIN || errno == EWOULDBLOCK) { return std::nullopt; }
                    return std::nullopt;
                }

                buffer.resize(static_cast<size_t>(n));
                return buffer;
#else
                return std::nullopt;
#endif
            }

            std::optional<std::string> UART::ReadString(size_t max_len) {
#ifdef __linux__
                std::string out;
                out.resize(max_len);

                ssize_t n = read(m_port, out.data(), max_len);

                if (n <= 0)
                {
                    //if (errno == EAGAIN || errno == EWOULDBLOCK) { return std::nullopt; }
                    return std::nullopt;
                }

                out.resize(static_cast<size_t>(n));
                return out;
#else
                return std::nullopt;
#endif
            }

            std::optional<std::string> UART::ReadLine(size_t max_len)
            {
#ifdef __linux__
                char buffer[256];

                while (true)
                {
                    ssize_t n = read(m_port, buffer, sizeof(buffer));

                    if (n < 0)
                    {
                        if (errno == EAGAIN || errno == EWOULDBLOCK) { break; }
                        return std::nullopt;
                    }

                    if (n == 0) { break; }

                    m_rx_buffer.append(buffer, static_cast<size_t>(n));
                }

                std::string::size_type new_line_pos = m_rx_buffer.find('\n');

                if (new_line_pos == std::string::npos) { return std::nullopt; }

                std::string line = m_rx_buffer.substr(0, new_line_pos);

                if (!line.empty() && line.back() == '\r') { line.pop_back(); }

                m_rx_buffer.erase(0, new_line_pos + 1);

                if (line.size() > max_len) { line.resize(max_len); }

                return line;
#else
                return std::nullopt;
#endif
            }

            UART::UARTConfig UART::GetConfig() { return m_config; }

            void UART::Periodic()
            {
            }
        }
    }
}