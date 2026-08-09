#pragma once

#include "buses/bus.hpp"
#include "logger.hpp"
#include <string>
#include <iostream>
#include <fcntl.h>
#include <cstring>
#include <errno.h>
#include <vector>
#include <optional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <chrono>
#include <functional>

#ifdef __linux__
#include <unistd.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/serial.h>
#endif

namespace bellman
{
	namespace hal
	{
		namespace bus
		{
			class UART : public Bus
			{
			public:
				enum class Parity
				{
					kNone,
					kOdd,
					kEven
				};

				struct UARTConfig
				{
					std::string channel;
					int baud;
					int data_bits;
					int stop_bits;
					UART::Parity parity;
					bool rs485_mode;
					std::chrono::microseconds response_timeout{ 1000 }; // Max time to wait for a full response before giving up on a transaction
					std::chrono::microseconds turnaround_delay{ 0 }; // Extra settle time between TX and RX, beyond the calculated TX wait
					size_t max_queue_depth{ 32 }; // Caps queue backlog if a device stops responding
				};

				// Called on the IO thread
				using TransactionCallback = std::function<void(bool success, std::vector<uint8_t>&& response)>;

				UART(UARTConfig config);
				~UART() override;
				bool m_initialized;
				void Periodic() override;

				bool WriteBytes(const uint8_t* data, size_t len);
				bool WriteString(const std::string& str);

				// Raw poll of whatever's in the RX buffer right now
				std::optional<std::vector<uint8_t>> ReadBytes(size_t len);
				std::optional<std::string> ReadString(size_t max_len = 256);
				std::optional<std::string> ReadLine(size_t max_len = 256);

				// Queues a write then wait transaction and returns immediately
				bool QueueTransaction(const uint8_t* tx_data, size_t tx_len, size_t expected_rx_len, TransactionCallback callback = nullptr);

				UARTConfig GetConfig();

			private:
				Logger* m_logger;

				int m_port;
				std::string m_rx_buffer; // Buffers read data to wait until newline

				UARTConfig m_config;

				struct WorkItem
				{
					std::vector<uint8_t> tx;
					size_t expected_rx_len;
					TransactionCallback callback;
				};

				// Thread variables
				std::thread m_write_thread;
				std::mutex m_write_mutex;
				std::condition_variable m_write_cv;
				std::queue<WorkItem> m_work_queue;
				bool m_stop_thread;

#ifdef __linux__
				void IOThread();
				bool ConfigureSerial(int fd, int baud, int stop_bits, int data_bits, UART::Parity parity);
				bool ConfigureRS485(bool enabled);
#endif
			};
		}
	}
}