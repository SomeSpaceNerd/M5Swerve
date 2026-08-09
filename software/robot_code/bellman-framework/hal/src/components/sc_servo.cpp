#include "components/sc_servo.hpp"

namespace bellman
{
	namespace hal
	{
		namespace component
		{
			SCServo::SCServo(bus::UART* uart_bus, uint8_t id) : m_uart(uart_bus), m_id(id)
			{

			}

			/**
			 * @brief Commands the servo to move to a target position
			 *
			 * @param steps Target position [0–1023]
			 * @param speed Steps per second; 0 = hardware maximum
			 * @param time_ms Travel time in ms; 0 = fastest possible
			 *
			 * @returns True if the command was successfully queued for transmission
			 */
			bool SCServo::SetPos(int steps, uint16_t speed, uint16_t time_ms)
			{
				if (steps < MIN_STEPS) { steps = MIN_STEPS; }
				else if (steps > MAX_STEPS) { steps = MAX_STEPS; }

				// Build big-endian payload
				uint8_t data[6] =
				{
					static_cast<uint8_t>((steps >> 8) & 0xFF),
					static_cast<uint8_t>(steps & 0xFF),
					static_cast<uint8_t>((time_ms >> 8) & 0xFF),
					static_cast<uint8_t>(time_ms & 0xFF),
					static_cast<uint8_t>((speed >> 8) & 0xFF),
					static_cast<uint8_t>(speed & 0xFF)
				};

				uint8_t buf[WRITE6_PACKET_SIZE];
				BuildWritePacket(buf, REG_GOAL_POSITION_L, data, 6);

				return m_uart->QueueTransaction(buf, sizeof(buf), ACK_RESPONSE_SIZE,
					[this](bool ok, std::vector<uint8_t>&& response)
					{
						m_last_ack_ok.store(ok && VerifyAck(response), std::memory_order_relaxed);
					});
			}

			/**
			 * @brief Reads the servo's current measured position
			 *
			 * @returns The last position reported by Periodic()'s readback in steps [0.0–1023.0], or -1.0 if nothing's been read back yet
			 */
			float SCServo::GetPos()
			{
				return m_last_pos.load(std::memory_order_relaxed);
			}

			/**
			 * @brief Holds the servo at its current position
			 *
			 * @returns True if the command was successfully queued for transmission
			 */
			bool SCServo::Stop()
			{
				float current = GetPos();
				if (current < 0.0f) { return false; } // No cached reading yet
				return SetPos(static_cast<int>(current));
			}

			// Queues a position readback, updates the cached value used by GetPos()
			void SCServo::Periodic()
			{
				uint8_t buf[READ_PACKET_SIZE];
				BuildReadPacket(buf, REG_PRESENT_POSITION_L, 2);

				m_uart->QueueTransaction(buf, sizeof(buf), READ2_RESPONSE_SIZE,
					[this](bool ok, std::vector<uint8_t>&& response)
					{
						if (!ok || !VerifyAck(response)) { return; }
						float pos = static_cast<float>((static_cast<uint16_t>(response[5]) << 8) | response[6]);
						m_last_pos.store(pos, std::memory_order_relaxed);
					});
			}

			void SCServo::Enable() { WriteTorque(0x01); }

			void SCServo::Disable() { WriteTorque(0x00); }

			bool SCServo::WriteTorque(uint8_t enable)
			{
				uint8_t data[1] = { enable };
				uint8_t buf[WRITE1_PACKET_SIZE];
				BuildWritePacket(buf, REG_TORQUE_ENABLE, data, 1);

				return m_uart->QueueTransaction(buf, sizeof(buf), ACK_RESPONSE_SIZE,
					[this](bool ok, std::vector<uint8_t>&& response)
					{
						m_last_ack_ok.store(ok && VerifyAck(response), std::memory_order_relaxed);
					});
			}

			void SCServo::BuildWritePacket(uint8_t* buf, uint8_t reg,
				const uint8_t* data, uint8_t data_len)
			{
				buf[0] = HEADER;
				buf[1] = HEADER;
				buf[2] = m_id;
				buf[3] = static_cast<uint8_t>(data_len + 3);
				buf[4] = INST_WRITE;
				buf[5] = reg;
				for (uint8_t i = 0; i < data_len; i++) {
					buf[6 + i] = data[i];
				}
				buf[6 + data_len] = Checksum(buf + 2, static_cast<size_t>(4 + data_len));
			}

			void SCServo::BuildReadPacket(uint8_t* buf, uint8_t reg, uint8_t read_len)
			{
				buf[0] = HEADER;
				buf[1] = HEADER;
				buf[2] = m_id;
				buf[3] = 0x04;
				buf[4] = INST_READ;
				buf[5] = reg;
				buf[6] = read_len;
				buf[7] = Checksum(buf + 2, 5);
			}

			uint8_t SCServo::Checksum(const uint8_t* buf, size_t len) const
			{
				uint8_t sum = 0;
				for (size_t i = 0; i < len; i++) {
					sum += buf[i];
				}
				return ~sum;
			}

			bool SCServo::VerifyAck(const std::vector<uint8_t>& response) const
			{
				if (response.size() < ACK_RESPONSE_SIZE) { return false; }
				if (response[0] != HEADER || response[1] != HEADER) { return false; }
				if (response[2] != m_id) { return false; }

				uint8_t expected = Checksum(response.data() + 2, response.size() - 3);
				if (expected != response.back()) { return false; }

				// Error byte is always at index 4 in both response types
				return response[4] == 0x00;
			}

			bool SCServo::LastCommandAcked() const
			{
				return m_last_ack_ok.load(std::memory_order_relaxed);
			}

		}
	}
}