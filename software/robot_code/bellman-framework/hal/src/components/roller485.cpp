#include "components/roller485.hpp"

namespace bellman
{
	namespace hal
	{
		namespace component
		{
			Roller485::Roller485(bus::UART* rs485_bus, uint8_t id) : m_rs485(rs485_bus), m_id(id)
			{
				SetSpeedMode(); // Initalize the motor mode to speed mode
				m_current = 1200; // Set the current to 1200mA (max) to start
			}

			/**
			* @brief Sets the speed of the motor
			*
			* @param speed The target speed of the motor in RPM
			*
			* @returns True if the command was successfully queued for transmission (not
			* whether the motor acked it - see LastCommandAcked())
			*/
			bool Roller485::SetSpeed(int32_t speed)
			{
				if (speed < -21000000) { speed = -21000000; }
				else if (speed > 21000000) { speed = 21000000; }

				memset(m_motor_data, 0, sizeof(m_motor_data));
				m_motor_data[0] = 0x20;
				m_motor_data[1] = m_id;
				int32_t speedBytes = (int32_t)(speed * 100);
				int32_t currentBytes = (int32_t)(m_current * 100);
				m_motor_data[2] = speedBytes & 0xFF;
				m_motor_data[3] = (speedBytes >> 8) & 0xFF;
				m_motor_data[4] = (speedBytes >> 16) & 0xFF;
				m_motor_data[5] = (speedBytes >> 24) & 0xFF;
				m_motor_data[6] = currentBytes & 0xFF;
				m_motor_data[7] = (currentBytes >> 8) & 0xFF;
				m_motor_data[8] = (currentBytes >> 16) & 0xFF;
				m_motor_data[9] = (currentBytes >> 24) & 0xFF;
				m_motor_data[sizeof(m_motor_data) - 1] = Roller485::M5CRC8(m_motor_data, sizeof(m_motor_data) - 1);

				return m_rs485->QueueTransaction(m_motor_data, sizeof(m_motor_data), ACK_RESPONSE_SIZE, [this](bool ok, std::vector<uint8_t>&& response) { m_last_ack_ok.store(ok && VerifyResponse(response, CMD_SPEED_MODE), std::memory_order_relaxed); });
			}

			/**
			* @brief Gets the motor's current speed in RPM
			*
			* @returns The last speed reported by Periodic()'s readback (cached, non-blocking)
			*/
			float Roller485::GetSpeed()
			{
				return m_last_speed.load(std::memory_order_relaxed);
			}

			/**
			* @brief Sets the motor's position
			*
			* @param position The motor's target position setting
			*
			* @returns True if the command was successfully queued for transmission
			*/
			bool Roller485::SetPos(int32_t position)
			{
				if (position < -21000000) { position = -21000000; }
				else if (position > 21000000) { position = 21000000; }

				memset(m_motor_data, 0, sizeof(m_motor_data));
				m_motor_data[0] = 0x22;
				m_motor_data[1] = m_id;
				int32_t positionBytes = (int32_t)(position * 100);
				int32_t currentBytes = (int32_t)(m_current * 100);
				m_motor_data[2] = positionBytes & 0xFF;
				m_motor_data[3] = (positionBytes >> 8) & 0xFF;
				m_motor_data[4] = (positionBytes >> 16) & 0xFF;
				m_motor_data[5] = (positionBytes >> 24) & 0xFF;
				m_motor_data[6] = currentBytes & 0xFF;
				m_motor_data[7] = (currentBytes >> 8) & 0xFF;
				m_motor_data[8] = (currentBytes >> 16) & 0xFF;
				m_motor_data[9] = (currentBytes >> 24) & 0xFF;
				m_motor_data[sizeof(m_motor_data) - 1] = M5CRC8(m_motor_data, sizeof(m_motor_data) - 1);

				return m_rs485->QueueTransaction(m_motor_data, sizeof(m_motor_data), ACK_RESPONSE_SIZE, [this](bool ok, std::vector<uint8_t>&& response) { m_last_ack_ok.store(ok && VerifyResponse(response, CMD_POS_MODE), std::memory_order_relaxed); });
			}

			/**
			* @brief Gets the motor's current position
			*
			* @returns The last position reported by Periodic()'s readback (cached, non-blocking)
			*/
			float Roller485::GetPos()
			{
				return m_last_pos.load(std::memory_order_relaxed);
			}

			/**
			* @brief Sets the maximum current the motor can draw
			*
			* @param current The maximum current the motor should draw in milliamps, between -1200 and 1200
			*/
			void Roller485::SetMaxCurrent(int32_t current)
			{
				if (current < -1200)
				{
					m_current = -1200;
					return;
				}
				else if (current > 1200)
				{
					m_current = 1200;
					return;
				}
				else
				{
					m_current = current;
					return;
				}
			}

			/**
			* @brief Stops the motor
			*/
			void Roller485::Stop()
			{
				SetSpeed(0.0f);
			}

			// Queues a single readback for speed and position
			void Roller485::Periodic()
			{
				uint8_t readback[READBACK_CMD_SIZE] = {};
				readback[0] = CMD_READBACK0;
				readback[1] = m_id;
				readback[READBACK_CMD_SIZE - 1] = M5CRC8(readback, READBACK_CMD_SIZE - 1);

				m_rs485->QueueTransaction(readback, READBACK_CMD_SIZE, READBACK0_RESPONSE_SIZE,
					[this](bool ok, std::vector<uint8_t>&& response)
					{
						if (!ok || !VerifyResponse(response, CMD_READBACK0)) { return; }

						const int32_t speed_raw = static_cast<int32_t>(response[2] | (response[3] << 8) | (response[4] << 16) | (response[5] << 24));
						const int32_t pos_raw = static_cast<int32_t>(response[6] | (response[7] << 8) | (response[8] << 16) | (response[9] << 24));

						m_last_speed.store(speed_raw / 100.0f, std::memory_order_relaxed);
						m_last_pos.store(pos_raw / 100.0f, std::memory_order_relaxed);
					});
			}

			void Roller485::Enable()
			{
				// Set motor mode to "motor enable"
				memset(m_motor_data, 0, sizeof(m_motor_data));
				m_motor_data[0] = 0x00;
				m_motor_data[1] = m_id;
				m_motor_data[2] = 0x01;
				m_motor_data[sizeof(m_motor_data) - 1] = Roller485::M5CRC8(m_motor_data, sizeof(m_motor_data) - 1);
				m_rs485->QueueTransaction(m_motor_data, sizeof(m_motor_data), ACK_RESPONSE_SIZE, [this](bool ok, std::vector<uint8_t>&& response) { m_last_ack_ok.store(ok && VerifyResponse(response, CMD_ENABLE), std::memory_order_relaxed); }); // Send it to the motor
			}

			void Roller485::Disable()
			{
				// Set motor mode to "motor disable"
				memset(m_motor_data, 0, sizeof(m_motor_data));
				m_motor_data[0] = 0x00;
				m_motor_data[1] = m_id;
				m_motor_data[2] = 0x00;
				m_motor_data[sizeof(m_motor_data) - 1] = Roller485::M5CRC8(m_motor_data, sizeof(m_motor_data) - 1);
				m_rs485->QueueTransaction(m_motor_data, sizeof(m_motor_data), ACK_RESPONSE_SIZE, [this](bool ok, std::vector<uint8_t>&& response) { m_last_ack_ok.store(ok && VerifyResponse(response, CMD_ENABLE), std::memory_order_relaxed); }); // Send it to the motor
			}

			// Sets the motor to speed control mode
			void Roller485::SetSpeedMode()
			{
				memset(m_motor_data, 0, sizeof(m_motor_data));
				m_motor_data[0] = 0x01;
				m_motor_data[1] = m_id;
				m_motor_data[2] = 0x01;
				m_motor_data[sizeof(m_motor_data) - 1] = Roller485::M5CRC8(m_motor_data, sizeof(m_motor_data) - 1);
				m_rs485->QueueTransaction(m_motor_data, sizeof(m_motor_data), ACK_RESPONSE_SIZE, [this](bool ok, std::vector<uint8_t>&& response) { m_last_ack_ok.store(ok && VerifyResponse(response, CMD_SET_MODE), std::memory_order_relaxed); }); // Send it to the motor
				m_current_mode = 0;
			}

			// Sets the motor to position control mode
			void Roller485::SetPosMode()
			{
				memset(m_motor_data, 0, sizeof(m_motor_data));
				m_motor_data[0] = 0x01;
				m_motor_data[1] = m_id;
				m_motor_data[2] = 0x02;
				m_motor_data[sizeof(m_motor_data) - 1] = Roller485::M5CRC8(m_motor_data, sizeof(m_motor_data) - 1);
				m_rs485->QueueTransaction(m_motor_data, sizeof(m_motor_data), ACK_RESPONSE_SIZE, [this](bool ok, std::vector<uint8_t>&& response) { m_last_ack_ok.store(ok && VerifyResponse(response, CMD_SET_MODE), std::memory_order_relaxed); }); // Send it to the motor
				m_current_mode = 1;
			}

			bool Roller485::LastCommandAcked() const
			{
				return m_last_ack_ok.load(std::memory_order_relaxed);
			}

			// Calculates the CRC for a message, implementation from Unit-Roller485-RS485-Protocol-EN.pdf page 4
			uint8_t Roller485::M5CRC8(uint8_t* data, uint8_t len)
			{
				uint8_t crc, i;
				crc = 0x00;
				while (len--) {
					crc ^= *data++;
					for (i = 0; i < 8; i++) {
						if (crc & 0x01) { crc = (crc >> 1) ^ 0x8c; }
						else { crc >>= 1; }
					}
				}
				return crc;
			}

			// Validates a response packet from the motor
			bool Roller485::VerifyResponse(const std::vector<uint8_t>& response, uint8_t sent_cmd)
			{
				if (response.size() < 2) { return false; }

				// CRC covers every byte except the final one
				uint8_t calculated = M5CRC8(const_cast<uint8_t*>(response.data()), static_cast<uint8_t>(response.size() - 1));
				if (calculated != response.back()) { return false; }

				// Motor echoes command + 0x10 in the first byte
				if (response[0] != static_cast<uint8_t>(sent_cmd + 0x10)) { return false; }

				return true;
			}
		}
	}
}