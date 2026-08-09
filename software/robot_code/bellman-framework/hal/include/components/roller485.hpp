#pragma once

#include "component.hpp"
#include "buses/uart.hpp"
#include <atomic>

namespace bellman
{
	namespace hal
	{
		namespace component
		{
			class Roller485 : public Component
			{
				static constexpr uint8_t CMD_ENABLE = 0x00;
				static constexpr uint8_t CMD_SET_MODE = 0x01;
				static constexpr uint8_t CMD_SPEED_MODE = 0x20;
				static constexpr uint8_t CMD_POS_MODE = 0x22;

				static constexpr uint8_t CMD_READBACK0 = 0x40;

				//static constexpr size_t MOTOR_DATA_SIZE = 11; // m_motor_data[15]/sizeof used instead
				static constexpr size_t READBACK_CMD_SIZE = 4;
				static constexpr size_t ACK_RESPONSE_SIZE = 15;
				static constexpr size_t READBACK0_RESPONSE_SIZE = 18;

			public:
				Roller485(bus::UART* rs485_bus, uint8_t id);
				~Roller485() override = default;

				void SetSpeedMode();
				void SetPosMode();

				bool SetSpeed(int32_t speed);
				float GetSpeed();
				bool SetPos(int32_t position);
				float GetPos();
				void SetMaxCurrent(int32_t current);
				void Stop();

				void Periodic() override;
				void Enable() override;
				void Disable() override;

				bool LastCommandAcked() const; // True if the last completed command was acked by the motor

			private:
				bus::UART* m_rs485;
				uint8_t m_id;

				int m_current_mode;
				int32_t m_current;

				uint8_t m_motor_data[15] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

				// Cached readback
				std::atomic<float> m_last_speed{ 0.0f };
				std::atomic<float> m_last_pos{ 0.0f };
				std::atomic<bool> m_last_ack_ok{ false };

				uint8_t M5CRC8(uint8_t* data, uint8_t len);
				bool VerifyResponse(const std::vector<uint8_t>& response, uint8_t sent_cmd);
			};
		}
	}
}