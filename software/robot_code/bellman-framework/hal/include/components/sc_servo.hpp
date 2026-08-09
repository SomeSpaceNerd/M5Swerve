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
			class SCServo : public Component
			{
				static constexpr uint8_t INST_READ = 0x02;
				static constexpr uint8_t INST_WRITE = 0x03;
				static constexpr uint8_t HEADER = 0xFF;

				static constexpr uint8_t REG_TORQUE_ENABLE = 0x28;
				static constexpr uint8_t REG_GOAL_POSITION_L = 0x2A;

				static constexpr uint8_t REG_PRESENT_POSITION_L = 0x38;

				static constexpr size_t WRITE1_PACKET_SIZE = 8;
				static constexpr size_t WRITE6_PACKET_SIZE = 13;
				static constexpr size_t READ_PACKET_SIZE = 8;
				static constexpr size_t ACK_RESPONSE_SIZE = 6;
				static constexpr size_t READ2_RESPONSE_SIZE = 8;

				static constexpr int32_t MAX_STEPS = 1023;
				static constexpr int32_t MIN_STEPS = 0;

			public:
				SCServo(bus::UART* uart_bus, uint8_t id);
				~SCServo() override = default;

				bool SetPos(int steps, uint16_t speed = 1000, uint16_t time_ms = 0);
				float GetPos();
				bool Stop();

				void Periodic() override;
				void Enable()  override;
				void Disable() override;

				bool LastCommandAcked() const; // True if the last completed command was acked by the servo

			private:
				bus::UART* m_uart;
				uint8_t    m_id;

				// Cached readback
				std::atomic<float> m_last_pos{ -1.0f };
				std::atomic<bool> m_last_ack_ok{ false };

				bool WriteTorque(uint8_t enable);
				void BuildWritePacket(uint8_t* buf, uint8_t reg, const uint8_t* data, uint8_t data_len);
				void BuildReadPacket(uint8_t* buf, uint8_t reg, uint8_t read_len);
				uint8_t Checksum(const uint8_t* buf, size_t len) const;
				bool VerifyAck(const std::vector<uint8_t>& response) const;
			};
		}
	}
}