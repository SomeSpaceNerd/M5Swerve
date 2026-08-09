#pragma once
#include "robot_to_ds.pb.h"
#include "ds_to_robot.pb.h"
#include "robot_advertisement.pb.h"
#include "robot_log.pb.h"
#include "device.hpp"
#include "state_control.hpp"
#include <string>
#include <chrono>
#include <cstring>
#include <cmath>
#include <iostream>
#include <optional>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <unordered_map>
#include <utility>
#include <vector>
#include <algorithm>
#include <cstdio>
// Nanopb
#include <pb.h>
#include <pb_common.h>
#include <pb_encode.h>
#include <pb_decode.h>
#ifdef __linux__
#include <fstream>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <net/if.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <cerrno>
#include <cstdlib>
#endif 
namespace bellman
{
	namespace hal
	{
		class DriverStation
		{
		public:
			enum Direction
			{
				kCenter = 0,
				kUp = 1,
				kRight = 2,
				kDown = 3,
				kLeft = 4,
				KRightUp = 5,
				KRightDown = 6,
				kLeftUp = 7,
				kLeftDown = 8
			};
			static DriverStation* GetInstance();
			void ConfigureDS();
			bool m_initialized;
			bool Periodic();
			void SendLogMessage(std::string level, std::string caller, std::string message);
			void SetBattery(float percent, float volts);
			bool GetHasDS();
			std::string GetActiveMode();
			std::optional<bool> GetButton(int joystick, int input);
			std::optional<float> GetAxis(int joystick, int input);
			std::optional<Direction> GetDirection(int joystick);
		private:
			DriverStation();
			~DriverStation();
			static DriverStation* instance;
			std::string m_multicast_ip;
			float GetCPUPercent();
			float GetRAMPercent();
			void RestartRobotCode();
			void RestartController();
			void RegisterEStop();
			void NetworkThread(bool has_ds_snapshot, long long last_received_snapshot, bool enabled_snapshot, bool clear_enable_snapshot, bool e_stopped_snapshot, bool send_ad_snapshot, std::vector<std::string> opmode_name_snapshot);
			void NetworkThreadLoop(); // Persistent worker body; parks on m_net_cv_work between ticks instead of being spawned fresh each tick
			State* m_state;
			Device* m_device;
			bool m_has_ds = false;
			bool m_e_stopped = false;
			long long m_last_received = 0;
			std::chrono::steady_clock::time_point m_now;
			RobotToDS m_outgoing_packet; // Staging buffer for main thread
			RobotLog m_outgoing_log_packet; // Staging log buffer for main thread
			DSToRobot m_inbound_packet; // Live buffer synced at loop start
			std::unordered_map<std::string, std::shared_ptr<RobotMode>> m_opmodes;
			std::vector<std::string> m_opmode_names;
			std::string m_active_opmode_name;

#ifdef __linux__
			float m_ram_percent = 0.0f;
			std::chrono::steady_clock::time_point m_last_ram_update;
			unsigned long long m_cpu_last_total = 0;
			unsigned long long m_cpu_last_idle = 0;
			float m_cpu_percent = 0.0f;
			std::chrono::steady_clock::time_point m_last_cpu_update;
#endif

			// Networking thread
			const long long STALE_TIMEOUT_US = 200000; // Longest stretch (in us) of no new packets until the data is determined stale
			// m_net_thread is started once and parks on m_net_cv_work between ticks instead of being
			// spawned/joined every tick; the mutex + pair of condition variables below form the same
			// one-iteration-at-a-time handshake the old join()-then-spawn pattern gave for free
			std::thread m_net_thread;
			std::mutex m_net_mutex;
			std::condition_variable m_net_cv_work; // main thread -> worker: a new snapshot is ready
			std::condition_variable m_net_cv_done; // worker -> main thread: the snapshot has been processed
			bool m_net_work_ready = false;
			bool m_net_work_done = true;
			bool m_net_shutdown = false;
			// Snapshot handed from the main thread to the worker under m_net_mutex; mirrors NetworkThread's parameters
			bool m_pending_has_ds = false;
			long long m_pending_last_received = 0;
			bool m_pending_enabled = false;
			bool m_pending_clear_enable = false;
			bool m_pending_e_stopped = false;
			bool m_pending_send_ad = false;
			std::vector<std::string> m_pending_opmode_names;
			RobotToDS m_thread_outgoing_packet;
			RobotLog m_thread_log_packet;
			RobotAd m_thread_ad_packet;
			DSToRobot m_thread_inbound_packet;
			bool m_thread_has_ds = false;
			long long m_thread_last_received = 0;
			int m_thread_invalid_packets = 0;
			std::chrono::steady_clock::time_point m_thread_now;
			bool m_thread_recv_ok = false;
			bool m_thread_disconnect = false;
			bool m_thread_decode_failed_hard = false;
			int m_thread_prev_receive_sequence = 0;
			int m_thread_send_sequence = 0;
			int m_thread_log_sequence = 0;
			bool m_driver_station_address_valid = false;
			bool m_log_socket_connecting = false;
			bool m_log_socket_connected = false;
			int m_ad_send_divider = 0;
#ifdef __linux__
			// OUTBOUND SOCKET
			int m_socket_out = -1;
			sockaddr_in m_addr_out{};
			sockaddr_in m_ad_addr{};
			// INBOUND SOCKET
			int m_socket_in = -1;
			// LOG SOCKET
			int m_socket_log = -1;
			sockaddr_in m_addr_log{};
#endif
		};
	}
}