#include "driver_station.hpp"

namespace bellman
{
	namespace hal
	{
		DriverStation::DriverStation()
		{

		}

		void DriverStation::ConfigureDS()
		{
			m_state = State::GetInstance();
			m_device = Device::GetInstance();
			m_has_ds = false;
			m_initialized = false;

			// Calculate the IP based on the device ID
			m_multicast_ip = "239." + std::to_string(m_device->GetDeviceInfo().ID / 100) + "." + std::to_string(m_device->GetDeviceInfo().ID % 100) + ".0";

			// Zero-init nanopb structures
			m_outgoing_packet = RobotToDS_init_zero;
			m_outgoing_log_packet = RobotLog_init_zero;
			m_inbound_packet = DSToRobot_init_zero;
			m_thread_outgoing_packet = RobotToDS_init_zero;
			m_thread_log_packet = RobotLog_init_zero;
			m_thread_ad_packet = RobotAd_init_zero;
			m_thread_inbound_packet = DSToRobot_init_zero;

			// Take a snapshot of the current opmodes
			m_opmodes = m_state->GetModes();
			for (const std::pair<const std::string, std::shared_ptr<RobotMode>>& entry : m_opmodes) { m_opmode_names.push_back(entry.first); }

#ifdef __linux__
			// OUTBOUND PORT
			// Setup the socket
			m_socket_out = socket(AF_INET, SOCK_DGRAM, 0);
			if (m_socket_out < 0)
			{
				std::cerr << "Failed to setup socket" << std::endl;
				m_initialized = false;
				return;
			}

			// Allow quick rebinding during restart
			int reuse = 1;
			if (setsockopt(m_socket_out, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0)
			{
				std::cerr << "Failed to set SO_REUSEADDR on outbound socket" << std::endl;
				m_initialized = false;
				return;
			}

			// Set the multicast TTL
			int ttl = 1;
			if (setsockopt(m_socket_out, IPPROTO_IP, IP_MULTICAST_TTL, &ttl, sizeof(ttl)) < 0)
			{
				std::cerr << "Failed to set TTL" << std::endl;
				m_initialized = false;
				return;
			}

			// Set the destination ports
			m_addr_out = {};
			m_addr_out.sin_family = AF_INET;
			m_addr_out.sin_port = htons(11140);
			m_ad_addr = {};
			m_ad_addr.sin_family = AF_INET;
			m_ad_addr.sin_port = htons(11150);
			if (inet_pton(AF_INET, m_multicast_ip.c_str(), &m_ad_addr.sin_addr) != 1)
			{
				std::cerr << "Invalid multicast address" << std::endl;
				close(m_socket_out);
				m_initialized = false;
				return;
			}
			m_addr_log = {};
			m_addr_log.sin_family = AF_INET;
			m_addr_log.sin_port = htons(11140);

			// Bind the socket to the robot network
			const char* iface = m_device->GetDeviceInfo().robot_network.c_str();

			if (setsockopt(m_socket_out, SOL_SOCKET, SO_BINDTODEVICE, iface, strlen(iface)) < 0)
			{
				std::cerr << "Failed to bind to interface " << m_device->GetDeviceInfo().robot_network << std::endl;
				m_initialized = false;
				return;
			}

			// INBOUND PORT
			// Setup the socket
			m_socket_in = socket(AF_INET, SOCK_DGRAM, 0);
			if (m_socket_in < 0)
			{
				std::cerr << "Failed to setup inbound port" << std::endl;
				m_initialized = false;
				return;
			}

			m_socket_log = -1;
			m_driver_station_address_valid = false;
			m_log_socket_connecting = false;
			m_log_socket_connected = false;
			m_thread_log_sequence = 0;
			m_ad_send_divider = 0;

			// Allow quick rebinding during restart
			reuse = 1;
			setsockopt(m_socket_in, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

			// Bind to interface
			iface = m_device->GetDeviceInfo().robot_network.c_str();

			if (setsockopt(m_socket_in, SOL_SOCKET, SO_BINDTODEVICE, iface, strlen(iface)) < 0)
			{
				std::cerr << "Failed to bind to interface " << m_device->GetDeviceInfo().robot_network << std::endl;
				m_initialized = false;
				return;
			}

			// Make socket non-blocking
			int flags = fcntl(m_socket_in, F_GETFL, 0);
			if (flags < 0 || fcntl(m_socket_in, F_SETFL, flags | O_NONBLOCK) < 0)
			{
				std::cerr << "Failed to set non-blocking mode" << std::endl;
				m_initialized = false;
				return;
			}

			// Set the address
			sockaddr_in addr{};
			addr.sin_family = AF_INET;
			addr.sin_addr.s_addr = INADDR_ANY;
			addr.sin_port = htons(11130);

			if (bind(m_socket_in, (sockaddr*)&addr, sizeof(addr)) < 0)
			{
				std::cerr << "Failed to bind to inbound port" << std::endl;
				m_initialized = false;
				return;
			}
#endif

			m_initialized = true;
		}

		DriverStation::~DriverStation()
		{
#ifdef __linux__
			// The worker parks indefinitely on m_net_cv_work now, so unlike the old spawn-per-tick
			// design a bare join() here would hang forever - wake it with the shutdown flag first,
			// then join, guaranteeing no NetworkThread() iteration is still touching the sockets
			// we're about to close below.
			if (m_net_thread.joinable())
			{
				{
					std::lock_guard<std::mutex> lock(m_net_mutex);
					m_net_shutdown = true;
				}
				m_net_cv_work.notify_one();
				m_net_thread.join();
			}

			if (m_socket_log >= 0)
			{
				close(m_socket_log);
				m_socket_log = -1;
			}

			if (m_socket_in >= 0)
			{
				close(m_socket_in);
				m_socket_in = -1;
			}

			if (m_socket_out >= 0)
			{
				close(m_socket_out);
				m_socket_out = -1;
			}
#endif

			m_initialized = false;
			m_has_ds = false;
			m_driver_station_address_valid = false;
			m_log_socket_connecting = false;
			m_log_socket_connected = false;
		}

		DriverStation* DriverStation::GetInstance()
		{
			if (instance == nullptr)
			{
				instance = new DriverStation();
			}
			return instance;
		}

		void DriverStation::SendLogMessage(std::string level, std::string caller, std::string message)
		{
			if (m_outgoing_log_packet.logs_count >= sizeof(m_outgoing_log_packet.logs) / sizeof(m_outgoing_log_packet.logs[0])) { return; }

			RobotLog_LogEntry& log = m_outgoing_log_packet.logs[m_outgoing_log_packet.logs_count++];
			std::snprintf(log.level, sizeof(log.level), "%s", level.c_str());
			std::snprintf(log.caller, sizeof(log.caller), "%s", caller.c_str());
			std::snprintf(log.contents, sizeof(log.contents), "%s", message.c_str());
		}

		/**
		* @brief Sets the battery percentage reported in the driver station
		*
		* @param battery The battery's current percent charged
		*/
		void DriverStation::SetBattery(float percent, float volts)
		{
			m_outgoing_packet.battery_percent = static_cast<int32_t>(std::lround(percent));
			m_outgoing_packet.battery_volts = std::round(volts * 100.0f) / 100.0f; // Limit the battery voltage to 2 decimal points
		}

		/**
		* @brief Gets the Driver Station's connection status
		*
		* @returns true if the driver station is connected
		*/
		bool DriverStation::GetHasDS() { return m_has_ds; }

		/**
		* @brief Gets the name of the currently active opmode
		*
		* @returns The active opmode's name
		*/
		std::string DriverStation::GetActiveMode() { return m_active_opmode_name; }

		float DriverStation::GetCPUPercent()
		{
#ifdef __linux__
			std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();

			// Update twice per second
			if (now - m_last_cpu_update > std::chrono::milliseconds(500))
			{
				m_last_cpu_update = now;

				std::ifstream stat("/proc/stat");

				std::string cpu;
				unsigned long long user, nice, system, idle, iowait, irq, softirq, steal;

				stat >> cpu >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;

				unsigned long long idle_time = idle + iowait;
				unsigned long long total_time = user + nice + system + idle + iowait + irq + softirq + steal;

				if (m_cpu_last_total != 0)
				{
					unsigned long long total_delta = total_time - m_cpu_last_total;
					unsigned long long idle_delta = idle_time - m_cpu_last_idle;

					if (total_delta != 0) { m_cpu_percent = 100.0f * (total_delta - idle_delta) / total_delta; }
				}

				m_cpu_last_total = total_time;
				m_cpu_last_idle = idle_time;
			}

			return m_cpu_percent;
#else
			return 0.0f;
#endif
		}

		/**
		* @brief Gets the current RAM usage of the robot controller
		*
		* @returns The percentage of the system's RAM that is used
		*/
		float DriverStation::GetRAMPercent()
		{
#ifdef __linux__
			std::chrono::time_point<std::chrono::steady_clock> now = std::chrono::steady_clock::now();

			// Update twice per second
			if (now - m_last_ram_update > std::chrono::milliseconds(500))
			{
				m_last_ram_update = now;

				std::ifstream meminfo("/proc/meminfo");

				std::string key;
				long value;
				std::string unit;

				long mem_total = 0;
				long mem_available = 0;

				while (meminfo >> key >> value >> unit)
				{
					if (key == "MemTotal:") { mem_total = value; }
					else if (key == "MemAvailable:") { mem_available = value; }
				}

				if (mem_total != 0)
				{
					long used = mem_total - mem_available;
					m_ram_percent = static_cast<float>(100.0 * used / mem_total);
				}
			}

			return m_ram_percent;
#else
			return 0.0f;
#endif
		}

		void DriverStation::RestartRobotCode()
		{
			m_state->Disable();
			std::cout << "Restarting robot code" << std::endl;
#ifdef __linux__
			std::system("systemctl restart bellman.service");
#endif
		}

		void DriverStation::RestartController()
		{
			m_state->Disable();
			std::cout << "Restarting robot controller" << std::endl;
#ifdef __linux__
			std::system("systemctl reboot");
#endif
		}

		void DriverStation::RegisterEStop()
		{
			if (!m_e_stopped) // Prevent constantly setting sticky fault
			{
				m_state->EStop();
				m_e_stopped = true;
				m_device->SetStickyFault("Driver station requested Emergency Stop");
			}
		}

		bool DriverStation::Periodic()
		{
			if (m_initialized)
			{
				bool clear_enable_snapshot = false;

				if (!m_net_thread.joinable())
				{
					// First tick: start the persistent worker. It immediately parks on m_net_cv_work,
					// so there's nothing to sync yet - same as the old code having no thread to join yet.
					m_net_thread = std::thread(&DriverStation::NetworkThreadLoop, this);
				}
				else
				{
					// Wait for the previous tick's network work to finish. This is the same rendezvous the old
					// join() gave us - the worker is guaranteed idle (parked back on m_net_cv_work) once this
					// returns, so it's safe to read the m_thread_* fields it just wrote without a race.
					std::unique_lock<std::mutex> lock(m_net_mutex);
					m_net_cv_done.wait(lock, [this] { return m_net_work_done; });
					lock.unlock();

					// Sync thread results to live state
					m_now = m_thread_now;
					m_last_received = m_thread_last_received;
					m_has_ds = m_thread_has_ds;
					if (m_thread_recv_ok) { m_inbound_packet = m_thread_inbound_packet; } // Only update on a clean receive

					// Apply E-Stop state
					if (m_inbound_packet.e_stop) { RegisterEStop(); }

					// Restart robot code/controller if requested
					if (m_inbound_packet.restart_code) { RestartRobotCode(); }
					if (m_inbound_packet.restart_controller) { RestartController(); }

					// Set the active opmode
					m_active_opmode_name = m_inbound_packet.opmode;

					// Apply state changes based on network results
					if (m_thread_disconnect || m_thread_decode_failed_hard)
					{
						m_state->Disable();
					}
					else if (m_thread_recv_ok)
					{
						if (m_inbound_packet.enabled && m_device->GetStickyFaults().empty()) { m_state->Enable(); }
						else if (m_inbound_packet.enabled && !m_device->GetStickyFaults().empty())
						{
							m_state->Disable();
							clear_enable_snapshot = true;
							for (const std::string& fault : m_device->GetStickyFaults())
							{
								std::cout << "Unable to enable due to sticky fault: " << fault << std::endl;
								SendLogMessage("FATAL", "Device Manager", "Unable to enable due to sticky fault: " + fault);
							}
						}
						else { m_state->Disable(); }
					}
					if (m_inbound_packet.e_stop) { RegisterEStop(); }
				}

				// Capture the just-completed iteration's status before handing off new work - once the worker
				// is signaled it may start overwriting m_thread_recv_ok on another core immediately.
				bool recv_ok_this_tick = m_thread_recv_ok;

				// Snapshot values the thread needs, taken after sync so they reflect the just-applied state
				bool has_ds_snapshot = m_has_ds;
				long long last_received_snapshot = m_last_received;
				bool enabled_snapshot = m_state->GetEnabled();
				bool e_stop_snapshot = m_state->GetEStopped();

				// Swap the staging outgoing packet to the thread (logs/battery written this loop go out next send)
				m_thread_outgoing_packet = m_outgoing_packet;
				m_outgoing_packet = RobotToDS_init_zero;
				m_thread_log_packet = m_outgoing_log_packet;
				m_outgoing_log_packet = RobotLog_init_zero;

				bool send_ad_snapshot = ((m_ad_send_divider++ % 2) == 0);

				// Hand the next iteration's snapshot to the worker and wake it - it runs in parallel with
				// the rest of the loop, same as the old spawn-a-thread-per-tick did.
				{
					std::lock_guard<std::mutex> lock(m_net_mutex);
					m_pending_has_ds = has_ds_snapshot;
					m_pending_last_received = last_received_snapshot;
					m_pending_enabled = enabled_snapshot;
					m_pending_clear_enable = clear_enable_snapshot;
					m_pending_e_stopped = e_stop_snapshot;
					m_pending_send_ad = send_ad_snapshot;
					m_pending_opmode_names = m_opmode_names;
					m_net_work_done = false;
					m_net_work_ready = true;
				}
				m_net_cv_work.notify_one();

				clear_enable_snapshot = false;

				return recv_ok_this_tick; // Status of the iteration that just completed
			}
			else { return false; }
		}

		// Persistent worker body. Parks on m_net_cv_work until Periodic() hands off a new
		// snapshot (or the destructor requests shutdown), runs exactly one NetworkThread()
		// iteration per wakeup, then reports back via m_net_cv_done - the same one-iteration-
		// at-a-time cadence the old spawn-a-thread-per-tick design gave for free.
		void DriverStation::NetworkThreadLoop()
		{
			while (true)
			{
				bool has_ds_snapshot;
				long long last_received_snapshot;
				bool enabled_snapshot;
				bool clear_enable_snapshot;
				bool e_stopped_snapshot;
				bool send_ad_snapshot;
				std::vector<std::string> opmode_name_snapshot;

				{
					std::unique_lock<std::mutex> lock(m_net_mutex);
					m_net_cv_work.wait(lock, [this] { return m_net_work_ready || m_net_shutdown; });

					if (m_net_shutdown)
					{
						// Discard any still-pending work - the destructor is about to close the sockets
						// and nothing is waiting on m_net_cv_done for this iteration anymore.
						return;
					}

					has_ds_snapshot = m_pending_has_ds;
					last_received_snapshot = m_pending_last_received;
					enabled_snapshot = m_pending_enabled;
					clear_enable_snapshot = m_pending_clear_enable;
					e_stopped_snapshot = m_pending_e_stopped;
					send_ad_snapshot = m_pending_send_ad;
					opmode_name_snapshot = m_pending_opmode_names;
					m_net_work_ready = false;
				}

				// Runs unlocked, same as the old per-tick thread's body did - only the handoff is guarded
				NetworkThread(has_ds_snapshot, last_received_snapshot, enabled_snapshot, clear_enable_snapshot, e_stopped_snapshot, send_ad_snapshot, opmode_name_snapshot);

				{
					std::lock_guard<std::mutex> lock(m_net_mutex);
					m_net_work_done = true;
				}
				m_net_cv_done.notify_one();
			}
		}

		void DriverStation::NetworkThread(bool has_ds_snapshot, long long last_received_snapshot, bool enabled_snapshot, bool clear_enable_snapshot, bool e_stopped_snapshot, bool send_ad_snapshot, std::vector<std::string> opmode_name_snapshot)
		{
			long long now_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
			if (!m_has_ds) { m_thread_send_sequence = 0; }

			m_thread_recv_ok = false;
			m_thread_disconnect = false;
			m_thread_decode_failed_hard = false;

			// Build the outgoing packet
			m_thread_outgoing_packet.sequence = m_thread_send_sequence;
			m_thread_send_sequence++;
			std::string version_str{ hal::Device::m_version };
			std::snprintf(m_thread_outgoing_packet.version, sizeof(m_thread_outgoing_packet.version), "%s", version_str.c_str());
			m_thread_outgoing_packet.cpu = GetCPUPercent(); // Set CPU percentage
			m_thread_outgoing_packet.ram = GetRAMPercent(); // Set RAM used percentage
			m_thread_outgoing_packet.has_ds = has_ds_snapshot; // Set has Driver Station
			m_thread_outgoing_packet.enabled = enabled_snapshot; // Set enabled
			m_thread_outgoing_packet.clear_enable = clear_enable_snapshot; // Set clear enabled
			m_thread_outgoing_packet.e_stopped = e_stopped_snapshot; // Set E-Stopped
			// Populate the opmodes
			m_thread_outgoing_packet.opmodes_count = std::min<pb_size_t>(opmode_name_snapshot.size(), 16);
			for (pb_size_t i = 0; i < m_thread_outgoing_packet.opmodes_count; ++i) { std::snprintf(m_thread_outgoing_packet.opmodes[i], sizeof(m_thread_outgoing_packet.opmodes[i]), "%s", opmode_name_snapshot[i].c_str()); }

			// Set the timestamp right before sending
			m_thread_now = std::chrono::steady_clock::now();
			long long sent_time = std::chrono::duration_cast<std::chrono::microseconds>(m_thread_now.time_since_epoch()).count();
			m_thread_outgoing_packet.timestamp_us = sent_time;

			// Build the robot ad packet when needed
			if (send_ad_snapshot)
			{
				m_thread_ad_packet = RobotAd_init_zero;
				m_thread_ad_packet.timestamp_us = sent_time;
				std::snprintf(m_thread_ad_packet.version, sizeof(m_thread_ad_packet.version), "%s", version_str.c_str());
				m_thread_ad_packet.battery_percent = m_thread_outgoing_packet.battery_percent;
				m_thread_ad_packet.battery_volts = m_thread_outgoing_packet.battery_volts;
				m_thread_ad_packet.cpu = m_thread_outgoing_packet.cpu;
				m_thread_ad_packet.ram = m_thread_outgoing_packet.ram;
				m_thread_ad_packet.has_ds = has_ds_snapshot;
				m_thread_ad_packet.enabled = enabled_snapshot;
				m_thread_ad_packet.e_stopped = e_stopped_snapshot;
				std::snprintf(m_thread_ad_packet.opmode, sizeof(m_thread_ad_packet.opmode), "%s", m_active_opmode_name.c_str());
			}

			uint8_t payload[RobotToDS_size];
			pb_ostream_t out_stream = pb_ostream_from_buffer(payload, sizeof(payload));
			if (!pb_encode(&out_stream, RobotToDS_fields, &m_thread_outgoing_packet))
			{
				std::cerr << "Failed to serialize protobuf" << std::endl;
				m_thread_last_received = last_received_snapshot;
				m_thread_has_ds = has_ds_snapshot;
				return;
			}
			size_t payload_len = out_stream.bytes_written;

			m_thread_outgoing_packet = RobotToDS_init_zero; // Clear the outgoing packet

#ifdef __linux__
			// ADVERTISEMENT
			if (send_ad_snapshot)
			{
				uint8_t ad_payload[RobotAd_size];
				pb_ostream_t ad_stream = pb_ostream_from_buffer(ad_payload, sizeof(ad_payload));
				if (pb_encode(&ad_stream, RobotAd_fields, &m_thread_ad_packet))
				{
					ssize_t ad_sent = sendto(m_socket_out, ad_payload, ad_stream.bytes_written, 0, reinterpret_cast<sockaddr*>(&m_ad_addr), sizeof(m_ad_addr));
					if (ad_sent < 0)
					{
						std::cerr << "Failed to send advertisement packet" << std::endl;
					}
				}
			}

			// INCOMING DATA
			uint8_t buffer[DSToRobot_size];

			sockaddr_in src{};
			socklen_t srclen = sizeof(src);

			ssize_t len = recvfrom(m_socket_in, buffer, sizeof(buffer), 0, (sockaddr*)&src, &srclen);
			if (len >= 0)
			{
				DSToRobot tmp = DSToRobot_init_zero;
				pb_istream_t in_stream = pb_istream_from_buffer(buffer, (size_t)len);
				if (!pb_decode(&in_stream, DSToRobot_fields, &tmp))
				{
					m_thread_invalid_packets++;
					if (m_thread_invalid_packets > 3)
					{
						m_thread_decode_failed_hard = true;
						std::cerr << "Failed to parse protobuf" << std::endl;
						m_thread_last_received = last_received_snapshot;
						m_thread_has_ds = false;
					}
					else
					{
						m_thread_last_received = last_received_snapshot;
						m_thread_has_ds = has_ds_snapshot;
					}
					return;
				}
				m_thread_inbound_packet = tmp;

				if (!m_driver_station_address_valid)
				{
					m_addr_out = {};
					m_addr_out.sin_family = AF_INET;
					m_addr_out.sin_addr = src.sin_addr;
					m_addr_out.sin_port = htons(11140);
					m_addr_log = m_addr_out;
					m_addr_log.sin_port = htons(11140);
					m_driver_station_address_valid = true;
					m_thread_prev_receive_sequence = 0;
					m_log_socket_connecting = false;
					m_log_socket_connected = false;
					if (m_socket_log >= 0) { close(m_socket_log); m_socket_log = -1; }
					std::cout << "Driver station connected from " << inet_ntoa(src.sin_addr) << std::endl;
				}
				else if (m_addr_out.sin_addr.s_addr != src.sin_addr.s_addr)
				{
					return;
				}
				m_thread_recv_ok = true;


			}
			else
			{
				// Check if DS data is stale with 200ms timeout
				if (((now_us - last_received_snapshot) > STALE_TIMEOUT_US) && (last_received_snapshot > 0))
				{
					m_thread_disconnect = true;
					std::cerr << "Driver Station disconnected" << std::endl;
					m_thread_last_received = 0;
					m_thread_has_ds = false;
					m_thread_prev_receive_sequence = 0;
					m_driver_station_address_valid = false;
					m_log_socket_connecting = false;
					m_log_socket_connected = false;
					if (m_socket_log >= 0) { close(m_socket_log); m_socket_log = -1; }
				}
				else
				{
					m_thread_last_received = last_received_snapshot;
					m_thread_has_ds = has_ds_snapshot;
				}
			}

			// OUTBOUND DATA
			if (m_driver_station_address_valid)
			{
				ssize_t sent = sendto(m_socket_out, payload, payload_len, 0, reinterpret_cast<sockaddr*>(&m_addr_out), sizeof(m_addr_out));
				if (sent < 0)
				{
					std::cerr << "Failed to send packet" << std::endl;
					m_thread_last_received = last_received_snapshot;
					m_thread_has_ds = has_ds_snapshot;
				}
			}

			// LOG DATA
			if (m_driver_station_address_valid)
			{
				if (m_socket_log < 0)
				{
					m_socket_log = socket(AF_INET, SOCK_STREAM, 0);
					if (m_socket_log >= 0)
					{
						int flags = fcntl(m_socket_log, F_GETFL, 0);
						if (flags >= 0) { fcntl(m_socket_log, F_SETFL, flags | O_NONBLOCK); }
						int nodelay = 1;
						setsockopt(m_socket_log, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
						m_addr_log = m_addr_out;
						m_addr_log.sin_port = htons(11140);
						int rc = connect(m_socket_log, reinterpret_cast<sockaddr*>(&m_addr_log), sizeof(m_addr_log));
						if (rc == 0)
						{
							m_log_socket_connected = true;
							m_log_socket_connecting = false;
						}
						else if (errno == EINPROGRESS || errno == EWOULDBLOCK || errno == EALREADY)
						{
							m_log_socket_connecting = true;
						}
						else
						{
							close(m_socket_log);
							m_socket_log = -1;
							m_log_socket_connected = false;
							m_log_socket_connecting = false;
						}
					}
				}

				if (m_socket_log >= 0 && !m_log_socket_connected)
				{
					pollfd pfd{};
					pfd.fd = m_socket_log;
					pfd.events = POLLOUT;
					if (poll(&pfd, 1, 0) > 0)
					{
						int err = 0;
						socklen_t err_len = sizeof(err);
						if (getsockopt(m_socket_log, SOL_SOCKET, SO_ERROR, &err, &err_len) == 0 && err == 0)
						{
							m_log_socket_connected = true;
							m_log_socket_connecting = false;
						}
						else
						{
							close(m_socket_log);
							m_socket_log = -1;
							m_log_socket_connected = false;
							m_log_socket_connecting = false;
						}
					}
				}

				if (m_socket_log >= 0 && m_log_socket_connected && m_thread_log_packet.logs_count > 0)
				{
					m_thread_log_packet.timestamp_us = sent_time;
					m_thread_log_packet.sequence = m_thread_log_sequence++;
					uint8_t log_payload[RobotLog_size + 8];
					pb_ostream_t log_stream = pb_ostream_from_buffer(log_payload, sizeof(log_payload));
					if (pb_encode_delimited(&log_stream, RobotLog_fields, &m_thread_log_packet))
					{
						ssize_t sent = send(m_socket_log, log_payload, log_stream.bytes_written, 0);
						if (sent < 0)
						{
							close(m_socket_log);
							m_socket_log = -1;
							m_log_socket_connected = false;
							m_log_socket_connecting = false;
						}
						else
						{
							m_thread_log_packet = RobotLog_init_zero;
						}
					}
				}
			}


#endif

			// Handle the incoming data
			if (m_thread_recv_ok)
			{
				// Check if any packets were dropped
				if (m_thread_inbound_packet.sequence == 0) { m_thread_prev_receive_sequence = 0; } // New or restarted connection DS-side
				// Widen to a signed 64-bit delta so a DS restart (sequence resetting below m_thread_prev_receive_sequence)
				// cannot underflow into a bogus, huge unsigned "dropped packets" count
				long long sequence_delta = static_cast<long long>(m_thread_inbound_packet.sequence) - static_cast<long long>(m_thread_prev_receive_sequence);
				if (sequence_delta > 5) // Only print this after 5+ consecutive packets are dropped, as WiFi is unreliable
				{
					std::cerr << "Dropped " << sequence_delta - 1 << " packets" << std::endl;
				}
				m_thread_prev_receive_sequence = m_thread_inbound_packet.sequence;

				m_thread_last_received = now_us;
				m_thread_invalid_packets = 0;
				m_thread_has_ds = true;
				m_thread_recv_ok = true;
			}

		}

		/**
				* @brief Gets the state of a button from the driver station
				*
				* @param joystick The joystick's ID number
				* @param input The button's ID number on the joystick
				*
				* @returns The state of the button
				*/
		std::optional<bool> DriverStation::GetButton(int joystick, int input)
		{
			if (!m_has_ds) { return std::nullopt; }
			for (pb_size_t i = 0; i < m_inbound_packet.controls_count; i++)
			{
				const DSToRobot_Joystick& js = m_inbound_packet.controls[i];
				if ((int)js.id == joystick)
				{
					if (input < 0 || input >= (int)js.buttons_count) { return std::nullopt; }
					return js.buttons[input];
				}
			}
			return std::nullopt;
		}


		/**
		* @brief Gets the state of a button from the driver station
		*
		* @param joystick The joystick's ID number
		* @param input The axis ID number on the joystick
		*
		* @returns The value of the axis (from -1 to 1)
		*/
		std::optional<float> DriverStation::GetAxis(int joystick, int input)
		{
			if (!m_has_ds) { return std::nullopt; }
			for (pb_size_t i = 0; i < m_inbound_packet.controls_count; i++)
			{
				const DSToRobot_Joystick& js = m_inbound_packet.controls[i];
				if ((int)js.id == joystick)
				{
					if (input < 0 || input >= (int)js.axes_count) { return std::nullopt; }
					return js.axes[input];
				}
			}
			return std::nullopt;
		}

		/**
		* @brief Gets the joystick's direction input (normally a d-pad)
		*
		* @param joystick The joystick's ID number
		*
		* @returns The Direction
		*/
		std::optional<DriverStation::Direction> DriverStation::GetDirection(int joystick)
		{
			if (!m_has_ds) { return std::nullopt; }
			for (pb_size_t i = 0; i < m_inbound_packet.controls_count; i++)
			{
				const DSToRobot_Joystick& js = m_inbound_packet.controls[i];
				if ((int)js.id == joystick)
				{
					return static_cast<DriverStation::Direction>(js.direction);
				}
			}
			return std::nullopt;
		}
	}
	hal::DriverStation* hal::DriverStation::instance = nullptr;
}