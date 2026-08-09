#include "main.hpp"

std::atomic<bool> running{ true };

void SignalHandler(int) { running = false; }

int main() 
{
	// Setup classes
	Robot robot = Robot(looptime);
	bellman::hal::IHal* hal = bellman::hal::IHal::GetInstance();
	bellman::hal::State* state_control = bellman::hal::State::GetInstance();
	bellman::hal::Logger* logger = bellman::hal::Logger::GetInstance();
	bellman::hal::Device* device = bellman::hal::Device::GetInstance();
	bellman::hal::DriverStation* ds = bellman::hal::DriverStation::GetInstance();
	
	// Setup the signal handler
	std::signal(SIGINT, SignalHandler);
	std::signal(SIGTERM, SignalHandler);

	// Setup variables before locking pages
	bool enabled = false;
	bool prev_enabled = false;
	unsigned int loopcounts = 0;
	bool e_stopped = false;
	std::shared_ptr<bellman::RobotMode> active_opmode;

	std::chrono::steady_clock::time_point end;
	std::chrono::milliseconds overrun;
	std::string message;
	std::chrono::steady_clock::time_point next_tick;

	#ifdef __linux__
	// Lock all current and future memory pages AFTER allocating and initializing classes
	if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
		std::cerr << "mlockall failed" << std::endl;
		return 1;
	}
	#endif

	// Log a startup message
	logger->LogMessage("runtime", "Started bellman runtime V" + std::string(device->m_version) + " on device " + std::string(device->GetDeviceInfo().device_name), bellman::hal::Logger::Level::kInfo);

	next_tick = std::chrono::steady_clock::now(); // Fixed-time loop code

	while (running)
	{
		next_tick += std::chrono::milliseconds(looptime); // Fixed-time loop code

		ds->Periodic(); // Always call DS Periodic BEFORE robot code (not with HAL)

		prev_enabled = enabled;
		enabled = state_control->GetEnabled();

		if (state_control->GetEStopped())
		{
			if (!e_stopped)
			{
				e_stopped = true;
				hal->Disable(e_stopped);
			}
		}

		// Only run hal/robot periodics if we have no sticky faults
		if (device->GetStickyFaults().empty())
		{
			// Run HAl enable/disable before robot code
			if (!enabled && prev_enabled) { hal->Disable(e_stopped); }
			if (enabled && !prev_enabled) 
			{ 
				hal->Enable(); 
				try { active_opmode = state_control->GetModes().at(ds->GetActiveMode()); } // Attempt to get the pointer to the active opmode
				catch (const std::out_of_range& e) // Gets thrown if the opmode is not found in the map
				{
					active_opmode = nullptr;
					logger->LogMessage("runtime", "Enable called with invalid active opmode", bellman::hal::Logger::Level::kError);
				}
			}

			// Run robot code
			robot.RobotPeriodic(loopcounts);
			if (enabled)
			{
				if (active_opmode != nullptr)
				{
					if (!prev_enabled) { active_opmode->Init(); }
					active_opmode->Periodic();
				}
			}
			else if (!enabled)
			{
				if (prev_enabled) { robot.DisabledInit(); }
				robot.DisabledPeriodic();
			}

			hal->Periodic(); // Call HAL Periodic AFTER robot code
		}

		loopcounts++; // Increment the loop counter
		
		// Fixed-time loop code block
		end = std::chrono::steady_clock::now();
		if (end > next_tick) // Looptime was overrun
		{
			overrun = std::chrono::duration_cast<std::chrono::milliseconds>(end - next_tick);
			if (loopcounts == 1) // The first loop often overruns, but that doesn't impact anything so add special handling
			{ 
				message = "Initial loop took " + std::to_string(overrun.count() + looptime) + "ms"; 
				logger->LogMessage("runtime", message, bellman::hal::Logger::Level::kInfo);
			}
			else 
			{ 
				message = "Looptime of " + std::to_string(looptime) + "ms overrun by " + std::to_string(overrun.count()) + "ms"; 
				logger->LogMessage("runtime", message, bellman::hal::Logger::Level::kWarning);
			}
			next_tick = end;
		}
		else
		{
			std::this_thread::sleep_until(next_tick);
		}
	}

	// Runs when SIGINT or SIGTERM is received
	hal->Disable(e_stopped);
	std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Wait a little while for all robot components to become disabled
	// Destructor calls should probably go here
	return 0;
}