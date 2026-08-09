#pragma once

namespace bellman
{
	namespace hal
	{
		namespace bus
		{
			class Bus
			{
			public:
				virtual ~Bus() = default;
				virtual void Periodic() = 0;
			};
		}
	}
}