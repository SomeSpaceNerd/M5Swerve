#pragma once

namespace bellman
{
	namespace hal
	{
		namespace component
		{
			class Component
			{
				public:
					virtual ~Component() = default;
					virtual void Periodic() = 0;
					virtual void Enable() = 0;
					virtual void Disable() = 0;
			};
		}
	}
}