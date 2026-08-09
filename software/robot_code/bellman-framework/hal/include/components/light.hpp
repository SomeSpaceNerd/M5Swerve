#pragma once

#include <cstdint>

namespace bellman
{
	namespace hal
	{
		namespace component
		{
			class Light
			{
				public:
					// Color correction values, taken from FastLED
					enum ColorCorrection
					{
						kSMD5050 = 0xFFB0F0, // 255, 176, 240
						k8mmPixel = 0xFFE08C, // 255, 224, 140
						kUncorrected = 0xFFFFFF, // 255, 255, 255
					};

					virtual ~Light() = default;
					virtual void SetOn(bool on) = 0;
					virtual void SetColor(uint8_t red, uint8_t green, uint8_t blue) = 0;
					virtual void SetCorrection(ColorCorrection correction) = 0;
			};
		}
	}
}