#include "Constants.hpp"

namespace WINDOW {
	const uint32_t WIDTH = 800;// 1024;
	const uint32_t HEIGHT = 600;// 768;

	const char* TITLE = "PhysicsEngine";

	const float MAX_DT = 0.05f; // Min 20fps
	const float TARGET_FPS = 60.0f;
	const float TARGET_DT = 1.0f / TARGET_FPS;
}

namespace COLOURS {
	const Framework::Colour BLACK{ 0x00, 0x00, 0x00 };
	const Framework::Colour WHITE{ 0xFF, 0xFF, 0xFF };
	const Framework::Colour RED{ 0xFF, 0x00, 0x00 };
	const Framework::Colour YELLOW{ 0xFF, 0xFF, 0x00 };
}