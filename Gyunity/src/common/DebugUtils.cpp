#include "DebugUtils.h"

GYT_NAMESPACE_BEGIN

namespace {
thread_local bool pixelDebugEnabled = false;
}

void SetPixelDebugEnabled(bool enabled) {
	pixelDebugEnabled = enabled;
}

bool IsPixelDebugEnabled() {
	return pixelDebugEnabled;
}

void SelectDebugPixel(real x, real y, real pixelX, real pixelY) {
	SetPixelDebugEnabled(x == pixelX && y == pixelY);
}

void SelectDebugRegion(real x, real y, real minX, real minY, real maxX, real maxY) {
	SetPixelDebugEnabled(x >= minX && x <= maxX && y >= minY && y <= maxY);
}


GYT_NAMESPACE_END