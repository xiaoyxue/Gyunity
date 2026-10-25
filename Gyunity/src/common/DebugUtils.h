#pragma once
#include "common/Core.h"

GYT_NAMESPACE_BEGIN

// Debug selection belongs to the calling thread, not a TBB arena index.
void SetPixelDebugEnabled(bool enabled);
bool IsPixelDebugEnabled();
void SelectDebugPixel(real x, real y, real pixelX, real pixelY);
void SelectDebugRegion(real x, real y, real minX, real minY, real maxX, real maxY);

GYT_NAMESPACE_END

// Compatibility with existing snippets; an explicit thread index is no longer required.
#define GYT_DEBUG_PIXEL(pixelX, pixelY, threadIndex) \
	((void)(threadIndex), Gyunity::SelectDebugPixel(x, y, (pixelX), (pixelY)))
#define GYT_DEBUG_PIXEL_V2(x, y, pixelX, pixelY, threadIndex) \
	((void)(threadIndex), Gyunity::SelectDebugPixel((x), (y), (pixelX), (pixelY)))
#define GYT_DEBUG_PIXEL_V3(x, y, minX, minY, maxX, maxY, threadIndex) \
	((void)(threadIndex), Gyunity::SelectDebugRegion((x), (y), (minX), (minY), (maxX), (maxY)))
#define GYT_DEBUG_PIXEL_IF(threadIndex) \
	if (((void)(threadIndex), Gyunity::IsPixelDebugEnabled()))
