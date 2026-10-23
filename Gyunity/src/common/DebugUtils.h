#pragma once
#include "common/Core.h"


GYT_NAMESPACE_BEGIN

extern int debugPixel[100];

#define GYT_DEBUG_PIXEL(pixelX, pixelY, threadIndex)\
if (x == (pixelX) && y == pixelY){ \
	debugPixel[(threadIndex)] = 1;				   \
}								   \
else{                              \
	debugPixel[(threadIndex)] = 0;				   \
}								   \

#define GYT_DEBUG_PIXEL_V2(x, y, pixelX, pixelY, threadIndex)	\
if (((x) == (pixelX)) && ((y) == (pixelY))){				\
	debugPixel[(threadIndex)] = 1;							\
}															\
else{														\
	debugPixel[(threadIndex)] = 0;							\
}															\


#define GYT_DEBUG_PIXEL_V3(x, y, pixelXmin, pixelYmin, pixelXmax, pixelYmax, threadIndex)	\
if (((x) >= (pixelXmin)) && ((y) >= (pixelYmin)) && ((x) <= (pixelXmax)) && ((x) <= (pixelYmax))){				\
	debugPixel[(threadIndex)] = 1;							\
}															\
else{														\
	debugPixel[(threadIndex)] = 0;							\
}															\

#define GYT_DEBUG_PIXEL_IF(threadIndex)\
if(debugPixel[(threadIndex)] == 1)		\

GYT_NAMESPACE_END