#include "Film.h"
#include "image/ImageIO.h"

GYT_NAMESPACE_BEGIN

bool Film::SaveImage() {
	WriteToPixelBuffer();
	return ImageIO::WriteImage(mFilename, mImageBuffer, mResX, mResY);
}

GYT_NAMESPACE_END