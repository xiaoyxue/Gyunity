#include "Film.h"
#include "image/ImageIO.h"

GYT_NAMESPACE_BEGIN

bool Film::SaveImage() {
	WriteToPixelBuffer();
	return ImageIO::WriteImage(mFilename, mImageBuffer, mResX, mResY);
}

bool Film::SaveImage(const ToneMapSettings& toneMap) {
	if (toneMap.op == ToneMapOperator::None && toneMap.scale == 1) return SaveImage();
	WriteToPixelBuffer();
	std::vector<Vec3> display = mImageBuffer;
	if (!ApplyToneMap(display, toneMap)) return false;
	return ImageIO::WriteSrgbLdrImage(mFilename, display, mResX, mResY);
}

GYT_NAMESPACE_END