#include "PinholeCamera.h"
#include "visual/Film.h"

GYT_NAMESPACE_BEGIN

//Vec3 PinHoleCamera::We(const Ray& ray) const  {
//	real pdfA = 1.0; // for pinhole camera
//	real area = mFilm->mArea;
//	real cosTheta = mCz.Dot(ray.mDir);
//	real cos2Theta = cosTheta * cosTheta;
//	real value = mFilmDistance * mFilmDistance * pdfA / (area * cos2Theta * cos2Theta);
//	return Vec3(value, value, value);
//}
//
//real PinHoleCamera::PdfDir(const Ray& cameraRay) const {
//	real filmArea = mFilm->Area();
//	real cosTheta = std::abs(mCz.Dot(cameraRay.mDir));
//	real cos2Theta = cosTheta * cosTheta;
//	return mFilmDistance * mFilmDistance / (filmArea * cos2Theta * cosTheta);
//}

GYT_NAMESPACE_END