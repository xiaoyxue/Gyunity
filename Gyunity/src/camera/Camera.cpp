#include "Camera.h"
#include "visual/Film.h"

GYT_NAMESPACE_BEGIN

void ProjectiveCamera::Initialize() {

	mCz = -(mPos - mLookAt).Norm(); // -1 * dir in raytracer
	mCx = -Cross(mUp, mCz).Norm();
	mCy = -Cross(mCz, mCx);

	CameraToWorld =
		Transform(mCx.x, mCy.x, mCz.x, mPos.x,
			mCx.y, mCy.y, mCz.y, mPos.y,
			mCx.z, mCy.z, mCz.z, mPos.z,
			0, 0, 0, 1.f);

	//Should get the same result
	//CameraToWorld = Transform::LookAt(mPos, mLookAt, mUp);

	WorldToCamera = Inverse(CameraToWorld);

	CameraToNDC = Transform::Perspective(mFovy, mpFilm->mAspectRatio, mFilmDistance, mFilmDistance, 100000.f);

	NDCToCamera = Inverse(CameraToNDC);

	NDCToRaster =
		Transform::Scale(real(mpFilm->mResX), real(mpFilm->mResY), 1) *
		Transform::Scale(0.5f, -0.5f, 1) *
		Transform::Translate(Vec3(1, -1, 0));

	RasterToNDC = Inverse(NDCToRaster);

	CameraToRaster = NDCToRaster * CameraToNDC;

	RasterToCamera = NDCToCamera * RasterToNDC;

	RasterToWorld = CameraToWorld * RasterToCamera;


	
	mpFilm->mHeight = mFilmDistance * std::tan(mFovy * 0.5f * PI / 180) * 2.f;
	mpFilm->mWidth = mpFilm->mHeight * mpFilm->mAspectRatio;
	mpFilm->mArea = mpFilm->mWidth * mpFilm->mHeight;


	Vec3 filmCenter = mPos + mCz * mFilmDistance;
	mpFilm->mLU = filmCenter + mCy * mpFilm->mHeight * 0.5 - mCx * mpFilm->mWidth * 0.5;
	mpFilm->mLL = filmCenter - mCy * mpFilm->mHeight * 0.5 - mCx * mpFilm->mWidth * 0.5;
	mpFilm->mRU = filmCenter + mCy * mpFilm->mHeight * 0.5 + mCx * mpFilm->mWidth * 0.5;
	mpFilm->mRL = filmCenter - mCy * mpFilm->mHeight * 0.5 + mCx * mpFilm->mWidth * 0.5;

	GYT_Print("Film center: {}\n", filmCenter);
	GYT_Print("LL: {} \n", mpFilm->mLL);
	GYT_Print("LU: {} \n", mpFilm->mLU);
	GYT_Print("RL: {} \n", mpFilm->mRL);
	GYT_Print("RU: {} \n", mpFilm->mRU);

}


ProjectiveCamera::ProjectiveCamera(std::shared_ptr<Film> pFilm, const Vec3& position, const Vec3& cz, const Vec3& cx, const Vec3& cy, real pFovy, real filmDis)
	: Camera(pFilm, position, cz, cx, cy, pFovy, filmDis)
{
	// Initialize() derives mCx/mCy/mCz from mLookAt/mUp, so synthesize them from
	// the supplied axes. Without this, mLookAt and mUp default to (0,0,0), which
	// causes Cross(mUp, mCz).Norm() to divide by zero and produce NaNs throughout
	// the camera basis (RasterToWorld, We(), PdfDir(), film corners, ...).
	mLookAt = position + cz;
	mUp = cy;
	Initialize();
}

ProjectiveCamera::ProjectiveCamera(std::shared_ptr<Film> pFilm, const Vec3& position, const Vec3& lookAt, const Vec3& up, real fovy, real filmDis)
	: Camera(pFilm, position, lookAt, up,fovy, filmDis)
{
	Initialize();
}

GYT_NAMESPACE_END

