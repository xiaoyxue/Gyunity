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

	CameraToNDC = Transform::Perspective(mFovy, mFilm->mAspectRatio, mFilmDistance, mFilmDistance, 100000.f);

	NDCToCamera = Inverse(CameraToNDC);

	NDCToRaster =
		Transform::Scale(real(mFilm->mResX), real(mFilm->mResY), 1) *
		Transform::Scale(0.5f, -0.5f, 1) *
		Transform::Translate(Vec3(1, -1, 0));

	RasterToNDC = Inverse(NDCToRaster);

	CameraToRaster = NDCToRaster * CameraToNDC;

	RasterToCamera = NDCToCamera * RasterToNDC;

	RasterToWorld = CameraToWorld * RasterToCamera;


	
	mFilm->mHeight = mFilmDistance * std::tan(mFovy * 0.5f * PI / 180) * 2.f;
	mFilm->mWidth = mFilm->mHeight * mFilm->mAspectRatio;
	mFilm->mArea = mFilm->mWidth * mFilm->mHeight;


	Vec3 filmCenter = mPos + mCz * mFilmDistance;
	mFilm->mLU = filmCenter + mCy * mFilm->mHeight * 0.5 - mCx * mFilm->mWidth * 0.5;
	mFilm->mLL = filmCenter - mCy * mFilm->mHeight * 0.5 - mCx * mFilm->mWidth * 0.5;
	mFilm->mRU = filmCenter + mCy * mFilm->mHeight * 0.5 + mCx * mFilm->mWidth * 0.5;
	mFilm->mRL = filmCenter - mCy * mFilm->mHeight * 0.5 + mCx * mFilm->mWidth * 0.5;

	GYT_Print("Film center: {}\n", filmCenter);
	GYT_Print("LL: {} \n", mFilm->mLL);
	GYT_Print("LU: {} \n", mFilm->mLU);
	GYT_Print("RL: {} \n", mFilm->mRL);
	GYT_Print("RU: {} \n", mFilm->mRU);

}


ProjectiveCamera::ProjectiveCamera(std::shared_ptr<Film> pFilm, const Vec3& position, const Vec3& cz, const Vec3& cx, const Vec3& cy, real pFovy, real filmDis)
	: Camera(pFilm, position, cx, cy, cz, pFovy, filmDis)
{
	Initialize();
}

ProjectiveCamera::ProjectiveCamera(std::shared_ptr<Film> pFilm, const Vec3& position, const Vec3& lookAt, const Vec3& up, real fovy, real filmDis)
	: Camera(pFilm, position, lookAt, up,fovy, filmDis)
{
	Initialize();
}

GYT_NAMESPACE_END

