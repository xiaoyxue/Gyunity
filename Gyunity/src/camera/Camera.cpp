#include "Camera.h"
#include "visual/Film.h"

GYT_NAMESPACE_BEGIN

namespace {
bool Finite(const Vec3& value) {
	return std::isfinite(value.x) && std::isfinite(value.y) && std::isfinite(value.z);
}

bool NonzeroFinite(const Vec3& value) {
	return Finite(value) && std::isfinite(value.Length2()) && value.Length2() > 0;
}

void CameraError(const char* message) {
	GetDefaultLogger()->Error(message, false);
}
}

void ProjectiveCamera::Initialize() {
	if (!mFilm || mFilm->mResX == 0 || mFilm->mResY == 0 ||
		!Finite(mPos) || !std::isfinite(mFovy) || mFovy <= 0 || mFovy >= 179 ||
		!std::isfinite(mFilmDistance) || mFilmDistance <= 0) {
		CameraError("Invalid projective camera film, position, FOV or film distance.");
		return;
	}
	const Vec3 axes[] = { mCx, mCy, mCz };
	for (int i = 0; i < 3; ++i) {
		if (!Finite(axes[i]) || std::abs(axes[i].Length2() - 1) > 1e-3f) {
			CameraError("Projective camera axes must be finite unit vectors.");
			return;
		}
		for (int j = 0; j < i; ++j) {
			if (std::abs(Dot(axes[i], axes[j])) > 1e-3f) {
				CameraError("Projective camera axes must be orthogonal.");
				return;
			}
		}
	}
	const real height = mFilmDistance * std::tan(mFovy * .5f * PI / 180) * 2;
	const real width = height * mFilm->mAspectRatio;
	if (!(height > 0) || !(width > 0) || !(width * height > 0) || !std::isfinite(width * height)) {
		CameraError("Projective camera film dimensions are not representable.");
		return;
	}

	mCameraToWorld =
		Transform(mCx.x, mCy.x, mCz.x, mPos.x,
			mCx.y, mCy.y, mCz.y, mPos.y,
			mCx.z, mCy.z, mCz.z, mPos.z,
			0, 0, 0, 1.f);

	//Should get the same result
	//CameraToWorld = Transform::LookAt(mPos, mLookAt, mUp);

	mWorldToCamera = Inverse(mCameraToWorld);

	mCameraToNdc = Transform::Perspective(mFovy, mFilm->mAspectRatio, mFilmDistance, mFilmDistance, 100000.f);

	mNdcToCamera = Inverse(mCameraToNdc);

	mNdcToRaster =
		Transform::Scale(real(mFilm->mResX), real(mFilm->mResY), 1) *
		Transform::Scale(0.5f, -0.5f, 1) *
		Transform::Translate(Vec3(1, -1, 0));

	mRasterToNdc = Inverse(mNdcToRaster);

	mCameraToRaster = mNdcToRaster * mCameraToNdc;

	mRasterToCamera = mNdcToCamera * mRasterToNdc;

	mRasterToWorld = mCameraToWorld * mRasterToCamera;


	
	mFilm->mHeight = mFilmDistance * std::tan(mFovy * 0.5f * PI / 180) * 2.f;
	mFilm->mWidth = mFilm->mHeight * mFilm->mAspectRatio;
	mFilm->mArea = mFilm->mWidth * mFilm->mHeight;


	Vec3 filmCenter = mPos + mCz * mFilmDistance;
	mFilm->mUpperLeft = filmCenter + mCy * mFilm->mHeight * 0.5 - mCx * mFilm->mWidth * 0.5;
	mFilm->mLowerLeft = filmCenter - mCy * mFilm->mHeight * 0.5 - mCx * mFilm->mWidth * 0.5;
	mFilm->mUpperRight = filmCenter + mCy * mFilm->mHeight * 0.5 + mCx * mFilm->mWidth * 0.5;
	mFilm->mLowerRight = filmCenter - mCy * mFilm->mHeight * 0.5 + mCx * mFilm->mWidth * 0.5;

	GYT_PRINT("Film center: {}\n", filmCenter);
	GYT_PRINT("LL: {} \n", mFilm->mLowerLeft);
	GYT_PRINT("LU: {} \n", mFilm->mUpperLeft);
	GYT_PRINT("RL: {} \n", mFilm->mLowerRight);
	GYT_PRINT("RU: {} \n", mFilm->mUpperRight);
	mValid = true;

}


ProjectiveCamera::ProjectiveCamera(std::shared_ptr<Film> film, const Vec3& pos, const Vec3& cz, const Vec3& cx, const Vec3& cy, real pFovy, real filmDis)
	: Camera(film, pos, cz, cx, cy, pFovy, filmDis)
{
	Initialize();
}

ProjectiveCamera::ProjectiveCamera(std::shared_ptr<Film> film, const Vec3& pos, const Vec3& lookAt, const Vec3& up, real fovy, real filmDis)
	: Camera(film, pos, lookAt, up,fovy, filmDis)
{
	if (!Finite(mPos) || !Finite(mLookAt) || !NonzeroFinite(mPos - mLookAt) || !NonzeroFinite(mUp)) {
		CameraError("Look-at camera requires finite positions, a nonzero view direction and a nonzero up vector.");
		return;
	}
	mCz = -(mPos - mLookAt).Norm();
	if (Cross(mUp, mCz).Length2() <= 1e-12f * mUp.Length2()) {
		Vec3 unused;
		CoordinateSystem(mCz, &mUp, &unused);
		GetDefaultLogger()->Warn("Camera up is parallel to its view; using a deterministic orthogonal up vector.");
	}
	mCx = -Cross(mUp, mCz).Norm();
	mCy = -Cross(mCz, mCx);
	Initialize();
}

GYT_NAMESPACE_END
