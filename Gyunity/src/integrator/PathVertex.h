#pragma once

#include "visual/Intersection.h"


GYT_NAMESPACE_BEGIN

class Light;
class Camera;

enum class PathVertexType { Surface, Camera, Light, Environment };

class PathVertex
{
public:

	Intersection mIsect;
	Vec3 mThroughput;
	real mPdfFwd = 0.0, mPdfPrev = 0.0;
	bool mIsDelta = false;
	// Actual successful delta continuation, before the legacy zero markers
	// replace its densities. Includes component selection, not geometry or eta^2.
	// Zero forward means no event; negative reverse means unsupported component.
	// Only meaningful when this vertex AND its sampled successor are selected.
	real mDeltaPdf = 0, mDeltaReversePdf = 0;
	PathVertexType mType = PathVertexType::Surface;
	const Light* mLight = nullptr;
	const Camera* mCamera = nullptr;
	Vec3 mDirection;
	real mPdfPos = 0, mPdfDir = 0, mLightSelectionPdf = 0;
};


GYT_NAMESPACE_END