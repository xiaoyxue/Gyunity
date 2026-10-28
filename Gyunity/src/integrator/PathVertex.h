#pragma once

#include "visual/Intersection.h"


GYT_NAMESPACE_BEGIN

class Light;
class Camera;

enum class PathVertexType { Surface, Camera, Light, Environment };

// Surface normal policy of a BDPT subpath. Geometric keeps the initial BDPT model (shading
// frame replaced by the geometric normal); Shading keeps interpolated normals with the
// adjoint-corrected transport of #148. Both subpaths of a render use the same policy.
enum class SurfaceNormalMode { Geometric, Shading };

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
	SurfaceNormalMode mNormalMode = SurfaceNormalMode::Geometric;
	const Light* mLight = nullptr;
	const Camera* mCamera = nullptr;
	Vec3 mDirection;
	real mPdfPos = 0, mPdfDir = 0, mLightSelectionPdf = 0;
};


GYT_NAMESPACE_END