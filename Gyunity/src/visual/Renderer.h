#pragma once

#include "integrator/Integrator.h"
#include "Scene.h"

GYT_NAMESPACE_BEGIN

class Renderer 
{
private:
	std::shared_ptr<Scene> mScene;
	std::shared_ptr<Integrator> mIntegrator;
	std::shared_ptr<Film> mFilm;
	std::shared_ptr<Camera> mCamera;
public:
	Renderer(const std::shared_ptr<Scene> &pScene, const std::shared_ptr<Camera> &pCamera, 
		const std::shared_ptr<Integrator> &pIntegrator, const std::shared_ptr<Film> &pFilm) 
		: mScene(pScene), mIntegrator(pIntegrator), mFilm(pFilm), mCamera(pCamera) 
	{

	}
	void Render() 
	{
		mIntegrator->Render(*mScene, *mCamera);
		mFilm->SaveImage();
	}
};

GYT_NAMESPACE_END