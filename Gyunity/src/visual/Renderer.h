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
	Renderer(const std::shared_ptr<Scene> &scene, const std::shared_ptr<Camera> &camera, 
		const std::shared_ptr<Integrator> &pIntegrator, const std::shared_ptr<Film> &film) 
		: mScene(scene), mIntegrator(pIntegrator), mFilm(film), mCamera(camera) 
	{

	}
	void Render() 
	{
		mIntegrator->Render(*mScene, *mCamera);
		mFilm->SaveImage();
	}
};

GYT_NAMESPACE_END